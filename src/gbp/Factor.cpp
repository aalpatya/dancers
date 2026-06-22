/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <Utils.h>
#include <gbp/GBPCore.h>
#include <gbp/Factor.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <raylib.h>

/*****************************************************************************************************/
// Factor constructor.
//  - connected_variables: keys of the variables this factor connects (order matters).
//  - groups: each variable's Lie group, in the same order; pass one name to use it for all of them.
//  - observation z: the measurement (the residual h(X) (-) z is taken against it).
//  - sigma_prior_list: per-dof measurement std-devs; precision is diag(sigma_prior_list)^-2.

// Equations eg (2.71) reference the PhD thesis at:
//      https://spiral.imperial.ac.uk/entities/publication/d7f69b3b-29a9-4d28-962a-2f7474454359
//      DOI={https://doi.org/10.25560/128135}
/*****************************************************************************************************/
Factor::Factor(Key fac_key, std::vector<Key> connected_variables, const std::vector<std::string>& groups,
        const Eigen::VectorXd& observation, Eigen::VectorXd sigma_prior_list)
        : fid_(fac_key.node_id_), rid_(fac_key.graph_id_), lid_(fac_key.lid_), key_(fac_key) {
        z_ = observation;
        connected_v_keys_ = connected_variables;

        // Reuse a single group name for every variable; otherwise there must be one name per variable.
        std::vector<std::string> grps = groups;
        if (grps.size() == 1 && connected_v_keys_.size() > 1){
            std::string g0 = grps[0];   // copy out first: assign() reallocates before reading grps[0]
            grps.assign(connected_v_keys_.size(), g0);
        }
        if (grps.size() != connected_v_keys_.size())
            throw std::runtime_error("Factor: number of groups (" + std::to_string(grps.size()) +
                ") must be 1 or match the number of connected variables (" +
                std::to_string(connected_v_keys_.size()) + ")");

        for (size_t i = 0; i < connected_v_keys_.size(); ++i)
            variable_groups_[connected_v_keys_[i]] = makeLieGroup(grps[i]);

        n_dofs_meas_ = z_.size();  // num dof of the observation z
        meas_model_lambda_ = sigma_prior_list.cwiseProduct(sigma_prior_list).cwiseInverse().asDiagonal();

        // Seed each mailbox with a zero-information message in that variable's group.
        for (auto k : connected_v_keys_) {
            Message zero_msg(variable_groups_.at(k));
            inbox_[k] = zero_msg;
            outbox_[k] = zero_msg;
        }

        other_rid_ = connected_v_keys_.back().graph_id_;   // the other robot, for inter-robot factors
    };

Factor::~Factor(){
}

/*****************************************************************************************************/
// Drawing. Base factor draws nothing; subclasses (e.g. DynamicsFactor) override to draw a line.
/*****************************************************************************************************/
void Factor::draw(Eigen::VectorXd p1, Eigen::VectorXd p2, Color color){
}

/*****************************************************************************************************/
// Finite-difference Jacobian of hfunc about the lin point X0, for computeResidualAndJacobian() when there is no
// closed form. Each variable is perturbed in the tangent of its group (X0[k] retracted by delta along
// basis dof j), so the result is dr/dtau. For a Euclidean group the retraction is plain addition.
/*****************************************************************************************************/
Eigen::MatrixXd Factor::jacobianFirstOrder(const std::vector<LieGroup>& X0, std::function<Eigen::MatrixXd(const std::vector<LieGroup>&)> hfunc){
    int total_dofs = 0;
    for (const auto& x : X0) total_dofs += x.dof();   // each variable in its own group (dofs may differ)
    Eigen::MatrixXd h0 = hfunc(X0);
    Eigen::MatrixXd jac_out = Eigen::MatrixXd::Zero(h0.size(), total_dofs);
    int col = 0;
    for (int k = 0; k < (int)X0.size(); k++){
        int dk = X0[k].dof();
        for (int j = 0; j < dk; j++){
            Eigen::VectorXd dtau = Eigen::VectorXd::Zero(dk); dtau(j) = delta_jac;
            std::vector<LieGroup> X = X0;
            X[k] = X0[k] + dtau;
            jac_out(Eigen::all, col + j) = (hfunc(X) - h0) / delta_jac;
        }
        col += dk;
    }
    return jac_out;
};

/*****************************************************************************************************/
// Robust kernel scale for the current residual. Returns 1 (no down-weighting) for a residual within
// the threshold, and a value in (0,1) beyond it, so an outlier factor contributes less. The factor
// energy is approximated by a scaled Gaussian whose information is multiplied by this value:
//   Huber:  scale = (2*c*M - c^2) / M^2   (matches the Huber energy past M = c; cf. gaussianbp.github.io)
//   DCS:    scale = (2*Phi/(Phi+M^2))^2 with Phi = c^2   (dynamic covariance scaling)
// where M = sqrt(r^T Lambda r) is the residual's Mahalanobis distance and c the threshold.
/*****************************************************************************************************/
double Factor::getRobustScale(const Eigen::VectorXd& residual) const {
    if (robust_kernel_ == RobustKernel::None) return 1.0;
    double M2 = residual.transpose() * meas_model_lambda_ * residual;   // squared Mahalanobis distance
    double c  = gbp::config().robust_threshold;
    if (M2 <= c*c) return 1.0;                                          // inside the kernel: plain quadratic
    double M = std::sqrt(M2);
    if (robust_kernel_ == RobustKernel::Huber) return (2.0*c*M - c*c) / M2;
    double Phi = c*c, s = 2.0*Phi / (Phi + M2);                         // DCS
    return s*s;
}

/*****************************************************************************************************/
// Refresh linpoints_ - the per-variable linearisation points used by computeResidualAndJacobian, conditioning,
// retraction and skipFactor. Each is the connected variable's full, current BELIEF mean.
// The message we receive from a variable (msg_in) is its belief WITHOUT this factor's contribution; we
// reconstruct the full belief by multiplying our own last message to it back in (belief = msg_in (x)
// last_outbox_). We only move a lin point when its belief has drifted past relinearise_threshold_.
/*****************************************************************************************************/
void Factor::updateLinearisationPoint(){
    // One lin point per connected variable, each in that variable's own group (its identity); (re)seed
    // if the inbox size changed (topology change).
    if ((int)linpoints_.size() != (int)inbox_.size()){
        linpoints_.clear();
        for (auto& [vkey, msg_in] : inbox_) linpoints_.push_back(variable_groups_.at(vkey));
    }

    int i = 0;
    for (auto& [vkey, msg_in] : inbox_) {
        LieGroup belief_mu = msg_in.mu; // fallback: use incoming mean (no last msg yet)

        auto it = last_outbox_.find(vkey);
        if (it != last_outbox_.end() && it->second.mu.dof() == msg_in.mu.dof()) {
            LieGroup       mu_in = msg_in.mu;
            // Get the deviation of our last sent message from the incoming message at the tangent-space to mu_in
            Eigen::VectorXd tau  = it->second.mu - mu_in;  // our last msg mean (-) incoming mean
            Eigen::MatrixXd J    = mu_in.drExp(tau);       // transport in THIS variable's group
            Eigen::MatrixXd Lf   = J.transpose() * it->second.lambda * J; // our last msg precision (warped)
            Eigen::VectorXd mu_full   = (msg_in.lambda + Lf).colPivHouseholderQr().solve(Lf * tau);
            if (mu_full.allFinite()) belief_mu = mu_in + mu_full;  // incoming (x) our last msg = belief
        }
        // Update linearisation point if the variable's belief is sufficiently different to our existing one.
        if ((belief_mu - linpoints_[i]).norm() > gbp::config().relinearise_threshold){
            linpoints_[i] = belief_mu;
        }
        ++i;
    }
}

/*****************************************************************************************************/
// Factor update.
// Linearise each connected variable at its belief mean (see below) to form the factor potential in the
// tangent space, then for each variable: condition on the others' beliefs (transported into the same
// tangent space), marginalise, and retract the message onto the group. Euclidean groups make the
// transports identities and the retraction addition, giving plain GBP.
/*****************************************************************************************************/
bool Factor::updateFactor(){

    updateLinearisationPoint();

    // Some factors skip work depending on state (e.g. the inter-robot factor when robots are far apart).
    // Send zero-information messages in that case. (skipFactor reads linpoints_.)
    if (this->skipFactor()){
        for (auto k : connected_v_keys_) this->outbox_[k] = Message(variable_groups_.at(k));
        return false;
    }

    // Calculate factor potential   (2.83)
    // A robust kernel scales the potential down when the residual is large (an outlier), so it pulls
    // the connected variables less; scale == 1 for the default (non-robust) factor.
    auto [residual, J] = computeResidualAndJacobian(linpoints_);
    double scale = getRobustScale(residual);
    Eigen::MatrixXd factor_lam_potential = scale * J.transpose() * meas_model_lambda_ * J;
    Eigen::VectorXd factor_eta_potential = scale * (J.transpose() * meas_model_lambda_) * -residual;

    // Condition on the other variables' beliefs
    // Marginalise, and send to each connected variable.
    int marginalisation_idx = 0;
    int vi = 0;
    for (auto [vkey_out, msg] : inbox_){
        int ndofs = msg.mu.dof();
        auto [conditioned_factor_potential_eta, conditioned_factor_potential_lambda] =
            conditionFactorPotential(vkey_out, factor_eta_potential, factor_lam_potential);

        // Marginalise down to this variable (Schur complement), giving the message info form {eta, lambda}.
        auto [m_eta, m_Lam] = marginaliseFactorDist(conditioned_factor_potential_eta, conditioned_factor_potential_lambda, marginalisation_idx, ndofs);

        // Solve in the tangent space at this variable's lin point, then retract onto ITS group.
        Eigen::VectorXd tau_out = m_Lam.colPivHouseholderQr().solve(m_eta);                             // (2.84)
        if (!tau_out.allFinite()) tau_out = Eigen::VectorXd::Zero(ndofs);
        const LieGroup& lp = linpoints_[vi];                                                            // this variable's lin point (its own group)
        LieGroup X = lp + tau_out;                                                                      // (2.85)
        Eigen::MatrixXd Jinv = lp.drExpInv(tau_out);
        Eigen::MatrixXd Lam_out = Jinv.transpose() * m_Lam * Jinv;                                      // (2.86)

        Message msg_out = Message(X, Lam_out);   // {mu, lambda}
        applyDamping(msg_out, vkey_out, gbp::config().damping);
        outbox_[vkey_out] = msg_out;

        marginalisation_idx += ndofs;
        ++vi;
    }
    return true;
};

/*****************************************************************************************************/
// Condition the factor potential on the beliefs of all connected variables except the receiving one.
// Each belief is transported into the tangent space at the factor's lin point before it's added in.
// Returns the conditioned {eta, lambda}; the input potential is left alone.
/*****************************************************************************************************/
std::pair<Eigen::VectorXd, Eigen::MatrixXd> Factor::conditionFactorPotential(Key vkey_out,
        const Eigen::VectorXd& factor_eta_potential, const Eigen::MatrixXd& factor_lam_potential){
    Eigen::VectorXd factor_eta = factor_eta_potential;
    Eigen::MatrixXd factor_lam = factor_lam_potential;
    int stateVectorIdx = 0;
    int vi = 0;
    for (auto [vkey_other, msg] : inbox_){
        int n_dofs = msg.mu.dof();
        if (vkey_other != vkey_out) {
            const LieGroup& lp = linpoints_[vi];           // the other variable's lin point (its own group)
            Eigen::VectorXd tau = msg.mu - lp;
            Eigen::MatrixXd Jt  = lp.drExp(tau);
            Eigen::MatrixXd Lam = Jt.transpose() * msg.lambda * Jt;
            Eigen::VectorXd eta = Lam * tau;
            factor_eta(seqN(stateVectorIdx, n_dofs)) += eta;
            factor_lam(seqN(stateVectorIdx, n_dofs), seqN(stateVectorIdx, n_dofs)) += Lam;
        }
        stateVectorIdx += n_dofs;
        ++vi;
    }
    return {factor_eta, factor_lam};
};

/*****************************************************************************************************/
// Marginalise the factor's {eta, lambda} down to the target variable's block (Schur complement).
// Works purely in information form; returns the marginalised {eta, lambda}.
/*****************************************************************************************************/
std::pair<Eigen::VectorXd, Eigen::MatrixXd> Factor::marginaliseFactorDist(const Eigen::VectorXd &eta, const Eigen::MatrixXd &Lam, int marg_idx, int n_dofs){
    // Nothing to marginalise if the factor connects a single variable
    if (eta.size() == n_dofs) return {eta, Lam};
    
    Eigen::VectorXd eta_a(n_dofs), eta_b(eta.size()-n_dofs);
    eta_a = eta(seqN(marg_idx, n_dofs));
    eta_b << eta(seq(0, marg_idx - 1)), eta(seq(marg_idx + n_dofs, last));
    
    Eigen::MatrixXd lam_aa(n_dofs, n_dofs), lam_ab(n_dofs, Lam.cols()-n_dofs);
    Eigen::MatrixXd lam_ba(Lam.rows()-n_dofs, n_dofs), lam_bb(Lam.rows()-n_dofs, Lam.cols()-n_dofs);
    lam_aa << Lam(seqN(marg_idx, n_dofs), seqN(marg_idx, n_dofs));
    lam_ab << Lam(seqN(marg_idx, n_dofs), seq(0, marg_idx - 1)), Lam(seqN(marg_idx, n_dofs), seq(marg_idx + n_dofs, last));
    lam_ba << Lam(seq(0, marg_idx - 1), seq(marg_idx, marg_idx + n_dofs - 1)), Lam(seq(marg_idx + n_dofs, last), seqN(marg_idx, n_dofs));
    lam_bb << Lam(seq(0, marg_idx - 1), seq(0, marg_idx - 1)), Lam(seq(0, marg_idx - 1), seq(marg_idx + n_dofs, last)),
    Lam(seq(marg_idx + n_dofs, last), seq(0, marg_idx - 1)), Lam(seq(marg_idx + n_dofs, last), seq(marg_idx + n_dofs, last));
    
    Eigen::MatrixXd lam_bb_inv = lam_bb.inverse();
    Eigen::VectorXd marg_eta = eta_a - lam_ab * lam_bb_inv * eta_b;
    Eigen::MatrixXd marg_lam = lam_aa - lam_ab * lam_bb_inv * lam_ba;
    if (!marg_lam.allFinite()){
        marg_eta.setZero();
        marg_lam.setZero();
    }
    
    return {marg_eta, marg_lam};
};

/*****************************************************************************************************/
// Message Damping
// Damp the new outgoing message using the old one. Damping happens as (1-d)*new + d*old
// First warp the old message into the tangent space of the new message, then apply damping 
// on eta and lambda
// Retract back onto the manifold to produce the damped output message.
/*****************************************************************************************************/
void Factor::applyDamping(Message& msg, Key vkey_out, double damping){
    damping = std::min(damping, 0.9999);
    if (last_outbox_.count(vkey_out)){
        auto& last = last_outbox_.at(vkey_out);
        Eigen::VectorXd tau_old = last.mu - msg.mu;   // old mean in the tangent at the new mean
        Eigen::MatrixXd lambda_d = (1.0 - damping) * msg.lambda + damping * last.lambda;
        Eigen::VectorXd eta_d    = damping * (last.lambda * tau_old);
        Eigen::VectorXd tau_d    = lambda_d.colPivHouseholderQr().solve(eta_d);
        if (!tau_d.allFinite()) tau_d.setZero();
        msg = Message(msg.mu + tau_d, lambda_d);
    }
    last_outbox_.insert_or_assign(vkey_out, msg);
}

/********************************************************************************************/
//                      GENERIC FACTORS
// A factor type provides computeResidualAndJacobian(): residual r(X) = h(X) (-) z and its Jacobian.
// Return a closed-form Jacobian directly (see SmoothnessFactor) or use jacobianFirstOrder() (see the
// planning factors in CustomFactorGraphLayers/Pathplanning.cpp).
/********************************************************************************************/

/*************************************************************************************** */
// SMOOTHNESS FACTOR: pulls two connected states together.   residual = X2 (-) X1 - z
/*************************************************************************************** */
SmoothnessFactor::SmoothnessFactor(Key fac_key, std::vector<Key> connected_variables, const std::string& group,
    const Eigen::VectorXd& observation, Eigen::VectorXd sigma_prior_list)
    : Factor{fac_key, connected_variables, group, observation, sigma_prior_list}{
};

std::pair<Eigen::VectorXd, Eigen::MatrixXd> SmoothnessFactor::computeResidualAndJacobian(const std::vector<LieGroup>& X){
    int n = X[0].dof();
    // r = X2 (-) X1 with EXACT analytic Jacobians from manif's right-minus (out-parameters):
    //   J_X2 = drExpInv(r),   J_X1 = -drExpInv(r) * Ad(X1^-1 * X2).
    Eigen::MatrixXd J_X2, J_X1;
    Eigen::VectorXd r = X[1].ominus(X[0], /*J_self=*/J_X2, /*J_rhs=*/J_X1);
    Eigen::MatrixXd J_h_X(n, n * 2);
    J_h_X.block(0, 0, n, n) = J_X1;   // J w.r.t. X1
    J_h_X.block(0, n, n, n) = J_X2;   // J w.r.t. X2
    Eigen::VectorXd residual = r - z_;
    return std::make_pair(residual, J_h_X);
};
