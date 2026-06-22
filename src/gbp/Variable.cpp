/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <Eigen/Core>
#include <Eigen/Dense>
#include <raylib.h>
#include <vector>
#include <math.h>
#include <Utils.h>
#include <gbp/GBPCore.h>

#include <gbp/Variable.h>

/***********************************************************************************************************/
// Variable constructor. The group LG carries the variable's value, which is its prior mean / initial
// estimate (e.g. LieGroup(manif::SO2d(0.5)), or LieGroup(Eigen::VectorXd{{x,y}}) for R^n).
//  - sigma_prior_list: per-dof prior standard deviations; the built-in prior precision is
//    diag(sigma_prior_list)^-2. An empty list means no built-in prior.

// Equations eg (2.71) reference the PhD thesis at:
//      https://spiral.imperial.ac.uk/entities/publication/d7f69b3b-29a9-4d28-962a-2f7474454359
//      DOI={https://doi.org/10.25560/128135}
/***********************************************************************************************************/
Variable::Variable(Key vkey, LieGroup prior_mean, const Eigen::VectorXd& sigma_prior_list)
            : vid_(vkey.node_id_), rid_(vkey.graph_id_), lid_(vkey.lid_), key_(vkey),
              LG(std::move(prior_mean)), n_dofs_(LG.dof()) {
                Eigen::MatrixXd lam_prior = Eigen::MatrixXd::Zero(n_dofs_, n_dofs_);
                if (sigma_prior_list.size() == n_dofs_ and (sigma_prior_list.array().abs() > 1e-12).all()){
                    lam_prior = sigma_prior_list.cwiseProduct(sigma_prior_list).cwiseInverse().asDiagonal();
                }
                // The prior mean is the variable's group value LG. The prior is applied as a residual in
                // updateBelief(), parametrised by its mean (a group element) and precision.
                prior_  = Message(LG, lam_prior);
                belief_ = prior_;
            };

/***********************************************************************************************************/
// Destructor: also deletes all connected factors.
/***********************************************************************************************************/
Variable::~Variable(){
    // Iterate over a copy: deleteFactor() erases from connected_f_keys_, which would invalidate the
    // range-for iterators (UB, and we saw intermittent heap corruption from it).
    std::vector<Key> keys = connected_f_keys_;
    for (auto fkey : keys){
        deleteFactor(fkey);
    }
}

/***********************************************************************************************************/
// Variable belief update.
//
// The belief is the product of the built-in prior (if any) and every incoming factor message, solved in
// the tangent space at the current estimation point `state`. The prior is linearised there as a unary factor
// (residual prior_mu (-) X, contributing JᵀΛJ and -JᵀΛr); each message N(mu_msg, lambda) is transported
// into the same tangent via tau = mu_msg (-) state with precision carried by drExp(tau). We solve for
// the tangent step and retract onto the group. For Euclidean groups drExp is the identity, so this is
// plain Gaussian BP.
//
// The message sent back to a factor is the belief minus that factor's own contribution; the prior is
// kept in, so it always rides along.
/***********************************************************************************************************/
void Variable::updateBelief(){
    LieGroup state = belief_.mu;
    Eigen::VectorXd eta_all = Eigen::VectorXd::Zero(n_dofs_);
    Eigen::MatrixXd lam_all = Eigen::MatrixXd::Zero(n_dofs_, n_dofs_);

    // Built-in unary prior, linearised on the manifold. For Euclidean J = -I, so this is just
    // eta = Lambda*(prior_mu - state).
    if (!prior_.lambda.isZero()){
        Eigen::VectorXd tau = prior_.mu - state;
        Eigen::MatrixXd J   = -LG.drExpInv(-tau);
        Eigen::MatrixXd JtL = J.transpose() * prior_.lambda;
        eta_all += -JtL * tau;
        lam_all +=  JtL * J;
    }

    // Computation of Incremental Belief using messages from connected factors.
    // "Incremental Belief at X" = Gaussian Belief on the tangent space to the Manifold at point X
    // Warp the incoming messages onto a common tangent space: the tangent space at the variable's current 'state'
    for (auto& [f_key, msg] : inbox_) { // From factor k:
        Eigen::VectorXd tau_k = msg.mu - state;                                 // (2.71)
        Eigen::MatrixXd J_tau_k  = LG.drExp(tau_k);
        Eigen::MatrixXd lam_k = J_tau_k.transpose() * msg.lambda * J_tau_k;     // (2.72)
        eta_all += lam_k * tau_k;                                               // (2.74)
        lam_all += lam_k;                                                       // (2.74)
    }
    // (eta_all, lam_all) is the incremental belief (on the tangent space at state)
    Eigen::VectorXd tau_v = lam_all.colPivHouseholderQr().solve(eta_all);       // (2.75)
    Eigen::MatrixXd J_tau_v_inv = LG.drExpInv(tau_v);
    LieGroup new_state = state + tau_v;                                         // (2.76)
    Eigen::MatrixXd new_lam = J_tau_v_inv.transpose() * lam_all * J_tau_v_inv;  // (2.76)
    belief_ = Message(new_state, new_lam);
    // Valid once the precision is invertible.
    valid_ = belief_.isValid();

    // Outgoing message to each factor = full belief minus that factor's own message.
    // All incoming messages must be warped into a common tangent space, for convenience
    // we use the current state of the variable.
    for (auto& [f_key, msg] : inbox_) {
        Eigen::VectorXd tau_k_new = msg.mu - state;                                         // (2.71')
        Eigen::MatrixXd J_tau_k_new  = LG.drExp(tau_k_new);
        Eigen::MatrixXd lam_k_new = J_tau_k_new.transpose() * msg.lambda * J_tau_k_new;     // (2.72')
        Eigen::VectorXd eta_out_k = eta_all - lam_k_new * tau_k_new;                        // (2.77)
        Eigen::MatrixXd lam_out_k = lam_all - lam_k_new;                                    // (2.77)
        Eigen::VectorXd tau_out_k = lam_out_k.colPivHouseholderQr().solve(eta_out_k);       // (2.78)
        if (!tau_out_k.allFinite()) tau_out_k = Eigen::VectorXd::Zero(n_dofs_);

        LieGroup X_out_k = state + tau_out_k;                                                   // (2.79)
        Eigen::MatrixXd J_tau_out_k_inv = LG.drExpInv(tau_out_k);
        Eigen::MatrixXd lam_out_k_new = J_tau_out_k_inv.transpose() * lam_out_k * J_tau_out_k_inv;  // (2.80)

        outbox_[f_key] = Message(X_out_k, lam_out_k_new);
    }
}

/***********************************************************************************************************/
// Change the built-in prior on the variable: its mean (minimal coordinates, like the constructor), and
// optionally its precision. updateBelief() linearises the prior at prior_.mu
/***********************************************************************************************************/
void Variable::changeVariablePrior(const LieGroup& new_mu, const Eigen::MatrixXd& new_lam){
    prior_.lambda = new_lam;
    prior_.mu     = new_mu;
    this->updateBelief();
};
void Variable::changeVariablePrior(const LieGroup& new_mu){
    changeVariablePrior(new_mu, prior_.lambda);
};

/***********************************************************************************************************/
// Register a factor with this variable and set up its mailboxes. The outgoing message's mean is seeded
// with the current belief mean, so the factor has a sensible first linearisation point.
/***********************************************************************************************************/
void Variable::addFactor(Key f_key){
    connected_f_keys_.push_back(f_key);
    inbox_[f_key] = Message(LG);
    outbox_[f_key] = Message(LG);
    outbox_[f_key].mu = belief_.mu;
}

/***********************************************************************************************************/
// Unregister a factor from this variable and drop its mailbox entries.
/***********************************************************************************************************/
void Variable::deleteFactor(Key fac_key){
    connected_f_keys_.erase(std::remove_if(connected_f_keys_.begin(), connected_f_keys_.end(), [&](auto v){return v==fac_key;}), connected_f_keys_.end());
    inbox_.erase(fac_key);
    outbox_.erase(fac_key);
}

/***********************************************************************************************************/
// Drawing. draw(Color) is a no-op; draw(position) updates the display position. Layers can recolour or
// override these for their own display.
/***********************************************************************************************************/
void Variable::draw(Color col){};
void Variable::draw(Eigen::VectorXd position, float height){
    double angle;
    if (LG.type() == LieGroupType::SE2) {
        position_ = belief_.mu.coeffs()({0,1});
    } else if (LG.type() == LieGroupType::SE3) {
        position_ = belief_.mu.coeffs()({0,1,2});
    }

    return;
};
