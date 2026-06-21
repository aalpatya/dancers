/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Utils.h>
#include <gbp/GBPCore.h>
#include <raylib.h>

using Eigen::seqN;
using Eigen::seq;
using Eigen::last;

class Variable;     // Forward declaration

// Robust kernel applied to a factor's energy. None is the standard quadratic (Gaussian) cost. Huber
// and DCS down-weight large residuals so outliers can't dominate the solution; the kernel only kicks
// in once the residual's Mahalanobis distance M = sqrt(r^T Lambda r) exceeds the threshold.
enum class RobustKernel { None, Huber, DCS };

/*****************************************************************************************/
// A GBP factor connecting one or more variables on a Lie group LG (R^n by default).
// Subclasses implement computeResidualAndJacobian(X): the residual r(X) = h(X) (-) z and Jacobian dr/dX at X.
// updateFactor() linearises at X, conditions on the other variables' beliefs (in the tangent space
// at X), marginalises, and retracts the outgoing message onto the group. For Euclidean LG the tangent
// transforms drop out and the retraction is addition.
/*****************************************************************************************/
class Factor {
    public:
    int fid_;                                  // Factor id
    int rid_;                                  // Robot id this factor belongs to
    int lid_;                                  // id of the GBP-stack layer this factor belongs to
    Key key_;                                   // Factor key = {rid_, fid_}
    std::vector<Key> connected_v_keys_{};
    int other_rid_;                             // id of the other connected robot, for inter-robot factors
    int n_dofs_;                                // dofs of each connected variable
    int n_dofs_meas_;                           // dofs of the observation z
    LieGroup LG;                                // group the connected variables live in (R^n by default)
    Eigen::VectorXd z_;                         // Measurement
    std::vector<LieGroup> linpoints_;           // current lin point, one group element per connected variable
                                                // (set in updateFactor; used by conditioning/retraction/skipFactor)
    Eigen::MatrixXd meas_model_lambda_;         // Precision of measurement model
    Mailbox inbox_, outbox_, last_outbox_;
    float delta_jac=1e-8;                       // Step size for the finite-difference Jacobian
    RobustKernel robust_kernel_ = RobustKernel::None;   // outlier down-weighting (off by default)
    virtual bool skipFactor(){                 // Subclasses override to skip work in some states
        return false;
    };

    // Factor connecting variables that live in group `group`, named as in a config / makeLieGroup:
    // "SO2"/"SE2"/"SO3"/"SE3", or "R<n>" for the Euclidean group R^n (e.g. "R2"). Measurement precision
    // is diag(sigma_prior_list)^-2 (use a constant list for isotropic noise); z is the observation.
    Factor(Key fac_key, std::vector<Key> connected_variables, const std::string& group,
            const Eigen::VectorXd& observation, Eigen::VectorXd sigma_prior_list);

    virtual ~Factor();

    virtual void draw(Eigen::VectorXd p1, Eigen::VectorXd p2, Color color);

    // Concrete factors implement this: the residual r(X) = h(X) (-) z and its Jacobian dr/dtau at the
    // lin point X, the connected variables' states as group elements indexed X[0], X[1], ... (read the
    // coords of a Euclidean one with X[k].coeffs()). With no analytic Jacobian, use
    // jacobianFirstOrder() below.
    virtual std::pair<Eigen::VectorXd, Eigen::MatrixXd> computeResidualAndJacobian(const std::vector<LieGroup>& X) = 0;

    // Finite-difference Jacobian of hfunc at the lin point X0: perturbs each variable in its tangent
    // (X0[k] (+) delta*e_j) and forward-differences. For Euclidean groups this is the ordinary Jacobian.
    Eigen::MatrixXd jacobianFirstOrder(const std::vector<LieGroup>& X0,
                                       std::function<Eigen::MatrixXd(const std::vector<LieGroup>&)> hfunc);

    // Make this factor robust: scale its potential by a Huber/DCS kernel once the residual's Mahalanobis
    // distance exceeds gbp::config().robust_threshold. (Off by default - the standard quadratic cost.)
    void setRobust(RobustKernel kernel){ robust_kernel_ = kernel; }
    // Kernel scale in [0,1] for the current residual (1 = no down-weighting).
    double getRobustScale(const Eigen::VectorXd& residual) const;

    bool updateFactor();
    // Refresh linpoints_ (one per connected variable) to each variable's reconstructed belief mean,
    // relinearising only when a belief drifts past gbp::config().relinearise_threshold.
    void updateLinearisationPoint();
    // Condition the factor potential on every connected variable's belief except the receiving one,
    // returning the conditioned {eta, lambda}. The input potential is left untouched.
    std::pair<Eigen::VectorXd, Eigen::MatrixXd> conditionFactorPotential(Key vkey_out,
        const Eigen::VectorXd& factor_eta_potential, const Eigen::MatrixXd& factor_lam_potential);
    void applyDamping(Message& msg, Key vkey_out, double damping);
    std::pair<Eigen::VectorXd, Eigen::MatrixXd> marginaliseFactorDist(const Eigen::VectorXd &eta, const Eigen::MatrixXd &Lam, int marg_idx, int n_dofs=1);
};

/********************************************************************************************/
//                      GENERIC FACTORS
// Problem-agnostic factors reused across layers. Layer-specific ones live with their layer
// (e.g. the planning factors in inc/CustomFactorGraphLayers/Pathplanning.h).
/* Smoothness factor: penalises the difference between two connected states (keeps them close). */
class SmoothnessFactor: public Factor {
    public:
    SmoothnessFactor(Key fac_key, std::vector<Key> connected_variables, const std::string& group,
        const Eigen::VectorXd& observation, Eigen::VectorXd sigma_prior_list);
    std::pair<Eigen::VectorXd, Eigen::MatrixXd> computeResidualAndJacobian(const std::vector<LieGroup>& X);
};