/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once

#include <vector>
#include <math.h>
#include <memory>

#include <Utils.h>
#include <gbp/GBPCore.h>

#include <raylib.h>
#include <Eigen/Core>
#include <Eigen/Dense>

class Factor;    // Forward declaration
/***********************************************************************************************************/
// A GBP variable living on a Lie group LG (R^n by default). updateBelief() gathers the incoming
// Gaussian messages in the tangent space at the current estimate, solves there, and retracts the
// result onto the group. For Euclidean LG the tangent is the space itself and the retraction is
// addition, so it reduces to ordinary Gaussian BP.
/***********************************************************************************************************/
class Variable {
    public:
        int rid_;                                          // id of robot this variable belongs to
        int vid_;                                          // id of variable
        int lid_;                                          // id of the GBP-stack layer this variable belongs to
        Key key_;                                           // Key {rid_, vid_}
        std::vector<Key> connected_f_keys_{};
        LieGroup LG;                                    // group this variable lives in (R^n by default)
        // Built-in unary prior, re-applied to the belief every iteration. Zero prior_.lambda = no prior.
        Message prior_;
        int n_dofs_;                                        // Degrees of freedom. In 2D, n_dofs_ = 4 ([x,y,xdot,ydot])
        Message belief_;                                    // Belief of variable (mean belief_.mu, precision belief_.lambda)
        Mailbox inbox_, outbox_;                            // Mailboxes for message storage
        bool valid_ = false;                                // Whether the belief's precision is invertible (has a finite covariance)
        Eigen::VectorXd position_;                          // Useful if Variables in other layers need a physical location for display

        // Variable on a group LG, whose value is the prior mean / initial estimate - e.g.
        // LieGroup(manif::SO2d(0.5)) or LieGroup(Eigen::VectorXd{{x,y}}). Pass sigma_prior_list for a
        // built-in prior N(mean, diag(sigma_prior_list)^2); leave it empty for none.
        Variable(Key vkey, LieGroup prior_mean, const Eigen::VectorXd& sigma_prior_list = Eigen::VectorXd());

        ~Variable();

        virtual void updateBelief();

        virtual void changeVariablePrior(const LieGroup& new_mu);
        virtual void changeVariablePrior(const LieGroup& new_mu, const Eigen::MatrixXd& new_lam);

        virtual void addFactor(Key f_key);
        void deleteFactor(Key fac_key);

        virtual void draw(Eigen::VectorXd position, float height);
        virtual void draw(Color color);

};