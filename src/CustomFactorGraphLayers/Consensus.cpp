/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <CustomFactorGraphLayers/Consensus.h>
#include <Robot.h>
#include <ShapeFormation.h>

/**************************************************************************************/
/*                                CONSENSUS LAYER                                   */
/**************************************************************************************/

/***************************************************************************/
// Create the consensus variable for the formation/decision, with a Lie prior factor.
/***************************************************************************/
void ConsensusLayer::initialiseLayerNodes(FactorGraph& owner){
    Robot* robot = static_cast<Robot*>(&owner);   // recover the application stack type
    const int layer = lid_;
    const Eigen::VectorXd& start = robot->start_;

    Eigen::VectorXd mu, sigma_list;
    double sigma_prior = SIGMA_CONSENSUS_VAR_PRIOR;
    std::shared_ptr<Variable> variable;

    if (robot->seed_robot_) robot->decision_ = globals.SEED_DECISION;
    if (robot->seed_robot_) sigma_prior = robot->SIGMA_POSE_FIXED;
    if (lieGroup_.type() == LieGroupType::SE2){
        mu = Eigen::VectorXd{{start(0), start(1), decision2angle(robot->decision_, robot->NUM_DECISIONS)}};
        sigma_list = Eigen::Vector3d::Constant(sigma_prior).cwiseProduct(Eigen::VectorXd{{10.,10.,M_PI}});
        robot->formationPoints_->liePose_ = manif::SE2d(mu(0), mu(1), mu(2));
    } else if (lieGroup_.type() == LieGroupType::SO2){
        mu = Eigen::VectorXd{{decision2angle(robot->decision_, robot->NUM_DECISIONS)}};
        sigma_list = Eigen::Vector<double, 1>::Constant(sigma_prior*M_PI);
        robot->formationPoints_->liePose_ = manif::SE2d(0., 0., mu(0));
    } else {   // R^n: consensus on an n-vector. Seed component 0 from the decision, rest zero.
        int n = lieGroup_.dof();
        mu = Eigen::VectorXd::Zero(n);
        mu(0) = quantiseInv(robot->decision_, robot->NUM_DECISIONS);
        sigma_list = Eigen::VectorXd::Constant(n, sigma_prior);
        robot->formationPoints_->liePose_ = manif::SE2d(0., 0., normalisedDecision2angle(mu(0)));
    }
    // The first consensus variable carries a built-in prior (mu + sigma_list) - no separate prior factor.
    variable = robot->addVariable<Variable>(layer, lieGroup_.fromCoeffs(mu), sigma_list);
    variable->position_ = robot->position_;   // display location for this consensus node

    robot->stack_[layer]->optimiseGBP(10);
}

/***************************************************************************/
// Create inter-robot smoothness (consensus) factors between this robot and another.
/***************************************************************************/
void ConsensusLayer::createInterrobotFactors(FactorGraph& owner, int other_robot_id){
    Robot* robot = static_cast<Robot*>(&owner);
    const int layer = lid_;
    int num_vars = robot->stack_[layer]->variables_.size();
    int n = lieGroup_.dof();
    for (int i = 0; i < num_vars; i++){
        Key this_var_key = robot->getVar(layer, i)->key_;
        Key other_var_key = Key(other_robot_id, this_var_key.node_id_, this_var_key.lid_, Node::VAR);

        std::shared_ptr<Factor> factor;
        float sigma_smoothness_prior = SIGMA_CONSENSUS_FACTOR_SMOOTHNESS;
        Eigen::VectorXd sigma_list_smoothness(n);
        if (lieGroup_.type() == LieGroupType::SE2){
            sigma_list_smoothness = Eigen::Vector3d::Constant(sigma_smoothness_prior).cwiseProduct(Eigen::VectorXd{{10.,10.,M_PI}});
        } else if (lieGroup_.type() == LieGroupType::SO2){
            sigma_list_smoothness = Eigen::VectorXd{{sigma_smoothness_prior}} * M_PI;
        } else {
            sigma_list_smoothness = Eigen::VectorXd::Constant(n, sigma_smoothness_prior);
        }
        factor = robot->addFactor<SmoothnessFactor>(layer, {this_var_key, other_var_key}, lieGroup_.name(), Eigen::VectorXd::Zero(n), sigma_list_smoothness);
        // addFactor registers the local variable; the cross-robot half lives in the robot inbox.
        robot->inbox_[std::make_pair(factor->key_, other_var_key)] = Message(factor->groupForKey(other_var_key));
    }
}

/***************************************************************************/
// Pre-GBP update: every SLIDING_WINDOW_TIMESTEP_FREQ timesteps, append a new
// consensus variable (with a temporal smoothness factor to the previous one),
// and drop the oldest variable beyond the sliding-window length, converting the
// message it carried into a prior on the new oldest variable.
/***************************************************************************/
void ConsensusLayer::preGBPUpdateNodes(FactorGraph& owner, uint32_t clock){
    Robot* robot = static_cast<Robot*>(&owner);
    const int layer = lid_;
    int n = lieGroup_.dof();

    const int K = std::max(NUM_CONSENSUS_VAR_SLIDING_WINDOW, 1);   // window keeps at least one variable

    // Add a variable every SLIDING_WINDOW_TIMESTEP_FREQ timesteps, and drain any beyond the window
    // length K. Nothing to do otherwise.
    bool do_slide = K>1 && clock>0 && clock % SLIDING_WINDOW_TIMESTEP_FREQ==0;
    bool oversize = (int)robot->stack_[layer]->variables_.size() > K;
    if (!do_slide && !oversize) return;

    if (do_slide){
        /****************** SLIDING WINDOW *************/
        // Create a new Consensus variable, along with a temporal smoothness factor
        std::shared_ptr<Variable> variable;
        auto last_consensus_var = robot->getVar(layer, -1);
        double sigma_sliding = SIGMA_CONSENSUS_VAR_SLIDING_WINDOW;
        Eigen::VectorXd sigma_list_smoothness(n);
        if (lieGroup_.type() == LieGroupType::SE2){
            sigma_list_smoothness = Eigen::Vector3d::Constant(sigma_sliding).cwiseProduct(Eigen::VectorXd{{10.,10.,M_PI}});
        } else if (lieGroup_.type() == LieGroupType::SO2){
            sigma_list_smoothness = Eigen::VectorXd{{sigma_sliding}} * M_PI;
        } else {
            sigma_list_smoothness = Eigen::VectorXd::Constant(n, sigma_sliding);
        }
        // Weak built-in prior at the last variable's belief mean (same convention as the first consensus
        // variable). Without it the fresh variable starts singular (belief_.lambda = 0) and can be yanked to
        // garbage by a not-yet-converged inter-robot factor whose remote half is still seeded at the origin.
        double sigma_new = SIGMA_CONSENSUS_VAR_PRIOR;
        if (robot->seed_robot_) sigma_new = robot->SIGMA_POSE_FIXED;
        Eigen::VectorXd sigma_list_prior;
        if (lieGroup_.type() == LieGroupType::SE2){
            sigma_list_prior = Eigen::Vector3d::Constant(sigma_new).cwiseProduct(Eigen::VectorXd{{10.,10.,M_PI}});
        } else if (lieGroup_.type() == LieGroupType::SO2){
            sigma_list_prior = Eigen::Vector<double,1>::Constant(sigma_new*M_PI);
        } else {
            sigma_list_prior = Eigen::VectorXd::Constant(n, sigma_new);
        }
        variable = robot->addVariable<Variable>(layer, last_consensus_var->belief_.mu, sigma_list_prior);
        variable->position_ = robot->position_;   // display location for this consensus node
        robot->unconnected_interrobot_variables_.push_back(variable);
        // Both connected variables are local, so addFactor wires the smoothness factor to each.
        robot->addFactor<SmoothnessFactor>(layer, {last_consensus_var->key_, variable->key_}, lieGroup_.name(), Eigen::VectorXd::Zero(n), sigma_list_smoothness);
    }

    // Drop the oldest variable(s) beyond length K, folding the message each was sending forward into
    // a prior on the new oldest variable (fixed-lag marginalisation).
    while ((int)robot->stack_[layer]->variables_.size() > K){
        auto oldest_var      = robot->getVar(layer, 0);   // front = oldest
        auto penultimate_var = robot->getVar(layer, 1);   // the new oldest, inherits the anchor

        // Get connecting factor (oldest -> penultimate) and the message it sends to penultimate.
        Key common_fkey = robot->stack_[layer]->findCommonFactor(oldest_var, penultimate_var);
        Message msgToBePrior = penultimate_var->inbox_.at(common_fkey);

        // Delete oldest variable, also remove from unconnected_interrobot_variables if not yet connected.
        robot->deleteVariable(oldest_var->key_);
        auto& uiv = robot->unconnected_interrobot_variables_;
        uiv.erase(std::remove_if(uiv.begin(), uiv.end(), [&](auto v){return v->key_==oldest_var->key_;}), uiv.end());

        // Fold that message into the now-oldest variable's built-in prior (mean + full precision), so it
        // keeps anchoring the window after the oldest variable is gone.
        penultimate_var->changeVariablePrior(msgToBePrior.mu, msgToBePrior.lambda);
    }
    robot->stack_[layer]->optimiseGBP(1);
}

/***************************************************************************/
// Per-timestep update: read the latest consensus variable back into the
// robot's formation pose and decision.
/***************************************************************************/
void ConsensusLayer::postGBPUpdateNodes(FactorGraph& owner){
    Robot* robot = static_cast<Robot*>(&owner);
    Eigen::VectorXd mu = robot->getVar(lid_, -1)->belief_.mu.coeffs();
    int n = lieGroup_.dof();

    // Update the consensus variable for the formation
    if (lieGroup_.type() == LieGroupType::SE2){
        robot->formationPoints_->liePose_ = manif::SE2d(mu(0), mu(1), mu(2));
        robot->decision_ = angle2decision(mu(n-1), robot->NUM_DECISIONS);
    } else if (lieGroup_.type() == LieGroupType::SO2){
        robot->formationPoints_->liePose_ = manif::SE2d(0., 0., mu(0));
        robot->decision_ = angle2decision(mu(n-1), robot->NUM_DECISIONS);
    } else {   // R^n (e.g. discrete consensus on R1)
        robot->formationPoints_->liePose_ = manif::SE2d(0., 0., normalisedDecision2angle(mu(0)));
        robot->decision_ = quantise(mu(n-1), robot->NUM_DECISIONS);
    }
}

/***************************************************************************/
// Draw the consensus layer: in the trigrid scenarios, the consensus variables at the robot's
// position; and tint the robot's display colour by its current decision (used for the model,
// drawn afterwards in Robot::draw()).
/***************************************************************************/
void ConsensusLayer::draw(Robot* robot, Simulator* /*sim*/, Color& col){
    if (globals.CONSENSUS_TYPE == ConsensusType::Discrete){
        for (auto [vkey, var] : variables_){
            var->draw(robot->position_({0,1}), robot->robot_radius_);
            var->position_ = robot->position_({0,1});
        }
        col = decisionColor(robot->decision_, globals.NUM_DECISIONS);   // tint the robot by its decision
    }
    else {
        if (globals.TEMP4>0) col = GREEN;
    }
}
