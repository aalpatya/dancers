/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <CustomFactorGraphLayers/ExampleLayer.h>
#include <Robot.h>

/**************************************************************************************/
/*                                  EXAMPLE LAYER                                     */
/*  A copy-me template. Read ExampleLayer.h first, then fill in the TODOs below.       */
/**************************************************************************************/

/*-----------------------------------------------------------------------------------------------------*/
/* CONSTRUCTOR: read this layer's tuning from its config `params:` block.                              */
/*-----------------------------------------------------------------------------------------------------*/
ExampleLayer::ExampleLayer(int graph_id, int lid, const LayerConfig& cfg)
    : FactorGraphLayer(graph_id, lid, cfg) {
    // TODO: read each param (numf/numi/flag/str), with a default as the 2nd argument.
    MY_SIGMA = cfg.params.numf("MY_SIGMA", 0.5f);
}

/*-----------------------------------------------------------------------------------------------------*/
/* initialiseLayerNodes: called once per robot. Build this layer's variables and its intra-robot        */
/* factors here. Use lid_ for every Key you create.                                                    */
/*-----------------------------------------------------------------------------------------------------*/
void ExampleLayer::initialiseLayerNodes(FactorGraph& owner){
    Robot* robot = static_cast<Robot*>(&owner);
    const int layer = lid_;

    // TODO: create your variables. addVariable<T>(layer, ctorArgs...) builds the node, assigns it a Key,
    // and registers it. This makes one 2D variable at the robot's position with a prior of strength MY_SIGMA.
    Eigen::VectorXd mu = robot->position_({0,1});
    Eigen::VectorXd sigma_list = Eigen::Vector2d::Constant(MY_SIGMA);
    robot->addVariable<ExampleVariable>(layer, mu, sigma_list);

    // TODO (optional): create intra-robot factors between your variables, e.g.
    //   robot->addFactor<ExampleFactor>(layer, {keyA, keyB}, MY_SIGMA);   // wires the factor to both vars
    // (see Pathplanning.cpp). addFactor assigns the Key and registers the factor with its local variables.

    // A few GBP sweeps so the belief is valid on the first frame.
    robot->stack_[layer]->optimiseGBP(10);
}

/*-----------------------------------------------------------------------------------------------------*/
/* createInterrobotFactors: called when this robot gains a neighbour. Couple this robot's variables to  */
/* other_robot_id's; the other robot's variable Key reuses this layer's id and the matching node id.    */
/* Delete this body if your layer has no inter-robot factors.                                          */
/*-----------------------------------------------------------------------------------------------------*/
void ExampleLayer::createInterrobotFactors(FactorGraph& owner, int other_robot_id){
    Robot* robot = static_cast<Robot*>(&owner);
    const int layer = lid_;

    // TODO: build the cross-robot factor you need. This connects this robot's variable 0 to the other
    // robot's variable 0 with an ExampleFactor.
    Key this_var_key  = robot->getVar(layer, 0)->key_;
    Key other_var_key = Key(other_robot_id, this_var_key.node_id_, layer, Node::VAR);

    // addFactor builds the factor, assigns its Key, and registers it with the LOCAL variable (this_var_key);
    // the other variable lives on another robot, so its half of the message goes in the robot inbox.
    auto fac = robot->addFactor<ExampleFactor>(layer, {this_var_key, other_var_key}, MY_SIGMA);
    robot->inbox_[std::make_pair(fac->key_, other_var_key)] = Message(fac->LG);
}

/*-----------------------------------------------------------------------------------------------------*/
/* preGBPUpdateNodes: each timestep, before the GBP iterations. Add/remove nodes, re-anchor priors,    */
/* advance a sliding window, etc. clock is the simulation timestep, handy for scheduling.              */
/*-----------------------------------------------------------------------------------------------------*/
void ExampleLayer::preGBPUpdateNodes(FactorGraph& /*owner*/, uint32_t /*clock*/){
    // TODO (optional). The Consensus layer slides a window here.
}

/*-----------------------------------------------------------------------------------------------------*/
/* postGBPUpdateNodes: each timestep, after the GBP iterations. Read the converged beliefs back out     */
/* (e.g. into robot state).                                                                            */
/*-----------------------------------------------------------------------------------------------------*/
void ExampleLayer::postGBPUpdateNodes(FactorGraph& /*owner*/){
    // TODO (optional).
}

/*-----------------------------------------------------------------------------------------------------*/
/* draw: each frame. Visualise this layer's variables/factors. col is the robot's running colour; read */
/* and/or tint it (the consensus layer recolours the robot model through it).                          */
/*-----------------------------------------------------------------------------------------------------*/
void ExampleLayer::draw(Robot* /*robot*/, Simulator* /*sim*/, Color& col){
    // TODO: draw what you like. variables_ are this layer's variables for this robot.
    for (auto& [vkey, var] : variables_) var->draw(col);
}

/**************************************************************************************/
/*               EXAMPLE-LAYER FACTOR AND VARIABLE DEFINITIONS                         */
/**************************************************************************************/

ExampleFactor::ExampleFactor(Key fac_key, std::vector<Key> connected_variables, float sigma)
    // Factor base args: (key, connected, group name, observation z, per-dof sigma). R^2, isotropic.
    : Factor{fac_key, connected_variables, "R2", Eigen::VectorXd::Zero(2), Eigen::VectorXd::Constant(2, sigma)} {
    // TODO (optional): set z_ here if nonzero (defaults to a zero vector of length n_dofs_meas).
    // e.g. z_ = Eigen::Vector2d(...);
}
Eigen::MatrixXd ExampleFactor::computeResidual(const std::vector<LieGroup>& X){
    // X[k] is connected variable k's state; .coeffs() gives its coords ([x,y] here, R^2).
    // TODO: your measurement model h(X). This placeholder says "the two should be equal".
    Eigen::VectorXd a = X[0].coeffs(), b = X[1].coeffs();
    Eigen::MatrixXd h(n_dofs_meas_, 1);
    h(0) = b(0) - a(0);
    h(1) = b(1) - a(1);
    return h - z_;
}
std::pair<Eigen::VectorXd, Eigen::MatrixXd> ExampleFactor::computeResidualAndJacobian(const std::vector<LieGroup>& X){
    // Residual plus a numerical Jacobian (swap in a closed form if you have one).
    Eigen::MatrixXd J = jacobianFirstOrder(X, [this](const std::vector<LieGroup>& x){ return computeResidual(x); });
    return { Eigen::VectorXd(computeResidual(X)), J };
}

ExampleVariable::ExampleVariable(Key vkey, const Eigen::VectorXd& mu_prior, const Eigen::VectorXd& sigma_prior_list)
    // Variable base args: (key, group-carrying-the-mean, prior sigma per dof). R^2 here.
    : Variable{vkey, LieGroup(mu_prior), sigma_prior_list} {
}
void ExampleVariable::draw(Color col){
    // belief_.mu is the belief mean, belief_.lambda.inverse() the covariance (valid_ is false until GBP produces a finite one).
    if (!valid_) return;
    // TODO: draw your variable. This draws a small sphere at the mean.
    DrawSphere(Vector3{(float)belief_.mu.coeffs()(0), height_3D_ + 0.1f, (float)belief_.mu.coeffs()(1)}, 0.2f, col);
}

