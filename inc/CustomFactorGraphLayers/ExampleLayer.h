/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once
#include <gbp/FactorGraphLayer.h>
#include <CustomFactorGraphLayers/DrawableLayer.h>
#include <gbp/Factor.h>
#include <gbp/Variable.h>

/*=====================================================================================================*
 *  EXAMPLE FACTOR-GRAPH LAYER  --  a copy-me template for adding your own layer.                       *
 *                                                                                                     *
 *  A layer is one GBP sub-problem each robot solves (planning, consensus, ...). Stack as many as you   *
 *  like, in any order. To add your own:                                                               *
 *                                                                                                     *
 *    1. Copy this pair of files into inc|src/CustomFactorGraphLayers/, rename files and classes              *
 *       (ExampleLayer -> MyLayer). The .cpp is compiled automatically.                                 *
 *    2. Fill in the parts marked  TODO.                                                                *
 *    3. Add one line to makeLayer() in src/Robot.cpp mapping a `type:` string to MyLayer.             *
 *    4. Use it in a config under FACTORGRAPH_LAYERS:                                                   *
 *         - name: MyLayer                                                                              *
 *           type: mylayer          # must match the string used in makeLayer()                        *
 *           params: { MY_SIGMA: 0.5 }                                                                  *
 *                                                                                                     *
 *  Compiles and runs as-is (type "example"). Working layers to crib from: Pathplanning and Consensus.  *
 *=====================================================================================================*/

/*-----------------------------------------------------------------------------------------------------*/
/* (OPTIONAL) A custom factor: a constraint between one or more variables. computeResidualAndJacobian() returns          */
/* h(X) - z, where h(X) is your measurement model and z the observation. Delete if the built-in         */
/* factors suffice.                                                                                     */
/*-----------------------------------------------------------------------------------------------------*/
class ExampleFactor: public Factor {
    public:
    ExampleFactor(Key fac_key, std::vector<Key> connected_variables, float sigma);

    // TODO: your measurement function. X is the stacked state of the connected variables (here
    // [x_i, y_i, x_j, y_j]). computeResidual() returns h(X) - z_; computeResidualAndJacobian() pairs it
    // with a numerical Jacobian (swap in a closed form if you have one).
    Eigen::MatrixXd computeResidual(const std::vector<LieGroup>& X);
    std::pair<Eigen::VectorXd, Eigen::MatrixXd> computeResidualAndJacobian(const std::vector<LieGroup>& X) override;
};

/*-----------------------------------------------------------------------------------------------------*/
/* (OPTIONAL) A custom variable: an unknown the graph estimates, with belief mean belief_.mu and        */
/* covariance belief_.lambda.inverse() (set by GBP). Override draw() to visualise it. Delete if the     */
/* built-in Variable suffices.                                                                          */
/*-----------------------------------------------------------------------------------------------------*/
class ExampleVariable: public Variable {
    public:
    float height_3D_ = 0.f;                     // out-of-plane height (3d visualisation only)
    ExampleVariable(Key vkey, const Eigen::VectorXd& mu_prior, const Eigen::VectorXd& sigma_prior_list);

    void draw(Color col) override;              // TODO: draw the variable (e.g. a marker at belief_.mu)
};

/*-----------------------------------------------------------------------------------------------------*/
/* The layer. Implement the hooks below; leave any you don't need empty.                               */
/*-----------------------------------------------------------------------------------------------------*/
class ExampleLayer : public FactorGraphLayer, public DrawableLayer {
public:
    // TODO: this layer's tuning. Read each value from the config param bag in the constructor.
    float MY_SIGMA;

    ExampleLayer(int graph_id, int lid, const LayerConfig& cfg = {});

    void initialiseLayerNodes(FactorGraph& owner) override;             // build this layer's nodes (once)
    void createInterrobotFactors(FactorGraph& owner, int other_id) override; // factors to another stack
    void preGBPUpdateNodes(FactorGraph& owner, uint32_t clock) override;     // each timestep, before GBP
    void postGBPUpdateNodes(FactorGraph& owner) override;                    // each timestep, after GBP
    void draw(Robot* robot, Simulator* sim, Color& col) override;            // each frame (DrawableLayer)
};
