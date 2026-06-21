/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once
#include <gbp/FactorGraphLayer.h>
#include <CustomFactorGraphLayers/DrawableLayer.h>
#include <gbp/Factor.h>
#include <gbp/Variable.h>

/********************************************************************************************/
// Factors and variables for the planning layer. A factor subclasses Factor and overrides
// computeResidualAndJacobian() to return {h(X) - z, Jacobian} (closed-form or via jacobianFirstOrder()).
// Layer-specific variables subclass Variable (see PathVariable).
/********************************************************************************************/
/* Dynamics factor: constant-velocity motion model between consecutive planned states */
class DynamicsFactor: public Factor {
    public:
    Eigen::MatrixXd J_lin_;

    DynamicsFactor(Key fac_key, std::vector<Key> connected_variables,
        float sigma, float dt, int dof_dynamics);

    Eigen::MatrixXd computeResidual(const std::vector<LieGroup>& X);                                   // h(X) - z
    std::pair<Eigen::VectorXd, Eigen::MatrixXd> computeResidualAndJacobian(const std::vector<LieGroup>& X) override;
    void draw(Eigen::VectorXd p1, Eigen::VectorXd p2, Color=BLACK) override;

};

/********************************************************************************************/
// Interrobot factor: collision avoidance between two robots. High energy when both robots plan to
// occupy the same position at the same timestep; zero energy (skip_ = true) beyond the safety distance.
/********************************************************************************************/
class InterrobotFactor: public Factor {
    public:
    double safety_distance_;
    double min_sep_;

    InterrobotFactor(Key fac_key, std::vector<Key> connected_variables,
        float sigma, float robot_radius, int dof);

    Eigen::MatrixXd computeResidual(const std::vector<LieGroup>& X);                                   // h(X) - z
    Eigen::MatrixXd computeJacobian(const std::vector<LieGroup>& X);                                    // closed-form Jacobian
    std::pair<Eigen::VectorXd, Eigen::MatrixXd> computeResidualAndJacobian(const std::vector<LieGroup>& X) override;
    bool skipFactor() override;

};

/********************************************************************************************/
// Obstacle factor for static obstacles, holding a pointer to the Simulator's obstacle image.
// White areas (value 1) are obstacles. The Jacobian delta is sized to one pixel of that image.
/********************************************************************************************/
class ObstacleFactor: public Factor {
    public:
    DistanceField* p_obstacleImgDist_;

    ObstacleFactor(Key fac_key, std::vector<Key> connected_variables,
        float sigma, DistanceField* p_obstacleImgDist, int dof);

    Eigen::MatrixXd computeResidual(const std::vector<LieGroup>& X);                                   // h(X) - z
    std::pair<Eigen::VectorXd, Eigen::MatrixXd> computeResidualAndJacobian(const std::vector<LieGroup>& X) override;

};

/********************************************************************************************/
/* Unicycle model factor: keeps velocity aligned with heading theta (y'/x' = tan(theta)) */
/********************************************************************************************/
class UnicycleModelFactor: public Factor {
    public:

    UnicycleModelFactor(Key fac_key, std::vector<Key> connected_variables,
        float sigma, int dof);

    Eigen::MatrixXd computeResidual(const std::vector<LieGroup>& X);                                   // h(X) - z
    std::pair<Eigen::VectorXd, Eigen::MatrixXd> computeResidualAndJacobian(const std::vector<LieGroup>& X) override;

};

/********************************************************************************************/
// Planned-path variable: a pose [x, y, theta, ...] drawn as a sphere at its mean.
/********************************************************************************************/
class PathVariable: public Variable {
    public:
    float height_3D_ = 0.f;                     // out-of-plane height (3d visualisation only)
    float size_ = 1.f;                          // display radius
    PathVariable(Key vkey, const Eigen::VectorXd& mu_prior, const Eigen::VectorXd& sigma_prior_list, float size, float height_3D);

    void draw(Color col);

};

/******************************************************************************************************/
// Planning layer: the planned path (pose variables linked by dynamics/obstacle/unicycle factors) plus
// inter-robot collision-avoidance factors.
/******************************************************************************************************/
class PlanningLayer : public FactorGraphLayer, public DrawableLayer {
public:
    int num_variables_ = 0;                     // number of variables in the planned path
    std::vector<int> variable_timesteps_{};     // timesteps at which planned-path variables are placed
    int dof_dynamics_ = 6;                       // x,y,θ,x',y',θ'

    // The timesteps at which planned-path variables are placed (pure function of horizon H, multiple M).
    static std::vector<int> getVariableTimesteps(int H, int M);

    // Layer tuning, read from this layer's config param bag.
    float SIGMA_VARIABLE_HORIZON, SIGMA_FACTOR_DYNAMICS, SIGMA_FACTOR_UNICYCLE,
          SIGMA_FACTOR_INTERROBOT, SIGMA_FACTOR_OBSTACLE;

    PlanningLayer(int graph_id, int lid, const LayerConfig& cfg = {}) : FactorGraphLayer(graph_id, lid, cfg) {
        SIGMA_VARIABLE_HORIZON  = cfg.params.numf("SIGMA_VARIABLE_HORIZON", 1e-2f);
        SIGMA_FACTOR_DYNAMICS   = cfg.params.numf("SIGMA_FACTOR_DYNAMICS",  0.1f);
        SIGMA_FACTOR_UNICYCLE   = cfg.params.numf("SIGMA_FACTOR_UNICYCLE",  0.001f);
        SIGMA_FACTOR_INTERROBOT = cfg.params.numf("SIGMA_FACTOR_INTERROBOT",0.01f);
        SIGMA_FACTOR_OBSTACLE   = cfg.params.numf("SIGMA_FACTOR_OBSTACLE",  0.01f);
    }
    void initialiseLayerNodes(FactorGraph& owner) override;
    void createInterrobotFactors(FactorGraph& owner, int other_id) override;
    void preGBPUpdateNodes(FactorGraph& owner, uint32_t clock) override;   // no-op for planning
    void postGBPUpdateNodes(FactorGraph& owner) override;
    void draw(Robot* robot, Simulator* sim, Color& col) override;

    // Per-timestep path operations, driven by the robot/simulator.
    void propogateCurrent(Robot* robot);                 // slide the current-state prior one timestep forward
    void propogateHorizon(Robot* robot, int towards=1);  // move the horizon state towards goal / formation
    void updateDisplayStats(Robot* robot);               // refresh the robot's position/speed/bearing/trajectory
};
