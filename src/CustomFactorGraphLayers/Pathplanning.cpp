/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <CustomFactorGraphLayers/Pathplanning.h>
#include <Robot.h>
#include <ShapeFormation.h>

/**************************************************************************************/
/*                                  PLANNING LAYER                                    */
/**************************************************************************************/

/***************************************************************************/
// Determine the timesteps at which variables in the planned path are placed.
/***************************************************************************/
std::vector<int> PlanningLayer::getVariableTimesteps(int lookahead_horizon, int lookahead_multiple){
    // Variables come in groups of lookahead_multiple; the spacing within a group grows by one per group.
    // e.g. for lookahead_multiple 3:   0,  1, 2, 3,  5, 7, 9,  12, 15, 18, ...
    // The aim: first variable one timestep ahead of the current state, early variables close together,
    // all at integer timesteps, spacing roughly exponential.
    std::vector<int> var_list{};
    int N = 1 + int(0.5*(-1 + sqrt(1 + 8*(float)lookahead_horizon/(float)lookahead_multiple)));

    for (int i=0; i<lookahead_multiple*(N+1); i++){
        int section = int(i/lookahead_multiple);
        int f = (i - section*lookahead_multiple + lookahead_multiple/2.*section)*(section+1);
        if (f>=lookahead_horizon){
            var_list.push_back(lookahead_horizon);
            break;
        }
        var_list.push_back(f);
    }

    return var_list;
}

/***************************************************************************/
// Build the planned-path variables and their intra-robot factors:
//   - PathVariables interpolated between start and horizon (fixed priors on the ends)
//   - DynamicsFactors between consecutive variables
//   - ObstacleFactors (if obstacles present)
//   - UnicycleModelFactors
/***************************************************************************/
void PlanningLayer::initialiseLayerNodes(FactorGraph& owner){
    Robot* robot = static_cast<Robot*>(&owner);
    const int layer = lid_;
    const Eigen::VectorXd& start   = robot->start_;
    const Eigen::VectorXd& horizon = robot->horizon_;

    // Spacing increases with timestep, so a horizon of e.g. 10 timesteps needs far fewer variables.
    variable_timesteps_ = getVariableTimesteps(robot->T_HORIZON / robot->T0, robot->LOOKAHEAD_MULTIPLE);
    num_variables_ = variable_timesteps_.size();
    const std::vector<int>& variable_timesteps = variable_timesteps_;

    /***************************************************************************/
    /* Create Variables with fixed pose priors on start and horizon variables. */
    /***************************************************************************/
    double sigma;
    Eigen::VectorXd mu(dof_dynamics_); Eigen::VectorXd sigma_list(dof_dynamics_);
    for (int i = 0; i < num_variables_; i++){
        // Interpolate mu between start and horizon
        mu = start + (horizon - start) * (float)(variable_timesteps[i]/(float)variable_timesteps.back());
        mu(2) = angleOp(mu(2));
        // Pin the start and horizon variables during optimisation
        sigma = (i==0 || i==num_variables_-1) ? robot->SIGMA_POSE_FIXED : 1e4;
        if (i==num_variables_-1) {
            sigma = SIGMA_VARIABLE_HORIZON;
        };
        sigma_list.setConstant(sigma);

        robot->addVariable<PathVariable>(layer, mu, sigma_list, robot->robot_radius_, robot->height_3D_);
    }

    /***************************************************************************/
    /* Create Dynamics factors between variables */
    /***************************************************************************/
    for (int i = 0; i < num_variables_-1; i++)
    {
        // T0 is the timestep between the current state and the first planned state.
        float delta_t = robot->T0 * (variable_timesteps[i + 1] - variable_timesteps[i]);
        std::vector<Key> connected_variables{robot->getVar(layer, i)->key_, robot->getVar(layer, i+1)->key_};
        robot->addFactor<DynamicsFactor>(layer, connected_variables, SIGMA_FACTOR_DYNAMICS, delta_t, dof_dynamics_);
    }

    /***************************************************************************/
    // Obstacle factors for all variables except start and horizon. (Platform-neutral now: the distance
    // field is filled from OpenCV on desktop and from a C++ transform on web - see Graphics.)
    /***************************************************************************/
    if (globals.obstacles_present_ && robot->p_obstacleImgDist_ && !robot->p_obstacleImgDist_->empty()){
        for (int i = 1; i < num_variables_-1; i++)
        {
            std::vector<Key> connected_variables{robot->getVar(layer, i)->key_};
            float sigma = SIGMA_FACTOR_OBSTACLE;
            if (i==num_variables_-1) sigma *= SIGMA_VARIABLE_HORIZON;
            robot->addFactor<ObstacleFactor>(layer, connected_variables, sigma, robot->p_obstacleImgDist_, dof_dynamics_);
        }
    }
    /***************************************************************************/
    // Unicycle model factors for all variables except horizon.
    /***************************************************************************/
    for (int i = 0; i < num_variables_-1; i++)
    {
        std::vector<Key> connected_variables{robot->getVar(layer, i)->key_};
        double sigma = SIGMA_FACTOR_UNICYCLE;
        if (i==num_variables_-1) sigma *= SIGMA_VARIABLE_HORIZON;
        robot->addFactor<UnicycleModelFactor>(layer, connected_variables, SIGMA_FACTOR_UNICYCLE, dof_dynamics_);
    }
    for (int j=0; j<10; j++){
        robot->stack_[layer]->factorIteration();
        robot->stack_[layer]->variableIteration();
    }

    // Seed the robot's displayed position/speed/bearing/trajectory from the built path.
    updateDisplayStats(robot);
}

/***************************************************************************/
// Inter-robot collision-avoidance factors between this robot and another.
/***************************************************************************/
void PlanningLayer::createInterrobotFactors(FactorGraph& owner, int other_robot_id){
    Robot* robot = static_cast<Robot*>(&owner);
    const int layer = lid_;
    // One factor per timestep, excluding the current state.
    int num_vars = robot->stack_[layer]->variables_.size();
    for (int i = 1; i < num_vars-1; i++){
        Key this_var_key = robot->getVar(layer, i)->key_;
        Key other_var_key = Key(other_robot_id, this_var_key.node_id_, this_var_key.lid_, Node::VAR);

        double sigma = SIGMA_FACTOR_INTERROBOT;
        auto factor = robot->addFactor<InterrobotFactor>(layer, {this_var_key, other_var_key}, sigma, robot->robot_radius_, dof_dynamics_);
        // addFactor registers the local variable; the cross-robot half lives in the robot inbox.
        robot->inbox_[std::make_pair(factor->key_, other_var_key)] = Message(factor->LG);
    }
}

/***************************************************************************/
// No pre-GBP node update for planning.
/***************************************************************************/
void PlanningLayer::preGBPUpdateNodes(FactorGraph& /*owner*/, uint32_t /*clock*/){
}

/***************************************************************************/
// Per-timestep update of the planned path: move the horizon towards the goal
// (or formation), then slide the current state one timestep forward.
/***************************************************************************/
void PlanningLayer::postGBPUpdateNodes(FactorGraph& owner){
    Robot* robot = static_cast<Robot*>(&owner);
    propogateHorizon(robot, static_cast<int>(!globals.TOWARDS_FORMATION));
    propogateCurrent(robot);

    // Pop the front waypoint once the horizon reaches it (keeping the last). Continuous-motion
    // formations (e.g. "random") refill their queues themselves.
    if (robot->waypoints_.size() > 1){
        Eigen::VectorXd horizon = robot->getVar(lid_, -1)->belief_.mu.coeffs()({0,1});
        Eigen::VectorXd horizon2goal = robot->waypoints_.front()({0,1}) - horizon;
        if (horizon2goal.norm() < 1.*robot->robot_radius_){
            robot->waypoints_.pop_front();
        }
    }
}

/***************************************************************************/
// Slide the current-state prior one timestep forward along the planned path.
/***************************************************************************/
void PlanningLayer::propogateCurrent(Robot* robot){
    Eigen::VectorXd increment = (robot->getVar(lid_, 1)->belief_.mu - robot->getVar(lid_, 0)->belief_.mu);
    increment(2) = angleOp(increment(2)); increment *= robot->TIMESTEP / robot->T0;
    // Clamp the position step so the robot doesn't jump when interrobot repulsion stretches the path.
    double pos_step = increment({0,1}).norm();
    if (pos_step > robot->max_speed_ * robot->TIMESTEP)
        increment({0,1}) *= (robot->max_speed_ * robot->TIMESTEP) / pos_step;
    robot->getVar(lid_, 0)->changeVariablePrior(robot->getVar(lid_, 0)->belief_.mu + increment);
    updateDisplayStats(robot);
}

/***************************************************************************/
// Refresh the robot's displayed position/bearing and append to its trajectory.
/***************************************************************************/
void PlanningLayer::updateDisplayStats(Robot* robot){
    robot->position_ = robot->getVar(lid_, 0)->belief_.mu.coeffs();
    robot->bearing_ = robot->position_(2);
    robot->trajectory_.push_back(robot->position_({0,1}));
    // Cap trajectory length if configured (-1 = keep all).
    if (globals.MAX_TRAJ_LEN>=0 && (int)robot->trajectory_.size() > globals.MAX_TRAJ_LEN){
        robot->trajectory_.erase(robot->trajectory_.begin(), robot->trajectory_.end() - globals.MAX_TRAJ_LEN);
    }
}

/***************************************************************************/
// Move the horizon state's prior, towards the next waypoint (towards==1) or
// towards the best free formation point (towards==0).
/***************************************************************************/
void PlanningLayer::propogateHorizon(Robot* robot, int towards){
    // horizon[t+1] = horizon[t] + dt * horizon.vel[t]
    auto horizon = robot->getVar(lid_, -1);

    // Velocity towards the goal waypoint or the formation.
    Eigen::VectorXd new_vel = Eigen::VectorXd::Zero(horizon->n_dofs_/2);
    if (robot->waypoints_.size()>0) {
        if (towards==1){
            new_vel = robot->getHorizonVelocityTowards(horizon->belief_.mu.coeffs()({0,1,2}), horizon->belief_.mu.coeffs()({3,4,5}), robot->waypoints_.front()({0,1,2}), robot->max_speed_, (robot->waypoints_.size()<=1), true);
        } else {
            /****************** FOR ATTRACTION TO SHAPE FORMATION *************/
            manif::SE2d& liePose = robot->formationPoints_->liePose_;
            manif::SE2d liePose_inv = robot->formationPoints_->liePose_.inverse();
            const float search_radius_others = pow(2. * globals.ROBOT_RADIUS, 2.);
            const float search_radius_mine = pow(1. * globals.ROBOT_RADIUS, 2.); // kept equal to the above
            // Occupancy weight applied to the 3rd (decay) dimension of the kd-tree formation points.
            const double HIGH = globals.OCC_WEIGHTING_HIGH_VAL;


            // Current position in the formation frame (formation defined at the origin).
            Eigen::Vector3d current_F; current_F << liePose_inv.act(Eigen::Vector2d(robot->getVar(lid_, 0)->belief_.mu.coeffs()({0,1}))), 0.;
            // Formation points within comms range are assumed unoccupied (3rd dim = 0); those out of range
            // decay their 3rd dim towards 0 (might still be occupied, so de-prioritise in the search).
            for (auto& p : robot->formationPoints_->points_){
                if (!globals.OCCUPANCY_WEIGHTING_DECAY) {p(2) = 0.; continue;};
                p(2) = ((p({0,1}) - current_F({0,1})).squaredNorm() < robot->COMMUNICATION_RADIUS) ? 0. : std::max(0., p(2)-1.);
            }

            // Mark formation points near a neighbour (xy only) with a HIGH 3rd dim, excluding them from the search.
            for (auto [crid, pos] : robot->neighbour_pos_){
                Eigen::Vector3d neighbour_F; neighbour_F << liePose_inv.act(Eigen::Vector2d(pos({0,1}))), 0.;
                auto [found, matches] = robot->formationPoints_->radiusSearch(neighbour_F, search_radius_others, 2);
                if (found){
                    for (auto& m : matches) robot->formationPoints_->points_[m.first](2) = HIGH;
                }
            }

            robot->formationPoints_->kdtree_formation_->index->buildIndex();

            // Nearest formation point including the 3rd dimension. The current position has 3rd dim 0,
            // so this prefers points known to be unoccupied.
            auto [found, idxs, dists_sqr] = robot->formationPoints_->knnSearch(current_F, 1, 3);
            int idx = 0;
            if (found && dists_sqr[idx] > search_radius_mine){
                Eigen::Vector2d goal = liePose.act(Eigen::Vector2d(robot->formationPoints_->points_[idxs[idx]]({0,1})));     // closest point in world frame
                new_vel = std::min(1., pow((goal - robot->getVar(lid_, 0)->belief_.mu.coeffs()({0,1})).norm()/globals.ROBOT_RADIUS, 2.)) * robot->getHorizonVelocityTowards(horizon->belief_.mu.coeffs()({0,1,2}), horizon->belief_.mu.coeffs()({3,4,5}), goal, robot->max_speed_, true, 1);
            }
        }
    }

    Eigen::VectorXd new_horz_state(horizon->n_dofs_);
    Eigen::VectorXd pos = horizon->belief_.mu.coeffs()({0,1,2}) + robot->TIMESTEP * new_vel;
    // Cap how far the horizon may lead the current state. The horizon advances open-loop toward the goal
    // each timestep; if the robot can't keep up (blocked, or GBP stalled e.g. high damping) the planned
    // path would otherwise stretch without bound. Pin it at max_lead ahead - using the same value for the
    // threshold and the clamp so it sits there steadily rather than sawtoothing (drift out, snap back).
    const double max_lead = 1.5 * robot->max_speed_ * globals.T_HORIZON;
    Eigen::VectorXd curr2pos = pos({0,1}) - robot->getVar(lid_, 0)->belief_.mu.coeffs()({0,1});
    if (curr2pos.norm() > max_lead){
        pos({0,1}) = robot->getVar(lid_, 0)->belief_.mu.coeffs()({0,1}) + max_lead * curr2pos.normalized();
    }
    new_horz_state << pos, new_vel;

    horizon->changeVariablePrior(new_horz_state);
}

/***************************************************************************/
// Draw the planning layer: planned path, executed trajectory, and waypoints.
/***************************************************************************/
void PlanningLayer::draw(Robot* robot, Simulator* sim, Color& col){
    // Planned path: the dynamics factors linking consecutive planned-path variables.
    if (globals.DRAW_PATH){
        for (auto [f_key, factor] : factors_) {
            if (factor->connected_v_keys_.size()!=2 || factor->other_rid_!=graph_id_) continue;
            factor->draw(robot->getVar(factor->connected_v_keys_[0])->belief_.mu.coeffs(),
                         robot->getVar(factor->connected_v_keys_[1])->belief_.mu.coeffs(), ColorAlpha(col, 0.5));
        }
    }
    // Executed trajectory: a fading line through past positions.
    if (globals.DRAW_TRAJ){
        int n = robot->trajectory_.size();
        for (int i=1; i<n; i++){
            DrawLine3D(Vector3{(float)robot->trajectory_[i-1](0), robot->height_3D_ + robot->robot_radius_, (float)robot->trajectory_[i-1](1)},
                       Vector3{(float)robot->trajectory_[i](0), robot->height_3D_ + robot->robot_radius_, (float)robot->trajectory_[i](1)},
                       ColorAlpha(robot->traj_col_, i/(float)n));
        }
    }
    // Waypoints: a cube at each outstanding goal.
    if (globals.DRAW_WAYPOINTS){
        for (int wp_idx=0; wp_idx<robot->waypoints_.size(); wp_idx++){
            DrawCubeV(Vector3{(float)robot->waypoints_[wp_idx](0), robot->height_3D_ + robot->robot_radius_, (float)robot->waypoints_[wp_idx](1)},
                      Vector3{0.5f*robot->robot_radius_, 0.5f*robot->robot_radius_, 0.5f*robot->robot_radius_}, col);
        }
    }
}


/********************************************************************************************/
//                      PLANNING-LAYER FACTORS AND VARIABLES
// computeResidual() = h(X) - z_, where h() is the measurement function at the current linearisation
// point and z_ the observation. computeResidualAndJacobian() pairs it with the Jacobian.
/********************************************************************************************/

/********************************************************************************************/
/* Dynamics factor: constant-velocity motion model between consecutive planned states */
/********************************************************************************************/
DynamicsFactor::DynamicsFactor(Key fac_key, std::vector<Key> connected_variables,
    float sigma, float dt, int dof_dynamics)
    : Factor{fac_key, connected_variables, "R" + std::to_string(dof_dynamics), Eigen::VectorXd::Zero(dof_dynamics), Eigen::VectorXd::Constant(dof_dynamics, sigma)}{
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_dofs_/2,n_dofs_/2);
        Eigen::MatrixXd O = Eigen::MatrixXd::Zero(n_dofs_/2,n_dofs_/2);
        Eigen::MatrixXd Qc_inv = pow(sigma, -2.) * I;

        Eigen::MatrixXd Qi_inv(n_dofs_, n_dofs_);
        Qi_inv << 12.*pow(dt, -3.) * Qc_inv,   -6.*pow(dt, -2.) * Qc_inv,
                  -6.*pow(dt, -2.) * Qc_inv,   4./dt * Qc_inv;   

        this->meas_model_lambda_ = Qi_inv;

        // Jacobian is constant (model is linear), so cache it.
        J_lin_ = Eigen::MatrixXd::Zero(n_dofs_, n_dofs_*2);
        J_lin_ << I, dt*I, -1*I,    O,
                  O,    I,    O, -1*I; 

    };

Eigen::MatrixXd DynamicsFactor::computeResidual(const std::vector<LieGroup>& X){
    Eigen::VectorXd x(n_dofs_*2);
    x << X[0].coeffs(), X[1].coeffs();   // the factor is linear in [state_i, state_{i+1}]
    Eigen::VectorXd h = J_lin_ * x;
    h(2) = angleOp(h(2));
    return h - z_;
}
std::pair<Eigen::VectorXd, Eigen::MatrixXd> DynamicsFactor::computeResidualAndJacobian(const std::vector<LieGroup>& X){
    Eigen::MatrixXd J = jacobianFirstOrder(X, [this](const std::vector<LieGroup>& x){ return computeResidual(x); });
    return { Eigen::VectorXd(computeResidual(X)), J };
}

void DynamicsFactor::draw(Eigen::VectorXd p1, Eigen::VectorXd p2, Color col){
    if (!globals.DRAW_PATH) return;
    float radius = 0.1*globals.ROBOT_RADIUS;
    DrawCylinderEx(Vector3{(float)p1(0), globals.ROBOT_RADIUS, (float)p1(1)},
                    Vector3{(float)p2(0), globals.ROBOT_RADIUS, (float)p2(1)}, 
                    radius, radius, 4, col);
}

/********************************************************************************************/
// Interrobot factor: collision avoidance between two robots. High energy when both plan to occupy
// the same position at the same timestep; zero energy (skip_ = true) beyond the safety distance.
/********************************************************************************************/

InterrobotFactor::InterrobotFactor(Key fac_key, std::vector<Key> connected_variables,
    float sigma, float robot_radius, int dof)
    : Factor{fac_key, connected_variables, "R" + std::to_string(dof), Eigen::VectorXd::Zero(1), Eigen::VectorXd::Constant(1, sigma)} {
        float eps = 0.2 * robot_radius;
        this->safety_distance_ = 10*robot_radius + eps;
        this->min_sep_ = 2.0*globals.ROBOT_RADIUS;
};


Eigen::MatrixXd InterrobotFactor::computeJacobian(const std::vector<LieGroup>& X){
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(n_dofs_meas_, n_dofs_*2);
    Eigen::VectorXd a = X[0].coeffs(), b = X[1].coeffs();
    double x1 = a(0), y1 = a(1), x2 = b(0), y2 = b(1);
    double r2 = Eigen::Vector2d(x2-x1, y2-y1).squaredNorm();

    if (r2 <= pow(safety_distance_, 2)){
        double r = sqrt(r2) + 1e-3;
        double k1 = -1/this->min_sep_;
        double K = (k1*exp(k1*r)) / r;
        J(0, 0) = K * (x1 - x2);
        J(0, 1) = K * (y1 - y2);
        J(0, n_dofs_) = K * (x2 - x1);
        J(0, n_dofs_+1) = K * (y2 - y1);
    }
    return J;
};

Eigen::MatrixXd InterrobotFactor::computeResidual(const std::vector<LieGroup>& X){
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(n_dofs_meas_, 1);
    Eigen::VectorXd a = X[0].coeffs(), b = X[1].coeffs();
    double x1 = a(0), y1 = a(1), x2 = b(0), y2 = b(1);
    double r2 = Eigen::Vector2d(x2-x1, y2-y1).squaredNorm();

    if (r2 <= pow(safety_distance_, 2)){
        double r = sqrt(r2) + 1e-3;
        double k1 = -1/this->min_sep_;
        h(0) = exp(k1*r);
    }

    return h - z_;
};

std::pair<Eigen::VectorXd, Eigen::MatrixXd> InterrobotFactor::computeResidualAndJacobian(const std::vector<LieGroup>& X){
    return { Eigen::VectorXd(computeResidual(X)), computeJacobian(X) };
}

bool InterrobotFactor::skipFactor(){
    // The two robots' current positions are the [x,y] of each variable's lin point.
    Eigen::VectorXd p1 = linpoints_[0].coeffs(), p2 = linpoints_[1].coeffs();
    double x1 = p1(0), y1 = p1(1), x2 = p2(0), y2 = p2(1);
    double r2 = Eigen::Vector2d(x2-x1, y2-y1).squaredNorm();

    bool should_skip_factor = ( r2 >= pow(safety_distance_, 2) );
    return should_skip_factor;
}


/********************************************************************************************/
// Obstacle factor for static obstacles, using the Simulator's obstacle distance transform.
// White areas (value 1) are obstacles. The Jacobian delta is sized to one pixel of the image.
/********************************************************************************************/
double getValueFromImgDist(int x, int y, DistanceField* p_obsImg){
    // The distance field returns 1 (free) out of bounds; 0 at/near an obstacle.
    return p_obsImg ? p_obsImg->value01(x, y) : 1.0;
}

ObstacleFactor::ObstacleFactor(Key fac_key, std::vector<Key> connected_variables,
    float sigma, DistanceField* p_obstacleImgDist, int dof)
    : Factor{fac_key, connected_variables, "R" + std::to_string(dof), Eigen::VectorXd::Zero(1), Eigen::VectorXd::Constant(1, sigma)}, p_obstacleImgDist_(p_obstacleImgDist){
        if (p_obstacleImgDist_ && p_obstacleImgDist_->cols() > 0)
            this->delta_jac = 2.*(float)globals.WORLD_SZ / (float)p_obstacleImgDist_->cols();
};
Eigen::MatrixXd ObstacleFactor::computeResidual(const std::vector<LieGroup>& X){
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(1,1);
    Eigen::VectorXd s = X[0].coeffs();
    // White areas are obstacles, so h(0) is 1 there.
    double x, y;
    if (n_dofs_!=6){
        double V = s({3,4}).norm(); double theta = s(2); double thetadot = s(5);
        x = s(0) + (V * cos(theta)) * globals.TIMESTEP;
        y = s(1) + (V * sin(theta)) * globals.TIMESTEP;
    } else {
        x = s(0) + s(2) * globals.TIMESTEP;
        y = s(1) + s(3) * globals.TIMESTEP;
    }
    int idx_x = (x + globals.WORLD_SZ/2) * (p_obstacleImgDist_->cols()/globals.WORLD_SZ);
    int idx_y = (y + globals.WORLD_SZ/2) * (p_obstacleImgDist_->rows()/globals.WORLD_SZ);
    h(0) = 1. - getValueFromImgDist(idx_x, idx_y, p_obstacleImgDist_);

    return h - z_;
};
std::pair<Eigen::VectorXd, Eigen::MatrixXd> ObstacleFactor::computeResidualAndJacobian(const std::vector<LieGroup>& X){
    Eigen::MatrixXd J = jacobianFirstOrder(X, [this](const std::vector<LieGroup>& x){ return computeResidual(x); });
    return { Eigen::VectorXd(computeResidual(X)), J };
};

/********************************************************************************************/
// Unicycle model factor: nonholonomic constraint keeping velocity along heading theta,
// i.e. V*cos(theta) = xdot, V*sin(theta) = ydot.
/********************************************************************************************/
UnicycleModelFactor::UnicycleModelFactor(Key fac_key, std::vector<Key> connected_variables,
    float sigma, int dof)
    : Factor{fac_key, connected_variables, "R" + std::to_string(dof), Eigen::VectorXd::Zero(2), Eigen::VectorXd::Constant(2, sigma)}{
};
Eigen::MatrixXd UnicycleModelFactor::computeResidual(const std::vector<LieGroup>& X){
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(n_dofs_meas_,1);
    Eigen::VectorXd s = X[0].coeffs();
    Eigen::VectorXd velocity = s({3,4}); double V = velocity.norm();
    h(0) = V*cos(s(2)) - s(3);
    h(1) = V*sin(s(2)) - s(4);
    return h - z_;
};
std::pair<Eigen::VectorXd, Eigen::MatrixXd> UnicycleModelFactor::computeResidualAndJacobian(const std::vector<LieGroup>& X){
    Eigen::MatrixXd J = jacobianFirstOrder(X, [this](const std::vector<LieGroup>& x){ return computeResidual(x); });
    return { Eigen::VectorXd(computeResidual(X)), J };
};

/********************************************************************************************/
// Planned-path variable: a pose drawn as a sphere at its belief mean.
/********************************************************************************************/
PathVariable::PathVariable(Key vkey, const Eigen::VectorXd& mu_prior, const Eigen::VectorXd& sigma_prior_list,
                            float size, float height_3D)
    : Variable{vkey, LieGroup(mu_prior), sigma_prior_list}, height_3D_(height_3D), size_(size) {   // R^N group at the mean
};

void PathVariable::draw(Color col){
    if (valid_) {
        DrawSphere(Vector3{(float)belief_.mu.coeffs()(0), height_3D_, (float)belief_.mu.coeffs()(1)}, 0.5*size_, ColorAlpha(col, 0.5));
    }
    return;
};

