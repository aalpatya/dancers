/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <Robot.h>
#include <ShapeFormation.h>
#include <CustomFactorGraphLayers/DrawableLayer.h>   // Robot::draw asks each layer if it's drawable
#include <CustomFactorGraphLayers/Pathplanning.h>
#include <CustomFactorGraphLayers/Consensus.h>
#include <CustomFactorGraphLayers/ExampleLayer.h>

// Build a layer from its config `type`. The one list of available layer types; to add your own, write
// the class (copy ExampleLayer) and add a line here.
static std::shared_ptr<FactorGraphLayer> makeLayer(const LayerConfig& cfg, int graph_id, int lid){
    const std::string& type = cfg.type;
    if (type == "planning")  return std::make_shared<PlanningLayer>(graph_id, lid, cfg);
    if (type == "consensus") return std::make_shared<ConsensusLayer>(graph_id, lid, cfg);
    if (type == "example")   return std::make_shared<ExampleLayer>(graph_id, lid, cfg);
    throw std::runtime_error("Unknown factor-graph layer type '" + type + "' (add it to makeLayer in Robot.cpp)");
}

/***************************************************************************/
// Build a robot: construct its configured layer stack, cache the tuning it needs, set up the
// start/horizon states, then let each layer build its own nodes. See Robot.h for the arguments.
/***************************************************************************/
Robot::Robot(int rid,
             std::deque<Eigen::VectorXd> waypoints,
             Color color,
             DistanceField* p_obstacleImgDist,
             int decision, bool seed_robot, bool is_special, double max_speed,
             float size, double turning_radius, float communication_radius) : FactorGraph(rid),
             rid_(rid),
             waypoints_(waypoints), decision_(decision), seed_robot_(seed_robot), is_special_(is_special),
             robot_radius_(size), color_(color), max_speed_(max_speed),
             min_turning_radius_(turning_radius), COMMUNICATION_RADIUS(communication_radius) {

    // Build the layer stack from FACTORGRAPH_LAYERS: makeLayer maps each config's `type` to its class,
    // adopt() wires it in.
    for (int id = 0; id < (int)layerConfigs().size(); id++)
        adopt(makeLayer(layerConfigs()[id], rid, id));

    // Stash the obstacle image the layers read when building their nodes.
    p_obstacleImgDist_ = p_obstacleImgDist;

    // Config-wide values the layers read off the robot (identical for every robot). Per-robot size,
    // speed, turning radius and comms radius come in via the constructor; the consensus Lie group lives
    // on the ConsensusLayer.
    TIMESTEP           = globals.TIMESTEP;
    T_HORIZON          = globals.T_HORIZON;
    T0                 = globals.T0;
    LOOKAHEAD_MULTIPLE = globals.LOOKAHEAD_MULTIPLE;
    SIGMA_POSE_FIXED   = globals.SIGMA_POSE_FIXED;
    NUM_DECISIONS      = globals.NUM_DECISIONS;

    formationPoints_ = new ShapeFormation();         

    // The robot always aims its horizon state at the next waypoint, popping a waypoint once reached.
    start_ = position_ = waypoints_[0];
    trajectory_.push_back(position_({0,1}));
    waypoints_.pop_front();
    Eigen::VectorXd goal = (waypoints_.size()>0) ? waypoints_[0] : start_;
    // Place the horizon towards the goal, at distance T_HORIZON * MAX_SPEED from the start, with its
    // velocity along the travel direction. Use the start->goal direction throughout (not goal-minus-
    // horizon, which is the zero vector whenever the goal is within reach -> a NaN/zero horizon velocity
    // that destabilises the planner, especially for large T_HORIZON). Fall back to the start heading if
    // start == goal so we never normalise a zero vector.
    Eigen::VectorXd start2goal = goal - start_;
    double dist = start2goal({0,1}).norm();
    Eigen::Vector2d dir = (dist > 1e-6) ? Eigen::Vector2d(start2goal({0,1}) / dist)
                                        : Eigen::Vector2d(cos(start_(2)), sin(start_(2)));
    horizon_ = goal;
    horizon_({0,1}) = start_({0,1}) + std::min(dist, 1.*T_HORIZON*max_speed_) * dir;
    horizon_(2) = start_(2);
    horizon_({3,4}) = max_speed_ * dir;

    // Let each layer build its own variables and intra-robot factors (the planning layer also
    // initialises the robot's display stats from the path it builds).
    initialiseLayers();

};

/***************************************************************************************************/
/* Destructor */
/***************************************************************************************************/
Robot::~Robot(){
}

/***************************************************************************************************/
// For new neighbours of a robot, create inter-robot factors if they don't exist. 
// Delete existing inter-robot factors for faraway robots
/***************************************************************************************************/
void Robot::updateInterrobotFactors(){
    
    // Search through currently connected rids. If any are not in neighbours, delete interrobot factors.
    std::vector<int> rids_to_delete{};
    for (auto rid : connected_r_ids_){
        if (std::find(neighbours_.begin(), neighbours_.end(), rid)==neighbours_.end()){
            deleteInterrobotFactors(rid);
            rids_to_delete.push_back(rid);
        };
    }
    for (auto r : rids_to_delete){
        // Remove other robot from current robot's connected rids
        auto it = std::find(connected_r_ids_.begin(), connected_r_ids_.end(), r);
        if (it != connected_r_ids_.end()) connected_r_ids_.erase(it);
    }

    // Search through neighbours. If any are not in currently connected rids, create interrobot factors.
    for (auto rid : neighbours_){
        if (std::find(connected_r_ids_.begin(), connected_r_ids_.end(), rid)==connected_r_ids_.end()){
            
            if (globals.SYMMETRIC_INTERROBOT_FACTORS || rid_ < rid){
                makeInterrobotFactors(rid);   // fans out to each layer
            }
            // Add the other robot to this robot's list of connected robots.
            connected_r_ids_.push_back(rid);            
        };
    }
}

/***************************************************************************************************/
// Draw the robot. This is a 2D problem; out-of-plane height is height_3D_.
/***************************************************************************************************/
void Robot::draw(Simulator* sim, Globals& globals){

    // Base colour: gray when inter-robot comms are off (e.g. simulated message failure); blink if
    // special.
    Color col = color_;
    if (is_special_) col = (sim->clock_%20 < 10) ? DARKBLUE : RED;
    col = (interrobot_comms_active_) ? col : GRAY;

    // Each drawable layer draws its own contribution (planning: path/trajectory/waypoints; consensus:
    // the consensus variables, and it may tint `col` for the robot model below).
    for (auto& [lyr, fg] : stack_)
        if (auto* d = dynamic_cast<DrawableLayer*>(fg.get())) d->draw(this, sim, col);

    // Inter-robot connections: a line to each connected robot. Robot-level (driven by connected_r_ids_),
    // so it renders regardless of which layers are active.
    if (globals.DRAW_INTERROBOT && interrobot_comms_active_){
        for (auto rid : connected_r_ids_){
            const auto& other = sim->robots_.at(rid);
            if (!other->interrobot_comms_active_) continue;
            DrawLine3D(Vector3{(float)position_(0), height_3D_ + robot_radius_, (float)position_(1)},
                       Vector3{(float)(other->position_(0)), other->height_3D_ + other->robot_radius_, (float)(other->position_(1))},
                       RED);
        }
    }

    // The robot model and formation overlay are robot-level, not layer-specific.
    DrawModelEx(sim->graphics->robotModel_, Vector3{(float)position_(0), height_3D_, (float)position_(1)}, Vector3{0, -1, 0}, bearing_*RAD2DEG,
                Vector3{robot_radius_, robot_radius_, robot_radius_}, col);
    formationPoints_->draw(globals.FORMATION_DISPLAY_TYPE, rid_, col);

};


Eigen::VectorXd Robot::getHorizonVelocityTowards(const Eigen::VectorXd& curr_pos, const Eigen::VectorXd& curr_vel, const Eigen::VectorXd& goal,
                                            double max_lin_speed, bool stop_at_waypoint, double smoothing_k){
    
    // Returns a horizon-state velocity that heads towards the goal under the unicycle model, capped at
    // max speed.
    Eigen::VectorXd horizon2goal = goal({0,1}) - curr_pos({0,1}); // Vector from horizon to goal

    // Angular velocity, capped by the turning radius.
    double angular_speed_desired = angleOp(atan2(horizon2goal(1), horizon2goal(0)) - curr_pos(2));
    double angular_speed_max = max_lin_speed/min_turning_radius_;
    double new_angular_speed = (angular_speed_desired<0) ? -1.*std::min(angular_speed_max, -angular_speed_desired) : std::min(angular_speed_max, angular_speed_desired);
    // Set linear velocity
    double lin_speed = (stop_at_waypoint) ? std::min(max_lin_speed, horizon2goal.norm()) : max_lin_speed;
    
    Eigen::VectorXd new_vel = cos(new_angular_speed) * lin_speed * Eigen::VectorXd{{cos(curr_pos(2)), sin(curr_pos(2))}};
    new_angular_speed *= (lin_speed/max_lin_speed);

    // Return velocity state
    Eigen::VectorXd new_horz_vel(curr_pos.size());

    if (smoothing_k>0){
        new_horz_vel({0,1}) = curr_vel({0,1}) + smoothing_k * (new_vel - curr_vel({0,1}));
        new_horz_vel(2) = curr_vel(2) + smoothing_k * (new_angular_speed - curr_vel(2));
        if (horizon2goal({0,1}).norm() < robot_radius_/4.) new_horz_vel.setZero();
        return new_horz_vel;
    }

    return Eigen::VectorXd{{new_vel(0), new_vel(1), new_angular_speed}};
};