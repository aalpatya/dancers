/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once
#include <Globals.h>
#include "Simulator.h"
#include <memory>
#include <vector>
#include <deque>
#include <Utils.h>
#include <gbp/FactorGraphLayer.h>
#include <gbp/GBPCore.h>
#include <gbp/Factor.h>
#include <gbp/FactorGraph.h>

class ShapeFormation;   // Robot holds a ShapeFormation* (full def in ShapeFormation.h, included by Robot.cpp)

extern Globals globals;

/***************************************************************************/
// A Robot IS a FactorGraph (a stack of factor-graph layers) plus physical state (waypoints,
// position, neighbours, ...) and bookkeeping the simulator drives. It builds its own layer stack
// from FACTORGRAPH_LAYERS.
//   rid             : robot id (from Simulator::next_rid_++)
//   waypoints       : deque of 6D states [x, y, theta, xdot, ydot, ...] the robot travels through
//   size / colour   : robot radius and display colour
//   p_obstacleImgDist: distance-transform obstacle image the planning layer reads
/***************************************************************************/
class Robot : public FactorGraph {
public:
    Robot(int rid,
          std::deque<Eigen::VectorXd> waypoints,
          Color color,
          DistanceField* p_obstacleImgDist,
          int decision=-1, bool seed_robot=false,
          bool is_special=false,
          double max_speed=globals.MAX_SPEED,
          float size=globals.ROBOT_RADIUS,
          double turning_radius=globals.TURNING_RADIUS,
          float communication_radius=globals.COMMUNICATION_RADIUS);
    ~Robot();

    int rid_ = 0;                               // Robot id
    std::deque<Eigen::VectorXd> waypoints_{};   // Dequeue of waypoints (whenever the robot reaches a point, it is popped off the front of the dequeue)
    float robot_radius_ = 1.;                   // Robot radius
    Color color_ = DARKGREEN;                   // Colour of robot
    Color traj_col_ = GRAY;                   // Colour of the robot's trajectory
    double max_speed_ = globals.MAX_SPEED;
    double min_turning_radius_ = globals.TURNING_RADIUS;

    std::vector<int> connected_r_ids_{};        // List of robot ids that are currently connected via inter-robot factors to this robot
    std::vector<int> neighbours_{};             // List of robot ids that are within comms radius of this robot
    std::map<int, Eigen::VectorXd> neighbour_pos_{};             // Positions of neighbours, keyed by robot id
    float height_3D_ = 0.f;                     // Height out of plane (for 3d visualisation only)
    Eigen::VectorXd position_;                  // Position of the robot (equivalent to taking the [x,y] of the current state of the robot)
    double bearing_ = 0.;
    bool is_special_ = false;
    bool seed_robot_ = false;
    int decision_ = -1;
    std::vector<std::shared_ptr<Variable>> unconnected_interrobot_variables_{};
    ShapeFormation* formationPoints_;
    std::vector<Eigen::Vector2d> trajectory_{};

    // Cached construction inputs the layers read when building their nodes.
    DistanceField* p_obstacleImgDist_ = nullptr;   // Distance-transform field of the obstacles
    Eigen::VectorXd start_;                      // Robot's start state
    Eigen::VectorXd horizon_;                    // Robot's initial horizon state


    float COMMUNICATION_RADIUS;                 // [m] per-robot comms radius (from ctor)

    // Config-wide values cached from globals (identical for every robot), read by the layers. Per-robot
    // size/speed/turning radius live in robot_radius_/max_speed_/min_turning_radius_; the consensus Lie
    // group lives on the ConsensusLayer (lieGroup_).
    float TIMESTEP;
    float T_HORIZON;
    float T0;
    int LOOKAHEAD_MULTIPLE;
    float SIGMA_POSE_FIXED;
    int NUM_DECISIONS;
    /****************************************/
    //Functions
    /****************************************/
    // preGBP/postGBP and inter-robot factor build/teardown live on the FactorGraph base.
    Eigen::VectorXd getHorizonVelocityTowards(const Eigen::VectorXd& curr_pos, const Eigen::VectorXd& curr_vel, const Eigen::VectorXd& goal,
                                            double max_lin_speed, bool stop_at_waypoint=true, double smoothing_k=0.);

    /***************************************************************************************************/
    // Create inter-robot factors for new neighbours, delete them for ones that have moved away.
    // Calls the base stack's makeInterrobotFactors/deleteInterrobotFactors; the neighbour bookkeeping
    // is robot-specific.
    /***************************************************************************************************/
    void updateInterrobotFactors();


    /***************************************************************************************************/
    // Drawing function
    /***************************************************************************************************/
    void draw(Simulator* sim, Globals& globals);

};

