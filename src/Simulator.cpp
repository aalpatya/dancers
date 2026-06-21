/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <iostream>
#include <gbp/GBPCore.h>
#include <ShapeFormation.h>
#include <Simulator.h>
#include <PathPlanningScenarios.h>
#include <Graphics.h>
#include <Robot.h>
#include <CustomFactorGraphLayers/Pathplanning.h>
#include <CustomFactorGraphLayers/Consensus.h>
#include <nanoflann.h>
#include <thread>
/*******************************************************************************/
// Raylib setup
/*******************************************************************************/
Simulator::Simulator(){
    // The window is created/owned by main() (so a config switch can rebuild the Simulator without
    // recreating the GL context). Here we just build the scene into the existing context.
    if (globals.DISPLAY) buildControlOptions();   // populate the controls-panel dropdown lists

    // Scenario
    scenario_fn_ = makeScenarioFn(globals.SCENARIO);

    // The robots' factor-graph stack: one layer per configured entry, id = its index.
    stack_layers_.clear();
    for (int id=0; id<(int)layerConfigs().size(); id++) stack_layers_.push_back(id);

    // Initialise kdtree for storing robot positions (needed for nearest neighbour check)
    treeOfRobots_ = new KDTree(2, robot_positions_, 50);  

    // Obstacle image: obstacles BLACK, background WHITE.
    graphics = new Graphics(globals.OBSTACLE_FILE.c_str());
};

/*******************************************************************************/
// Destructor
/*******************************************************************************/
Simulator::~Simulator(){
    delete treeOfRobots_;
    int n = robots_.size();
    for (int i = 0; i < n; ++i) robots_.erase(i);
    // Unload GL resources but leave the window open - main() owns the window (and closes it at exit).
    if (globals.DISPLAY) delete graphics;
};

/*******************************************************************************/
// Drawing graphics.
/*******************************************************************************/
void Simulator::draw(){
    if (!globals.DISPLAY) return;

    // 1) Render the 3D world into the off-screen square (SCREEN_SZ x SCREEN_SZ -> 1:1 aspect).
    BeginTextureMode(graphics->world_rt_);
        ClearBackground(WHITE);
        BeginMode3D(graphics->camera3d);
            DrawModel(graphics->groundModel_, graphics->groundModelpos_, 1.f, RAYWHITE);
            for (auto [rid, robot] : robots_) robot->draw(this, globals);
            // Coordinate frame
            DrawLine3D(Vector3{0.f, 1.f, 0.f}, Vector3{1.f, 1.f, 0.f}, RED);
            DrawLine3D(Vector3{0.f, 1.f, 0.f}, Vector3{0.f, 2.f, 0.f}, GREEN);
            DrawLine3D(Vector3{0.f, 1.f, 0.f}, Vector3{0.f, 1.f, 1.f}, BLUE);
        EndMode3D();
    EndTextureMode();

    // 2) Composite: blit the world to the right of the panel, then draw the 2D overlays on top.
    BeginDrawing();
        ClearBackground(RAYWHITE);
        // Blit the (HighDPI-resolution) world texture into the SCREEN_SZ logical square to the right of
        // the panel. Source height is negated because RenderTextures are stored bottom-up.
        float ts = (float)graphics->world_rt_.texture.width;
        DrawTexturePro(graphics->world_rt_.texture,
                       Rectangle{0, 0, ts, -ts},
                       Rectangle{(float)PANEL_W, 0, (float)globals.SCREEN_SZ, (float)globals.SCREEN_SZ},
                       Vector2{0, 0}, 0.f, WHITE);
        drawControlsPanel();   // full-height left controls panel (Graphics.cpp)
        drawDecisionHud();     // decision HUD over the world, trigrid scenarios (Graphics.cpp)
        drawInfo(clock_);     // Help overlay (only shown while SIM_MODE == Help)
    EndDrawing();
};

void Simulator::logResults(){
    if (!globals.LOG_RESULTS) return;
    if (globals.SIM_MODE!=SimMode::Timestep) return;

    /******************************************* */
    // Convergence on Consensus Variable
    const int consensus_id = layerIdOfType("consensus");
    // The consensus Lie group is owned by the consensus layer (same for every robot); read it off any
    // robot's layer.
    LieGroup lg;   // defaults to SE2
    if (consensus_id >= 0 && !robots_.empty())
        lg = static_cast<ConsensusLayer*>(robots_.begin()->second->stack_[consensus_id].get())->lieGroup_;

    Eigen::VectorXd diff_sum = Eigen::VectorXd::Zero(lg.dof());
    int cnt = 0;
    if (consensus_id >= 0){
        // Iterate the robots that exist (rids aren't contiguous 0..NUM_ROBOTS in dynamic scenarios,
        // where robots are removed and re-added with fresh ids).
        for (auto it = robots_.begin(); it != robots_.end(); ++it){
            for (auto jt = std::next(it); jt != robots_.end(); ++jt){
                // Euclidean groups (e.g. discrete R1) have no heading/position convergence metric.
                if (!lg.isEuclidean()){
                    diff_sum += (it->second->getVar(consensus_id, -1)->belief_.mu - jt->second->getVar(consensus_id, -1)->belief_.mu).cwiseAbs();
                }
                cnt++;
            }
        }
    }
    diff_sum /= (float)cnt;
    double error_heading = diff_sum(lg.dof()-1);
    double error_pos = (lg.dof()>1) ? diff_sum({0,1}).norm() : 0;
    double THRESH_heading = 0.01;
    double THRESH_pos = 0.1;

    /******************************************* */
    // Consensus on discrete decisions
    std::map<int, int> decisions{};
    for (auto [rid, robot] : robots_){;
        int d = robot->decision_;
        decisions[d] = decisions.count(d) ? decisions[d] + 1 : 1;
    }
    // Pick the convergence metric from the active layers / consensus settings: shape formation measures
    // formation fill; discrete consensus measures decision agreement (on the fixed seed decision when
    // seed robots are present); continuous consensus measures heading/pose error.
    const bool has_consensus = (consensus_id >= 0);
    const bool is_discrete   = (globals.CONSENSUS_TYPE == ConsensusType::Discrete);
    const bool has_seed      = (globals.SEED_ROBOT_PROPORTION > 0.f);
    const bool is_formation  = globals.TOWARDS_FORMATION;

    auto [max_d, num_majority] = getMax(decisions);
    if (is_discrete && has_seed) num_majority = decisions.count(globals.SEED_DECISION) ? decisions.at(globals.SEED_DECISION) : 0;
    m_majority_num.push_back(num_majority/(float)robots_.size());

    /******************************************* */
    // Shape Formation (sample the formation off any existing robot; they share the same target)
    double unoccupied = 0.;
    int fp_sz = 0;
    double filled_proportion = 1.;
    if (!robots_.empty() && robots_.begin()->second->formationPoints_){
        auto& fp = robots_.begin()->second->formationPoints_;
        for (auto& p : fp->points_){
            Eigen::VectorXd query_pt = fp->liePose_.act(Eigen::Vector2d(p({0,1})));
            const float search_radius = pow(globals.ROBOT_RADIUS*2,2.);
            std::vector<nanoflann::ResultItem<size_t, double>> matches;
            const size_t nMatches = treeOfRobots_->index->radiusSearch(&query_pt[0], search_radius, matches);
            if (nMatches==0) unoccupied+=1.0;
        }
        fp_sz = fp->points_.size();
        filled_proportion = (fp_sz==0) ? 1. : 1. - (unoccupied / (double)fp_sz);
    }

    /******************************************* */
    // Avg Connectivity
    mean_num_neighbours_time_ += mean_num_neighbours_;
    double avg_connectivity = mean_num_neighbours_time_/(double)(clock_);

    /******************************************* */
    // Consensus Check

    if (is_formation) {
        m_consensus_reached = (filled_proportion>=1.);
    } else if (!has_consensus) {
        m_consensus_reached = 0;
    } else if (is_discrete && has_seed) {
        m_consensus_reached = (num_majority==robots_.size() && max_d==globals.SEED_DECISION);
    } else if (is_discrete) {
        m_consensus_reached = (int)(num_majority==robots_.size());
    } else {   // continuous consensus
        m_consensus_reached = (int)((error_heading < THRESH_heading) && (error_pos < THRESH_pos));
    }
    /******************************************* */
    // Log printing
    if (clock_>0 && clock_%100==0){
        print("timesteps: ", clock_, " |  proportion majority: ", m_majority_num.back(),
        " | mean error_heading ", error_heading, " | mean error_pos ", error_pos, " | filled prop: ", filled_proportion);
    }

    /******************************************* */
    // Final log print if consensus
    if (clock_<globals.MAX_TIME && !m_consensus_reached) return;
    
    if (m_consensus_reached==1){
        globals.SIM_MODE = SimMode::Paused;
        m_final_decision = max_d;
        print("Consensus reached. \tFinal decision: [" + std::to_string(m_final_decision) + "]");
    } else {
        print("MAX TIME ELASPED! No Consensus reached. ");
    }
    m_final_iterations = iterations_;
    m_final_connectivity = avg_connectivity;
    m_final_error_heading = error_heading;
    m_final_error_pos = error_pos;
    print("\t\t\tFinal iterations: [" + std::to_string(m_final_iterations) + "]");
    print("\t\t\tMean connectivity: [" + std::to_string(m_final_connectivity) + " neighbours/robot]");
    print("\t\t\tMean formation parameter error_heading: [" + std::to_string(m_final_error_heading) + " rad].");
    print("\t\t\tMean formation parameter error_pos: [" + std::to_string(m_final_error_pos) + " m]");

    print("");

    if (!globals.DISPLAY){
        print("Test complete");
        print("***************************************");
        globals.RUN = false;
    }
}

/*******************************************************************************/
// Timestep loop of simulator.
/*******************************************************************************/
void Simulator::timestep(){
    if (globals.SIM_MODE!=SimMode::Timestep && globals.SIM_MODE!=SimMode::TimestepSingle) return;
    // If the communications failure rate is non-zero, activate/deactivate robot comms
    setCommsFailure(globals.COMMS_FAILURE_RATE);

    // Sense neighbours
    calculateRobotNeighbours(robots_);

    // Pre-GBP per-layer node updates across all robots (e.g. the consensus layer slides its window).
    all_robots_.preGBP();

    // Create/destroy inter-robot factors based on each robot's neighbours.
    for (auto [rid, robot] : robots_) robot->updateInterrobotFactors();

    // Run GBP: 2*num_iters internal iterations, num_iters inter-robot iterations.
    optimiseGBP(globals.NUM_ITERS);

    // Post-GBP read-back (planning moves the path, consensus reads its decision).
    all_robots_.postGBP();

    // Advance the clock, keeping the robot group's clock in sync (it drives the layers' time-based
    // scheduling in preGBP()).
    if (globals.SIM_MODE==SimMode::TimestepSingle) globals.SIM_MODE=SimMode::Paused;
    if (++clock_ >= globals.MAX_TIME ) globals.RUN = false;
    all_robots_.clock_ = clock_;

};


void Simulator::calculateRobotNeighbours(std::map<int,std::shared_ptr<Robot>>& robots){
    for (auto [rid, robot] : robots){
        robot_positions_.at(rid) = robot->position_;
        robot->neighbour_pos_.clear();
    }
    treeOfRobots_->index->buildIndex(); 
    mean_num_neighbours_ = 0.;
    for (auto [rid, robot] : robots){
        // Find nearest neighbors in radius
        robot->neighbours_.clear();
        Eigen::VectorXd query_pt = robots[rid]->position_;
        const float search_radius = pow(robots[rid]->COMMUNICATION_RADIUS, 2.);
        std::vector<nanoflann::ResultItem<size_t, double>> matches;
        nanoflann::SearchParameters params; params.sorted = true;
        const size_t nMatches = treeOfRobots_->index->radiusSearch(&query_pt[0], search_radius, matches, params);
        for(size_t i = 0; i < nMatches; i++){
            // matches[i].first indexes the KD-tree's dataset (robot_positions_), so resolve it there;
            // robots_ can differ in size/order and walk out of bounds.
            if (matches[i].first >= robot_positions_.size()) continue;
            auto it = robot_positions_.begin(); std::advance(it, matches[i].first);
            int other_rid = it->first;
            if (other_rid==rid) continue;
            auto rit = robots.find(other_rid);
            if (rit==robots.end() || !rit->second->interrobot_comms_active_) continue; // skip missing / muted robots
            robot->neighbours_.push_back(other_rid);
            robot->neighbour_pos_[other_rid] = it->second({0,1});
        }
        mean_num_neighbours_ += nMatches - 1.;
    }
    mean_num_neighbours_ /= (double)(robots.size());
};

/*******************************************************************************/
// Set a proportion of robots to not perform inter-robot communications
/*******************************************************************************/
void Simulator::setCommsFailure(float failure_rate, bool reset){
    if (failure_rate==0 && !reset) return;
    // Get all the robot ids and then shuffle them      
    std::vector<int> range{}; for (auto& [rid, robot] : robots_) range.push_back(rid);
    std::shuffle(range.begin(), range.end(), gen_uniform);
    // Set a proportion of the robots as inactive using their interrobot_comms_active_ flag.
    int num_inactive = round(failure_rate*robots_.size());
    for (int i=0; i<range.size(); i++){
        robots_.at(range[i])->interrobot_comms_active_ = (i>=num_inactive);
    }
}

/*******************************************************************************/
// Handles keypresses and mouse input, and updates camera.
/*******************************************************************************/
void Simulator::eventHandler(){
#ifndef __EMSCRIPTEN__
    // Close the app when the window's X (or the OS close request) is hit, same as ESC below. Guard on
    // DISPLAY: with no window (headless), WindowShouldClose() returns true and would end the run at once.
    if (globals.DISPLAY && WindowShouldClose()) globals.RUN = false;
#endif

    // Deal with Keyboard key press
    int key = GetKeyPressed();
    switch (key)
    {
    case KEY_ESCAPE:
            globals.RUN = false;                                                    break;
    case KEY_H:
            globals.LAST_SIM_MODE = (globals.SIM_MODE==SimMode::Help) ? globals.LAST_SIM_MODE : globals.SIM_MODE;
            globals.SIM_MODE = (globals.SIM_MODE==SimMode::Help) ? globals.LAST_SIM_MODE: SimMode::Help;break;
    case KEY_SPACE:
            graphics->camera_transition_ = !graphics->camera_transition_;           break;
    default:
        break;
    }

    // Mouse input handling. The world is drawn offset by PANEL_W into a SCREEN_SZ square, so unproject
    // the mouse relative to that square (subtract the panel width; use SCREEN_SZ as the viewport size).
    Vector2 mouse_screen = GetMousePosition();
    Ray ray = GetScreenToWorldRayEx(Vector2{mouse_screen.x - PANEL_W, mouse_screen.y},
                                    graphics->camera3d, globals.SCREEN_SZ, globals.SCREEN_SZ);
    Vector3 mouse_gnd = Vector3Add(ray.position, Vector3Scale(ray.direction, -ray.position.y/ray.direction.y));
    Vector2 mouse_pos{mouse_gnd.x, mouse_gnd.z};        // Position on the ground plane
    // Clicks over the GUI panel are swallowed so they don't fall through to the world (robot picking /
    // waypoint placement / obstacle painting).
    bool mouse_over_gui = CheckCollisionPointRec(GetMousePosition(), guiPanelRect());   // collapsed -> only the header swallows clicks
    // Crosshair cursor in waypoint-adder mode (arrow over the GUI so its toggle stays clickable).
    SetMouseCursor((waypoint_adder_mode_ && !mouse_over_gui) ? MOUSE_CURSOR_CROSSHAIR : MOUSE_CURSOR_DEFAULT);

    // Left button: dragging pans the camera; a plain click (press+release, no drag) does the mode action.
    if (!mouse_over_gui && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)){
        left_press_pos_ = GetMousePosition();
        left_dragging_ = false;
    }
    if (!mouse_over_gui && IsMouseButtonDown(MOUSE_BUTTON_LEFT)){
        if (Vector2Distance(GetMousePosition(), left_press_pos_) > 5.f) left_dragging_ = true;  // moved enough -> drag
        // Camera drag (rotate / shift-pan), disabled while placing waypoints.
        if (left_dragging_ && !waypoint_adder_mode_){
            if (IsKeyDown(KEY_LEFT_SHIFT)) graphics->pan(GetMouseDelta());    // shift+drag pans
            else                           graphics->orbit(GetMouseDelta());  // drag rotates
        }
    }
    if (!mouse_over_gui && IsMouseButtonReleased(MOUSE_BUTTON_LEFT)){
        if (waypoint_adder_mode_){
            // Waypoint-adder mode: drop a waypoint at the release position for every robot.
            for (auto& [rid, robot] : robots_){
                robot->waypoints_.push_back(Eigen::VectorXd{{mouse_pos.x, mouse_pos.y, 0., 0., 0., 0.}});
            }
        }
    }

    graphics->updateCamera(!mouse_over_gui);
}

/*******************************************************************************/
// Create new robots if needed. Handles deletion of robots out of bounds. 
// New formations must modify the vectors "robots to create" and optionally "robots_to_delete"
// by appending (push_back()) a shared pointer to a Robot class.
/*******************************************************************************/
void Simulator::createOrDeleteRobots(){

    // 1. Ask the active scenario for any new robots to spawn this timestep, and register them.
    for (auto& robot : scenario_fn_(*this, clock_)){
        robot_positions_[robot->rid_] = robot->position_;
        robots_[robot->rid_] = robot;
    }

    // 2. Remove any robot that has left the world bounds.
    std::vector<std::shared_ptr<Robot>> robots_to_delete{};
    for (auto& [rid, robot] : robots_){
        if (std::abs(robot->position_(0)) > globals.WORLD_SZ/2 || std::abs(robot->position_(1)) > globals.WORLD_SZ/2){
            robots_to_delete.push_back(robot);
        }
    }
    for (auto& robot : robots_to_delete){
        deleteRobot(robot);
    }

    // Reaching/popping the front waypoint is handled in PlanningLayer::postGBPUpdateNodes.

};

/*******************************************************************************/
// Deletes the robot from the simulator's robots_, as well as any variable/factors associated.
/*******************************************************************************/
void Simulator::deleteRobot(std::shared_ptr<Robot> robot){
    robots_.erase(robot->rid_);
    robot_positions_.erase(robot->rid_);
}

void Simulator::optimiseGBP(int num_iters){
    all_robots_.optimiseGBP(num_iters);   // exchange external messages + internal factor/variable sweeps
    iterations_ += num_iters;
}