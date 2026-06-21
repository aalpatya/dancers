/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once

#include <map>
#include <memory>
#include <algorithm>
#include <functional>
#include <deque>

#include <Globals.h>
#include <Utils.h>
#include <gbp/GBPCore.h>
#include <gbp/FactorGraph.h>   // FactorGraphGroup: the multi-robot GBP loop
#include <Graphics.h>
#include <gbp/Variable.h>

#include <raylib.h>
#include <rlights.h>
#include <nanoflann.h>
#include <KDTreeAdaptors.h>
#include <random>
#include <filesystem>

#define SUPPORT_SCREEN_CAPTURE
class Robot;
class Graphics;
class TreeOfRobots;

/************************************************************************************/
// The main Simulator.
/************************************************************************************/
class Simulator {
public:
    friend class Robot;
    friend class Factor;
    // Constructor
    Simulator();
    ~Simulator();

    // Pointer to Graphics class which hold all the camera, graphics and models for display
    Graphics* graphics;

    // kd-tree of robot positions, for fast neighbour lookups each timestep.
    typedef KDTreeMapOfVectorsAdaptor<std::map<int,Eigen::VectorXd>> KDTree;
    std::map<int, Eigen::VectorXd> robot_positions_{{0,Eigen::VectorXd{{0.,0.}}}};
    KDTree* treeOfRobots_;

    std::vector<int> stack_layers_{};
    int next_rid_ = 0;                              // rid for the next new robot; increment after use
    uint32_t clock_ = 0;                            // Simulation clock (timesteps)
    std::map<int, std::shared_ptr<Robot>> robots_;  // All robots, keyed by rid
    std::map<std::pair<int, int>, int> last_created_robots_{};  // <road, lane> -> rid, for new-robot collision avoidance in junction formations
    int last_junction_creation_clock_ = -1;          // clock_ when junction robots were last spawned (avoids re-spawn while paused)

    // The active scenario: a function that, given the simulator and current timestep, returns any new
    // robots to spawn this step ({} if none). Static problems return all robots when robots_ is empty;
    // dynamic problems spawn over time and can read/modify state via `sim`. See PathPlanningScenarios.cpp.
    using ScenarioFn = std::function<std::vector<std::shared_ptr<Robot>>(Simulator&, uint32_t)>;
    ScenarioFn scenario_fn_;

    // Cached decision-HUD swatch colours/labels (depend only on NUM_DECISIONS), filled lazily so draw()
    // doesn't recompute ColorFromHSV/to_string each frame.
    std::vector<Color> decisionColors_{};
    std::vector<std::string> decisionHudLabels_{};

    // Width (px) of the docked controls panel on the left. The 3D world is a SCREEN_SZ x SCREEN_SZ
    // square drawn to its RIGHT, so the window is (PANEL_W + SCREEN_SZ) wide by SCREEN_SZ tall.
    static constexpr int PANEL_W = 250;
    // The controls panel occupies the full-height strip on the left; eventHandler() uses these bounds
    // to swallow clicks so they don't fall through to robot picking / waypoint placement.
    Rectangle guiPanelRect() const { return Rectangle{0, 0, (float)PANEL_W, (float)globals.SCREEN_SZ}; }
    bool waypoint_adder_mode_ = false;   // left-clicks add a waypoint to all robots (crosshair cursor)
    bool left_dragging_ = false;         // a left-button drag is in progress (camera pan, not a click)
    Vector2 left_press_pos_{};           // screen pos where the current left-press started

    // --- Controls-panel widget state ----------------------------------------------------------------
    // Value-boxes for the live comms controls (raygui needs an int buffer + an "is editing" flag each).
    int  comms_box_value_ = 10;   bool comms_box_edit_ = false;   // comms radius
    int  fail_box_value_  = 0;    bool fail_box_edit_  = false;   // comms failure %
    bool iters_edit_ = false;                                     // GBP iters / timestep (edits globals.NUM_ITERS live)

    // Dropdown option lists, built once in the ctor (buildControlOptions()): config files + shape images
    // are scanned from the filesystem; scenarios/models are fixed. Labels are what the user sees.
    std::vector<std::string> cfg_paths_, shape_paths_;            // selectable values (config / shape-image paths)
    std::vector<std::string> cfg_opts_, shape_opts_;              // labels shown in those dropdowns
    // Scrollable-dropdown state: active index, whether open, and the list's scroll offset (one set each).
    int cfg_dd_=0;   bool cfg_dd_open_=false;   int cfg_scroll_=0;   int cfg_loaded_=0;   // cfg_loaded_ = config in use
    int model_dd_=0; bool model_dd_open_=false; int model_scroll_=0;
    int shape_dd_=0; bool shape_dd_open_=false; int shape_scroll_=0;
    bool dd_consumed_press_=false;   // a dropdown is consuming the current mouse press; lock the rest of the panel until release
    // Staged restart-param edits (mirror the curated set; committed together by "Apply & Reset").
    int  stage_num_robots_=0, stage_seed_=0, stage_world_sz_=0, stage_seed_decision_=0;
    bool num_edit_=false, seed_edit_=false, world_edit_=false, seed_decision_edit_=false;
    char t_horizon_buf_[16] = "";   bool t_horizon_edit_=false;   // T_HORIZON is a float -> edited as text
    void buildControlOptions();          // populate the dropdown lists + seed staged values from globals
    void requestRestart();               // gather staged edits + selected config into a restart request

    // Metrics structure
    int iterations_ = 0;
    double mean_num_neighbours_ = 0.;
    double mean_num_neighbours_time_ = 0.;
    std::vector<float> m_majority_num{}; // Time evolution of majority decisions robots
    int m_final_decision = -1;
    int m_consensus_reached = 0;
    int m_final_iterations = 0;
    double m_final_connectivity = 0.;
    double m_final_error_heading = 0.;
    double m_final_error_pos = 0.;

    // The multi-robot GBP loop runs on this group, referencing robots_ and stack_layers_. Message
    // routing and iteration live in FactorGraphGroup; the Simulator drives it once per timestep via
    // optimiseGBP().
    FactorGraphGroup<Robot> all_robots_{robots_, stack_layers_};
    void optimiseGBP(int num_iters);


    /*******************************************************************************/
    // Create new robots if needed. Handles deletion of robots out of bounds. 
    // New formations must modify the vectors "robots to create" and optionally "robots_to_delete"
    // by appending (push_back()) a shared pointer to a Robot class.
    /*******************************************************************************/    
    void createOrDeleteRobots();

    /*******************************************************************************/
    // Set a proportion of robots to not perform inter-robot communications
    /*******************************************************************************/
    void setCommsFailure(float failure_rate=globals.COMMS_FAILURE_RATE, bool reset=false);

    /*******************************************************************************/
    // Timestep loop of simulator.
    /*******************************************************************************/
    void timestep();

    /*******************************************************************************/
    // Drawing graphics.
    /*******************************************************************************/
    void draw();
    // GUI / 2D overlay drawing (defined in Graphics.cpp, alongside the raygui implementation).
    void drawControlsPanel();   // top-left controls panel (draw-flag toggles, comms radius, sim mode)
    void drawDecisionHud();     // bottom decision-distribution HUD (trigrid scenarios)

    /*******************************************************************************/
    // Use a kd-tree to perform a radius search for neighbours of a robot within comms. range
    // (Updates the neighbours_ of a robot)
    /*******************************************************************************/    
    void calculateRobotNeighbours(std::map<int,std::shared_ptr<Robot>>& robots);

    /*******************************************************************************/
    // Handles keypresses and mouse input, and updates camera.
    /*******************************************************************************/
    void eventHandler();

    /*******************************************************************************/
    // Deletes the robot from the simulator's robots_, as well as any variable/factors associated.
    /*******************************************************************************/
    void deleteRobot(std::shared_ptr<Robot> robot);

    /*******************************************************************************/
    // Function for handling logging of results
    /*******************************************************************************/
    void logResults();

    /***************************************************************************************************************/
    // RANDOM NUMBER GENERATOR.
    // Usage: randomNumber("normal", mean, sigma) or randomNumber("uniform", lower, upper)
    /***************************************************************************************************************/
    std::mt19937 gen_normal = std::mt19937(globals.RNG_SEED);
    std::mt19937 gen_uniform = std::mt19937(globals.RNG_SEED);
    std::mt19937 gen_uniform_int = std::mt19937(globals.RNG_SEED);
    template<typename T>
    T randomNumber(std::string distribution, T param1, T param2){
        if (distribution=="normal") return std::normal_distribution<T>(param1, param2)(gen_normal);
        if (distribution=="uniform") return std::uniform_real_distribution<T>(param1, param2)(gen_uniform);
        return (T)0;
    }
    Eigen::VectorXd randomNumberVec(std::string distribution, double param1, double param2, int dim){
        Eigen::VectorXd returnvec(dim); returnvec.setZero();
        for (int i=0; i<dim; i++){
            if (distribution=="normal"){
                returnvec(i) = std::normal_distribution<double>(param1, param2)(gen_normal);
            } else if (distribution=="uniform"){
                returnvec(i) = std::uniform_real_distribution<double>(param1, param2)(gen_uniform);
            }
        }
        return returnvec;
    }
    int randomInt(int lower, int upper){
        return std::uniform_int_distribution<int>(lower, upper)(gen_uniform_int);
    }

};

double getValueFromImgDist(int x, int y, DistanceField* p_obsImg);