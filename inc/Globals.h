/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once
#include <cmath>
#include <raylib.h>
#include <DArgs.h>
#include <fstream>
#include <filesystem>
#include <string>
#include <vector>
#include <stdexcept>
#include "json.hpp"

struct ParamBag;   // defined in gbp/FactorGraphLayer.h

// Simulation modes
enum class SimMode {Paused, Timestep, Help, TimestepSingle};

// How the Consensus layer represents its state: a continuous Lie-group value, or one of
// NUM_DECISIONS discrete options.
enum class ConsensusType {Continuous, Discrete};

/***********************************************************************************************/
// Global config. Most values come from the YAML config file (default in CONFIG_FILE below).
/***********************************************************************************************/
class Globals {
    public:
    nlohmann::json jsonfile_out;                              // Log file
    std::string SIMULATION_NAME;                              // Name of this simulation (result-file key + startup banner)
    std::string OUTPUT_FILE;                                  // Test name
    bool LOG_RESULTS;                                         // If true, Simulator::logResults() runs (convergence metrics + result logging)
    std::string RUN_DIR = std::filesystem::current_path().string();   // .string() needed on Windows (native path is wstring)

    // Basic parameters
    const char* WINDOW_TITLE = "DANCeRS: A Distributed Algorithm for Negotiating Consensus in Robot Swarms";
    bool RUN = true;
    std::string CONFIG_FILE = "config/joint_consensus_shape_formation.yaml";   // Default config file (relative to project root)
    std::string OBSTACLE_FILE;                                // Binary image for obstacles
    std::string FORMATION_IMG_FILE;                           // Path to image file for shape formation
    std::string ASSETS_DIR;                                   // Directory for Assets
    SimMode SIM_MODE = SimMode::Paused;                       // Simulation mode to begin with (0: Paused, 1: Running)
    SimMode LAST_SIM_MODE = SimMode::Timestep;                // Storage of Simulation mode (if time is paused eg.)
    bool SYMMETRIC_INTERROBOT_FACTORS = false;

    // ---- Problem setup (see config) ----
    // The layer stack lives in layerConfigs(), filled from the config's FACTORGRAPH_LAYERS list by
    // parseGlobalArgs. These helpers copy a built-in layer's params into the Globals fields those
    // layers read; custom layers read cfg_ directly.
    void applyPlanningParams(const ParamBag& p);
    void applyConsensusParams(const ParamBag& p);
    ConsensusType CONSENSUS_TYPE = ConsensusType::Continuous; // Consensus layer state: continuous (space R^N|SE2|SO2) or discrete (num_decisions)
    
    // Display parameters
    bool DISPLAY;                                             // 0: no graphical display, 1: graphical display
    int WORLD_SZ;                                             // [m] world size (square)
    int SCREEN_SZ;                                            // Size of display window (pixels)
    bool DRAW_INTERROBOT;                                     // Draw inter-robot factors (communication links)
    bool DRAW_PATH;                                           // Draw robot planned paths
    bool DRAW_TRAJ;                                           // Draw robot executed trajectories
    int MAX_TRAJ_LEN = -1;                                    // Maximum length of executed trajectory to draw (-1 = no limit)
    bool DRAW_WAYPOINTS;                                      // Draw robot waypoints
    float robot_length_;

    // Simulation parameters
    int RNG_SEED;                                             // Random Number Generator seed
    float TIMESTEP;                                           // [s] Seconds per simulation timestep
    int MAX_TIME;                                             // Max timesteps for simulation (-1 = no limit)
    int NUM_ROBOTS;                                           // Number of robots (if no new robots are to be added)
    float T_HORIZON;                                          // [s] Time horizon for planning (how far into the future to plan)
    float ROBOT_RADIUS;                                       // [m] Robot radius. For 'bus' models, this is the width and half the length of the bus.
    float COMMUNICATION_RADIUS;                               // [m] Robot comms radius (inter-robot factors created within this range)
    float MAX_SPEED;                                          // [m/s] Maximum speed of robot
    float TURNING_RADIUS;                                     // [m] Turning radius of robot with unicycle model
    float COMMS_FAILURE_RATE;                                 // Proportion of robots [0,1] that do not communicate
    float SEED_ROBOT_PROPORTION;                              // Proportion [0,1] of robots fixed as "seed/informed" robots
    int SEED_DECISION = 3;                              // The decision seed robots hold fixed (discrete consensus)
    int LOOKAHEAD_MULTIPLE = 3;                               // Parameter affecting how planned path is spaced out in time
    std::string SCENARIO;                                     // random | circle | junction | junction_twoway | trigrid | file (with SCENARIO_FILE)
    std::string SCENARIO_FILE;                                // Path to json file containing waypoints and/or consensus decisions (if SCENARIO=file)
    int TIMESTEPS_BEFORE_NEW_ROBOTS_SPAWNED = 50;                                    // Ticks between robot spawns in dynamic scenarios (junction)
    float T0;                                                 // Time between current state and next state of planned path
    double OCC_WEIGHTING_HIGH_VAL = 1e4;
    std::string ROBOT_MODEL = "bus";                          // bus | sphere | robot
    std::string FORMATION_DISPLAY_TYPE = "none";              // none | points (draw shape formation points) | full (draw shape formation overlay)
    bool obstacles_present_ = true;                           // Whether obstacles are present (used to skip obstacle factors if not)

    // GBP parameters. Per-layer factor sigmas live in each layer (see PlanningLayer / ConsensusLayer);
    // only app-wide values are here.
    float SIGMA_POSE_FIXED = 1e-8;                            // Sigma for Unary pose factor on current and horizon states
    int NUM_ITERS;                                            // Number of external GBP iterations per timestep
    float DAMPING = 0.;                                       // Damping amount (not used in this work)
    bool spawn_special_robot_ = false;                        // One-shot latch: arms a special (fast, black) robot in the junction scenarios
    bool TOWARDS_FORMATION = false;                           // true: robots move towards the shape; false: towards their waypoints

    bool OCCUPANCY_WEIGHTING_DECAY;                           // If true, the occupancy weighting of formation points decays over time.
    double TEMP4;                                            // debug toggle: tints continuous-consensus robots green (Consensus::draw)
    double NUM_DECISIONS;                                     // Number of discrete consensus decisions
    Globals();
    int parseGlobalArgs(DArgs::DArgs& dargs);
    void parseGlobalArgs(std::ifstream& config_file);
    // Apply CLI/URL parameter overrides of the form "KEY=VALUE;KEY=VALUE" on top of the parsed config
    // (used by the --set argument; the web build builds this string from its config form).
    void applyConfigOverrides(const std::string& spec);
    void postParsing();

    // ---- Runtime restart request -----------------------------------------------------------------
    // The controls panel sets these to ask for a rebuild with a different config / staged parameter
    // edits. The main loop consumes the request: desktop rebuilds the Simulator in-process; the web
    // build reloads the page with the new config. (Live params don't use this - they apply instantly.)
    bool restart_requested_ = false;
    std::string pending_config_;      // project-root-relative config path to switch to ("" = keep current)
    std::string pending_overrides_;   // staged "KEY=VALUE;..." restart-param edits to apply after parsing

};

// The application's single global config instance (defined in main.cpp). The gbp library has its own
// gbp::config() and doesn't touch this; the simulator/app code reads it widely.
extern Globals globals;