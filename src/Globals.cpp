/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <Globals.h>
#include <Utils.h>
#include <gbp/FactorGraphLayer.h>   // ParamBag/LayerConfig + layerConfigs()
#include "yaml.hpp"   // fkYAML: header-only YAML reader
#include <algorithm>

// nlohmann-style value(key, default) for fkYAML: returns node[key] converted to T if the key is
// present (and node is a mapping), otherwise the default. Keeps every config key optional.
template <typename T>
static T yamlValue(fkyaml::node& node, const char* key, T def) {
    if (node.is_mapping() && node.contains(key)) {
        try { return node[key].template get_value<T>(); }
        catch (const fkyaml::exception&) { return def; }
    }
    return def;
}

// Returns the child mapping at `key`, or a null node if it is absent. Passing the
// result to yamlValue() then yields the defaults (a null node is not a mapping).
static fkyaml::node yamlChild(fkyaml::node& node, const char* key) {
    if (node.is_mapping() && node.contains(key)) return node[key];
    return fkyaml::node{};
}

// Resolve a path relative to the project root (baked in at compile time via PROJECT_ROOT).
// Absolute paths and empty strings are returned unchanged.
static std::string resolvePath(const std::string& path) {
    // is_absolute() handles both POSIX ("/...") and Windows ("C:\...") absolute paths.
    if (path.empty() || std::filesystem::path(path).is_absolute()) return path;
    return makePath({PROJECT_ROOT, path});
}

// Resolve a user-supplied config path. A relative path is interpreted relative to
// the current working directory first (standard CLI behaviour, so e.g. running
// `../config/x.json` from inside build/ works), and only falls back to the project
// root if no such file exists there (so the built-in default works from anywhere).
static std::string resolveConfigPath(const std::string& path) {
    if (path.empty() || std::filesystem::path(path).is_absolute()) return path;
    if (std::filesystem::exists(path)) return path;   // relative to the current working directory
    return makePath({PROJECT_ROOT, path});           // fall back to the project root
}

static std::string toLower(std::string s){
    for (auto& c : s) c = (char)std::tolower((unsigned char)c);
    return s;
}

// Flatten a YAML mapping (a layer's `params:` block) into a typed ParamBag the layer can read.
static ParamBag paramBagFromNode(fkyaml::node& node){
    ParamBag bag;
    if (!node.is_mapping()) return bag;
    for (auto it = node.begin(); it != node.end(); ++it){
        std::string key = it.key().get_value<std::string>();
        fkyaml::node& v = *it;
        if      (v.is_string())       bag.strs[key] = v.get_value<std::string>();
        else if (v.is_boolean())      bag.nums[key] = v.get_value<bool>() ? 1.0 : 0.0;
        else if (v.is_integer())      bag.nums[key] = (double)v.get_value<int64_t>();
        else if (v.is_float_number()) bag.nums[key] = v.get_value<double>();
    }
    return bag;
}

// Mirror just the robot kinematics into Globals (the rest of the app reads them); PlanningLayer reads
// its own sigmas straight from its param bag.
void Globals::applyPlanningParams(const ParamBag& p){
    ROBOT_RADIUS   = p.numf("ROBOT_RADIUS",   1.0f);
    MAX_SPEED      = p.numf("MAX_SPEED",       2.f);
    TURNING_RADIUS = p.numf("TURNING_RADIUS", 1.0f);
    T_HORIZON      = p.numf("T_HORIZON",       1.5f);
}
// Mirror just the app-wide consensus settings (space/decisions/seed robots) into Globals; ConsensusLayer
// reads its own sigmas from its param bag.
void Globals::applyConsensusParams(const ParamBag& p){
    // The Lie group (TYPE/SPACE) is owned by the ConsensusLayer (lieGroup_); the CONSENSUS_TYPE flag
    // mirrored here is just for the metrics/HUD.
    CONSENSUS_TYPE = (p.str("TYPE", "continuous")=="discrete") ? ConsensusType::Discrete
                                                              : ConsensusType::Continuous;
    NUM_DECISIONS         = p.numi("NUM_DECISIONS",          10);
    SEED_ROBOT_PROPORTION = p.numf("SEED_ROBOTS_PROPORTION", 0.f);
    SEED_DECISION   = p.numi("SEED_DECISION",   3);
    // Robot display size. A consensus-only config has no planning layer to set ROBOT_RADIUS, but robots
    // are still drawn and scaled by it, so this layer supplies it. Keep a planning layer's value if set
    // (>0); otherwise use this layer's param, defaulting to 1.
    ROBOT_RADIUS          = p.numf("ROBOT_RADIUS", (ROBOT_RADIUS > 0.f) ? ROBOT_RADIUS : 1.0f);
}

/*****************************************************************/
// Reads the appropriate sections from the config (YAML) file
/*****************************************************************/
void Globals::parseGlobalArgs(std::ifstream& config_file){

    fkyaml::node j = fkyaml::node::deserialize(config_file);

    // Every key below is optional: if absent from the config, the default given here is used. A user's
    // config need only contain the keys that define their problem; the tuning knobs fall back to defaults.

    // Resolved first: other paths below (OBSTACLE_FILE, FORMATION_IMG_FILE) are relative to ASSETS_DIR.
    ASSETS_DIR = resolvePath(yamlValue(j, "ASSETS_DIR", std::string("assets/")));

    // ---- Bookkeeping / IO -------------------------------------------------------------------------
    SIMULATION_NAME = yamlValue(j, "SIMULATION_NAME", std::string("test"));
    OUTPUT_FILE = resolvePath(yamlValue(j, "OUTPUT_FILE", std::string("results/output_results.json")));
    LOG_RESULTS = yamlValue(j, "LOG_RESULTS", true);

    // ---- World / robot ----------------------------------------------------------------------------
    WORLD_SZ = yamlValue(j, "WORLD_SZ", 100);
    TIMESTEP = yamlValue(j, "TIMESTEP", 0.1f);
    MAX_TIME = yamlValue(j, "MAX_TIME", -1);
    SIM_MODE = static_cast<SimMode>(yamlValue(j, "SIM_MODE", 0));
    ROBOT_MODEL = yamlValue(j, "ROBOT_MODEL", std::string("bus"));

    // ---- Display ----------------------------------------------------------------------------------
    DISPLAY = yamlValue(j, "DISPLAY", 1);
    SCREEN_SZ = yamlValue(j, "SCREEN_SZ", 1000);
    DRAW_INTERROBOT = yamlValue(j, "DRAW_INTERROBOT", 1);
    DRAW_PATH = yamlValue(j, "DRAW_PATH", 0);
    DRAW_TRAJ = yamlValue(j, "DRAW_TRAJ", 0);
    MAX_TRAJ_LEN = yamlValue(j, "MAX_TRAJ_LEN", -1);   // missing key -> unbounded
    DRAW_WAYPOINTS = yamlValue(j, "DRAW_WAYPOINTS", 0);

    // ---- Problem setup ----------------------------------------------------------------------------
    // FACTORGRAPH_LAYERS: the robot's GBP stack, as an ordered list. Each entry is
    //   { name: <label>, type: <layer type>, params: { ... } }
    // The order is the stack order, and each layer's id is its index. The `type` string is mapped to a
    // layer class by makeLayer() in src/Robot.cpp.
    fkyaml::node fg_layers = yamlChild(j, "FACTORGRAPH_LAYERS");
    layerConfigs().clear();
    if (fg_layers.is_sequence()){
        for (auto it = fg_layers.begin(); it != fg_layers.end(); ++it){
            fkyaml::node entry = *it;
            LayerConfig lc;
            lc.name = yamlValue(entry, "name", std::string(""));
            lc.type = toLower(yamlValue(entry, "type", lc.name));   // default type = name
            fkyaml::node params = yamlChild(entry, "params");
            lc.params = paramBagFromNode(params);
            layerConfigs().push_back(lc);
        }
    }

    // A few built-in layers' params are read across the whole app (robot kinematics, consensus
    // space/decisions/seed robots), so mirror those into Globals. Each layer reads its own sigmas from
    // its param bag, so a new layer needs none of this.
    for (auto& lc : layerConfigs()){
        if      (lc.type=="planning")  applyPlanningParams(lc.params);
        else if (lc.type=="consensus") applyConsensusParams(lc.params);
    }

    // -- shape formation parameters (overlay used by the planning layer) --
    fkyaml::node shape_formation = yamlChild(j, "SHAPE_FORMATION");   // overlay, not a layer: stays top-level
    FORMATION_DISPLAY_TYPE = yamlValue(shape_formation, "FORMATION_DISPLAY_TYPE", std::string("none"));
    std::string formation_img = yamlValue(shape_formation, "FORMATION_IMG_FILE", std::string(""));
    FORMATION_IMG_FILE = formation_img.empty() ? std::string("") : ASSETS_DIR + formation_img;
    TOWARDS_FORMATION = yamlValue(shape_formation, "ROBOTS_MOVE_TOWARDS_SHAPE", false);
    OCCUPANCY_WEIGHTING_DECAY = yamlValue(shape_formation, "OCCUPANCY_WEIGHTING_DECAY", true);

    // ---- Scenario ---------------------------------------------------------------------------------
    SCENARIO = yamlValue(j, "SCENARIO", std::string("random"));
    SCENARIO_FILE = resolveConfigPath(yamlValue(j, "SCENARIO_FILE", std::string("")));   // per-robot scenario for SCENARIO=="file"
    TIMESTEPS_BEFORE_NEW_ROBOTS_SPAWNED = std::max(1, yamlValue(j, "TIMESTEPS_BEFORE_NEW_ROBOTS_SPAWNED", 50));
    NUM_ROBOTS = yamlValue(j, "NUM_ROBOTS", 20);
    COMMUNICATION_RADIUS = yamlValue(j, "COMMUNICATION_RADIUS", 10.f);
    RNG_SEED = yamlValue(j, "RNG_SEED", 1);
    COMMS_FAILURE_RATE = yamlValue(j, "COMMS_FAILURE_RATE", 0.f);
    std::string obstacle_file = yamlValue(j, "OBSTACLE_FILE", std::string(""));
    OBSTACLE_FILE  = obstacle_file.empty() ? std::string("") : ASSETS_DIR + obstacle_file;

    // ---- GBP solver / misc (layer-specific sigmas live in the planning/consensus blocks above) -----
    NUM_ITERS = yamlValue(j, "NUM_ITERS", 2);
    DAMPING = yamlValue(j, "DAMPING", 0.f);
    SYMMETRIC_INTERROBOT_FACTORS = yamlValue(j, "SYMMETRIC_INTERROBOT_FACTORS", false);
    TEMP4 = yamlValue(j, "TEMP4", 0);

    OCC_WEIGHTING_HIGH_VAL = WORLD_SZ / (0.5 * MAX_SPEED * TIMESTEP);
    robot_length_ = 2.* ROBOT_RADIUS;

    return;
}

Globals::Globals(){};

/*****************************************************************/
// Apply "KEY=VALUE;KEY=VALUE" overrides on top of the parsed config. Only a curated set of numeric
// parameters is settable this way (the ones the web config form exposes); unknown keys are ignored.
/*****************************************************************/
void Globals::applyConfigOverrides(const std::string& spec){
    auto trim = [](std::string s){
        size_t a = s.find_first_not_of(" \t"); if (a==std::string::npos) return std::string();
        size_t b = s.find_last_not_of(" \t"); return s.substr(a, b-a+1);
    };
    size_t pos = 0;
    while (pos < spec.size()){
        size_t semi = spec.find(';', pos);
        std::string item = spec.substr(pos, semi==std::string::npos ? std::string::npos : semi-pos);
        pos = (semi==std::string::npos) ? spec.size() : semi+1;
        size_t eq = item.find('=');
        if (eq==std::string::npos) continue;
        std::string key = trim(item.substr(0, eq)), val = trim(item.substr(eq+1));
        if (key.empty()) continue;   // empty value is allowed (e.g. FORMATION_IMG_FILE= clears the shape)
        try {
            if      (key=="NUM_ROBOTS")           NUM_ROBOTS = std::stoi(val);
            else if (key=="RNG_SEED")             RNG_SEED = std::stoi(val);
            else if (key=="NUM_ITERS")            NUM_ITERS = std::stoi(val);
            else if (key=="MAX_TIME")             MAX_TIME = std::stoi(val);
            else if (key=="WORLD_SZ")             WORLD_SZ = std::stoi(val);
            else if (key=="SEED_DECISION")        SEED_DECISION = std::stoi(val);
            else if (key=="T_HORIZON")            T_HORIZON = std::stof(val);
            else if (key=="COMMUNICATION_RADIUS") COMMUNICATION_RADIUS = std::stof(val);
            else if (key=="DAMPING")              DAMPING = std::stof(val);
            else if (key=="SCENARIO")             SCENARIO = val;
            else if (key=="ROBOT_MODEL")          ROBOT_MODEL = val;
            else if (key=="FORMATION_IMG_FILE")   FORMATION_IMG_FILE = val;   // full path (or "" for none)
            else { print("Ignoring unknown config override: ", key); continue; }
        } catch (...) { print("Bad config override value: ", item); }
    }
}

/*****************************************************************/
// Allows for parsing of an external config file
/*****************************************************************/
int Globals::parseGlobalArgs(DArgs::DArgs &dargs)
{
    // Argument parser
    this->CONFIG_FILE = resolveConfigPath(dargs("-c", "--cfg", "config_file", this->CONFIG_FILE));
    // Optional per-parameter overrides applied on top of the config file: --set "KEY=VALUE;KEY=VALUE".
    // (The web build builds this string from its config form; also handy on the desktop CLI.)
    std::string overrides = dargs("--set", "config overrides KEY=VALUE;KEY=VALUE", std::string(""));

    std::ifstream my_config_file(CONFIG_FILE);
    assert(my_config_file && "Couldn't find the config file");
    parseGlobalArgs(my_config_file);
    applyConfigOverrides(overrides);

    postParsing();
    // ASCII ART FOR DANCERS
    print(R"(
     ____    _    _   _  ____     ____  ____
    |  _ \  / \  | \ | |/ ___|___|  _ \/ ___|
    | | | |/ _ \ |  \| | |   / _ \ |_) \___ \
    | |_| / ___ \| |\  | |__|  __/  _ < ___) |
    |____/_/   \_\_| \_|\____\___|_| \_\____/ 

    © Aalok Patwardhan 2026
    )");

    return 0;
};

/*****************************************************************/
// Any checks on the input configs should go here.
/*****************************************************************/
void Globals::postParsing()
{
    // Cap max speed, since it should be <= ROBOT_RADIUS/2.f / TIMESTEP:
    // In one timestep a robot should not move more than half of its radius
    // (since we plan for discrete timesteps)
    if (MAX_SPEED > ROBOT_RADIUS/2.f/TIMESTEP){
        MAX_SPEED = ROBOT_RADIUS/2.f/TIMESTEP;
        print("Capping MAX_SPEED parameter at ", MAX_SPEED);
    }
    T0 = ROBOT_RADIUS/2.f / MAX_SPEED; // Time between current state and next state of planned path

    // The formation overlay ("points"/"full") needs a shape image ("full" uses its pixels, "points" its
    // dimensions). Without one, disable the overlay so we don't try to load a missing image.
    if ((FORMATION_DISPLAY_TYPE=="points" || FORMATION_DISPLAY_TYPE=="full")
        && (FORMATION_IMG_FILE.empty() || !std::filesystem::exists(FORMATION_IMG_FILE))){
        print("Warning: FORMATION_DISPLAY_TYPE '", FORMATION_DISPLAY_TYPE, "' needs a formation image, but none found at '", FORMATION_IMG_FILE, "'. Disabling formation overlay.");
        FORMATION_DISPLAY_TYPE = "none";
    }

}
