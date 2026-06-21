/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
// PATH-PLANNING SCENARIOS
//
// A scenario decides which robots exist and where they want to go. Each one is a function
// (Simulator&, timestep) -> the NEW robots to spawn this step (see PathPlanningScenarios.h for
// the full contract). This file is laid out in three parts:
//
//   1. Helpers      - build robots from waypoint lists / JSON files / python generators.
//   2. Scenarios    - the actual scenarios (circle, trigrid, file, random, junction, default).
//   3. Factory      - makeScenarioFn(), which maps a name to its scenario. Register yours there.
/**************************************************************************************/
#include <PathPlanningScenarios.h>
#include <Simulator.h>
#include <Robot.h>
#include <Globals.h>
#include <Utils.h>

#include <cmath>
#include <deque>
#include <map>
#include <set>
#include <random>
#include <fstream>
#include <filesystem>
#include <numeric>
#include <algorithm>

extern Globals globals;

/**************************************************************************************/
// 1. HELPERS - shared machinery the scenarios below build on.
/**************************************************************************************/

// One robot's entry in a scenario file.
struct RobotScenario {
    std::vector<Eigen::Vector2d> xy;   // waypoint positions
    int decision = 0;
    bool seed_robot = false;
    bool has_hue = false;              // if false, the robot is coloured by its index
    double hue = 0.;
};

// Build static robots from per-robot scenario entries. Each waypoint's heading
// points toward the next one (the final waypoint keeps the previous heading).
// A robot is coloured by its explicit hue if given, else evenly around the hue wheel.
static std::vector<std::shared_ptr<Robot>> makeStaticRobots(Simulator& sim,
        const std::vector<RobotScenario>& robots){
    std::vector<std::shared_ptr<Robot>> out;
    int n = (int)robots.size();
    for (int i=0; i<n; i++){
        const auto& r = robots[i];
        if (r.xy.empty()) continue;
        std::deque<Eigen::VectorXd> waypoints;
        for (size_t k=0; k<r.xy.size(); k++){
            Eigen::Vector2d dir = (k+1<r.xy.size()) ? (r.xy[k+1]-r.xy[k])
                                 : (r.xy.size()>1 ? (r.xy[k]-r.xy[k-1]) : Eigen::Vector2d{1., 0.});
            double angle = atan2(dir.y(), dir.x());
            waypoints.push_back(Eigen::VectorXd{{r.xy[k].x(), r.xy[k].y(), angle, 0., 0., 0.}});
        }
        Color color = r.has_hue ? ColorFromHSV(r.hue, 1., 0.75) : ColorFromHSV(i*360./(float)n, 1., 0.75);
        out.push_back(std::make_shared<Robot>(sim.next_rid_++, waypoints, color, &sim.graphics->obstacleDist_, r.decision, r.seed_robot));
    }
    return out;
}

// Load a scenario JSON and build the robots. Schema (per-robot data is keyed by
// the robot's index, as a string, in parallel maps):
//   {
//     "num_robots": 3,                                          // optional; caps how many robots spawn
//     "waypoints":   { "0": [[x0,y0], [x1,y1], ...], ... },     // required
//     "decisions":   { "0": 0, ... },                           // optional, default 0
//     "seed_robots": ["0", "2", ...],                           // optional; ids of the seed robots
//     "hues":        { "0": 123.0, ... }                        // optional; if absent, coloured by index
//   }
// Only "waypoints" is required; "decisions"/"hues" may be partial (missing keys
// take defaults), and "seed_robots" lists only the seed robots' ids.
static std::vector<std::shared_ptr<Robot>> loadScenarioFile(Simulator& sim, const std::string& path){
    std::vector<std::shared_ptr<Robot>> out;
    std::ifstream f(path);
    if (!f.is_open()){
        print("Could not open scenario file: ", path);
        globals.NUM_ROBOTS = 0;   // no robots, but don't leave loops indexing missing ones
        return out;
    }
    nlohmann::json j; f >> j;
    if (!j.contains("waypoints")){
        print("Scenario json has no 'waypoints' map: ", path);
        globals.NUM_ROBOTS = 0;
        return out;
    }
    // Build robots keyed by integer index so they end up sorted, regardless of the
    // order the JSON map happens to list them in.
    std::map<int, RobotScenario> byIndex;
    for (auto& el : j["waypoints"].items()){
        int idx = std::stoi(el.key());
        RobotScenario r;
        for (auto& wp : el.value()) r.xy.push_back(Eigen::Vector2d{(double)wp[0], (double)wp[1]});
        byIndex[idx] = r;
    }
    if (j.contains("decisions"))
        for (auto& el : j["decisions"].items())
            if (auto it = byIndex.find(std::stoi(el.key())); it != byIndex.end()) it->second.decision = el.value();
    if (j.contains("seed_robots"))
        for (auto& id : j["seed_robots"]){
            int idx = id.is_number() ? (int)id : std::stoi(id.get<std::string>());
            if (auto it = byIndex.find(idx); it != byIndex.end()) it->second.seed_robot = true;
        }
    if (j.contains("hues"))
        for (auto& el : j["hues"].items())
            if (auto it = byIndex.find(std::stoi(el.key())); it != byIndex.end()){ it->second.has_hue = true; it->second.hue = el.value(); }

    std::vector<RobotScenario> robots;
    for (auto& [idx, r] : byIndex) robots.push_back(r);   // std::map iterates in ascending key order
    // Optional num_robots caps how many of the listed robots are spawned.
    if (j.contains("num_robots")){
        int n = j["num_robots"];
        if (n >= 0 && n < (int)robots.size()) robots.resize(n);
    }
    globals.NUM_ROBOTS = (int)robots.size();   // keep the rest of the sim (metrics, loops) consistent
    return makeStaticRobots(sim, robots);
}

// Geometric scenario generators (circle, trigrid). Each builds a RobotScenario list and hands it to
// makeStaticRobots - the geometry lives here in C++ rather than in external python scripts.

// Robots evenly spaced on a circle, each travelling to the antipodal point.
static std::vector<std::shared_ptr<Robot>> makeCircleRobots(Simulator& sim){
    int n = globals.NUM_ROBOTS;
    double min_spacing = 5.0 * globals.ROBOT_RADIUS, min_radius = 0.25 * globals.WORLD_SZ;
    std::vector<RobotScenario> robots;
    for (int i = 0; i < n; i++){
        double radius = (n==1) ? min_radius
                     : std::max(min_radius, std::sqrt(min_spacing / (2.0 - 2.0*std::cos(2.0*M_PI/n))));
        double a = 2.0*M_PI*i/n;
        RobotScenario r;
        r.xy.push_back(Eigen::Vector2d{radius*std::cos(a),        radius*std::sin(a)});
        r.xy.push_back(Eigen::Vector2d{radius*std::cos(a+M_PI),   radius*std::sin(a+M_PI)});
        robots.push_back(r);
    }
    globals.NUM_ROBOTS = (int)robots.size();
    return makeStaticRobots(sim, robots);
}

// Grow a connected blob of `n` nodes on an equilateral-triangle grid, map to Cartesian, and give each
// robot a random decision + seed flag + decision-derived hue.
static std::vector<std::shared_ptr<Robot>> makeTrigridRobots(Simulator& sim){
    using P = std::pair<int,int>;
    auto neighbours = [](P p){ return std::vector<P>{
        {p.first+1,p.second}, {p.first-1,p.second}, {p.first,p.second+1},
        {p.first,p.second-1}, {p.first+1,p.second-1}, {p.first-1,p.second+1} }; };
    auto has = [](const std::vector<P>& v, P x){ return std::find(v.begin(), v.end(), x) != v.end(); };

    int size = globals.NUM_ROBOTS;
    std::mt19937 rng((unsigned)globals.RNG_SEED);
    std::vector<P> nodes{{0,0}}, pool;
    for (auto& nb : neighbours({0,0})) pool.push_back(nb);
    for (int i = 0; i < size-1 && !pool.empty(); i++){
        size_t r = std::uniform_int_distribution<size_t>(0, pool.size()-1)(rng);
        P pn = pool[r]; pool.erase(pool.begin() + r);
        nodes.push_back(pn);
        for (auto& nb : neighbours(pn)) if (!has(nodes,nb) && !has(pool,nb)) pool.push_back(nb);
    }

    int n = (int)nodes.size(), num_decisions = std::max(1, (int)globals.NUM_DECISIONS);
    double L = 5.0 * globals.ROBOT_RADIUS;
    int num_seed = (int)std::round(std::min((double)globals.SEED_ROBOT_PROPORTION, 1.0) * n);
    std::vector<int> order(n); std::iota(order.begin(), order.end(), 0);
    std::shuffle(order.begin(), order.end(), rng);
    std::set<int> seed_set(order.begin(), order.begin() + num_seed);

    std::vector<RobotScenario> robots;
    for (int i = 0; i < n; i++){
        double gx = nodes[i].first * L, gy = nodes[i].second * L;
        RobotScenario r;
        r.xy.push_back(Eigen::Vector2d{gx + gy*std::sin(M_PI/6), gy*std::cos(M_PI/6)});   // skewed -> Cartesian
        r.decision = std::uniform_int_distribution<int>(0, num_decisions-1)(rng);
        r.seed_robot = seed_set.count(i) > 0;
        r.has_hue = true; r.hue = r.decision * 180.0 / num_decisions;
        robots.push_back(r);
    }
    globals.NUM_ROBOTS = n;
    return makeStaticRobots(sim, robots);
}

/**************************************************************************************/
// 2. SCENARIOS
// STATIC scenarios place all robots on the first call and return {} afterwards; DYNAMIC
// ones keep spawning / topping up waypoints over time.
/**************************************************************************************/

/**************************************************************************************/
// STATIC: robots start on a circle and travel to the diametrically opposite side.
/**************************************************************************************/
static std::vector<std::shared_ptr<Robot>> circleScenario(Simulator& sim, uint32_t /*t*/){
    if (!sim.robots_.empty()) return {};   // static: spawn once
    return makeCircleRobots(sim);
}

/**************************************************************************************/
// STATIC: triangular grid; each robot also gets a decision and some are seed robots (used by the
// consensus layer).
/**************************************************************************************/
static std::vector<std::shared_ptr<Robot>> trigridScenario(Simulator& sim, uint32_t /*t*/){
    if (!sim.robots_.empty()) return {};   // static: spawn once
    return makeTrigridRobots(sim);
}

/**************************************************************************************/
// STATIC (file-driven): load robots from globals.SCENARIO_FILE
// (config "SCENARIO_FILE" or --scenario). See loadScenarioFile() for the schema.
/**************************************************************************************/
static std::vector<std::shared_ptr<Robot>> fileScenario(Simulator& sim, uint32_t /*t*/){
    if (!sim.robots_.empty()) return {};   // static: spawn once
    if (globals.SCENARIO_FILE.empty()){
        print("SCENARIO=file but SCENARIO_FILE is empty - set it in the config or pass --scenario");
        return {};
    }
    return loadScenarioFile(sim, globals.SCENARIO_FILE);
}

// Sample a random free-space position, keeping clear of obstacles (via the distance field, which is
// empty when there are no obstacles - then nothing is rejected).
static Eigen::VectorXd sampleFreePosition(Simulator& sim, double extent){
    DistanceField& df = sim.graphics->obstacleDist_;
    Eigen::VectorXd pos;
    do { pos = sim.randomNumberVec("uniform", -1., 1., 2) * extent * globals.WORLD_SZ; }
    while (df.cols() > 0 &&
        getValueFromImgDist((pos(0) + globals.WORLD_SZ/2) * (df.cols()/globals.WORLD_SZ),
                            (pos(1) + globals.WORLD_SZ/2) * (df.rows()/globals.WORLD_SZ), &df) < 0.5);
    return pos;
}

/**************************************************************************************/
// DYNAMIC: robots wander forever, getting a fresh random goal each time they arrive.
/**************************************************************************************/
static std::vector<std::shared_ptr<Robot>> randomScenario(Simulator& sim, uint32_t /*t*/){
    std::vector<std::shared_ptr<Robot>> out;
    const int num_init_wp = 3;

    if (sim.robots_.empty()){
        std::vector<Eigen::Vector2d> positions{};
        for (int i=0; i<globals.NUM_ROBOTS; i++){
            std::deque<Eigen::VectorXd> waypoints{};
            for (int j=0; j<num_init_wp; j++){
                Eigen::VectorXd rand_pos;
                // De-conflict the START so robots don't spawn on top of each other. The start is the LAST
                // point generated (push_front makes it waypoints_[0]); `positions` holds the other robots'
                // starts.
                do {
                    rand_pos = sim.randomNumberVec("uniform", -1., 1., 2) * 0.5 * globals.WORLD_SZ;
                } while (j==num_init_wp-1 && [&](){ bool coll = false;
                                for (auto& p : positions){
                                    if ((rand_pos - p).norm() < 4*globals.ROBOT_RADIUS){ coll = true; break; }
                                }
                                return coll; }()
                );
                double angle = (j==0) ? sim.randomNumber("uniform", 0., 2.*PI)
                                      : atan2(waypoints.front()(1) - rand_pos(1), waypoints.front()(0) - rand_pos(0));
                waypoints.push_front(Eigen::VectorXd{{rand_pos(0), rand_pos(1), angle, 0., 0., 0.}});
            }
            positions.push_back(waypoints.front()({0,1}));   // the robot's start (de-conflicted above)
            Color robot_color = ColorFromHSV(i*360./(float)globals.NUM_ROBOTS, 1., 0.75);
            int decision_idx = sim.randomInt(0, (int)globals.NUM_DECISIONS - 1);
            out.push_back(std::make_shared<Robot>(sim.next_rid_++, waypoints, robot_color, &sim.graphics->obstacleDist_, decision_idx));
        }
        return out;
    }

    // Keep each robot supplied with goals so it keeps wandering. The simulator pops
    // the front waypoint once reached; here we top the queue back up.
    const int planning_id = layerIdOfType("planning");
    for (auto& [rid, robot] : sim.robots_){
        if (planning_id < 0 || !robot->hasLayer(planning_id)) continue;
        while ((int)robot->waypoints_.size() < num_init_wp){
            Eigen::VectorXd horizon = robot->getVar(planning_id, -1)->belief_.mu.coeffs()({0,1});
            // don't spawn new point too close, otherwise the robot spins around
            Eigen::VectorXd new_pos = sampleFreePosition(sim, 0.45);
            double new_heading = atan2((new_pos - horizon)(1), (new_pos - horizon)(0));
            robot->waypoints_.push_back(Eigen::VectorXd{{new_pos(0), new_pos(1), new_heading, 0., 0., 0.}});
        }
    }
    return out;   // empty: no brand-new robots, only waypoint top-ups
}

/**************************************************************************************/
// DYNAMIC: cross-roads junction. One-way traffic per road; robots spawn periodically.
// Handles both "junction" (2 roads, 2 lanes, straight only) and "junction_twoway"
// (4 roads, 3 lanes, can turn). Out-of-bounds robots are culled generically by the
// simulator, so this only handles spawning.
/**************************************************************************************/
static std::vector<std::shared_ptr<Robot>> junctionScenario(Simulator& sim, uint32_t t){
    std::vector<std::shared_ptr<Robot>> out;
    bool twoway = (globals.SCENARIO=="junction_twoway");

    int n_roads = twoway ? 4 : 2;
    int n_lanes = twoway ? 3 : 2;
    double lane_width = 4.*globals.ROBOT_RADIUS /3. * 2.;
    // Spawn once every TIMESTEPS_BEFORE_NEW_ROBOTS_SPAWNED clock ticks, not every paused frame.
    if (t%globals.TIMESTEPS_BEFORE_NEW_ROBOTS_SPAWNED!=0 || (int)t == sim.last_junction_creation_clock_) return out;
    sim.last_junction_creation_clock_ = (int)t;

    // Draw RNG ONLY here, on actual spawn ticks (after the guard) - never every frame. Otherwise the
    // number of draws depends on how many (paused / variable-FPS) frames have rendered, so robot
    // generation would diverge run-to-run even with the same RNG_SEED.
    int trigger_road = sim.randomInt(0, n_roads-1);
    // Chance per spawn tick of designating a special (fast, black) robot. Tune the 5 to taste.
    if (sim.randomInt(0,5)==0) globals.spawn_special_robot_ = true;

    Eigen::VectorXd starting, turning, ending;
    Eigen::VectorXd start_pos, turn_pos, end_pos;
    for (int r = 0; r < n_roads; r++){
        bool collision_with_existing = false;
        int road, lane, turn;
        double max_speed;
        std::pair<int,int> id;
        // Bound the retry: if every lane on this road is blocked, give up rather
        // than spinning forever, and just don't spawn a robot on it this tick.
        const int kMaxSpawnAttempts = 50;
        int attempts = 0;
        bool gave_up = false;
        do {
            if (++attempts > kMaxSpawnAttempts){ gave_up = true; break; }
            collision_with_existing = false;
            // Define one road (going left), then rotate the positions for the others.
            road = r;
            Eigen::Matrix4d rot; rot.setZero();
            rot.topLeftCorner(2,2)     << cos(PI/2.*road), -sin(PI/2.*road), sin(PI/2.*road), cos(PI/2.*road);
            rot.bottomRightCorner(2,2) << cos(PI/2.*road), -sin(PI/2.*road), sin(PI/2.*road), cos(PI/2.*road);

            lane = sim.randomInt(0, n_lanes-1);
            turn = twoway ? sim.randomInt(0,2) : 1;
            max_speed = (globals.spawn_special_robot_ && (r==trigger_road)) ? 2.0*globals.MAX_SPEED : (1.+lane*0.1)*globals.MAX_SPEED;
            double start_angle = PI/2.*road, end_angle = PI/2.*(road-1+turn); double turn_angle = start_angle;
            double lane_v_offset = twoway ? (0.5*(1-2.*n_lanes)+lane)*lane_width : (-0.5 + (lane+1)/(float)(n_lanes+1))*n_lanes*2*lane_width;
            double omega = globals.MAX_SPEED / globals.TURNING_RADIUS;
            double lane_h_offset = (1-turn)*(0.5+lane-n_lanes)*lane_width - 1.*max_speed/omega;

            starting = rot * Eigen::VectorXd{{-globals.WORLD_SZ/2., lane_v_offset, max_speed, 0.}};
            start_pos = Eigen::VectorXd{{starting(0), starting(1), start_angle, starting(2), starting(3), 0.}};
            turning = rot * Eigen::VectorXd{{lane_h_offset, lane_v_offset, (turn%2)*max_speed, (turn-1)*max_speed}};
            turn_pos = Eigen::VectorXd{{turning(0), turning(1), turn_angle, turning(2), turning(3), 0.}};
            ending = rot * Eigen::VectorXd{{lane_h_offset + (turn%2)*globals.WORLD_SZ*1., lane_v_offset + (turn-1)*globals.WORLD_SZ*1., 0., 0.}};
            end_pos = Eigen::VectorXd{{ending(0), ending(1), end_angle, ending(2), ending(3), 0.}};

            id = std::make_pair(road,lane);
            if (sim.last_created_robots_.count(id)){
                if (!sim.robots_.count(sim.last_created_robots_.at(id))) continue;
                auto rob = sim.robots_.at(sim.last_created_robots_.at(id));
                collision_with_existing = ((start_pos({0,1}) - rob->position_({0,1})).norm() <= globals.robot_length_);
            }
        } while (collision_with_existing);
        if (gave_up) continue;   // no free lane on this road: skip spawning here

        std::deque<Eigen::VectorXd> waypoints{start_pos, turn_pos, end_pos};
        // Colour by turn direction: straight=GOLD, left=LIME, right=VIOLET.
        Color robot_color = (turn==1) ? GOLD : (turn==2) ? LIME : VIOLET;
        bool is_special = false;
        if (globals.spawn_special_robot_ && (r==trigger_road)){
            robot_color = BLACK;
            globals.spawn_special_robot_ = false;
            is_special = true;
        }
        sim.last_created_robots_[id] = sim.next_rid_;   // rid the robot is about to be assigned
        auto robot = std::make_shared<Robot>(sim.next_rid_++, waypoints, robot_color, &sim.graphics->obstacleDist_, -1, false, is_special, max_speed);
        out.push_back(robot);
    }
    return out;
}

/**************************************************************************************/
// Fallback for an unrecognised scenario name: random static start positions.
/**************************************************************************************/
static std::vector<std::shared_ptr<Robot>> defaultScenario(Simulator& sim, uint32_t /*t*/){
    std::vector<std::shared_ptr<Robot>> out;
    if (!sim.robots_.empty()) return out;
    print("Scenario not recognised - using random placement. Define new scenarios in PathPlanningScenarios.cpp!");
    for (int i=0; i<globals.NUM_ROBOTS; i++){
        Eigen::VectorXd start_pos = sim.randomNumberVec("uniform", -0.99, 0.99, 2) * 0.5 * globals.WORLD_SZ;
        Eigen::VectorXd end_pos = start_pos;
        double angle = atan2((end_pos - start_pos)(1), (end_pos - start_pos)(0));
        std::deque<Eigen::VectorXd> waypoints{ Eigen::VectorXd{{start_pos(0), start_pos(1), angle, 0., 0., 0.}} };
        Color robot_color = ColorFromHSV(i*360./(float)globals.NUM_ROBOTS, 1., 0.75);
        int d = sim.randomInt(0, (int)globals.NUM_DECISIONS - 1);
        out.push_back(std::make_shared<Robot>(sim.next_rid_++, waypoints, robot_color, &sim.graphics->obstacleDist_, d));
    }
    return out;
}

/**************************************************************************************/
// 3. FACTORY: map a scenario name (globals.SCENARIO) to its function.
// >>> Register your own scenario here. <<<
/**************************************************************************************/
std::function<std::vector<std::shared_ptr<Robot>>(Simulator&, uint32_t)> makeScenarioFn(const std::string& name){
    if (name=="circle")           return circleScenario;
    if (name=="file")             return fileScenario;
    if (name=="random")           return randomScenario;
    if (name.substr(0,8)=="junction") return junctionScenario;
    if (name.substr(0,7)=="trigrid") return trigridScenario;
    return defaultScenario;
}
