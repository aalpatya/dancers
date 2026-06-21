/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once

#include <functional>
#include <vector>
#include <memory>
#include <string>
#include <cstdint>

class Simulator;
class Robot;

/**************************************************************************************/
// SCENARIOS
//
// A scenario is just a function:
//
//     std::vector<std::shared_ptr<Robot>> myScenario(Simulator& sim, uint32_t timestep);
//
// It is called once per timestep and returns any NEW robots to spawn this step
// (return {} if none). 
// It has full access to `sim` so it can: 
//      - read the current robots (sim.robots_)
//      - use the random number generators (sim.randomInt / sim.randomNumberVec),
//      - query obstacles, etc. 
//      - create a robot with
//          std::make_shared<Robot>(sim.next_rid_++, waypoints, color, &sim.graphics->obstacleImgCVDist_, ...)
//          (the Robot constructor builds its own configured factor graph stack).
//
//  A Scenario is useful when you want to procedurally generate starting/intermediate waypoints.
//  It can handle:
//   - STATIC problems (e.g. circle): a fixed set of start/goal waypoints THAT DEPEND ON ROBOT SIZE.
//      Spawn all robots on the first call (when sim.robots_ is empty) and return {} afterwards.
//   - DYNAMIC problems (e.g. junction, random): spawn robots over time, and/or top up
//     existing robots' waypoints.
//
//  NOTE: The simulator (and not the Scenario) is responsible for:
//     (a) deleting robots that leave the world bounds
//     (b) popping a robot's front waypoint once it is reached
//
// To ADD YOUR OWN SCENARIO: write a function with the signature above in PathPlanningScenarios.cpp
// and register it by name in makeScenarioFn() at the bottom of that file.
/**************************************************************************************/

// Returns the scenario function for the given scenario name (from globals.SCENARIO).
std::function<std::vector<std::shared_ptr<Robot>>(Simulator&, uint32_t)> makeScenarioFn(const std::string& name);
