/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once
#include <raylib.h>   // Color

// App-side wrapper for layers that draw themselves. Drawing needs the rendering context (the owning
// Robot and the Simulator's graphics), so it lives here rather than in the gbp library's
// FactorGraphLayer base. Robot::draw() dynamic_casts each layer to DrawableLayer and calls draw().
class Robot;
class Simulator;

class DrawableLayer {
public:
    virtual ~DrawableLayer() = default;

    // Draw this layer for the given robot. col is the robot's running display colour; a layer may read
    // it to tint its geometry and/or modify it (the consensus layer tints the robot model, drawn by
    // Robot::draw() afterwards).
    virtual void draw(Robot* robot, Simulator* sim, Color& col) = 0;
};
