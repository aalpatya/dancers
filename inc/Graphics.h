/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once
#include <Globals.h>
#include <Simulator.h>
#include <raymath.h>
#include <rcamera.h>
/**************************************************************************/
// Graphics class that deals with the nitty-gritty of display.
// Camera is also included here. You can set different camera positions/trajectories
// and then during simulation cycle through them using the SPACEBAR

// Please note: Raylib camera defines the world with positive X = right, positive Z = down, and positive Y = out-of-plane
// But in our work we use the standard convention of positive X = right, positive Y = down, and positive Z = into-plane
/**************************************************************************/

// Draw a Texture2D flat on the ground plane (at height h), centred on `position`
// and scaled.
void DrawTextureFlat(Texture2D texture, Vector2 position, float rotation, Vector3 rotationAxis, float scale, Color tint, float h);

class Graphics {
public:
    Graphics(std::string obstacle_image_path);
    ~Graphics();
    
    DistanceField obstacleDist_;                          // Obstacle distance field the planning layer's obstacle factor reads

    Model robotModel_;                              // Raylib Model representing a robot. This can be changed.
    Model groundModel_;                             // Model representing the ground plane
    Vector3 groundModelpos_;                        // Ground plane position
    Shader lightShader_;                            // Light shader
    
    // The 3D world is rendered into this off-screen SCREEN_SZ x SCREEN_SZ square, then Simulator::draw()
    // blits it to the right of the controls panel - so the world stays a fixed square whatever PANEL_W is.
    RenderTexture2D world_rt_ = { 0 };

    Camera3D camera3d = { 0 };                      // Define the camera to look into our 3d world
    // These represent a set of camera transition frames.
    std::vector<Vector3> camera_positions_{};
    std::vector<Vector3> camera_ups_{};
    std::vector<Vector3> camera_targets_{};
    int camera_idx_ = 0;
    uint32_t camera_clock_=0;    
    bool camera_transition_ = false; 

    // Update camera from mouse/key input. allow_mouse_input=false skips the wheel-zoom and middle-drag
    // (e.g. when the cursor is over the controls panel, so the wheel scrolls a dropdown instead); the
    // SPACEBAR camera transition still runs either way.
    void updateCamera(bool allow_mouse_input=true);
    // Pan the camera across the ground plane by a screen-space mouse delta (scaled to world units by zoom).
    void pan(Vector2 delta);
    // Orbit the camera around its target by a screen-space mouse delta (yaw + pitch).
    void orbit(Vector2 delta);
    // Load the obstacle PNG with raylib for the ground texture, and build obstacleDist_ with a C++
    // chamfer distance transform.
    void buildObstacleGround(const std::string& path);
};