/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
// Compile the rlights implementation here (once). It must precede <Graphics.h>, which transitively pulls
// in rlights.h. Keeping it in this TU lets the Graphics ctor reset rlights' internal `lightsCount` so a
// rebuilt Graphics (config switch) re-creates its light at index 0 instead of drifting to a new slot.
#define RLIGHTS_IMPLEMENTATION
#include <Graphics.h>

// Compile the raygui (immediate-mode GUI) implementation here, once. Graphics.h transitively pulls in
// raylib.h, which raygui needs. Other translation units just #include <raygui.h> for the declarations.
#define RAYGUI_IMPLEMENTATION
#include <raygui.h>

#include <Simulator.h>   // for the Simulator GUI/HUD overlay methods defined at the bottom of this file
#include <Robot.h>
#include <rlgl.h>        // low-level matrix stack used by DrawTextureFlat
#include <vector>
#include <algorithm>
#include <cmath>
#include <filesystem>    // scanning config/ and shapes/ for the controls-panel dropdowns
#ifdef __EMSCRIPTEN__
#include <emscripten.h>          // EM_ASM (via em_asm.h): a web "restart" is a page reload with the new config
#endif

// Draw a Texture2D flat on the ground plane (at height h), centred on `position`
// and scaled. (rotationAxis is unused, kept for call-site compatibility.)
void DrawTextureFlat(Texture2D texture, Vector2 position, float rotation, Vector3 rotationAxis, float scale, Color tint, float h)
{
    rlPushMatrix();
        rlTranslatef(position.x, h, position.y);
        rlRotatef(rotation, 0.f, -1.f, 0.f);
        rlRotatef(90.f, 1.f, 0.f, 0.f);
        rlTranslatef(-0.5f*texture.width*scale, -0.5f*texture.height*scale, 0.f);
        Rectangle source = { 0.0f, 0.0f, (float)texture.width, (float)texture.height };
        Rectangle dest = { 0.f, 0.f, (float)texture.width*scale, (float)texture.height*scale };
        Vector2 origin = { 0.0f, 0.0f };

        DrawTexturePro(texture, source, dest, origin, 0., tint);
        rlEnd();
    rlPopMatrix();
}

/**************************************************************************/
// Graphics class that deals with the nitty-gritty of display.
// Camera is also included here. You can set different camera positions/trajectories
// and then during simulation cycle through them using the SPACEBAR

// Please note: Raylib camera defines the world with positive X = right, positive Z = down, and positive Y = out-of-plane
// But in our work we use the standard convention of positive X = right, positive Y = down, and positive Z = into-plane
/**************************************************************************/
Graphics::Graphics(std::string obstacle_image_path){
    if (!globals.DISPLAY) return;

    // Camera is defined by a forward vector (target - position), as well as an up vector (see raylib for more info)
    // These are vectors for each camera transition. Cycle through them in the simulation with the SPACEBAR
    camera3d.fovy = 60.0f;
    camera_positions_ = {Vector3{0.,(float)(0.5/tan(camera3d.fovy*0.5*DEG2RAD))*globals.WORLD_SZ, 0.},
                        (Vector3){20., 15, 20},
                        (Vector3){ 0., 0.85f*globals.WORLD_SZ, 0.9f*globals.WORLD_SZ }};
    // This line is uncommented for regular camera rotation
    // camera_ups_ = {Vector3{1.,0.,0.}, (Vector3){-0.325, 0.9, -0.316}, (Vector3){0.,0.,-1.}};
    // The next line is uncommented if you want camera to begin rotated 90 degrees
    camera_ups_ = {Vector3{0.,0.,-1.}, (Vector3){-0.325, 0.9, -0.316}, (Vector3){0.,0.,-1.}};
    camera_targets_ = {Vector3{0.,0.,0.}, (Vector3){1.363, 0, 1.463}, (Vector3){0.,0.,0.}};


    camera3d.position = camera_positions_[camera_idx_];
    camera3d.target = camera_targets_[camera_idx_];
    camera3d.up = camera_ups_[camera_idx_];
    camera3d.projection = CAMERA_PERSPECTIVE;

    // Lighting shader. GLSL_DIR selects the shader dialect: "glsl330" for desktop GL, "glsl100" for
    // WebGL ES2. It's a compile definition set by CMake (see the GLSL_DIR define alongside PROJECT_ROOT).
    lightShader_ = LoadShader(makePath({globals.ASSETS_DIR, "shaders", GLSL_DIR, "base_lighting.vs"}).c_str(),
                            makePath({globals.ASSETS_DIR, "shaders", GLSL_DIR, "lighting.fs"}).c_str());
    lightShader_.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(lightShader_, "viewPos");
    // Ambient light level (some basic lighting)
    int ambientLoc = GetShaderLocation(lightShader_, "ambient");
    float temp[4] = {0.1f, 0.1f, 0.1f, 1.0f};
    SetShaderValue(lightShader_, ambientLoc, temp, SHADER_UNIFORM_VEC4);

    // Assign our lighting shader to robot model
    if (globals.ROBOT_MODEL=="bus"){
        robotModel_ = LoadModel(makePath({globals.ASSETS_DIR, "models", "bus.glb"}).c_str());
        robotModel_.transform = MatrixRotateY(-PI/2.f);//MatrixMultiply(MatrixRotateY(-PI/2.f), MatrixRotateZ(-PI/2.f));
    } else if (globals.ROBOT_MODEL=="sphere"){
        robotModel_ = LoadModelFromMesh(GenMeshSphere(1., 16.0f, 16.0f));
        robotModel_.transform = MatrixTranslate(0.f, 1.f, 0.f);
    } else {
        robotModel_ = LoadModel(makePath({globals.ASSETS_DIR, "models", "robot.obj"}).c_str());
    }

    for (int i = 0; i < robotModel_.materialCount; i++){
        robotModel_.materials[i].shader = lightShader_;
        robotModel_.materials[i].maps[MATERIAL_MAP_DIFFUSE].color = WHITE;
    }

    // Load the obstacle PNG (raylib), use it as the ground texture, and build the distance field the
    // planning layer's obstacle factor reads. Falls back to a plain checker ground if no image is present.
    buildObstacleGround(obstacle_image_path);

    // Create lights. Reset rlights' global light counter so a rebuilt Graphics re-creates its light at
    // index 0 (otherwise it climbs past MAX_LIGHTS and the new shader is left unlit).
    lightsCount = 0;
    Light lights[MAX_LIGHTS] = { 0 };
    Vector3 target = camera3d.target;
    Vector3 position = Vector3{target.x+100,target.y+200,target.z+100};
    lights[0] = CreateLight(LIGHT_POINT, position, target, LIGHTGRAY, lightShader_);

    // Off-screen target the 3D world is rendered into (see world_rt_), blitted right of the panel. Size
    // it to SCREEN_SZ * supersample, where supersample follows the HighDPI scale so the world isn't
    // rendered at half resolution on a retina display and upscaled when blitted (looks pixelated).
    // On web the world render is the framerate bottleneck (lit 3D on WebGL ES2), so cap the supersample:
    // a full 2x is 4x the fragments and halves the framerate. 1.0 keeps 60fps; raise toward 2.0 for a
    // sharper world at the cost of fps.
    float supersample = GetWindowScaleDPI().x;
#ifdef __EMSCRIPTEN__
    supersample = std::min(supersample, 1.0f);
#endif
    int rt_sz = (int)(globals.SCREEN_SZ * supersample);
    world_rt_ = LoadRenderTexture(rt_sz, rt_sz);
    SetTextureFilter(world_rt_.texture, TEXTURE_FILTER_BILINEAR);
}

// Load the obstacle PNG (black = obstacle, white = free) as the ground texture, then build obstacleDist_
// with a chamfer (3,4) distance transform capped at 20 px and normalised to 0..255 (0 at an obstacle,
// 255 far).
void Graphics::buildObstacleGround(const std::string& path){
    const int SZ = 1000;
    groundModel_ = LoadModelFromMesh(GenMeshPlane(1.f*globals.WORLD_SZ, 1.f*globals.WORLD_SZ, 1, 1));
    groundModelpos_ = {0., 0., 0.};

    if (path.empty() || !FileExists(path.c_str())){
        globals.obstacles_present_ = false;
        Image blank = GenImageColor(SZ, SZ, RAYWHITE);   // no obstacle image -> plain blank ground
        groundModel_.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = LoadTextureFromImage(blank);
        UnloadImage(blank);
        return;
    }

    Image img = LoadImage(path.c_str());
    ImageResize(&img, SZ, SZ);
    ImageFormat(&img, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
    groundModel_.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = LoadTextureFromImage(img);

    const Color* px = (const Color*)img.data;
    const int INF = 1 << 29;
    std::vector<int> d(SZ*SZ);
    for (int i = 0; i < SZ*SZ; i++) d[i] = (px[i].r < 250 || px[i].g < 250 || px[i].b < 250) ? 0 : INF;   // 0 at obstacle, INF elsewhere
    UnloadImage(img);

    auto at = [&](int x, int y) -> int& { return d[y*SZ + x]; };
    auto relax = [&](int x, int y, int nx, int ny, int w){
        if (nx < 0 || ny < 0 || nx >= SZ || ny >= SZ) return;
        if (d[ny*SZ + nx] + w < at(x,y)) at(x,y) = d[ny*SZ + nx] + w;
    };
    for (int y = 0; y < SZ; y++) for (int x = 0; x < SZ; x++){     // forward pass
        relax(x,y,x-1,y,3); relax(x,y,x,y-1,3); relax(x,y,x-1,y-1,4); relax(x,y,x+1,y-1,4);
    }
    for (int y = SZ-1; y >= 0; y--) for (int x = SZ-1; x >= 0; x--){ // backward pass
        relax(x,y,x+1,y,3); relax(x,y,x,y+1,3); relax(x,y,x+1,y+1,4); relax(x,y,x-1,y+1,4);
    }

    const double cap = 20.0;   // chamfer units / 3 ~= pixels; cap and normalise like createDistanceField
    obstacleDist_.w = SZ; obstacleDist_.h = SZ; obstacleDist_.data.resize((size_t)SZ*SZ);
    for (int i = 0; i < SZ*SZ; i++){
        double dist = std::min(d[i] / 3.0, cap);
        obstacleDist_.data[i] = (unsigned char)std::lround(dist / cap * 255.0);
    }
    globals.obstacles_present_ = true;
}

Graphics::~Graphics(){
    // Unload GL resources so a rebuilt Graphics (config switch) doesn't leak them.
    UnloadTexture(groundModel_.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture);
    UnloadModel(groundModel_);
    UnloadModel(robotModel_);
    UnloadShader(lightShader_);
    UnloadRenderTexture(world_rt_);
};

/******************************************************************************************/
// Use captured mouse input and keypresses and modify the camera view.
// Also transition between camera viewframes if necessary.
/******************************************************************************************/
void Graphics::pan(Vector2 delta)
{
    // Scale the screen-space (pixel) delta to world units at the target plane, so panning tracks the
    // cursor regardless of zoom: world-per-pixel = 2*distance*tan(fovy/2) / screenHeight.
    float dist = Vector3Distance(camera3d.position, camera3d.target);
    float world_per_px = 2.f * dist * tanf(camera3d.fovy*0.5f*DEG2RAD) / (float)GetScreenHeight();
    delta = Vector2Scale(delta, world_per_px);
    CameraMoveRight(&camera3d, -delta.x, true);
    Vector3 D = GetCameraUp(&camera3d); D.y = 0.;
    D = Vector3Scale(Vector3Normalize(D), delta.y);
    camera3d.position = Vector3Add(camera3d.position, D);
    camera3d.target = Vector3Add(camera3d.target, D);
}

void Graphics::orbit(Vector2 delta)
{
    const float k = 0.012f;   // rotation sensitivity (radians per pixel)
    CameraPitch(&camera3d, -delta.y*k, true, true, true);                       // pitch around target
    camera3d.up = Vector3RotateByAxisAngle(camera3d.up, Vector3{0.,1.,0.}, -k*delta.x);  // yaw around world up
    Vector3 forward = Vector3Subtract(camera3d.target, camera3d.position);
    forward = Vector3RotateByAxisAngle(forward, Vector3{0.,1.,0.}, -k*delta.x);
    camera3d.position = Vector3Subtract(camera3d.target, forward);
}

void Graphics::updateCamera(bool allow_mouse_input)
{
    // Skip wheel-zoom / middle-drag when the cursor is over the GUI (so the wheel scrolls a dropdown
    // instead of zooming the world). The camera transition below still runs.
    if (allow_mouse_input){
        float zoomscale = IsKeyDown(KEY_LEFT_SHIFT) ? 100. :10.;
        float zoom = -(float)GetMouseWheelMove() * zoomscale;
        CameraMoveToTarget(&camera3d, zoom);
        if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE))
        {
            Vector2 del = GetMouseDelta();
            // FOR UP {0,0,-1} and TOWARDS STRAIGHT DOWN
            if (IsKeyDown(KEY_LEFT_SHIFT)){
                orbit(del);
            } else if (IsKeyDown(KEY_LEFT_CONTROL)){
                float zoom = del.y*0.1;
                CameraMoveToTarget(&camera3d, zoom);
            } else {
                // Camera movement
                pan(del);
            }
        }
    }
    if (camera_transition_){
        int camera_transition_time = 100;
        if (camera_clock_==camera_transition_time){
            camera_transition_ = false;
            camera_idx_ = (camera_idx_+1)%camera_positions_.size();
            camera_clock_ = 0;
        }
        camera3d.position = Vector3Lerp(camera_positions_[camera_idx_], camera_positions_[(camera_idx_+1)%camera_positions_.size()], (camera_clock_%camera_transition_time)/(float)camera_transition_time);
        camera3d.up = Vector3Lerp(camera_ups_[camera_idx_], camera_ups_[(camera_idx_+1)%camera_ups_.size()], (camera_clock_%camera_transition_time)/(float)camera_transition_time);
        camera3d.target = Vector3Lerp(camera_targets_[camera_idx_], camera_targets_[(camera_idx_+1)%camera_targets_.size()], (camera_clock_%camera_transition_time)/(float)camera_transition_time);
        camera_clock_++;

    }    
}

/*******************************************************************************/
// Simulator 2D GUI / HUD overlays. These are Simulator member functions, defined here
// alongside the raygui implementation so the immediate-mode widgets live in one place and
// Simulator.cpp stays focused on the simulation. Called each frame from Simulator::draw().
/*******************************************************************************/

// Fixed dropdown option lists (the config + shape lists are scanned from disk in buildControlOptions).
namespace {
    const std::vector<std::string> MODEL_OPTS = {"bus","sphere","robot"};
    int indexOf(const std::vector<std::string>& v, const std::string& x){
        for (size_t i=0;i<v.size();i++) if (v[i]==x) return (int)i;
        return 0;
    }
}

/*******************************************************************************/
// Populate the controls-panel dropdown lists (called once from the Simulator ctor): scan the config
// directory and the shape-image directory under PROJECT_ROOT, and seed the staged values + selected
// indices from the current globals. PROJECT_ROOT is the project dir on desktop and the preloaded
// virtual-FS mount ("/app") on the web, so the same scan works on both.
/*******************************************************************************/
void Simulator::buildControlOptions(){
    namespace fs = std::filesystem;
    std::error_code ec;
    const std::string root = PROJECT_ROOT;

    // Config files: every *.yaml under <root>/config (recursive). Stored as root-relative paths.
    cfg_paths_.clear();
    std::vector<std::pair<std::string,std::string>> cfgs;   // (label, root-relative path), sorted by label
    const std::string cfg_dir = makePath({root, "config"});
    if (fs::exists(cfg_dir, ec))
        for (auto& e : fs::recursive_directory_iterator(cfg_dir, ec))
            if (e.path().extension() == ".yaml"){
                std::string lbl = fs::relative(e.path(), cfg_dir, ec).generic_string();
                if (lbl.size() > 5) lbl.resize(lbl.size()-5);   // drop ".yaml"
                cfgs.push_back({lbl, fs::relative(e.path(), root, ec).generic_string()});
            }
    std::sort(cfgs.begin(), cfgs.end());
    cfg_opts_.clear();
    for (auto& c : cfgs){ cfg_opts_.push_back(c.first); cfg_paths_.push_back(c.second); }
    cfg_dd_ = 0;
    std::string cur_cfg = fs::relative(globals.CONFIG_FILE, root, ec).generic_string();
    for (size_t i=0;i<cfg_paths_.size();i++) if (cfg_paths_[i]==cur_cfg) cfg_dd_=(int)i;
    cfg_loaded_ = cfg_dd_;   // baseline: Apply only switches config if the selection differs from this

    // Shape images: "(none)" plus every *.png under <root>/assets/imgs/shapes (stored as full paths).
    shape_paths_.assign(1, std::string());           // index 0 = "(none)"
    shape_opts_.assign(1, "(none)");
    std::vector<std::pair<std::string,std::string>> shps;   // (label, full path), sorted by label
    const std::string shp_dir = makePath({root, "assets", "imgs", "shapes"});
    if (fs::exists(shp_dir, ec))
        for (auto& e : fs::directory_iterator(shp_dir, ec))
            if (e.path().extension() == ".png")
                shps.push_back({e.path().stem().string(), e.path().generic_string()});
    std::sort(shps.begin(), shps.end());
    for (auto& s : shps){ shape_opts_.push_back(s.first); shape_paths_.push_back(s.second); }
    shape_dd_ = 0;
    std::string cur_shape = fs::path(globals.FORMATION_IMG_FILE).filename().string();
    if (!cur_shape.empty())
        for (size_t i=1;i<shape_paths_.size();i++)
            if (fs::path(shape_paths_[i]).filename().string()==cur_shape) shape_dd_=(int)i;

    // Fixed dropdown + staged numeric values mirror the current config.
    model_dd_ = indexOf(MODEL_OPTS, globals.ROBOT_MODEL);
    stage_num_robots_    = globals.NUM_ROBOTS;
    stage_seed_          = globals.RNG_SEED;
    stage_world_sz_      = globals.WORLD_SZ;
    stage_seed_decision_ = globals.SEED_DECISION;
    std::snprintf(t_horizon_buf_, sizeof(t_horizon_buf_), "%g", globals.T_HORIZON);
}

/*******************************************************************************/
// Commit a restart: globals.pending_config_ / pending_overrides_ have already been set by the caller.
// Desktop: main()'s loop rebuilds the Simulator. Web: reload the page with the config + overrides in
// the URL (the loader passes them to --cfg / --set), since the GL context can't be safely rebuilt.
/*******************************************************************************/
void Simulator::requestRestart(){
    globals.restart_requested_ = true;
#ifdef __EMSCRIPTEN__
    // Build the reload URL in C++ and let JS percent-encode the parts (config paths / overrides contain
    // '/', ';', '=' but no quotes, so single-quoted JS string literals are safe here).
    std::string js = "var u='index.html?config='+encodeURIComponent('" + globals.pending_config_ + "');";
    if (!globals.pending_overrides_.empty())
        js += "u+='&set='+encodeURIComponent('" + globals.pending_overrides_ + "');";
    js += "window.location.href=u;";
    emscripten_run_script(js.c_str());
#endif
}

/*******************************************************************************/
// Full-height controls panel docked on the left (width PANEL_W). Top: title + FPS + timestep. Then
// SIMULATION (play/step), CONFIG (file dropdown -> restart on change), LIVE (apply instantly), and
// RESTART REQUIRED (staged edits committed together by "Apply & Reset"). Dropdowns are drawn last so
// an open list paints over the widgets below it; while one is open the others are locked.
/*******************************************************************************/
void Simulator::drawControlsPanel(){
    const float pad=10, x0=pad, w=PANEL_W-2*pad;
    const float rowH=22, gap=6, lblH=15, cs=18, secGap=10;
    const Color bg   = GetColor(GuiGetStyle(DEFAULT, BACKGROUND_COLOR));
    const Color line = GetColor(GuiGetStyle(DEFAULT, LINE_COLOR));
    const Color txt  = GetColor(GuiGetStyle(DEFAULT, TEXT_COLOR_NORMAL));

    DrawRectangle(0, 0, PANEL_W, globals.SCREEN_SZ, bg);
    DrawLine(PANEL_W, 0, PANEL_W, globals.SCREEN_SZ, line);   // divider against the world

    float y = pad;
    auto section = [&](const char* t){ y+=6; DrawLine((int)x0,(int)y,(int)(x0+w),(int)y,line); y+=5;
                                       DrawText(t,(int)x0,(int)y,11,Fade(txt,0.6f)); y+=16; };

    // --- Header: title + live status -------------------------------------------------------------
    DrawText("DANCeRS", (int)x0, (int)y, 22, txt); y+=26;
    DrawText(TextFormat("FPS %d    timestep %u", GetFPS(), clock_), (int)x0, (int)y, 11, Fade(txt,0.7f)); y+=16;

    // Dropdowns are interactive but drawn LAST (an open list must paint over everything below it);
    // reserve their rects now. While any dropdown is open, the rest of the panel is locked so clicks on
    // the (later-drawn) open list don't fall through to a widget beneath it.
    Rectangle cfgRect, modelRect, shapeRect;
    // Snapshot each dropdown's open-state at frame start so each is drawn exactly once per frame (a box
    // opened this frame shows its list next frame; drawing both in one frame would eat the click).
    bool cfgOpen0 = cfg_dd_open_, modelOpen0 = model_dd_open_, shapeOpen0 = shape_dd_open_;
    bool anyOpen = cfgOpen0 || modelOpen0 || shapeOpen0;
    // raygui widgets fire on mouse *release* over their bounds, not on press. If we only lock the panel
    // while a dropdown is open, the release that picks a popup item (and closes it) lands on the next
    // frame - when the panel is unlocked again - and falls through to the widget the popup was covering
    // (e.g. Apply & Reset). So latch from the press while open through to the release, keeping the rest
    // of the panel locked for the whole click.
    if (anyOpen && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) dd_consumed_press_ = true;
    bool lockBelow = anyOpen || dd_consumed_press_;
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) dd_consumed_press_ = false;
    if (lockBelow) GuiLock();

    // --- SIMULATION ------------------------------------------------------------------------------
    section("SIMULATION");
    float bw = (w-gap)*0.62f, bws = w-gap-bw;
    bool playing = (globals.SIM_MODE==SimMode::Timestep), was_playing = playing;
    GuiToggle(Rectangle{x0, y, bw, rowH}, playing ? "#132#Pause" : "#131#Play", &playing);
    if (playing != was_playing) globals.SIM_MODE = playing ? SimMode::Timestep : SimMode::Paused;
    if (GuiButton(Rectangle{x0+bw+gap, y, bws, rowH}, "#134#Step")) globals.SIM_MODE = SimMode::TimestepSingle;
    y += rowH + secGap;

    // --- Configuration (config file + staged params; nothing happens until Apply) -----------------
    // Most rows are |Label        [control]| with the control on the right; Config is wider (it starts
    // just after its label) since config names are long.
    const float ctlW = w * 0.55f;                          // right-hand control width (model/shape/value boxes)
    section("Configuration (requires reset)");
    { DrawText("Config", (int)x0, (int)(y+4), 10, txt);
      float lw = (float)MeasureText("Config", 10) + 10;
      cfgRect = Rectangle{x0 + lw, y, w - lw, rowH};        // wide: starts just after the label
      y += rowH + gap; }
    auto intRow = [&](const char* lbl, int* val, bool* edit, int lo, int hi){
        DrawText(lbl, (int)x0, (int)(y+4), 10, txt);
        if (GuiValueBox(Rectangle{x0+w-ctlW, y, ctlW, rowH}, NULL, val, lo, hi, *edit)) *edit = !*edit;
        y += rowH + gap;
    };
    auto ddRow = [&](const char* lbl)->Rectangle{
        DrawText(lbl, (int)x0, (int)(y+4), 10, txt);
        Rectangle r{x0 + w - ctlW, y, ctlW, rowH};
        y += rowH + gap;
        return r;
    };
    intRow("Num robots", &stage_num_robots_, &num_edit_,   1, 1000);
    intRow("RNG Seed",       &stage_seed_,       &seed_edit_,  0, 100000);
    intRow("World size", &stage_world_sz_,   &world_edit_, 10, 1000);
    intRow("Seed decision", &stage_seed_decision_, &seed_decision_edit_, 0, 1000);
    // T_HORIZON is a float, so it's edited as text (no float value-box in raygui).
    DrawText("Horizon (timesteps)", (int)x0, (int)(y+4), 10, txt);
    int tb_al = GuiGetStyle(TEXTBOX, TEXT_ALIGNMENT);
    // Centre to match the value-boxes above, but left-align while editing: raygui positions the text
    // cursor assuming left alignment, so a centred box puts the cursor out of sync with the text.
    GuiSetStyle(TEXTBOX, TEXT_ALIGNMENT, t_horizon_edit_ ? TEXT_ALIGN_LEFT : TEXT_ALIGN_CENTER);
    if (GuiTextBox(Rectangle{x0+w-ctlW, y, ctlW, rowH}, t_horizon_buf_, sizeof(t_horizon_buf_), t_horizon_edit_)) t_horizon_edit_ = !t_horizon_edit_;
    GuiSetStyle(TEXTBOX, TEXT_ALIGNMENT, tb_al);
    y += rowH + gap;
    modelRect = ddRow("Robot model");
    shapeRect = ddRow("Shape image");
    bool apply = GuiButton(Rectangle{x0, y, w, rowH+2}, "#75#Apply & Reset");
    y += rowH + secGap;

    // --- Editable Parameters (applied instantly) -------------------------------------------------
    section("Editable Parameters");
    GuiCheckBox(Rectangle{x0, y, cs, cs}, "Planned paths",    &globals.DRAW_PATH);       y += cs+gap;
    GuiCheckBox(Rectangle{x0, y, cs, cs}, "Trajectories",     &globals.DRAW_TRAJ);       y += cs+gap;
    GuiCheckBox(Rectangle{x0, y, cs, cs}, "Waypoints",        &globals.DRAW_WAYPOINTS);  y += cs+gap;
    GuiCheckBox(Rectangle{x0, y, cs, cs}, "Interrobot comms", &globals.DRAW_INTERROBOT); y += cs+gap;

    const float boxW = 50;
    // GBP iterations per timestep: written straight to globals.NUM_ITERS (the GBP loop reads it each step).
    DrawText("GBP Iters / timestep", (int)x0, (int)(y+4), 10, txt);
    if (GuiValueBox(Rectangle{x0+w-boxW, y, boxW, rowH}, NULL, &globals.NUM_ITERS, 1, 100, iters_edit_)) iters_edit_ = !iters_edit_;
    y += rowH + gap;

    // Comms radius: slider + value-box (1..WORLD_SZ), pushed live to globals and every robot.
    DrawText("Comms radius", (int)x0, (int)y, 10, txt); y += 12;
    float comms_prev = globals.COMMUNICATION_RADIUS;
    GuiSlider(Rectangle{x0, y+1, w-boxW-6, rowH-2}, NULL, NULL, &globals.COMMUNICATION_RADIUS, 0.f, (float)globals.WORLD_SZ);
    if (!comms_box_edit_) comms_box_value_ = (int)std::round(globals.COMMUNICATION_RADIUS);
    if (GuiValueBox(Rectangle{x0+w-boxW, y, boxW, rowH}, NULL, &comms_box_value_, 0, globals.WORLD_SZ, comms_box_edit_)) comms_box_edit_ = !comms_box_edit_;
    if (comms_box_edit_) globals.COMMUNICATION_RADIUS = (float)comms_box_value_;
    if (globals.COMMUNICATION_RADIUS != comms_prev)
        for (auto& [rid, robot] : robots_) robot->COMMUNICATION_RADIUS = globals.COMMUNICATION_RADIUS;
    y += rowH + gap;

    // Comms failure %: slider + value-box (0..100), mapped to COMMS_FAILURE_RATE [0,1].
    DrawText("Comms failure %", (int)x0, (int)y, 10, txt); y += 12;
    float fail_pct = globals.COMMS_FAILURE_RATE * 100.f;
    GuiSlider(Rectangle{x0, y+1, w-boxW-6, rowH-2}, NULL, NULL, &fail_pct, 0.f, 100.f);
    if (!fail_box_edit_) fail_box_value_ = (int)std::round(fail_pct);
    if (GuiValueBox(Rectangle{x0+w-boxW, y, boxW, rowH}, NULL, &fail_box_value_, 0, 100, fail_box_edit_)) fail_box_edit_ = !fail_box_edit_;
    globals.COMMS_FAILURE_RATE = (fail_box_edit_ ? (float)fail_box_value_ : fail_pct) / 100.f;
    if (globals.COMMS_FAILURE_RATE == 0.0f) this->setCommsFailure(0.0f, true);
    else                                    globals.DRAW_INTERROBOT = true;   // show comms when failures exist
    y += rowH + gap;

    // Attraction target (TOWARDS_FORMATION) and formation overlay (FORMATION_DISPLAY_TYPE).
    DrawText("Move towards", (int)x0, (int)y, 10, txt); y += lblH;
    int towards = globals.TOWARDS_FORMATION ? 1 : 0, towards_prev = towards;
    GuiToggleGroup(Rectangle{x0, y, (w-gap)/2.f, cs}, "Waypoints;Shape", &towards);
    if (towards != towards_prev) globals.TOWARDS_FORMATION = (towards == 1);
    y += cs + gap;

    DrawText("Formation overlay", (int)x0, (int)y, 10, txt); y += lblH;
    int fdisp = (globals.FORMATION_DISPLAY_TYPE=="full") ? 2 : (globals.FORMATION_DISPLAY_TYPE=="points") ? 1 : 0, fdisp_prev = fdisp;
    GuiToggleGroup(Rectangle{x0, y, (w-2*gap)/3.f, cs}, "None;Points;Shape", &fdisp);
    if (fdisp != fdisp_prev) globals.FORMATION_DISPLAY_TYPE = (fdisp==2) ? "full" : (fdisp==1) ? "points" : "none";
    y += cs + gap;

    bool wp_prev = waypoint_adder_mode_;
    GuiToggle(Rectangle{x0, y, w, rowH}, "#21#Waypoint adder", &waypoint_adder_mode_);
    if (waypoint_adder_mode_ != wp_prev) globals.DRAW_WAYPOINTS = waypoint_adder_mode_;
    y += rowH + secGap;

    if (lockBelow) GuiUnlock();

    // --- Dropdowns: scrollable popups (raygui's GuiDropdownBox can't scroll, and the shape list is long).
    //     Each is a header button showing the current choice; clicking opens a GuiListView popup with a
    //     scrollbar. Closed headers are drawn first (locked while another popup is open); the open one's
    //     popup is drawn LAST so it overlays cleanly. Selections are only staged (applied on Apply). ----
    // A dropdown header button: centred label with an arrow icon flush-right. Returns true if clicked.
    auto ddHeaderButton = [&](Rectangle r, const std::string& label, int iconId)->bool{
        int al = GuiGetStyle(BUTTON, TEXT_ALIGNMENT);
        GuiSetStyle(BUTTON, TEXT_ALIGNMENT, TEXT_ALIGN_CENTER);
        bool clicked = GuiButton(r, label.c_str());
        GuiSetStyle(BUTTON, TEXT_ALIGNMENT, al);
        const int icon = 16;                                // RAYGUI_ICON_SIZE
        Color tc = GetColor(GuiGetStyle(BUTTON, TEXT_COLOR_NORMAL));
        GuiDrawIcon(iconId, (int)(r.x + r.width - icon - 4), (int)(r.y + (r.height - icon)*0.5f), 1, tc);
        return clicked;
    };
    auto ddHeader = [&](Rectangle r, const std::vector<std::string>& opts, int active, bool was_open, bool* open){
        if (was_open) return;                               // the open one's header is drawn in ddPopup
        if (lockBelow) GuiLock();
        if (ddHeaderButton(r, opts.empty() ? "" : opts[active], ICON_ARROW_DOWN_FILL)) *open = true;
        if (lockBelow) GuiUnlock();
    };
    auto ddPopup = [&](Rectangle r, const std::vector<std::string>& opts, int* active, int* scroll, bool* open){
        // Open: header shows an up-arrow; clicking it closes the list.
        if (ddHeaderButton(r, opts.empty() ? "" : opts[*active], ICON_ARROW_UP_FILL)) *open = false;
        int n = (int)opts.size();
        Rectangle pop{r.x, r.y + r.height + 2, r.width, std::min(n, 9) * 24.f + 4.f};
        DrawRectangleRec(pop, GetColor(GuiGetStyle(DEFAULT, BACKGROUND_COLOR)));   // opaque under the list
        std::vector<const char*> ptrs; ptrs.reserve(n);
        for (auto& s : opts) ptrs.push_back(s.c_str());
        int prev = *active, focus = -1;
        GuiListViewEx(pop, ptrs.data(), n, scroll, active, &focus);
        if (*active != prev) *open = false;                 // picked an item -> close
        else if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) &&                     // clicked away -> close
                 !CheckCollisionPointRec(GetMousePosition(), pop) &&
                 !CheckCollisionPointRec(GetMousePosition(), r)) *open = false;
    };
    ddHeader(cfgRect,   cfg_opts_,   cfg_dd_,   cfgOpen0,   &cfg_dd_open_);
    ddHeader(modelRect, MODEL_OPTS,  model_dd_, modelOpen0, &model_dd_open_);
    ddHeader(shapeRect, shape_opts_, shape_dd_, shapeOpen0, &shape_dd_open_);
    if (cfgOpen0)   ddPopup(cfgRect,   cfg_opts_,   &cfg_dd_,   &cfg_scroll_,   &cfg_dd_open_);
    if (modelOpen0) ddPopup(modelRect, MODEL_OPTS,  &model_dd_, &model_scroll_, &model_dd_open_);
    if (shapeOpen0) ddPopup(shapeRect, shape_opts_, &shape_dd_, &shape_scroll_, &shape_dd_open_);

    // Apply: switch to the chosen config (loaded fresh) if it changed; otherwise apply the staged params.
    if (apply){
        globals.pending_config_ = cfg_paths_.empty() ? std::string() : cfg_paths_[cfg_dd_];
        if (cfg_dd_ != cfg_loaded_){
            globals.pending_overrides_.clear();             // different config -> load it as-is
        } else {
            std::string spec;
            auto add = [&](const std::string& k, const std::string& v){ if (!spec.empty()) spec+=";"; spec+=k+"="+v; };
            add("NUM_ROBOTS",        std::to_string(stage_num_robots_));
            add("RNG_SEED",          std::to_string(stage_seed_));
            add("WORLD_SZ",          std::to_string(stage_world_sz_));
            add("SEED_DECISION",     std::to_string(stage_seed_decision_));
            add("T_HORIZON",         std::string(t_horizon_buf_));
            add("ROBOT_MODEL",       MODEL_OPTS[model_dd_]);
            add("FORMATION_IMG_FILE", shape_dd_ > 0 ? shape_paths_[shape_dd_] : std::string(""));
            globals.pending_overrides_ = spec;
        }
        requestRestart();
    }
}

/*******************************************************************************/
// Bottom decision-distribution HUD: a coloured bar of the NUM_DECISIONS discrete
// options with a dot per robot at its current decision. Only shown for the trigrid
// scenarios when globals.CONSENSUS_TYPE is Discrete.
/*******************************************************************************/
void Simulator::drawDecisionHud(){
    if (globals.CONSENSUS_TYPE != ConsensusType::Discrete) return;

    const int   len      = globals.SCREEN_SZ*0.25;
    const int   len_1    = len / globals.NUM_DECISIONS;
    // The world is drawn offset by PANEL_W, so centre the HUD on the world region, not the whole window.
    const int   start_x  = PANEL_W + globals.SCREEN_SZ*0.5 - len/2;
    const float max_bar  = globals.SCREEN_SZ*0.05f;          // height of a bar at 100%
    const float baseline = globals.SCREEN_SZ - 80.f;         // bars grow upward from here; labels below

    if ((int)decisionColors_.size() != (int)globals.NUM_DECISIONS){
        decisionColors_.clear(); decisionHudLabels_.clear();
        for (int d=0; d<globals.NUM_DECISIONS; d++){
            decisionColors_.push_back(decisionColor(d, globals.NUM_DECISIONS));
            decisionHudLabels_.push_back(std::to_string(d));
        }
    }

    // Count robots per decision.
    std::vector<int> counts(globals.NUM_DECISIONS, 0);
    for (auto& [rid, robot] : robots_)
        if (robot->decision_ >= 0 && robot->decision_ < globals.NUM_DECISIONS) counts[robot->decision_]++;
    const int total = (int)robots_.size();

    const float pad     = 8.f;
    const float label_y = baseline + 6.f;    // per-decision digit labels
    const float title_y = baseline + 26.f;   // "Decision" caption below them

    // Opaque background panel behind the whole HUD (bars + labels + caption).
    DrawRectangle(start_x - pad, baseline - max_bar - pad, len + 2*pad, max_bar + pad + 46.f, RAYWHITE);
    DrawRectangleLines(start_x - pad, baseline - max_bar - pad, len + 2*pad, max_bar + pad + 46.f, ColorAlpha(GRAY, 0.5));

    // One bar per decision; height is that decision's share of all robots.
    for (int d=0; d<globals.NUM_DECISIONS; d++){
        float bar_h = (total>0) ? counts[d]/(float)total * max_bar : 0.f;
        int   x = start_x + d*len_1;
        DrawRectangle(x, baseline - bar_h, len_1, bar_h, decisionColors_[d]);
        DrawRectangleLines(x, baseline - max_bar, len_1, max_bar, ColorAlpha(GRAY, 0.4));   // bin outline
        int tw = MeasureText(decisionHudLabels_[d].c_str(), 16);
        DrawText(decisionHudLabels_[d].c_str(), x + 0.5*len_1 - tw/2, label_y, 16, BLACK);
    }

    // "Decision" caption, centred under the bar labels.
    const char* title = "Decision";
    int ttw = MeasureText(title, 18);
    DrawText(title, start_x + len/2 - ttw/2, title_y, 18, BLACK);
}