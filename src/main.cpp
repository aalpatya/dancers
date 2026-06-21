/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)

// Define all parameters in the appropriate config file (default: config/joint_consensus_shape_formation.yaml)
/**************************************************************************************/
#include <iostream>
#include <cstdlib>
#include <fstream>
#ifdef __linux__
#include <unistd.h>   // execv / setenv, used for the OpenMP re-exec below (Linux only)
#endif
#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>   // browser build: drive the loop via requestAnimationFrame
#endif
#include <Utils.h>

#include <DArgs.h>

#include <Globals.h>
#include <Simulator.h>

#include <manif/SE2.h>
#include <manif/SO2.h>
#define SUPPORT_SCREEN_CAPTURE
Globals globals;

// One simulation+render frame. The browser build calls this via emscripten_set_main_loop (it can't run
// an infinite while-loop); the desktop build calls it from the while-loop in main().
static void doFrame(Simulator* sim){
    sim->eventHandler();                // Capture keypresses or mouse events
    sim->createOrDeleteRobots();
    sim->timestep();
    sim->draw();
#ifndef __EMSCRIPTEN__
    sim->logResults();                 // results logging writes to disk - skip in the browser
#endif
}

// Hand the gbp library the few defaults it reads (it doesn't see `globals`).
static void syncGbpConfig(){
    gbp::config().damping = globals.DAMPING;
}

#ifndef __EMSCRIPTEN__
// Consume a restart request from the controls panel (desktop): rebuild the Simulator with the chosen
// config + staged parameter edits, keeping the same window/GL context. On the web a restart is a page
// reload instead (Simulator::requestRestart), so this is desktop-only.
static Simulator* applyRestartIfRequested(Simulator* sim){
    if (!globals.restart_requested_) return sim;
    // Capture the request before we reset globals.
    std::string cfg = globals.pending_config_.empty() ? globals.CONFIG_FILE
                                                       : makePath({PROJECT_ROOT, globals.pending_config_});
    std::string overrides = globals.pending_overrides_;

    delete sim;                          // tear down robots + graphics (GL); the window stays open
    globals = Globals{};                 // reset to defaults so keys absent from the new config don't leak across
    globals.CONFIG_FILE = cfg;
    std::ifstream f(globals.CONFIG_FILE);
    globals.parseGlobalArgs(f);
    if (!overrides.empty()) globals.applyConfigOverrides(overrides);
    globals.postParsing();
    srand((unsigned)globals.RNG_SEED);
    syncGbpConfig();
    globals.RUN = true;
    if (globals.DISPLAY) SetWindowSize(Simulator::PANEL_W + globals.SCREEN_SZ, globals.SCREEN_SZ);
    return new Simulator();
}
#endif

int main(int argc, char *argv[]){

    // Make idle OpenMP threads sleep instead of spin-waiting (they otherwise peg all cores). The
    // re-exec relies on /proc/self/exe + execv, which only exist on Linux.
#ifdef __linux__
    if (!getenv("OMP_WAIT_POLICY")) {
        setenv("OMP_WAIT_POLICY", "passive", 1);
        execv("/proc/self/exe", argv);
    }
#endif

    DArgs::DArgs dargs(argc, argv);                             // Parse config file argument --cfg <file.yaml>
    if (globals.parseGlobalArgs(dargs)) return EXIT_FAILURE;
    srand((unsigned)globals.RNG_SEED);
    syncGbpConfig();

    // Create the window here (not in the Simulator) so a config switch can rebuild the Simulator without
    // recreating the GL context. The world is a SCREEN_SZ square; the controls panel adds PANEL_W on the left.
    if (globals.DISPLAY){
        SetTraceLogLevel(LOG_ERROR);
        // HighDPI on both so the UI/blit are crisp on a retina display. The expensive part - the lit 3D
        // world - is supersampled separately and capped on web (see world_rt_ in Graphics.cpp).
        unsigned int flags = FLAG_MSAA_4X_HINT | FLAG_WINDOW_HIGHDPI;
#ifndef __EMSCRIPTEN__
        flags |= FLAG_VSYNC_HINT;   // desktop: vsync to the display
#endif
        SetConfigFlags(flags);
        InitWindow(Simulator::PANEL_W + globals.SCREEN_SZ, globals.SCREEN_SZ, globals.WINDOW_TITLE);
#ifndef __EMSCRIPTEN__
        SetTargetFPS(60);
#endif
        // On web both VSYNC and SetTargetFPS are skipped: the emscripten main loop (fps=0 ->
        // requestAnimationFrame, see below) already paces at the display rate. Either one would add a
        // blocking wait in EndDrawing that overruns the rAF budget and halves the rate to ~30fps.
    }

    Simulator* sim = new Simulator();       // Initialise the simulator
    globals.RUN = true;

#ifdef __EMSCRIPTEN__
    // The browser owns the event loop: hand it a per-frame callback. fps=0 paces it with
    // requestAnimationFrame (the display rate). simulate_infinite_loop=1 keeps the runtime (and `sim`) alive.
    emscripten_set_main_loop_arg([](void* s){ doFrame(static_cast<Simulator*>(s)); }, sim, 0, 1);
#else
    while (globals.RUN){
        doFrame(sim);
        sim = applyRestartIfRequested(sim);   // rebuild in-place if the panel asked for a config/param change
    }
    delete sim;
    if (globals.DISPLAY) CloseWindow();
#endif

    return 0;
}
