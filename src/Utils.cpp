/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <filesystem>
#include <Utils.h>
#include <Globals.h>
extern Globals globals;

/**************************************************************************************/
// Cross-platform filepath builder (see Utils.h for the documented contract).
// Joins the parts with std::filesystem's path operator (which understands '/' on
// every platform), then make_preferred() rewrites separators to the host's native
// one ('\' on Windows, '/' elsewhere). This is why nothing else in the codebase
// needs to hand-concatenate path separators.
/**************************************************************************************/
std::string makePath(const std::vector<std::string>& parts, bool make_absolute){
    std::filesystem::path p;
    for (const auto& part : parts){
        if (part.empty()) continue;     // skip empties so {"", "a"} -> "a", not "/a"
        p /= std::filesystem::path(part);
    }
    if (make_absolute){
        std::error_code ec;
        std::filesystem::path abs = std::filesystem::absolute(p, ec);
        if (!ec) p = abs;               // on failure, fall back to the relative path
    }
    return p.make_preferred().string();
}

/**************************************************************************************/
// Help overlay (toggled with H). The live status (FPS / timestep) now lives in the controls panel;
// this is just the modal key/mouse reference, centred on the window.
/**************************************************************************************/
void drawInfo(uint32_t time_cnt){
    if (globals.SIM_MODE != SimMode::Help) return;

    const int w = 500, h = 360;
    const int x = GetScreenWidth()/2 - w/2, y = GetScreenHeight()/2 - h/2;
    DrawRectangle(x, y, w, h, Fade(SKYBLUE, 0.7f));
    DrawRectangleLines(x, y, w, h, BLACK);

    std::vector<std::string> texts{
        "Esc : \t\t\t\t Exit Simulation",
        "H : \t\t\t\t\t\t Close Help",
        "SPACE : \t Camera Transition",
        "",
        "Mouse Wheel Scroll : Zoom",
        "Mouse Wheel Drag : Pan",
        "Mouse Wheel Drag + SHIFT : Rotate",
    };
    const int offset = 40;
    for (size_t t=0; t<texts.size(); t++)
        DrawText(texts[t].c_str(), x + 20, y + (int)(t+1)*offset, 20, BLACK);
}


