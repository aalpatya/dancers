/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
// Interactive whiteboard to author binary shape images (standalone, no Simulator).
//
// Draw a shape on a 500x500 whiteboard (or load an image, which is binarised on load), then
// "Save Binary Image" to write a clean black-on-white PNG. Point a config's FORMATION_IMG_FILE at it
// and the simulator turns it into formation points (ShapeFormation::generateFormationPoints). The
// Output-file text box sets the load/save path.
//
// Build & run:   make -C build shape_editor  &&  ./build/helpers/shape_editor
/**************************************************************************************/
#include <raylib.h>
#define RAYGUI_IMPLEMENTATION
#include <raygui.h>

#include <cstring>
#include <string>

// ── Layout constants ─────────────────────────────────────────────────────────
static constexpr int CANVAS_SIZE = 500;        // on-screen whiteboard (px)
static constexpr int OUT_SIZE    = 500;        // saved binary image is OUT_SIZE x OUT_SIZE px
static constexpr int PANEL_W     = 230;
static constexpr int CANVAS_X    = PANEL_W + 20;
static constexpr int CANVAS_Y    = 20;
static constexpr int INIT_W      = CANVAS_X + CANVAS_SIZE + 20;
static constexpr int INIT_H      = CANVAS_Y + CANVAS_SIZE + 90;

// Build a black-shape-on-white-background copy of img: pixels darker than mid-grey become the shape.
static Image binarize(const Image& img){
    Image out = GenImageColor(img.width, img.height, WHITE);
    Color* px = LoadImageColors(img);
    for (int y = 0; y < img.height; y++)
        for (int x = 0; x < img.width; x++){
            const Color& c = px[y*img.width + x];
            if ((c.r + c.g + c.b) / 3 < 128) ImageDrawPixel(&out, x, y, BLACK);
        }
    UnloadImageColors(px);
    return out;
}

static void clearCanvas(RenderTexture2D& canvas){
    BeginTextureMode(canvas);
        ClearBackground(WHITE);
    EndTextureMode();
}

// Load an image from `path`, binarise it, and blit the mask fit/centred/upright onto the canvas.
static bool loadBinarizedToCanvas(RenderTexture2D& canvas, const char* path){
    if (!FileExists(path)) return false;
    Image img = LoadImage(path);
    if (img.width == 0 || img.height == 0){ UnloadImage(img); return false; }

    Image bin = binarize(img);
    UnloadImage(img);

    float s = fminf(CANVAS_SIZE / (float)bin.width, CANVAS_SIZE / (float)bin.height);
    int nw = (bin.width  * s < 1) ? 1 : (int)(bin.width  * s);
    int nh = (bin.height * s < 1) ? 1 : (int)(bin.height * s);
    ImageResize(&bin, nw, nh);
    Texture2D tex = LoadTextureFromImage(bin);
    UnloadImage(bin);

    int ox = (CANVAS_SIZE - nw) / 2, oy = (CANVAS_SIZE - nh) / 2;
    BeginTextureMode(canvas);
        ClearBackground(WHITE);
        // Source height positive: the canvas display below applies the render-texture flip, so this
        // lands upright on screen.
        DrawTexturePro(tex, Rectangle{0, 0, (float)nw, (float)nh},
                       Rectangle{(float)ox, (float)oy, (float)nw, (float)nh}, Vector2{0, 0}, 0.f, WHITE);
    EndTextureMode();
    UnloadTexture(tex);
    return true;
}

// Read the canvas back, downscale to OUT_SIZE, threshold to a clean black/white mask, and export a PNG.
// Returns the number of shape (black) pixels, or -1 on failure.
static int saveBinaryPng(RenderTexture2D& canvas, const char* path){
    Image img = LoadImageFromTexture(canvas.texture);
    ImageFlipVertical(&img);                 // GL framebuffer is bottom-up; flip to screen order
    ImageResize(&img, OUT_SIZE, OUT_SIZE);   // bilinear

    Image out = binarize(img);               // re-threshold so the downscaled result stays binary
    UnloadImage(img);

    Color* px = LoadImageColors(out);
    int n_shape = 0;
    for (int i = 0; i < OUT_SIZE*OUT_SIZE; i++) if (px[i].r < 128) n_shape++;
    UnloadImageColors(px);

    bool ok = ExportImage(out, path);
    UnloadImage(out);
    return ok ? n_shape : -1;
}

// A centred modal over a dimmed screen: a framed box sized to the message, the message drawn in full
// (left-aligned, multi-line ok, so long paths are never clipped), and one or two buttons. Returns the
// clicked button index (0 = btnA, 1 = btnB) or -1 while still open. Pass btnB = nullptr for one button.
static int modalDialog(const Font& font, const char* title, const char* msg, const char* btnA, const char* btnB){
    const float ts = 18, pad = 18, titleH = 28, btnH = 30, btnW = 110;
    int sw = GetScreenWidth(), sh = GetScreenHeight();
    Vector2 m = MeasureTextEx(font, msg, ts, 1.f);
    float bw = fminf((float)(sw - 20), fmaxf(360.f, m.x + 2*pad));
    float bh = titleH + pad + m.y + pad + btnH + pad;
    float bx = (sw - bw)/2, by = (sh - bh)/2;

    DrawRectangle(0, 0, sw, sh, Color{0, 0, 0, 110});
    GuiUnlock();
    GuiPanel(Rectangle{bx, by, bw, bh}, title);
    DrawTextEx(font, msg, Vector2{bx + pad, by + titleH + pad}, ts, 1.f, DARKGRAY);

    int result = -1;
    float yb = by + bh - pad - btnH;
    if (btnB){
        if (GuiButton(Rectangle{bx + bw - pad - btnW, yb, btnW, btnH}, btnB)) result = 1;
        if (GuiButton(Rectangle{bx + bw - 2*(pad + btnW) + pad, yb, btnW, btnH}, btnA)) result = 0;
    } else if (GuiButton(Rectangle{bx + (bw - btnW)/2, yb, btnW, btnH}, btnA)) result = 0;
    return result;
}

int main(){
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    SetTraceLogLevel(LOG_WARNING);
    InitWindow(INIT_W, INIT_H, "Binary Shape Whiteboard");
    SetTargetFPS(60);

    // Load a crisp TTF for the UI if one is available (the built-in raylib font is hard to read).
    Font ui = GetFontDefault();
    bool custom_font = false;
    for (const char* fp : {"/System/Library/Fonts/Supplemental/Arial.ttf",
                           "/System/Library/Fonts/Helvetica.ttc",
                           "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                           "C:/Windows/Fonts/arial.ttf"}){
        if (FileExists(fp)){
            Font f = LoadFontEx(fp, 36, nullptr, 0);
            if (f.texture.id != 0){ ui = f; custom_font = true; SetTextureFilter(f.texture, TEXTURE_FILTER_BILINEAR); GuiSetFont(f); }
            break;
        }
    }
    GuiSetStyle(DEFAULT, TEXT_SIZE, 18);
    GuiSetStyle(DEFAULT, TEXT_SPACING, 1);

    RenderTexture2D canvas = LoadRenderTexture(CANVAS_SIZE, CANVAS_SIZE);
    clearCanvas(canvas);

    // Default output: a binary PNG under the shape-image folder (PROJECT_ROOT is baked in by CMake).
    char outfile[512] = PROJECT_ROOT "/assets/imgs/shapes/drawn_shape.png";

    float brush = 6.0f;
    bool  draw_on = false;
    bool  outfile_edit = false;
    bool  have_prev = false;
    Vector2 prev{};
    std::string status = "Draw a shape or load an image, then Save Binary Image.";
    std::string popup;            // non-empty -> show the "saved" confirmation overlay
    bool confirm = false;         // true -> show the "overwrite existing file?" prompt

    // Write the canvas to `outfile` and report the result in the status line / "saved" popup.
    auto doSave = [&]{
        int n = saveBinaryPng(canvas, outfile);
        if (n < 0) status = "Error: could not save image.";
        else { status = "Saved binary image."; popup = TextFormat("Saved %dx%d binary image (%d shape px) to:\n\n%s", OUT_SIZE, OUT_SIZE, n, outfile); }
    };

    while (!WindowShouldClose()){
        Vector2 mouse = GetMousePosition();
        float b = (brush < 1.f) ? 1.f : brush;

        // ── Painting onto the canvas ─────────────────────────────────────────
        bool in_canvas = (mouse.x >= CANVAS_X && mouse.x <= CANVAS_X + CANVAS_SIZE &&
                          mouse.y >= CANVAS_Y && mouse.y <= CANVAS_Y + CANVAS_SIZE);
        if (draw_on && in_canvas && popup.empty() && !confirm && IsMouseButtonDown(MOUSE_BUTTON_LEFT)){
            Vector2 c{mouse.x - CANVAS_X, mouse.y - CANVAS_Y};
            BeginTextureMode(canvas);
                if (have_prev) DrawLineEx(prev, c, b*2, BLACK);
                DrawCircleV(c, b, BLACK);
            EndTextureMode();
            prev = c; have_prev = true;
        } else {
            have_prev = false;
        }

        // ── Render ───────────────────────────────────────────────────────────
        BeginDrawing();
        ClearBackground(Color{235, 235, 235, 255});
        if (!popup.empty() || confirm) GuiLock();   // freeze the panel under an overlay

        GuiPanel(Rectangle{10, 10, (float)PANEL_W, (float)(INIT_H - 70)}, "Whiteboard");
        float x = 20, w = PANEL_W - 20, y = 40;

        GuiLabel(Rectangle{x, y, w, 18}, TextFormat("Brush size: %d", (int)b)); y += 20;
        GuiSlider(Rectangle{x + 5, y, w - 10, 18}, "", "", &brush, 1.0f, 40.0f); y += 32;

        GuiToggle(Rectangle{x, y, w, 28}, draw_on ? "Draw: ON" : "Draw: OFF", &draw_on); y += 38;

        if (GuiButton(Rectangle{x, y, w, 28}, "Load image (from path)")){
            if (loadBinarizedToCanvas(canvas, outfile)) status = std::string("Loaded & binarised ") + GetFileName(outfile);
            else                                        status = "Failed to load image at output path.";
        }
        y += 34;

        if (GuiButton(Rectangle{x, y, w, 28}, "Clear canvas")){ clearCanvas(canvas); status = "Canvas cleared."; }
        y += 40;

        if (GuiButton(Rectangle{x, y, w, 34}, "Save Binary Image")){
            if (FileExists(outfile)) confirm = true;   // ask before overwriting
            else                     doSave();
        }
        y += 40;

        if (GuiButton(Rectangle{x, y, w, 28}, "Exit")) break;

        // ── Canvas ───────────────────────────────────────────────────────────
        DrawRectangleLines(CANVAS_X - 1, CANVAS_Y - 1, CANVAS_SIZE + 2, CANVAS_SIZE + 2, DARKGRAY);
        DrawTexturePro(canvas.texture, Rectangle{0, 0, (float)CANVAS_SIZE, -(float)CANVAS_SIZE},
                       Rectangle{(float)CANVAS_X, (float)CANVAS_Y, (float)CANVAS_SIZE, (float)CANVAS_SIZE},
                       Vector2{0, 0}, 0.f, WHITE);

        // ── Output path + status along the bottom ────────────────────────────
        int sh = GetScreenHeight(), sw = GetScreenWidth();
        GuiLabel(Rectangle{15, (float)(sh - 58), 90, 22}, "Output file:");
        if (GuiTextBox(Rectangle{105, (float)(sh - 60), (float)(sw - 120), 26}, outfile, 512, outfile_edit))
            outfile_edit = !outfile_edit;
        if (custom_font) DrawTextEx(ui, status.c_str(), Vector2{15, (float)(sh - 28)}, 18, 1, DARKBLUE);
        else             DrawText(status.c_str(), 15, sh - 26, 16, DARKBLUE);

        // ── Overwrite prompt ─────────────────────────────────────────────────
        if (confirm){
            int r = modalDialog(ui, "File exists", TextFormat("This file already exists:\n%s\n\nReplace it?", outfile), "Cancel", "Replace");
            if (r == 1){ confirm = false; doSave(); }
            else if (r == 0){ confirm = false; status = "Save cancelled."; }
        }

        // ── "Saved" confirmation overlay ─────────────────────────────────────
        if (!popup.empty()){
            if (modalDialog(ui, "Image saved", popup.c_str(), "OK", nullptr) == 0) popup.clear();
        }

        EndDrawing();
    }

    UnloadRenderTexture(canvas);
    if (custom_font) UnloadFont(ui);
    CloseWindow();
    return 0;
}
