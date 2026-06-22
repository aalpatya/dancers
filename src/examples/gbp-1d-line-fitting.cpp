/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
// Interactive 1D line fitting with the GBP library (see gbp-two-layer-factorgraph.cpp for the basics first).
//
// A chain of N height variables (LineVariable, each in R^1) is tied together by smoothness factors; each
// data point adds a DataFactor pulling the chain toward it. The nodes start at the bottom and rise as
// GBP passes messages. LineVariable and DataFactor each draw themselves, so main() just builds the graph
// (add variables, add factors), runs GBP, and calls those draws.
//
// Build & run:   make -C build gbp-1d-line-fitting  &&  ./build/examples/gbp-1d-line-fitting
/**************************************************************************************/
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>
#include <functional>
#include <random>
#include <set>

#include <raylib.h>
#define RAYGUI_IMPLEMENTATION
#include <raygui.h>

#include <gbp/FactorGraph.h>
#include <gbp/Variable.h>
#include <gbp/Factor.h>
#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>   // the browser drives the frame loop (no blocking while-loop on web)
#endif

// ---- model + canvas constants, and the value <-> screen mappings used by the draws below ----
const int    N = 40;                  // number of nodes in the fitted chain
const double SIGMA_SMOOTH = 1.0;      // smoothness factor strength (between neighbouring nodes)
const double SIGMA_DATA   = 1.0;      // measurement factor strength (data point -> chain)
const double ROBUST_C     = 1.0;      // robust-kernel threshold, in Mahalanobis distance
const double YMIN = -10, YMAX = 10;   // value range a node can take
const int    SCREEN = 1000, HEIGHT = 640;
const float  NODE_R = 13.f;            // on-screen radius of a chain-node dot (drawn size AND click target)
const float  X0 = 80, Y0 = 100, W = SCREEN - 2*X0, H = HEIGHT - Y0 - 50;
static float  toScreenX(float x){ return X0 + x/(N-1)*W; }
static float  toScreenY(double y){ return Y0 + (float)((YMAX - y)/(YMAX - YMIN))*H; }
static float  toNodeX(float sx){ return std::clamp((sx - X0)/W*(N-1), 0.f, (float)(N-1)); }
static double toValueY(float sy){ return std::clamp((double)(YMAX - (sy - Y0)/H*(YMAX - YMIN)), YMIN, YMAX); }

// ---- factor-graph entities (each draws itself) ----

// A chain node: a height in R^1. Draws the fitted-line segment to the next node, its +/-1 std bar, and
// its mean as a dot.
class LineVariable : public Variable {
public:
    using Variable::Variable;
    int       idx_  = 0;      // position along the chain (also its screen column)
    LineVariable* next_ = nullptr; // next node in the chain, for the fitted line (nullptr at the end)
    void draw(Color c, float radius = NODE_R, bool highlight = false){
        double mu = belief_.mu.coeffs()(0);
        double sd = valid_ ? std::sqrt(std::max(0.0, belief_.lambda.inverse()(0,0))) : (YMAX - YMIN);
        if (next_)
            DrawLineEx(Vector2{toScreenX(idx_), toScreenY(mu)},
                       Vector2{toScreenX(next_->idx_), toScreenY(next_->belief_.mu.coeffs()(0))}, 2.5f, c);
        DrawLineEx(Vector2{toScreenX(idx_), toScreenY(std::clamp(mu+sd, YMIN, YMAX))},
                   Vector2{toScreenX(idx_), toScreenY(std::clamp(mu-sd, YMIN, YMAX))}, 3.f, ColorAlpha(c, 0.22f));
        Vector2 ctr{toScreenX(idx_), toScreenY(mu)};
        if (highlight){                                      // hovered node: halo + ring to read as clickable
            DrawCircleV(ctr, radius + 6, ColorAlpha(c, 0.25f));
            DrawCircleLines((int)ctr.x, (int)ctr.y, radius + 6, c);
        }
        DrawCircleV(ctr, radius, c);
    }
};

// A data measurement at fractional position x_ between nodes floor(x_) and floor(x_)+1: it pulls the
// chain toward value z, with residual = (1-gamma)*x_left + gamma*x_right - z. Draws as a red dot.
class DataFactor : public Factor {
public:
    float  x_;        // fractional node position (screen column)
    double gamma_;    // interpolation weight on the right node
    DataFactor(Key key, std::vector<Key> connected, const std::string& group,
               const Eigen::VectorXd& z, Eigen::VectorXd sigma, float x, double gamma)
        : Factor{key, connected, group, z, sigma}, x_(x), gamma_(gamma) {}
    std::pair<Eigen::VectorXd, Eigen::MatrixXd> computeResidualAndJacobian(const std::vector<LieGroup>& X) override {
        double xl = X[0].coeffs()(0), xr = X[1].coeffs()(0);   // the two node heights
        Eigen::VectorXd r(1); r << (1 - gamma_)*xl + gamma_*xr - z_(0);
        Eigen::MatrixXd J(1, 2); J << (1 - gamma_), gamma_;
        return {r, J};
    }
    void draw(){ DrawCircleV(Vector2{toScreenX(x_), toScreenY(z_(0))}, 5.f, RED); }
};

static RobustKernel kernelOf(int k){ return k==1 ? RobustKernel::Huber : k==2 ? RobustKernel::DCS : RobustKernel::None; }

int main(){
    // Synchronous GBP oscillates on dense/random data; damping (blend each new message with the last)
    // settles it. (The gaussianBP.github.io demo uses message dropout for the same purpose.)
    gbp::config().damping = 0.5f;
    gbp::config().robust_threshold = ROBUST_C;   // shared by every robust factor

    // State is `static` so it survives Emscripten unwinding main()'s stack when it hands the frame
    // callback to the browser. On desktop this is identical (main runs once anyway).
    static const LieGroup R1 = Eigen::VectorXd::Zero(1);   // each node is a height in R^1
    static FactorGraph fg(0);
    static int  layer_id = 0;
    static std::vector<std::shared_ptr<LineVariable>> nodes;
    static std::vector<std::shared_ptr<DataFactor>> data;
    static long iters = 0;

    static RobustKernel kernel = RobustKernel::None;
    static auto applyKernel = [&]{ for (auto& [k, f] : fg.layer(layer_id).factors_) f->setRobust(kernel); };
    // Prime: one variable sweep so every node sends its prior message to its factors. Beliefs don't move
    // yet, but the outgoing messages now carry the prior precision, so factors aren't singular on the
    // first update.
    static auto prime = [&]{ fg.layer(layer_id).variableIteration(); };

    // Rebuild from scratch: N nodes at the bottom joined by smoothness factors, no data points. The weak
    // bottom prior (large sigma) barely influences the fit but keeps every belief well-posed, and is what
    // starts the nodes at the bottom before any messages arrive.
    static auto reset = [&]{
        fg = FactorGraph(0);
        layer_id = fg.addLayer("1DLineFittingLayer")->lid_;
        nodes.clear(); data.clear(); iters = 0;
        Eigen::VectorXd bottom = Eigen::VectorXd::Constant(1, YMIN);
        for (int i = 0; i < N; i++){
            auto n = fg.addVariable<LineVariable>(layer_id, LieGroup(bottom), Eigen::VectorXd{{50.0}});
            n->idx_ = i;
            nodes.push_back(n);
        }
        for (int i = 0; i + 1 < N; i++){
            nodes[i]->next_ = nodes[i+1].get();
            fg.addFactor<SmoothnessFactor>(layer_id, {nodes[i]->key_, nodes[i+1]->key_}, "R1",
                                           Eigen::VectorXd::Zero(1), Eigen::VectorXd::Constant(1, SIGMA_SMOOTH));
        }
        applyKernel();
        prime();
    };
    // Add a data factor at fractional node position x, pulling the chain toward value y.
    static auto addPoint = [&](float x, double y){
        int i = std::clamp((int)std::floor(x), 0, N-2);
        auto f = fg.addFactor<DataFactor>(layer_id, {nodes[i]->key_, nodes[i+1]->key_}, "R1",
                     Eigen::VectorXd::Constant(1, y), Eigen::VectorXd::Constant(1, SIGMA_DATA), x, x - i);
        f->setRobust(kernel);
        data.push_back(f);
    };
    static auto addRow = [&](const std::function<double(float)>& fn){
        for (float x = 1; x <= N-2; x += 1) addPoint(x, std::clamp(fn(x), YMIN, YMAX));
        prime();
    };
    // One asynchronous GBP update centred on node i (matches the gaussianBP.github.io demo): each
    // neighbour sharing a factor with node i re-gathers ALL its factors - so it feels both node i's
    // belief and its own data - then updates. Node i itself is not updated, so its belief pushes outward.
    static auto asyncFromNode = [&](int i){
        FactorGraphLayer& L = fg.layer(layer_id);
        std::set<Key> neighbours;
        for (const Key& fk : nodes[i]->connected_f_keys_)
            for (const Key& vk : fg.getFac(fk)->connected_v_keys_)
                if (vk != nodes[i]->key_) neighbours.insert(vk);
        for (const Key& vk : neighbours){
            for (const Key& fk : fg.getVar(vk)->connected_f_keys_) L.factorIteration(fk);
            L.variableIteration(vk);
        }
        iters++;
    };

    reset();

    static bool running = false;     // synchronous GBP iterating?
    static bool asyncMode = false;   // asynchronous mode: click a node to update its neighbourhood
    static int  kernelIdx = 0;       // dropdown index: 0 None, 1 Huber, 2 DCS
    static bool kernelEdit = false;  // dropdown open?
    static int  held = -1;           // index of the data point being dragged, or -1
    static std::mt19937 rng(1);

    SetTraceLogLevel(LOG_WARNING);
    InitWindow(SCREEN, HEIGHT, "DANCeRS - 1D line fitting");
    SetTargetFPS(60);

    static Rectangle bRun{X0, 14, 185, 26}, bStepGBP{X0+191, 14, 30, 26}, bAsync{X0+227, 14, 170, 26}, bKernel{X0+W-150, 14, 150, 26};
    static Rectangle bOutlier{X0+180, 56, 84, 26}, bStep{X0+272, 56, 64, 26}, bRandom{X0+344, 56, 86, 26}, bClear{X0+438, 56, 64, 26};

    static std::function<void()> frame = [&](){
        Vector2 mouse = GetMousePosition();

        // ---- keyboard ----
        if (IsKeyPressed(KEY_S)){ fg.optimiseGBP(1); iters++; }                 // single sweep
        if (IsKeyPressed(KEY_R)){ kernelIdx = (kernelIdx+1)%3; kernel = kernelOf(kernelIdx); applyKernel(); }

        // ---- mouse (locked out while the dropdown is open) ----
        bool in_plot = !kernelEdit && mouse.x >= X0 && mouse.x <= X0+W && mouse.y >= Y0 && mouse.y <= Y0+H;
        // Async mode: a click acts only ON a node, so find the one under the cursor (within a grab radius).
        int hoverNode = -1;
        if (asyncMode && in_plot){
            // Circles overlap (nodes ~21px apart, radius 13), so pick the NEAREST node within the disk,
            // not the first one found - otherwise overlap zones always snap to the left neighbour.
            float best = NODE_R*NODE_R;          // click target matches the drawn node size
            for (int i = 0; i < N; i++){
                float dx = toScreenX(nodes[i]->idx_) - mouse.x;
                float dy = toScreenY(nodes[i]->belief_.mu.coeffs()(0)) - mouse.y;
                float d2 = dx*dx + dy*dy;
                if (d2 < best){ best = d2; hoverNode = i; }
            }
        }
        SetMouseCursor(hoverNode >= 0 ? MOUSE_CURSOR_POINTING_HAND : MOUSE_CURSOR_DEFAULT);
        if (asyncMode){
            // Async mode drives GBP only: clicking ON a node updates its neighbourhood. (Data from presets.)
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && hoverNode >= 0)
                asyncFromNode(hoverNode);
        } else if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && in_plot){
            held = -1;                                           // grab a nearby point, else add a new one
            for (int i = 0; i < (int)data.size(); i++){
                float dx = toScreenX(data[i]->x_) - mouse.x, dy = toScreenY(data[i]->z_(0)) - mouse.y;
                if (dx*dx + dy*dy < 12*12){ held = i; break; }
            }
            if (held < 0){ addPoint(toNodeX(mouse.x), toValueY(mouse.y)); held = (int)data.size() - 1; }
        }
        if (!asyncMode && IsMouseButtonDown(MOUSE_BUTTON_LEFT) && held >= 0) data[held]->z_(0) = toValueY(mouse.y);
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) held = -1;

        if (running){ fg.optimiseGBP(1); iters++; }              // one sweep a frame while running (sync)

        BeginDrawing();
        ClearBackground(RAYWHITE);

        // ---- GBP controls (sync run + step, or async click-stepping; the two modes are exclusive) ----
        bool wasRunning = running;
        GuiToggle(bRun, "Run Synchronous GBP", &running);
        if (running && !wasRunning) asyncMode = false;
        if (GuiButton(bStepGBP, "#134#")){ fg.optimiseGBP(1); iters++; }
        GuiToggle(bAsync, "Asynchronous GBP", &asyncMode);
        if (asyncMode) running = false;
        DrawText(TextFormat("iteration: %ld", iters), (int)(bAsync.x + bAsync.width + 14), 19, 18, DARKGRAY);
        DrawText("Robust Kernel:", (int)(bKernel.x - 130), 19, 18, DARKGRAY);

        // ---- "Set Data Factors" presets (each rebuilds the chain, GBP left paused) ----
        DrawText("Set Data Factors:", (int)X0, 60, 18, DARKGRAY);
        if (GuiButton(bOutlier, "Outlier")){ reset(); addRow([](float x){ return -3.0 + 0.22*x; }); addPoint((N-1)/2.f, 9.0); running = false; }
        if (GuiButton(bStep,    "Step"))   { reset(); addRow([](float x){ return x < (N-1)/2.f ? -4.0 : 4.0; }); running = false; }
        if (GuiButton(bRandom,  "Random")) { reset(); std::uniform_real_distribution<double> u(-7,7); addRow([&](float){ return u(rng); }); running = false; }
        if (GuiButton(bClear,   "Clear"))  { reset(); running = false; }

        // ---- plot: frame + zero line, then let each entity draw itself ----
        DrawRectangleLines((int)X0, (int)Y0, (int)W, (int)H, LIGHTGRAY);
        DrawLine((int)X0, (int)toScreenY(0), (int)(X0+W), (int)toScreenY(0), LIGHTGRAY);
        Color line = kernelIdx==0 ? BLUE : kernelIdx==1 ? DARKGREEN : ORANGE;
        for (auto& n : nodes)                   // fitted line + uncertainty bars + node means
            n->draw(line, NODE_R, n->idx_ == hoverNode);   // hover halo highlights the node in async mode
        for (auto& d : data)  d->draw();        // data points

        const char* hint = asyncMode
            ? "async mode - click a variable node to push its belief to its neighbours    (set data factors with the presets)"
            : "left-click: set a data factor    drag: move it    S: single sweep";
        DrawText(hint, (int)X0, (int)(Y0+H+14), 16, GRAY);

        // dropdown last so its open list draws on top of everything
        int prev = kernelIdx;
        if (GuiDropdownBox(bKernel, "None;Huber;DCS", &kernelIdx, kernelEdit)) kernelEdit = !kernelEdit;
        if (kernelIdx != prev){ kernel = kernelOf(kernelIdx); applyKernel(); }
        EndDrawing();
    };

#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop([]{ frame(); }, 0, 1);   // browser calls frame() each animation tick
#else
    while (!WindowShouldClose()) frame();
    CloseWindow();
#endif
    return 0;
}
