/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
// Interactive SE(2) localisation with GBP - the GBP counterpart of manif's se2_sam.cpp.
//
// You drive a ground-truth robot around; GBP estimates its pose from noisy odometry and landmark
// observations. The factor graph mixes groups in one layer: the robot path is a chain of SE2 pose
// variables, the landmarks are R2 variables.
//
//   - left click            add a landmark (R2) at the cursor
//   - up arrow              drive the true robot forward
//   - left / right arrows   turn the true robot
//
// Every KEYFRAME_EVERY frames a new pose variable is added, tied to the previous one by an odometry
// factor (the true relative motion plus Gaussian noise). Whenever the true robot is within OBSERVE_RADIUS
// of a landmark, a LandmarkFactor measures the landmark's position in the robot frame (true value plus
// Gaussian noise). The estimated pose and its position covariance are drawn each frame.
//
// Build & run:  make -C build gbp-se2-localisation && ./build/examples/gbp-se2-localisation
/**************************************************************************************/
#include <random>
#include <vector>
#include <cmath>

#include <gbp/FactorGraph.h>
#include <gbp/Variable.h>
#include <gbp/Factor.h>

#include <Eigen/Dense>
#include <raylib.h>
#include <functional>
#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>   // the browser drives the frame loop (no blocking while-loop on web)
#endif

// ---- World <-> screen ------------------------------------------------------------------------------
constexpr int   SCREEN_W = 1100, SCREEN_H = 750;
constexpr float SCALE = 45.f;                       // pixels per metre
constexpr float CX = SCREEN_W / 2.f, CY = SCREEN_H / 2.f;

static Vector2 worldToScreen(const Eigen::Vector2d& p) {
    return Vector2{ CX + (float)(p.x() * SCALE), CY - (float)(p.y() * SCALE) };
}
static Eigen::Vector2d screenToWorld(Vector2 s) {
    return Eigen::Vector2d((s.x - CX) / SCALE, -(s.y - CY) / SCALE);
}
// wrapAngle (to (-pi, pi]) comes from Utils.h.

// ---- Factors ---------------------------------------------------------------------------------------
// Both factors use EXACT analytic Jacobians, not finite differences. They come from manif's convention
// of passing an optional output-Jacobian argument into each group operation, e.g.
//     y = X.inverse(J_y_X);            // J_y_X = d(X^-1)/d(X)
//     q = X.act(p, J_q_X, J_q_p);      // J_q_X = d(q)/d(X),  J_q_p = d(q)/d(p)
//     rel = a.between(b, J_rel_a, J_rel_b);
//     tau = a.ominus(b, J_tau_a, J_tau_b);
// each Jacobian being the derivative of that operation w.r.t. its input, in the right-tangent space.
// We build a factor's residual by composing these primitives and chain-rule their Jacobians (matrix
// products) to get d(residual)/d(each variable). The LieGroup wrapper (inc/gbp/LieGroups.h) forwards
// these calls straight to manif.

// Odometry: a measured relative motion Z between two SE2 poses. residual = (Xprev^-1 Xcurr) (-) Z.
class OdometryFactor : public Factor {
public:
    LieGroup Z_;
    OdometryFactor(Key key, std::vector<Key> vars, const LieGroup& z, const Eigen::VectorXd& sigma)
        : Factor(key, vars, "SE2", Eigen::VectorXd::Zero(3), sigma), Z_(z) {}

    std::pair<Eigen::VectorXd, Eigen::MatrixXd> computeResidualAndJacobian(const std::vector<LieGroup>& X) override {
        // Xprev is added before Xcurr, so the factor's Key order is {prev, curr}.
        Eigen::MatrixXd J_rel_prev, J_rel_curr, J_r_rel, J_r_Z;
        LieGroup rel = X[0].between(X[1], J_rel_prev, J_rel_curr);   // rel = Xprev^-1 Xcurr
        Eigen::VectorXd r = rel.ominus(Z_, J_r_rel, J_r_Z);          // r = rel (-) Z
        Eigen::MatrixXd J(3, 6);
        J << J_r_rel * J_rel_prev, J_r_rel * J_rel_curr;            // chain through `between`
        return { r, J };
    }
};

// Landmark observation: an SE2 pose measures the position of an R2 landmark in its own frame.
// residual = pose^-1 . landmark - z. This is the manif se2_localization measurement - a plain Cartesian
// residual with no angle wrap and no 1/range term, so it stays well-conditioned even when the pose
// estimate has drifted a long way before the landmark is first seen.
class LandmarkFactor : public Factor {
public:
    LandmarkFactor(Key key, std::vector<Key> vars, const Eigen::VectorXd& z, const Eigen::VectorXd& sigma)
        : Factor(key, vars, std::vector<std::string>{"SE2", "R2"}, z, sigma) {}

    // The landmark's position in the pose's frame.
    static Eigen::Vector2d predict(const LieGroup& pose, const LieGroup& landmark) {
        Eigen::MatrixXd Js, Jp;
        return pose.inverse().act(landmark.coeffs(), Js, Jp);
    }

    std::pair<Eigen::VectorXd, Eigen::MatrixXd> computeResidualAndJacobian(const std::vector<LieGroup>& X) override {
        // X arrives in the factor's Key order, which is not necessarily {pose, landmark}, so pick them
        // out by group: the SE2 pose has 3 dofs, the R2 landmark has 2.
        int ip = (X[0].dof() == 3) ? 0 : 1, il = 1 - ip;
        const LieGroup& pose = X[ip];
        const LieGroup& landmark = X[il];

        // q = pose^-1 . landmark, chaining the inverse and action Jacobians (manif style):
        //   xi = pose^-1 (J_xi_pose),  q = xi . landmark (J_q_xi),  d(q)/d(pose) = J_q_xi * J_xi_pose.
        Eigen::MatrixXd J_xi_pose, J_q_xi, J_q_point;
        LieGroup xi = pose.inverse(J_xi_pose);
        Eigen::VectorXd q = xi.act(landmark.coeffs(), J_q_xi, J_q_point);   // landmark in robot frame

        Eigen::MatrixXd blocks[2];
        blocks[ip] = J_q_xi * J_xi_pose;     // 2x3, d(q)/d(pose)
        blocks[il] = J_q_point;              // 2x2, d(q)/d(landmark)
        Eigen::MatrixXd J(2, 5);
        J << blocks[0], blocks[1];           // columns in X order

        return { Eigen::VectorXd(q - z_), J };
    }
};

// ---- Drawing ---------------------------------------------------------------------------------------
static void drawRobot(const LieGroup& pose, float radius, Color body, Color heading, bool filled) {
    Eigen::Vector3d c = pose.coeffs();   // [x, y, theta]
    Eigen::Vector2d p(c(0), c(1));
    Vector2 centre = worldToScreen(p);
    if (filled) DrawCircleV(centre, radius, body);
    else        DrawCircleLinesV(centre, radius, body);
    Eigen::Vector2d nose = p + 0.45 * Eigen::Vector2d(std::cos(c(2)), std::sin(c(2)));
    DrawLineEx(centre, worldToScreen(nose), 2.f, heading);
}

// 2-sigma position-covariance ellipse, from the 2x2 position block of a pose belief.
static void drawCovEllipse(const Eigen::Vector2d& mean, Eigen::Matrix2d cov, Color col) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
    if (es.info() != Eigen::Success) return;
    Eigen::Vector2d axis = es.eigenvalues().cwiseMax(0.0).cwiseSqrt().cwiseMin(20.0);   // clamp when unconstrained
    Eigen::Matrix2d V = es.eigenvectors();
    Vector2 prev{};
    const int N = 48;
    for (int i = 0; i <= N; ++i) {
        double t = 2.0 * PI * i / N;
        Eigen::Vector2d e = mean + 2.0 * (V.col(0) * axis(0) * std::cos(t) + V.col(1) * axis(1) * std::sin(t));
        Vector2 s = worldToScreen(e);
        if (i > 0) DrawLineV(prev, s, col);
        prev = s;
    }
}

static void drawGrid() {
    for (int gx = -12; gx <= 12; ++gx)
        DrawLineV(worldToScreen({(double)gx, -8}), worldToScreen({(double)gx, 8}), Fade(LIGHTGRAY, 0.5f));
    for (int gy = -8; gy <= 8; ++gy)
        DrawLineV(worldToScreen({-12, (double)gy}), worldToScreen({12, (double)gy}), Fade(LIGHTGRAY, 0.5f));
}

// A flat button (drawn with a hover highlight). The caller does the click hit-test against the same rect.
static void drawButton(Rectangle r, const char* label) {
    bool hover = CheckCollisionPointRec(GetMousePosition(), r);
    DrawRectangleRec(r, hover ? Fade(SKYBLUE, 0.7f) : Fade(LIGHTGRAY, 0.7f));
    DrawRectangleLinesEx(r, 1.f, GRAY);
    int fs = 16, tw = MeasureText(label, fs);
    DrawText(label, (int)(r.x + (r.width - tw) / 2.f), (int)(r.y + (r.height - fs) / 2.f), fs, DARKGRAY);
}

// One legend row: a swatch (filled circle, or a ring) and its label.
static void legendEntry(float x, float y, Color c, bool filled, const char* label) {
    if (filled) DrawCircleV({ x, y }, 7.f, c);
    else        DrawCircleLinesV({ x, y }, 8.f, c);
    DrawText(label, (int)x + 18, (int)y - 8, 16, DARKGRAY);
}

// ---- Tuning ----------------------------------------------------------------------------------------
constexpr int    GBP_ITERS      = 10;     // GBP sweeps per frame
constexpr double FORWARD_STEP   = 0.06;   // metres per frame, up arrow
constexpr double TURN_STEP      = 0.03;   // radians per frame, left/right arrows
constexpr double KEYFRAME_DIST  = 0.5;    // new keyframe after this much translation ...
constexpr double KEYFRAME_ANGLE = 0.3;    // ... or this much rotation
constexpr double OBSERVE_RADIUS = 4.0;    // landmarks within this range are observed
constexpr double HALF_PI        = 1.5707963267948966;   // a 90-degree turn (the rotate buttons)

// On-screen buttons for discrete moves (screen-space rects, hit-tested against the mouse).
const Rectangle BTN_FORWARD = { 12, 62, 110, 30 };
const Rectangle BTN_CCW     = { 130, 62, 150, 30 };
const Rectangle BTN_CW      = { 288, 62, 150, 30 };
const Rectangle BTN_RESET   = { 446, 62, 80, 30 };

int main() {
    // State is `static` so it survives Emscripten unwinding main()'s stack when it hands the frame
    // callback to the browser. On desktop this is identical (main runs once anyway).
    static std::mt19937 rng(0);
    static std::normal_distribution<double> gauss(0.0, 1.0);
    static auto randn = [&](double sigma) { return sigma * gauss(rng); };

    static const Eigen::Vector3d odom_sigma(0.04, 0.04, 0.02);   // odometry noise (x, y, theta)
    static const Eigen::Vector2d meas_sigma(0.10, 0.10);         // landmark-position noise (x, y in robot frame)

    // The true robot, and the factor graph that estimates it.
    static LieGroup true_robot, last_keyframe;
    static FactorGraph fg(0);
    static int lid = 0;
    static Key current_pose;
    static std::vector<Key> pose_keys, landmark_keys;
    static std::vector<Eigen::Vector2d> landmark_true;
    static int timestep = 0;

    // (Re)build an empty graph anchored at the origin. Run once at startup and by the Reset button.
    static auto reset = [&]() {
        true_robot = LieGroup(LieGroupType::SE2, Eigen::Vector3d(0, 0, 0));
        last_keyframe = true_robot;
        fg = FactorGraph(0);
        lid = fg.addLayer("se2-localisation")->lid_;
        // First pose: anchored with a strong prior so the trajectory has a fixed origin.
        current_pose = fg.addVariable<Variable>(lid, true_robot, Eigen::VectorXd::Constant(3, 1e-4))->key_;
        pose_keys = { current_pose };
        landmark_keys.clear();
        landmark_true.clear();
        timestep = 0;
    };
    reset();

    SetTraceLogLevel(LOG_WARNING);   // hide raylib's INFO startup logs
    InitWindow(SCREEN_W, SCREEN_H, "GBP SE(2) localisation");
    SetTargetFPS(60);

    static std::function<void()> frame = [&]() {
        // -- input: drive the true robot ------------------------------------------------------------
        Eigen::Vector3d u(0, 0, 0);
        if (IsKeyDown(KEY_UP))    u(0) += FORWARD_STEP;
        if (IsKeyDown(KEY_LEFT))  u(2) += TURN_STEP;
        if (IsKeyDown(KEY_RIGHT)) u(2) -= TURN_STEP;
        if (u.norm() > 0) true_robot = true_robot + u;

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            Vector2 m = GetMousePosition();
            if (CheckCollisionPointRec(m, BTN_FORWARD))   true_robot = true_robot + Eigen::Vector3d(1.0, 0.0, 0.0);
            else if (CheckCollisionPointRec(m, BTN_CCW))  true_robot = true_robot + Eigen::Vector3d(0.0, 0.0,  HALF_PI);
            else if (CheckCollisionPointRec(m, BTN_CW))   true_robot = true_robot + Eigen::Vector3d(0.0, 0.0, -HALF_PI);
            else if (CheckCollisionPointRec(m, BTN_RESET)) reset();
            else {
                // Click on the canvas: add a landmark (an R2 variable with a strong prior) at the cursor.
                Eigen::Vector2d p = screenToWorld(m);
                Key k = fg.addVariable<Variable>(lid, LieGroup(LieGroupType::RN, p), Eigen::VectorXd::Constant(2, 1e-3))->key_;
                landmark_keys.push_back(k);
                landmark_true.push_back(p);
            }
        }

        // -- new keyframe once the robot has actually moved enough (not every frame, so we don't pile up
        //    near-coincident poses on the same landmarks - that makes the graph pathologically loopy) ---
        ++timestep;
        double moved_dist = (true_robot.translation() - last_keyframe.translation()).norm();
        double moved_ang  = std::abs(wrapAngle(true_robot.coeffs()(2) - last_keyframe.coeffs()(2)));
        if (moved_dist > KEYFRAME_DIST || moved_ang > KEYFRAME_ANGLE) {
            LieGroup delta_meas = last_keyframe.between(true_robot)
                                + Eigen::Vector3d(randn(odom_sigma(0)), randn(odom_sigma(1)), randn(odom_sigma(2)));
            LieGroup new_init = fg.getVar(current_pose)->belief_.mu * delta_meas;   // dead-reckoned start
            // A weak prior at the dead-reckoned init regularises the new (otherwise zero-precision) pose,
            // so loopy-GBP overconfidence can't drive its precision to infinity.
            Key new_pose = fg.addVariable<Variable>(lid, new_init, Eigen::VectorXd::Constant(3, 5.0))->key_;
            fg.addFactor<OdometryFactor>(lid, { current_pose, new_pose }, delta_meas, odom_sigma);

            Eigen::Vector2d robot_xy = true_robot.translation();
            for (size_t i = 0; i < landmark_keys.size(); ++i) {
                if ((landmark_true[i] - robot_xy).norm() > OBSERVE_RADIUS) continue;
                Eigen::Vector2d z = LandmarkFactor::predict(true_robot, LieGroup(LieGroupType::RN, landmark_true[i]));
                z(0) += randn(meas_sigma(0));
                z(1) += randn(meas_sigma(1));
                fg.addFactor<LandmarkFactor>(lid, { new_pose, landmark_keys[i] }, z, meas_sigma);
            }

            current_pose = new_pose;
            pose_keys.push_back(new_pose);
            last_keyframe = true_robot;
        }

        fg.optimiseGBP(GBP_ITERS);

        // -- draw -----------------------------------------------------------------------------------
        BeginDrawing();
        ClearBackground(RAYWHITE);
        drawGrid();

        // Estimated trajectory through the pose keyframes.
        for (size_t i = 1; i < pose_keys.size(); ++i) {
            Eigen::Vector3d a = fg.getVar(pose_keys[i - 1])->belief_.mu.coeffs();
            Eigen::Vector3d b = fg.getVar(pose_keys[i])->belief_.mu.coeffs();
            DrawLineV(worldToScreen(a.head<2>()), worldToScreen(b.head<2>()), GRAY);
        }

        // Every pose variable: its belief mean (dot) and position-covariance ellipse.
        for (Key k : pose_keys) {
            const auto& b = fg.getVar(k)->belief_;
            Eigen::Vector3d m = b.mu.coeffs();
            drawCovEllipse(m.head<2>(), b.lambda.inverse().topLeftCorner<2, 2>(), Fade(BLUE, 0.25f));
            DrawCircleV(worldToScreen(m.head<2>()), 3.f, Fade(BLUE, 0.7f));
        }

        // Landmarks (true position) and the observation radius around the true robot.
        DrawCircleLinesV(worldToScreen(true_robot.translation()), OBSERVE_RADIUS * SCALE, Fade(GREEN, 0.35f));
        for (size_t i = 0; i < landmark_keys.size(); ++i) {
            DrawCircleV(worldToScreen(landmark_true[i]), 6.f, ORANGE);
            if ((landmark_true[i] - true_robot.translation()).norm() <= OBSERVE_RADIUS)
                DrawLineV(worldToScreen(fg.getVar(current_pose)->belief_.mu.coeffs().head<2>()),
                          worldToScreen(landmark_true[i]), Fade(MAROON, 0.5f));
        }

        // Estimated pose + its position covariance, then the ground-truth robot on top.
        {
            const auto& belief = fg.getVar(current_pose)->belief_;
            Eigen::Vector3d est = belief.mu.coeffs();
            Eigen::Matrix3d cov = belief.lambda.inverse();
            drawCovEllipse(est.head<2>(), cov.topLeftCorner<2, 2>(), Fade(BLUE, 0.9f));
            drawRobot(belief.mu, 9.f, BLUE, DARKBLUE, true);
        }
        drawRobot(true_robot, 9.f, GREEN, DARKGREEN, true);

        // Buttons for discrete moves.
        drawButton(BTN_FORWARD, "Forward 1 m");
        drawButton(BTN_CCW, "Rotate +90 (CCW)");
        drawButton(BTN_CW, "Rotate -90 (CW)");
        drawButton(BTN_RESET, "Reset");

        // HUD: controls hint (top), status (bottom).
        DrawText("left click: add landmark    arrows: drive    buttons: discrete move / reset", 12, 12, 18, DARKGRAY);
        DrawText(TextFormat("frame %d   poses %d   landmarks %d", timestep, (int)pose_keys.size(), (int)landmark_keys.size()),
                 12, SCREEN_H - 26, 18, DARKGRAY);

        // Legend (left).
        {
            int px = 10, py = 104, pw = 188, ph = 122;
            DrawRectangle(px, py, pw, ph, Fade(RAYWHITE, 0.88f));
            DrawRectangleLines(px, py, pw, ph, Fade(GRAY, 0.4f));
            DrawText("Key", px + 10, py + 8, 14, GRAY);
            float cx = px + 18, y0 = py + 34;
            legendEntry(cx, y0,      GREEN,            true,  "true robot");
            legendEntry(cx, y0 + 26, BLUE,             true,  "estimated pose");
            legendEntry(cx, y0 + 52, Fade(BLUE, 0.7f), false, "pose 2-sigma");
            legendEntry(cx, y0 + 78, ORANGE,           true,  "landmark");
        }
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
