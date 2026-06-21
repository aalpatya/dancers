/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once
#include <iostream>
#include <chrono>
#include <math.h>
#include <algorithm>
#include <numeric>
#include <random>
#include <Eigen/Dense>
#include <map>
#include <string>
#include <vector>
#include <filesystem>
#include <raylib.h>

// Obstacle distance field read by the planning layer's obstacle factor. Built by Graphics from the
// obstacle PNG with a C++ chamfer distance transform. Row-major, 0 = at/near an obstacle .. 255 = far /
// free space.
struct DistanceField {
    int w = 0, h = 0;
    std::vector<unsigned char> data;
    int  cols()  const { return w; }
    int  rows()  const { return h; }
    bool empty() const { return data.empty(); }
    double value01(int x, int y) const {                 // in [0,1]; out of bounds = 1 (free space)
        if (x < 0 || y < 0 || x >= w || y >= h || data.empty()) return 1.0;
        return data[(size_t)y * w + x] / 255.0;
    }
};

// Distinct display colour for integer category `i` of `n`. The hue ramp [0, n) is shuffled once
// (fixed seed) so neighbouring categories get well-separated hues. Used for discrete-decision tints
// (robot colour and the decision HUD share this, so they match).
inline Color decisionColor(int i, int n){
    static std::vector<int> perm; static int perm_n = -1;
    if (perm_n != n){
        perm.resize(n); std::iota(perm.begin(), perm.end(), 0);
        std::shuffle(perm.begin(), perm.end(), std::mt19937(12345));
        perm_n = n;
    }
    int d = (i >= 0 && i < n) ? i : 0;
    return ColorFromHSV(perm[d] / (float)n * 180.f, 1., 1.);
}

/*******************************************************************************/
// Python-style print: print(a, b, "blah blah").
// For an Eigen expression (e.g. a slice), call .eval() first:
//   Eigen::VectorXd v{{0,1,2}}; print(v({0,1}).eval());
/*******************************************************************************/
template <typename T> void print(const T& t) {
    std::cout << t << std::endl;
}
template <typename First, typename... Rest> void print(const First& first, const Rest&... rest) {
    std::cout << first << " ";
    print(rest...); // recursive call using pack expansion syntax
}

/*******************************************************************************/
// Wrap an angle to (-pi, pi].
/*******************************************************************************/
inline double angleOp(const double& a){
    return atan2(sin(a), cos(a));
};
/*******************************************************************************/
// Draws the FPS and time on screen, plus the help screen.
/*******************************************************************************/
void drawInfo( uint32_t time_cnt);

/*******************************************************************************/
// Cross-platform filepath builder. Paths in this codebase are written unix-style
// (with '/'); this normalises them for the host platform (no-op on macOS/Linux,
// '/' -> '\' on Windows).
//   - `parts` are joined with the platform separator, skipping empty parts.
//   - if `make_absolute` is true, the result is resolved against the cwd (already-
//     absolute parts are kept).
// Examples:
//   makePath({globals.ASSETS_DIR, "models/bus.glb"})
//   makePath({globals.RUN_DIR, "temp", "out.json"})
//   makePath({"results", "out.json"}, /*make_absolute=*/true)
/*******************************************************************************/
std::string makePath(const std::vector<std::string>& parts, bool make_absolute = false);

// Variadic convenience: pathOf("a", "b/c", "d.json") == makePath({"a","b/c","d.json"}).
template <typename... Parts>
std::string pathOf(const Parts&... parts){
    return makePath(std::vector<std::string>{ std::string(parts)... });
}

/*******************************************************************************/
// Wall-clock timing helper.
// Usage:
//   auto start = std::chrono::steady_clock::now();
//   std::cout << "Elapsed(us): " << since(start).count() << std::endl;
/*******************************************************************************/
template <
    class result_t   = std::chrono::microseconds,
    class clock_t    = std::chrono::high_resolution_clock,
    class duration_t = std::chrono::microseconds
>
auto since(std::chrono::time_point<clock_t, duration_t> const& start)
{
    return std::chrono::duration_cast<result_t>(clock_t::now() - start);
}


static float distPoint2line(const Eigen::Vector2d& p, const Eigen::Vector2d& A, const Eigen::Vector2d& B){
    float x3 = p.x(), y3 = p.y(), x1 = A.x(), y1 = A.y(), x2 = B.x(), y2 = B.y();
    Eigen::Vector2d direction = B - A;
    float u = ((x3 - x1) * (x2-x1) + (y3 - y1) * (y2-y1)) / direction.squaredNorm();
    float dist = (u < 0.f) ? (p - A).norm() : (u > 1.f) ? (p - B).norm() : (p - (A + u * direction)).norm();
    return dist;
};

static bool linesIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3, const Eigen::Vector2d& p4){
    float x1 = p1.x(), y1 = p1.y(), x2 = p2.x(), y2 = p2.y(), x3 = p3.x(), y3 = p3.y(), x4 = p4.x(), y4 = p4.y();
    float denom = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1);
    float ua = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / denom;
    float ub = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / denom;
    return ((ua >= 0.f && ua <= 1.f) && (ub >= 0.f && ub <= 1.f));
};

static std::pair<Eigen::Vector2d, Eigen::Vector2d> getSegment(int side, float x, float y, float t, float L, float W){
    std::vector<Eigen::Vector2d> corners{
        Eigen::Vector2d{{x+0.5f*(L*cos(t)-W*sin(t)), y+0.5f*(L*sin(t)+W*cos(t))}},
        Eigen::Vector2d{{x+0.5f*(L*cos(t)+W*sin(t)), y+0.5f*(L*sin(t)-W*cos(t))}},
        Eigen::Vector2d{{x+0.5f*(W*sin(t)-L*cos(t)), y+0.5f*(-L*sin(t)-W*cos(t))}},
        Eigen::Vector2d{{x+0.5f*(-W*sin(t)-L*cos(t)), y+0.5f*(-1.f*L*sin(t)+W*cos(t))}}
    };
        
    return {corners[side], corners[(side+1)%4]};
};  

static float dist_rect2rect(float x1, float y1, float t1, float x2, float y2, float t2, float L, float W){
    float shortest_dist = 1e9; Eigen::Vector2d pA{{0., 0.}}, pB{{0., 0.}}, p{{0., 0.}};
    for (int i=0; i<4; i++){
        for (int j=0; j<4; j++){
            auto [p1, p2] = getSegment(i, x1, y1, t1, L, W);
            auto [p3, p4] = getSegment(j, x2, y2, t2, L, W);
            if (linesIntersect(p1, p2, p3, p4)){return 0.f;}

            std::vector<float> dists {distPoint2line(p1, p3, p4),
                                      distPoint2line(p2, p3, p4),
                                      distPoint2line(p3, p1, p2),
                                      distPoint2line(p4, p1, p2)};
            auto it = std::min_element(dists.begin(), dists.end());
            float min_dist = *it;
            if (min_dist <= shortest_dist) {
                shortest_dist = min_dist;
            };
        }
    }

    return shortest_dist;

};

template<typename KeyType, typename ValueType> 
static std::pair<KeyType,ValueType> getMax( const std::map<KeyType,ValueType>& x ) {
  using pairtype=std::pair<KeyType,ValueType>; 
  return *std::max_element(x.begin(), x.end(), [] (const pairtype & p1, const pairtype & p2) {
        return p1.second < p2.second;
  }); 
}