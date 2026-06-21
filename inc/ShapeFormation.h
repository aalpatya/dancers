/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once

#include <string>
#include <tuple>
#include <vector>

#include <Globals.h>
#include <gbp/GBPCore.h>
#include <KDTreeAdaptors.h>
#include <nanoflann.h>

class ShapeFormation {
public:
    ShapeFormation();
    ~ShapeFormation();
    // Draw the formation per the display type ("none" | "points" | "full"), at this tree's liePose_.
    // rid (robot id) and col are optional, used only by the "full" texture overlay's depth/tint.
    void draw(const std::string& display_type, int rid = -1, Color col = GRAY);
    void updateKdtree();
    std::tuple<bool, std::vector<size_t>, std::vector<double>> knnSearch(Eigen::Vector3d query_point, size_t k, int dims);
    std::pair<bool, std::vector<nanoflann::ResultItem<size_t, double>>> radiusSearch(Eigen::Vector3d query_point, double search_radius, int dims);

    using KDTree_formation = KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector3d>>;
    std::vector<Eigen::Vector3d> points_;
    KDTree_formation* kdtree_formation_;
    manif::SE2d liePose_ = manif::SE2d::Identity();
    double image_scale_ = 1.;

    // Both overlay textures are built up-front (when a formation image is configured) so the display
    // type can be switched live in the GUI. "points" = white squares per formation point; "full" = the
    // inverted shape image. tex_loaded_ is false when no image is available.
    Texture2D tex_points_{};
    Texture2D tex_full_{};
    bool tex_loaded_ = false;
};