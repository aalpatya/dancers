/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <ShapeFormation.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>
#include <utility>
#include <algorithm>

#include <Graphics.h>

// Generate the formation points from the shape image, using raylib's image API: binarise the shape
// (dark pixels are "inside"), then shrink the mask and grow the grid spacing together until it only just
// fits N points, subsample to N, and centre/scale. Returns the points (metres, centred on the origin)
// and the image scale factor used by the overlay texture.
namespace {
// luminance < 0.5 (skimage rgb2gray weights) means a dark pixel, i.e. inside the shape.
static bool insidePixel(Color c){ return (0.2125*c.r + 0.7154*c.g + 0.0721*c.b) / 255.0 < 0.5; }

// Re-threshold an RGBA image back to pure black/white after a (smoothing) resize.
static void rebinarise(Image& im){
    Color* p = (Color*)im.data;
    for (int i = 0; i < im.width*im.height; i++){ unsigned char v = p[i].r > 127 ? 255 : 0; p[i] = Color{v,v,v,255}; }
}

static std::pair<std::vector<std::pair<double,double>>, double>
generateShapeFormationPoints(const std::string& image_path, int N, double min_spacing, unsigned seed){
    Image src = LoadImage(image_path.c_str());
    ImageFormat(&src, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
    const int orig_h = src.height;

    // mask: white where inside the shape, black outside.
    Image mask = GenImageColor(src.width, src.height, BLACK);
    ImageFormat(&mask, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
    { Color* s = (Color*)src.data; Color* m = (Color*)mask.data;
      for (int i = 0; i < src.width*src.height; i++) m[i] = insidePixel(s[i]) ? WHITE : BLACK; }
    UnloadImage(src);

    // Grid points (in mask pixel coords) lying inside the mask at the given spacing.
    auto gridPoints = [](const Image& m, double spacing){
        std::vector<std::pair<double,double>> pts;
        const Color* px = (const Color*)m.data;
        for (double y = 0; y < m.height; y += spacing)
            for (double x = 0; x < m.width; x += spacing)
                if (px[(int)y * m.width + (int)x].r > 127) pts.emplace_back(x, y);
        return pts;
    };

    std::vector<std::pair<double,double>> kept;
    int last_w = 0, last_h = 0; double last_spacing = 0, spacing = 5.0;
    for (int it = 0; it < 1000; it++){
        auto pts = gridPoints(mask, spacing);
        if ((int)pts.size() < N) break;
        kept = std::move(pts); last_w = mask.width; last_h = mask.height; last_spacing = spacing;
        Image next = ImageCopy(mask);
        ImageResize(&next, (int)std::round(mask.width*0.9), (int)std::round(mask.height*0.9));
        rebinarise(next);
        UnloadImage(mask); mask = next;
        spacing *= 1.02;
    }
    UnloadImage(mask);

    if (kept.empty() || last_spacing <= 0) return {{}, 1.0};

    // Keep exactly N (random subset, like np.random.choice; the exact subset needn't match python).
    if ((int)kept.size() > N){
        std::mt19937 rng(seed);
        std::shuffle(kept.begin(), kept.end(), rng);
        kept.resize(N);
    }
    const double scale = min_spacing / last_spacing;
    const double image_scale = (double)orig_h / (scale * last_h);
    std::vector<std::pair<double,double>> out;
    out.reserve(kept.size());
    for (auto& p : kept) out.emplace_back((p.first - last_w/2.0) * scale, (p.second - last_h/2.0) * scale);
    return {out, image_scale};
}
}  // namespace

ShapeFormation::ShapeFormation(){
    // Build the formation points and both overlay textures whenever a formation image is configured and
    // present, regardless of FORMATION_DISPLAY_TYPE, so the GUI can switch display type (None/Points/
    // Shape) live. postParsing() already forced the type to "none" if the image was missing, so an
    // empty/absent image just skips this.
    if (!globals.FORMATION_IMG_FILE.empty() && std::filesystem::exists(globals.FORMATION_IMG_FILE)){
        // Generate the formation points from the shape image (see generateShapeFormationPoints above).
        auto [positions, scale] = generateShapeFormationPoints(globals.FORMATION_IMG_FILE,
                                      globals.NUM_ROBOTS, globals.ROBOT_RADIUS*4, (unsigned)globals.RNG_SEED);
        image_scale_ = scale;
        for (auto& p : positions) points_.push_back(Eigen::Vector3d{p.first, p.second, 0.});

        // Build both overlay textures up-front. The "points" and "full" variants differ only in how the
        // source `img` is produced; they share the tail (black background -> transparent, then upload).
        if (globals.DISPLAY){
            auto buildTex = [&](bool full) -> Texture2D {
                Image img = LoadImage(globals.FORMATION_IMG_FILE.c_str());
                if (full){
                    // Shape is black on white; invert so the shape is white and the background black (-> transparent).
                    ImageColorInvert(&img);
                } else {   // "points": a blank canvas (formation-image size) with one white square per point.
                    // A point at p metres maps to pixel (centre + p*image_scale_), matching where draw()'s
                    // DrawTextureFlat (centred, scaled by 1/image_scale_) expects it.
                    int w = img.width, h = img.height;
                    UnloadImage(img);
                    img = GenImageColor(w, h, BLACK);
                    int rect_width = (int)std::round(globals.ROBOT_RADIUS * image_scale_);
                    for (auto& p : points_){
                        int cx = w/2 + (int)std::round(p(0) * image_scale_);
                        int cy = h/2 + (int)std::round(p(1) * image_scale_);
                        ImageDrawRectangleLines(&img, Rectangle{(float)(cx - rect_width/2), (float)(cy - rect_width/2), (float)rect_width, (float)rect_width}, 1, WHITE);
                    }
                }
                ImageFormat(&img, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
                Color *pixels = (Color *)img.data;
                int pixelCount = img.width * img.height;
                for (int i = 0; i < pixelCount; i++) {
                    if (pixels[i].r == 0 && pixels[i].g == 0 && pixels[i].b == 0) pixels[i].a = 0;   // black -> transparent
                }
                Texture2D t = LoadTextureFromImage(img);
                UnloadImage(img);
                return t;
            };
            tex_points_ = buildTex(false);
            tex_full_   = buildTex(true);
            tex_loaded_ = true;
        }
    }

    // Generate the KD Tree with points_
    if (points_.size()==0){
        points_.push_back(Eigen::Vector3d{Eigen::Infinity, Eigen::Infinity, Eigen::Infinity});
        kdtree_formation_ = new KDTree_formation(3, points_, 10);
        points_.clear();
    } else {
        kdtree_formation_ = new KDTree_formation(3, points_, 10);
    }
    updateKdtree();
}

ShapeFormation::~ShapeFormation(){
    if (tex_loaded_){ UnloadTexture(tex_points_); UnloadTexture(tex_full_); }
    delete kdtree_formation_;
}

void ShapeFormation::draw(const std::string& display_type, int rid, Color col){
    if (display_type=="none" || !tex_loaded_) return;   // nothing to draw without textures
    Eigen::VectorXd v(3); v << liePose_.translation(), liePose_.angle();
    if (display_type=="points"){
        DrawTextureFlat(tex_points_, Vector2{(float)v(0), (float)v(1)},
            RAD2DEG*v(manif::SE2d::DoF - 1),
            Vector3{0.f, 1.f, 0.f}, 1.f/image_scale_, ColorAlpha(col, 0.5), -1 + 0.5*rid/(float)globals.NUM_ROBOTS);
    } else if (display_type=="full"){
        DrawTextureFlat(tex_full_, Vector2{(float)v(0), (float)v(1)},
            RAD2DEG*v(manif::SE2d::DoF - 1),
            Vector3{0.f, 1.f, 0.f}, 1.f/image_scale_, ColorAlpha(col, 0.2), -1 + 0.5*rid/(float)globals.NUM_ROBOTS);
    }
}

std::tuple<bool, std::vector<size_t>, std::vector<double>> ShapeFormation::knnSearch(Eigen::Vector3d query_pt, size_t k, int dims){
    kdtree_formation_->set_DIM(dims);
    // do a knn search
    std::vector<size_t> ret_indexes(k);
    std::vector<double> out_dists_sqr(k);

    nanoflann::KNNResultSet<double> resultSet(k);
    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
    bool found = (kdtree_formation_->kdtree_get_point_count()) ?
        kdtree_formation_->index->findNeighbors(resultSet, &query_pt[0], {}) : false;
    return {found, ret_indexes, out_dists_sqr};
}

std::pair<bool, std::vector<nanoflann::ResultItem<size_t, double>>> ShapeFormation::radiusSearch(Eigen::Vector3d query_pt, double search_radius, int dims){
    kdtree_formation_->set_DIM(dims);
    std::vector<nanoflann::ResultItem<size_t, double>> IndicesDists{};
    nanoflann::SearchParameters searchParams{};
    bool found = kdtree_formation_->index->radiusSearch(&query_pt[0], search_radius, IndicesDists, searchParams);
    return {found, IndicesDists};
}

void ShapeFormation::updateKdtree(){
    kdtree_formation_->index->buildIndex();
}