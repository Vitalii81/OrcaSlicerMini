#include "PerimeterGenerator.hpp"
#include "AABBTreeLines.hpp"
//#include "BridgeDetector.hpp"
#include "ClipperUtils.hpp"
// #include "ExtrusionEntity.hpp"
// #include "ExtrusionEntityCollection.hpp"
#include "PrintConfig.hpp"
//#include "ShortestPath.hpp"
//#include "VariableWidth.hpp"
//#include "CurveAnalyzer.hpp"
#include "Clipper2Utils.hpp"
//#include "Arachne/WallToolPaths.hpp"
#include "Geometry/ConvexHull.hpp"
#include "ExPolygonCollection.hpp"
#include "Geometry.hpp"
#include "Line.hpp"
#include <cmath>
#include <cassert>
#include <random>
#include <unordered_set>
#include <thread>
#include "libslic3r/AABBTreeLines.hpp"
#include "Print.hpp"
#include "Algorithm/LineSplit.hpp"
//#include "libnoise/noise.h"
static const int overhang_sampling_number = 6;
static const double narrow_loop_length_threshold = 10;
static const double min_degree_gap = 0.1;
static const int max_overhang_degree = overhang_sampling_number - 1;
//BBS: when the width of expolygon is smaller than
//ext_perimeter_width + ext_perimeter_spacing  * (1 - SMALLER_EXT_INSET_OVERLAP_TOLERANCE),
//we think it's small detail area and will generate smaller line width for it
static constexpr double SMALLER_EXT_INSET_OVERLAP_TOLERANCE = 0.22;

//#define DEBUG_FUZZY

namespace Slic3r {

// Produces a random value between 0 and 1. Thread-safe.
static double random_value() {
    thread_local std::random_device rd;
    // Hash thread ID for random number seed if no hardware rng seed is available
    thread_local std::mt19937 gen(rd.entropy() > 0 ? rd() : std::hash<std::thread::id>()(std::this_thread::get_id()));
    thread_local std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(gen);
}

// class UniformNoise: public noise::module::Module {
//     public:
//         UniformNoise(): Module (GetSourceModuleCount ()) {};
//
//         virtual int GetSourceModuleCount() const { return 0; }
//         virtual double GetValue(double x, double y, double z) const { return random_value() * 2 - 1; }
// };

// Hierarchy of perimeters.
class PerimeterGeneratorLoop {
public:
    // Polygon of this contour.
    Polygon                             polygon;
    // Is it a contour or a hole?
    bool                                is_contour;
    // BBS: is perimeter using smaller width
    bool is_smaller_width_perimeter;
    // Depth in the hierarchy. External perimeter has depth = 0. An external perimeter could be both a contour and a hole.
    unsigned short                      depth;
    // Children contour, may be both CCW and CW oriented (outer contours or holes).
    std::vector<PerimeterGeneratorLoop> children;

    PerimeterGeneratorLoop(const Polygon &polygon, unsigned short depth, bool is_contour, bool is_small_width_perimeter = false) :
        polygon(polygon), is_contour(is_contour), is_smaller_width_perimeter(is_small_width_perimeter), depth(depth) {}
    // External perimeter. It may be CCW or CW oriented (outer contour or hole contour).
    bool is_external() const { return this->depth == 0; }
    // An island, which may have holes, but it does not have another internal island.
    bool is_internal_contour() const;
};

// static std::unique_ptr<noise::module::Module> get_noise_module(const FuzzySkinConfig& cfg) {
//     if (cfg.noise_type == NoiseType::Perlin) {
//         auto perlin_noise = noise::module::Perlin();
//         perlin_noise.SetFrequency(1 / cfg.noise_scale);
//         perlin_noise.SetOctaveCount(cfg.noise_octaves);
//         perlin_noise.SetPersistence(cfg.noise_persistence);
//         return std::make_unique<noise::module::Perlin>(perlin_noise);
//     } else if (cfg.noise_type == NoiseType::Billow) {
//         auto billow_noise = noise::module::Billow();
//         billow_noise.SetFrequency(1 / cfg.noise_scale);
//         billow_noise.SetOctaveCount(cfg.noise_octaves);
//         billow_noise.SetPersistence(cfg.noise_persistence);
//         return std::make_unique<noise::module::Billow>(billow_noise);
//     } else if (cfg.noise_type == NoiseType::RidgedMulti) {
//         auto ridged_multi_noise = noise::module::RidgedMulti();
//         ridged_multi_noise.SetFrequency(1 / cfg.noise_scale);
//         ridged_multi_noise.SetOctaveCount(cfg.noise_octaves);
//         return std::make_unique<noise::module::RidgedMulti>(ridged_multi_noise);
//     } else if (cfg.noise_type == NoiseType::Voronoi) {
//         auto voronoi_noise = noise::module::Voronoi();
//         voronoi_noise.SetFrequency(1 / cfg.noise_scale);
//         voronoi_noise.SetDisplacement(1.0);
//         return std::make_unique<noise::module::Voronoi>(voronoi_noise);
//     } else {
//         return std::make_unique<UniformNoise>();
//     }
// }
//
// // Thanks Cura developers for this function.
// static void fuzzy_polyline(Points& poly, bool closed, coordf_t slice_z, const FuzzySkinConfig& cfg)
// {
//     std::unique_ptr<noise::module::Module> noise = get_noise_module(cfg);
//
//     const double min_dist_between_points = cfg.point_distance * 3. / 4.; // hardcoded: the point distance may vary between 3/4 and 5/4 the supplied value
//     const double range_random_point_dist = cfg.point_distance / 2.;
//     double dist_left_over = random_value() * (min_dist_between_points / 2.); // the distance to be traversed on the line before making the first new point
//     Point* p0 = &poly.back();
//     Points out;
//     out.reserve(poly.size());
//     for (Point &p1 : poly)
//     {
//         if (!closed) {
//             // Skip the first point for open path
//             closed = true;
//             p0 = &p1;
//             continue;
//         }
//         // 'a' is the (next) new point between p0 and p1
//         Vec2d  p0p1      = (p1 - *p0).cast<double>();
//         double p0p1_size = p0p1.norm();
//         double p0pa_dist = dist_left_over;
//         for (; p0pa_dist < p0p1_size;
//             p0pa_dist += min_dist_between_points + random_value() * range_random_point_dist)
//         {
//             Point pa = *p0 + (p0p1 * (p0pa_dist / p0p1_size)).cast<coord_t>();
//             double r = noise->GetValue(unscale_(pa.x()), unscale_(pa.y()), slice_z) * cfg.thickness;
//             out.emplace_back(pa + (perp(p0p1).cast<double>().normalized() * r).cast<coord_t>());
//         }
//         dist_left_over = p0pa_dist - p0p1_size;
//         p0 = &p1;
//     }
//     while (out.size() < 3) {
//         size_t point_idx = poly.size() - 2;
//         out.emplace_back(poly[point_idx]);
//         if (point_idx == 0)
//             break;
//         -- point_idx;
//     }
//     if (out.size() >= 3)
//         poly = std::move(out);
// }

using PerimeterGeneratorLoops = std::vector<PerimeterGeneratorLoop>;

struct PolylineWithDegree
{
    PolylineWithDegree(Polyline polyline, double overhang_degree) : polyline(polyline), overhang_degree(overhang_degree){};
    Polyline polyline;
    double   overhang_degree = 0;
};

static std::deque<PolylineWithDegree> split_polyline_by_degree(const Polyline &polyline_with_insert_points, const std::deque<double> &points_overhang)
{
    std::deque<PolylineWithDegree> out;
    Polyline left;
    Polyline right;
    Polyline temp_copy = polyline_with_insert_points;

    size_t   poly_size = polyline_with_insert_points.size();
    // BBS: merge degree in limited range
    //find first degee base
    double degree_base = int(points_overhang[points_overhang.size() - 1] / min_degree_gap) * min_degree_gap + min_degree_gap;
    degree_base = degree_base > max_overhang_degree ? max_overhang_degree : degree_base;
    double short_poly_len = 0;
    for (int point_idx = points_overhang.size() - 2; point_idx > 0; --point_idx) {

        double degree = points_overhang[point_idx];

        if ( degree <= degree_base && degree >= degree_base - min_degree_gap )
            continue;

        temp_copy.split_at_index(point_idx, &left, &right);

        temp_copy = std::move(left);
        out.push_back(PolylineWithDegree(right, degree_base));

        degree_base = int(degree / min_degree_gap) * min_degree_gap + min_degree_gap;
        degree_base = degree_base > max_overhang_degree ? max_overhang_degree : degree_base;
    }

    if (!temp_copy.empty()) {
        out.push_back(PolylineWithDegree(temp_copy, degree_base));
    }

    return out;

}
static void insert_point_to_line( double              left_point_degree,
                                  Point               left_point,
                                  double              right_point_degree,
                                  Point               right_point,
                                  std::deque<double> &points_overhang,
                                  Polyline&           polyline,
                                  double              mini_length)
{
    Line   line_temp(left_point, right_point);
    double line_length = line_temp.length();
    if (std::abs(left_point_degree - right_point_degree) <= 0.5 * min_degree_gap || line_length<scale_(1.5))
        return;

    Point middle_pt((left_point + right_point) / 2);
    std::deque<double>  left_points_overhang;
    std::deque<double> right_points_overhang;

    double middle_degree = (left_point_degree + right_point_degree) / 2;
    Polyline left_polyline;
    Polyline right_polyline;

    insert_point_to_line(left_point_degree, left_point, middle_degree, middle_pt, left_points_overhang, left_polyline, mini_length);
    insert_point_to_line(middle_degree, middle_pt, right_point_degree, right_point, right_points_overhang, right_polyline, mini_length);

    if (!left_polyline.empty()) {
        polyline.points.insert(polyline.points.end(), std::make_move_iterator(left_polyline.points.begin()), std::make_move_iterator(left_polyline.points.end()));
        points_overhang.insert(points_overhang.end(), std::make_move_iterator(left_points_overhang.begin()), std::make_move_iterator(left_points_overhang.end()));
    }

    polyline.append(middle_pt);
    points_overhang.emplace_back(middle_degree);

    if (!right_polyline.empty()) {
        polyline.points.insert(polyline.points.end(), std::make_move_iterator(right_polyline.points.begin()), std::make_move_iterator(right_polyline.points.end()));
        points_overhang.insert(points_overhang.end(), std::make_move_iterator(right_points_overhang.begin()), std::make_move_iterator(right_points_overhang.end()));
    }
}

class OverhangDistancer
{
    std::vector<Linef>                lines;
    AABBTreeIndirect::Tree<2, double> tree;

public:
    OverhangDistancer(const Polygons layer_polygons)
    {
        for (const Polygon &island : layer_polygons) {
            for (const auto &line : island.lines()) {
                lines.emplace_back(line.a.cast<double>(), line.b.cast<double>());
            }
        }
        tree = AABBTreeLines::build_aabb_tree_over_indexed_lines(lines);
    }

    float distance_from_perimeter(const Vec2f &point) const
    {
        Vec2d  p = point.cast<double>();
        size_t hit_idx_out{};
        Vec2d  hit_point_out = Vec2d::Zero();
        auto   distance      = AABBTreeLines::squared_distance_to_indexed_lines(lines, tree, p, hit_idx_out, hit_point_out);
        if (distance < 0) { return std::numeric_limits<float>::max(); }

        distance          = sqrt(distance);
        return distance;
    }
};

static std::deque<PolylineWithDegree> detect_overahng_degree(Polygons        lower_polygons,
                                                             Polylines       middle_overhang_polyines,
                                                             const double    &lower_bound,
                                                             const double    &upper_bound,
                                                             Polylines       &too_short_polylines)
{
    // BBS: collect lower_polygons points
    //Polylines;
    Points lower_polygon_points;
    std::vector<size_t> polygons_bound;

    std::unique_ptr<OverhangDistancer> prev_layer_distancer;
    prev_layer_distancer = std::make_unique<OverhangDistancer>(lower_polygons);
    std::deque<PolylineWithDegree> out;
    std::deque<double>             points_overhang;
    //BBS: get overhang degree and split path
    for (size_t polyline_idx = 0; polyline_idx < middle_overhang_polyines.size(); ++polyline_idx) {
        //filter too short polyline
        Polyline middle_poly = middle_overhang_polyines[polyline_idx];
        if (middle_poly.length() < scale_(1.0)) {
            too_short_polylines.push_back(middle_poly);
            continue;
        }

        Polyline polyline_with_insert_points;
        points_overhang.clear();
        double last_degree = 0;
        // BBS : calculate overhang dist
        for (size_t point_idx = 0; point_idx < middle_poly.points.size(); ++point_idx) {
            Point pt = middle_poly.points[point_idx];

            float overhang_dist = prev_layer_distancer->distance_from_perimeter(pt.cast<float>());
            overhang_dist       = overhang_dist > upper_bound ? upper_bound : overhang_dist;
            // BBS : calculate overhang degree
            int    max_overhang = max_overhang_degree;
            int    min_overhang = 0;
            double t            = (overhang_dist - lower_bound) / (upper_bound - lower_bound);
            t = t > 1.0 ? 1: t;
            t = t < EPSILON ? 0 : t;
            double this_degree  = (1.0 - t) * min_overhang + t * max_overhang;
            // BBS: intert points
            if (point_idx > 0) {
                insert_point_to_line(last_degree, middle_poly.points[point_idx - 1], this_degree, pt, points_overhang, polyline_with_insert_points,
                                     upper_bound - lower_bound);
            }
            points_overhang.push_back(this_degree);

            polyline_with_insert_points.append(pt);
            last_degree = this_degree;

        }

        // BBS : split path by degree
        std::deque<PolylineWithDegree> polyline_with_merged_degree = split_polyline_by_degree(polyline_with_insert_points, points_overhang);
        out.insert(out.end(),  std::make_move_iterator(polyline_with_merged_degree.begin()),  std::make_move_iterator(polyline_with_merged_degree.end()));
    }

    return out;
}

std::pair<double, double> PerimeterGenerator::dist_boundary(double width)
{
    std::pair<double, double> out;
    float nozzle_diameter = print_config->nozzle_diameter.get_at(config->wall_filament - 1);
    float start_offset = -0.5 * width;
    float end_offset = 0.5 * nozzle_diameter;
    double degree_0 = scale_(start_offset + 0.5 * (end_offset - start_offset) / (overhang_sampling_number - 1));
    out.first = 0;
    out.second = scale_(end_offset) - degree_0;
    return out;
}

template<class _T>
static bool detect_steep_overhang(const PrintRegionConfig *config,
                                  bool                     is_contour,
                                  const BoundingBox       &extrusion_bboxs,
                                  double                   extrusion_width,
                                  const _T                 extrusion,
                                  const ExPolygons        *lower_slices,
                                  bool                    &steep_overhang_contour,
                                  bool                    &steep_overhang_hole)
{
    double threshold = config->overhang_reverse_threshold.get_abs_value(extrusion_width);
    // Special case: reverse on every even (from GUI POV) layer
    if (threshold < EPSILON) {
        if (is_contour) {
            steep_overhang_contour = true;
        } else {
            steep_overhang_hole = true;
        }

        return true;
    }

    Polygons lower_slcier_chopped = ClipperUtils::clip_clipper_polygons_with_subject_bbox(*lower_slices, extrusion_bboxs, true);

    // All we need to check is whether we have lines outside `threshold`
    double off = threshold - 0.5 * extrusion_width;

    auto limiton_polygons = offset(lower_slcier_chopped, float(scale_(off)));

    auto remain_polylines = diff_pl(extrusion, limiton_polygons);
    if (!remain_polylines.empty()) {
        if (is_contour) {
            steep_overhang_contour = true;
        } else {
            steep_overhang_hole = true;
        }

        return true;
    }

    return false;
}

static bool should_fuzzify(const FuzzySkinConfig& config, const int layer_id, const size_t loop_idx, const bool is_contour)
{
    const auto fuzziy_type = config.type;

    if (fuzziy_type == FuzzySkinType::None) {
        return false;
    }
    if (!config.fuzzy_first_layer && layer_id <= 0) {
        // Do not fuzzy first layer unless told to
        return false;
    }

    const bool fuzzify_contours = loop_idx == 0 || fuzziy_type == FuzzySkinType::AllWalls;
    const bool fuzzify_holes    = fuzzify_contours && (fuzziy_type == FuzzySkinType::All || fuzziy_type == FuzzySkinType::AllWalls);

    return is_contour ? fuzzify_contours : fuzzify_holes;
}


static ClipperLib_Z::Paths clip_extrusion(const ClipperLib_Z::Path& subject, const ClipperLib_Z::Paths& clip, ClipperLib_Z::ClipType clipType)
{
    ClipperLib_Z::Clipper clipper;
    clipper.ZFillFunction([](const ClipperLib_Z::IntPoint& e1bot, const ClipperLib_Z::IntPoint& e1top, const ClipperLib_Z::IntPoint& e2bot,
        const ClipperLib_Z::IntPoint& e2top, ClipperLib_Z::IntPoint& pt) {
            ClipperLib_Z::IntPoint start = e1bot;
            ClipperLib_Z::IntPoint end = e1top;

            if (start.z() <= 0 && end.z() <= 0) {
                start = e2bot;
                end = e2top;
            }

            assert(start.z() >= 0 && end.z() >= 0);

            // Interpolate extrusion line width.
            double length_sqr = (end - start).cast<double>().squaredNorm();
            double dist_sqr = (pt - start).cast<double>().squaredNorm();
            double t = std::sqrt(dist_sqr / length_sqr);

            pt.z() = start.z() + coord_t((end.z() - start.z()) * t);
        });

    clipper.AddPath(subject, ClipperLib_Z::ptSubject, false);
    clipper.AddPaths(clip, ClipperLib_Z::ptClip, true);

    ClipperLib_Z::Paths    clipped_paths;
    {
        ClipperLib_Z::PolyTree clipped_polytree;
        clipper.Execute(clipType, clipped_polytree, ClipperLib_Z::pftNonZero, ClipperLib_Z::pftNonZero);
        ClipperLib_Z::PolyTreeToPaths(std::move(clipped_polytree), clipped_paths);
    }

    // Clipped path could contain vertices from the clip with a Z coordinate equal to zero.
    // For those vertices, we must assign value based on the subject.
    // This happens only in sporadic cases.
    for (ClipperLib_Z::Path& path : clipped_paths)
        for (ClipperLib_Z::IntPoint& c_pt : path)
            if (c_pt.z() == 0) {
                // Now we must find the corresponding line on with this point is located and compute line width (Z coordinate).
                if (subject.size() <= 2)
                    continue;

                const Point pt(c_pt.x(), c_pt.y());
                Point       projected_pt_min;
                auto        it_min = subject.begin();
                auto        dist_sqr_min = std::numeric_limits<double>::max();
                Point       prev(subject.front().x(), subject.front().y());
                for (auto it = std::next(subject.begin()); it != subject.end(); ++it) {
                    Point curr(it->x(), it->y());
                    Point projected_pt = pt.projection_onto(Line(prev, curr));
                    if (double dist_sqr = (projected_pt - pt).cast<double>().squaredNorm(); dist_sqr < dist_sqr_min) {
                        dist_sqr_min = dist_sqr;
                        projected_pt_min = projected_pt;
                        it_min = std::prev(it);
                    }
                    prev = curr;
                }

                assert(dist_sqr_min <= SCALED_EPSILON);
                assert(std::next(it_min) != subject.end());

                const Point  pt_a(it_min->x(), it_min->y());
                const Point  pt_b(std::next(it_min)->x(), std::next(it_min)->y());
                const double line_len = (pt_b - pt_a).cast<double>().norm();
                const double dist = (projected_pt_min - pt_a).cast<double>().norm();
                c_pt.z() = coord_t(double(it_min->z()) + (dist / line_len) * double(std::next(it_min)->z() - it_min->z()));
            }

    assert([&clipped_paths = std::as_const(clipped_paths)]() -> bool {
        for (const ClipperLib_Z::Path& path : clipped_paths)
            for (const ClipperLib_Z::IntPoint& pt : path)
                if (pt.z() <= 0)
                    return false;
        return true;
    }());

    return clipped_paths;
}

// struct PerimeterGeneratorArachneExtrusion
// {
//     Arachne::ExtrusionLine* extrusion = nullptr;
//     // Indicates if closed ExtrusionLine is a contour or a hole. Used it only when ExtrusionLine is a closed loop.
//     bool is_contour = false;
// };


Polylines reconnect_polylines(const Polylines &polylines, double limit_distance)
{
    if (polylines.empty())
        return polylines;

    std::unordered_map<size_t, Polyline> connected;
    connected.reserve(polylines.size());
    for (size_t i = 0; i < polylines.size(); i++) {
        if (!polylines[i].empty()) {
            connected.emplace(i, polylines[i]);
        }
    }

    for (size_t a = 0; a < polylines.size(); a++) {
        if (connected.find(a) == connected.end()) {
            continue;
        }
        Polyline &base = connected.at(a);
        for (size_t b = a + 1; b < polylines.size(); b++) {
            if (connected.find(b) == connected.end()) {
                continue;
            }
            Polyline &next = connected.at(b);
            if ((base.last_point() - next.first_point()).cast<double>().squaredNorm() < limit_distance * limit_distance) {
                base.append(std::move(next));
                connected.erase(b);
            } else if ((base.last_point() - next.last_point()).cast<double>().squaredNorm() < limit_distance * limit_distance) {
                base.points.insert(base.points.end(), next.points.rbegin(), next.points.rend());
                connected.erase(b);
            } else if ((base.first_point() - next.last_point()).cast<double>().squaredNorm() < limit_distance * limit_distance) {
                next.append(std::move(base));
                base = std::move(next);
                base.reverse();
                connected.erase(b);
            } else if ((base.first_point() - next.first_point()).cast<double>().squaredNorm() < limit_distance * limit_distance) {
                base.reverse();
                base.append(std::move(next));
                base.reverse();
                connected.erase(b);
            }
        }
    }

    Polylines result;
    for (auto &ext : connected) {
        result.push_back(std::move(ext.second));
    }

    return result;
}



#define EXTRA_PERIMETER_OFFSET_PARAMETERS ClipperLib::jtSquare, 0.
// #define EXTRA_PERIM_DEBUG_FILES
// Function will generate extra perimeters clipped over nonbridgeable areas of the provided surface and returns both the new perimeters and
// Polygons filled by those clipped perimeters
// std::tuple<std::vector<ExtrusionPaths>, Polygons> generate_extra_perimeters_over_overhangs(ExPolygons               infill_area,
//                                                                                            const Polygons          &lower_slices_polygons,
//                                                                                            int                      perimeter_count,
//                                                                                            const Flow              &overhang_flow,
//                                                                                            double                   scaled_resolution,
//                                                                                            const PrintObjectConfig &object_config,
//                                                                                            const PrintConfig       &print_config)
// {
//     coord_t anchors_size = std::min(coord_t(scale_(EXTERNAL_INFILL_MARGIN)), overhang_flow.scaled_spacing() * (perimeter_count + 1));
//
//     BoundingBox infill_area_bb = get_extents(infill_area).inflated(SCALED_EPSILON);
//     Polygons optimized_lower_slices = ClipperUtils::clip_clipper_polygons_with_subject_bbox(lower_slices_polygons, infill_area_bb);
//     Polygons overhangs  = diff(infill_area, optimized_lower_slices);
//
//     if (overhangs.empty()) { return {}; }
//
//     AABBTreeLines::LinesDistancer<Line> lower_layer_aabb_tree{to_lines(optimized_lower_slices)};
//     Polygons                            anchors             = intersection(infill_area, optimized_lower_slices);
//     Polygons                            inset_anchors       = diff(anchors,
//                                                                    expand(overhangs, anchors_size + 0.1 * overhang_flow.scaled_width(), EXTRA_PERIMETER_OFFSET_PARAMETERS));
//     Polygons                            inset_overhang_area = diff(infill_area, inset_anchors);
//
// #ifdef EXTRA_PERIM_DEBUG_FILES
//     {
//         BoundingBox bbox = get_extents(inset_overhang_area);
//         bbox.offset(scale_(1.));
//         ::Slic3r::SVG svg(debug_out_path("inset_overhang_area").c_str(), bbox);
//         for (const Line &line : to_lines(inset_anchors)) svg.draw(line, "purple", scale_(0.25));
//         for (const Line &line : to_lines(inset_overhang_area)) svg.draw(line, "red", scale_(0.15));
//         svg.Close();
//     }
// #endif
//
//     Polygons inset_overhang_area_left_unfilled;
//
//     std::vector<ExtrusionPaths> extra_perims; // overhang region -> extrusion paths
//     for (const ExPolygon &overhang : union_ex(to_expolygons(inset_overhang_area))) {
//         Polygons overhang_to_cover = to_polygons(overhang);
//         Polygons expanded_overhang_to_cover = expand(overhang_to_cover, 1.1 * overhang_flow.scaled_spacing());
//         Polygons shrinked_overhang_to_cover = shrink(overhang_to_cover, 0.1 * overhang_flow.scaled_spacing());
//
//         Polygons real_overhang = intersection(overhang_to_cover, overhangs);
//         if (real_overhang.empty()) {
//             inset_overhang_area_left_unfilled.insert(inset_overhang_area_left_unfilled.end(), overhang_to_cover.begin(),
//                                                      overhang_to_cover.end());
//             continue;
//         }
//         ExtrusionPaths &overhang_region = extra_perims.emplace_back();
//
//         Polygons anchoring         = intersection(expanded_overhang_to_cover, inset_anchors);
//         Polygons perimeter_polygon = offset(union_(expand(overhang_to_cover, 0.1 * overhang_flow.scaled_spacing()), anchoring),
//                                             -overhang_flow.scaled_spacing() * 0.6);
//
//         Polygon anchoring_convex_hull = Geometry::convex_hull(anchoring);
//         double  unbridgeable_area     = area(diff(real_overhang, {anchoring_convex_hull}));
//
//        // auto [dir, unsupp_dist] = detect_bridging_direction(real_overhang, anchors);
//
// #ifdef EXTRA_PERIM_DEBUG_FILES
//         {
//             BoundingBox bbox = get_extents(anchoring_convex_hull);
//             bbox.offset(scale_(1.));
//             ::Slic3r::SVG svg(debug_out_path("bridge_check").c_str(), bbox);
//             for (const Line &line : to_lines(perimeter_polygon)) svg.draw(line, "purple", scale_(0.25));
//             for (const Line &line : to_lines(real_overhang)) svg.draw(line, "red", scale_(0.20));
//             for (const Line &line : to_lines(anchoring_convex_hull)) svg.draw(line, "green", scale_(0.15));
//             for (const Line &line : to_lines(anchoring)) svg.draw(line, "yellow", scale_(0.10));
//             for (const Line &line : to_lines(diff_ex(perimeter_polygon, {anchoring_convex_hull}))) svg.draw(line, "black", scale_(0.10));
//             for (const Line &line : to_lines(diff_pl(to_polylines(diff(real_overhang, anchors)), expand(anchors, float(SCALED_EPSILON)))))
//                 svg.draw(line, "blue", scale_(0.30));
//             svg.Close();
//         }
// #endif
//
//         if (unbridgeable_area < 0.2 * area(real_overhang) && unsupp_dist < total_length(real_overhang) * 0.2) {
//             inset_overhang_area_left_unfilled.insert(inset_overhang_area_left_unfilled.end(),overhang_to_cover.begin(),overhang_to_cover.end());
//             perimeter_polygon.clear();
//         } else {
//             //  fill the overhang with perimeters
//             int continuation_loops = 2;
//             while (continuation_loops >= 0) {
//                 auto prev = perimeter_polygon;
//                 // prepare next perimeter lines
//                 Polylines perimeter = intersection_pl(to_polylines(perimeter_polygon), shrinked_overhang_to_cover);
//
//                 // do not add the perimeter to result yet, first check if perimeter_polygon is not empty after shrinking - this would mean
//                 //  that the polygon was possibly too small for full perimeter loop and in that case try gap fill first
//                 perimeter_polygon = union_(perimeter_polygon, anchoring);
//                 perimeter_polygon = intersection(offset(perimeter_polygon, -overhang_flow.scaled_spacing()), expanded_overhang_to_cover);
//
//                 if (perimeter_polygon.empty()) { // fill possible gaps of single extrusion width
//                     Polygons shrinked = intersection(offset(prev, -0.3 * overhang_flow.scaled_spacing()), expanded_overhang_to_cover);
//                     if (!shrinked.empty()) {
//                         extrusion_paths_append(overhang_region, reconnect_polylines(perimeter, overhang_flow.scaled_spacing()),
//                                                ExtrusionRole::erOverhangPerimeter, overhang_flow.mm3_per_mm(), overhang_flow.width(),
//                                                overhang_flow.height());
//                     }
//
//                     Polylines  fills;
//                     ExPolygons gap = shrinked.empty() ? offset_ex(prev, overhang_flow.scaled_spacing() * 0.5) : to_expolygons(shrinked);
//
//                     for (const ExPolygon &ep : gap) {
//                         ep.medial_axis(0.75 * overhang_flow.scaled_width(), 3.0 * overhang_flow.scaled_spacing(), &fills);
//                     }
//                     if (!fills.empty()) {
//                         fills = intersection_pl(fills, shrinked_overhang_to_cover);
//                         extrusion_paths_append(overhang_region, reconnect_polylines(fills, overhang_flow.scaled_spacing()),
//                                                ExtrusionRole::erOverhangPerimeter, overhang_flow.mm3_per_mm(), overhang_flow.width(),
//                                                overhang_flow.height());
//                     }
//                     break;
//                 } else {
//                     extrusion_paths_append(overhang_region, reconnect_polylines(perimeter, overhang_flow.scaled_spacing()),
//                                            ExtrusionRole::erOverhangPerimeter, overhang_flow.mm3_per_mm(), overhang_flow.width(),
//                                            overhang_flow.height());
//                 }
//
//                 if (intersection(perimeter_polygon, real_overhang).empty()) { continuation_loops--; }
//
//                 if (prev == perimeter_polygon) {
// #ifdef EXTRA_PERIM_DEBUG_FILES
//                     BoundingBox bbox = get_extents(perimeter_polygon);
//                     bbox.offset(scale_(5.));
//                     ::Slic3r::SVG svg(debug_out_path("perimeter_polygon").c_str(), bbox);
//                     for (const Line &line : to_lines(perimeter_polygon)) svg.draw(line, "blue", scale_(0.25));
//                     for (const Line &line : to_lines(overhang_to_cover)) svg.draw(line, "red", scale_(0.20));
//                     for (const Line &line : to_lines(real_overhang)) svg.draw(line, "green", scale_(0.15));
//                     for (const Line &line : to_lines(anchoring)) svg.draw(line, "yellow", scale_(0.10));
//                     svg.Close();
// #endif
//                     break;
//                 }
//             }
//
//             perimeter_polygon = expand(perimeter_polygon, 0.5 * overhang_flow.scaled_spacing());
//             perimeter_polygon = union_(perimeter_polygon, anchoring);
//             inset_overhang_area_left_unfilled.insert(inset_overhang_area_left_unfilled.end(), perimeter_polygon.begin(),perimeter_polygon.end());
//
// #ifdef EXTRA_PERIM_DEBUG_FILES
//             BoundingBox bbox = get_extents(inset_overhang_area);
//             bbox.offset(scale_(2.));
//             ::Slic3r::SVG svg(debug_out_path("pre_final").c_str(), bbox);
//             for (const Line &line : to_lines(perimeter_polygon)) svg.draw(line, "blue", scale_(0.05));
//             for (const Line &line : to_lines(anchoring)) svg.draw(line, "green", scale_(0.05));
//             for (const Line &line : to_lines(overhang_to_cover)) svg.draw(line, "yellow", scale_(0.05));
//             for (const Line &line : to_lines(inset_overhang_area_left_unfilled)) svg.draw(line, "red", scale_(0.05));
//             svg.Close();
// #endif
//             overhang_region.erase(std::remove_if(overhang_region.begin(), overhang_region.end(),
//                                                  [](const ExtrusionPath &p) { return p.empty(); }),
//                                   overhang_region.end());
//
//             if (!overhang_region.empty()) {
//                 // there is a special case, where the first (or last) generated overhang perimeter eats all anchor space.
//                 // When this happens, the first overhang perimeter is also a closed loop, and needs special check
//                 // instead of the following simple is_anchored lambda, which checks only the first and last point (not very useful on closed
//                 // polyline)
//                 bool first_overhang_is_closed_and_anchored =
//                     (overhang_region.front().first_point() == overhang_region.front().last_point() &&
//                      !intersection_pl(overhang_region.front().polyline, optimized_lower_slices).empty());
//
//                 auto is_anchored = [&lower_layer_aabb_tree](const ExtrusionPath &path) {
//                     return lower_layer_aabb_tree.distance_from_lines<true>(path.first_point()) <= 0 ||
//                            lower_layer_aabb_tree.distance_from_lines<true>(path.last_point()) <= 0;
//                 };
//                 if (!first_overhang_is_closed_and_anchored) {
//                     std::reverse(overhang_region.begin(), overhang_region.end());
//                 } else {
//                     size_t min_dist_idx = 0;
//                     double min_dist = std::numeric_limits<double>::max();
//                     for (size_t i = 0; i < overhang_region.front().polyline.size(); i++) {
//                         Point p = overhang_region.front().polyline[i];
//                         if (double d = lower_layer_aabb_tree.distance_from_lines<true>(p) < min_dist) {
//                             min_dist = d;
//                             min_dist_idx = i;
//                         }
//                     }
//                     std::rotate(overhang_region.front().polyline.begin(), overhang_region.front().polyline.begin() + min_dist_idx,
//                                 overhang_region.front().polyline.end());
//                 }
//                 auto first_unanchored          = std::stable_partition(overhang_region.begin(), overhang_region.end(), is_anchored);
//                 int  index_of_first_unanchored = first_unanchored - overhang_region.begin();
//                 overhang_region = sort_extra_perimeters(overhang_region, index_of_first_unanchored, overhang_flow.scaled_spacing());
//             }
//         }
//     }
//
// #ifdef EXTRA_PERIM_DEBUG_FILES
//     BoundingBox bbox = get_extents(inset_overhang_area);
//     bbox.offset(scale_(2.));
//     ::Slic3r::SVG svg(debug_out_path(("final" + std::to_string(rand())).c_str()).c_str(), bbox);
//     for (const Line &line : to_lines(inset_overhang_area_left_unfilled)) svg.draw(line, "blue", scale_(0.05));
//     for (const Line &line : to_lines(inset_overhang_area)) svg.draw(line, "green", scale_(0.05));
//     for (const Line &line : to_lines(diff(inset_overhang_area, inset_overhang_area_left_unfilled))) svg.draw(line, "yellow", scale_(0.05));
//     svg.Close();
// #endif
//
//     inset_overhang_area_left_unfilled = union_(inset_overhang_area_left_unfilled);
//
//     return {extra_perims, diff(inset_overhang_area, inset_overhang_area_left_unfilled)};
// }

// void PerimeterGenerator::apply_extra_perimeters(ExPolygons &infill_area)
// {
//     if (!m_spiral_vase && this->lower_slices != nullptr && this->config->detect_overhang_wall && this->config->extra_perimeters_on_overhangs &&
//         this->config->wall_loops > 0 && this->layer_id > this->object_config->raft_layers) {
//         // Generate extra perimeters on overhang areas, and cut them to these parts only, to save print time and material
//         auto [extra_perimeters, filled_area] = generate_extra_perimeters_over_overhangs(infill_area, this->lower_slices_polygons(),
//                                                                                         this->config->wall_loops, this->overhang_flow,
//                                                                                         this->m_scaled_resolution, *this->object_config,
//                                                                                         *this->print_config);
//         if (!extra_perimeters.empty()) {
//             ExtrusionEntityCollection *this_islands_perimeters = static_cast<ExtrusionEntityCollection *>(this->loops->entities.back());
//             ExtrusionEntityCollection  new_perimeters{};
//             new_perimeters.no_sort = this_islands_perimeters->no_sort;
//             for (const ExtrusionPaths &paths : extra_perimeters) {
//                 new_perimeters.append(paths);
//             }
//             new_perimeters.append(this_islands_perimeters->entities);
//             this_islands_perimeters->swap(new_perimeters);
//
//             SurfaceCollection orig_surfaces = *this->fill_surfaces;
//             this->fill_surfaces->clear();
//             for (const auto &surface : orig_surfaces.surfaces) {
//                 auto new_surfaces = diff_ex({surface.expolygon}, filled_area);
//                 this->fill_surfaces->append(new_surfaces, surface);
//             }
//         }
//     }
// }

// Reorient loop direction
// static void reorient_perimeters(ExtrusionEntityCollection &entities, bool steep_overhang_contour, bool steep_overhang_hole, bool reverse_internal_only)
// {
//     if (steep_overhang_hole || steep_overhang_contour) {
//         for (auto entity : entities) {
//             if (entity->is_loop()) {
//                 ExtrusionLoop *eloop = static_cast<ExtrusionLoop *>(entity);
//                 // Only reverse when needed
//                 bool need_reverse = ((eloop->loop_role() & elrHole) == elrHole) ? steep_overhang_hole : steep_overhang_contour;
//
//                 bool isExternal = false;
//                 if(reverse_internal_only){
//                     for(auto path : eloop->paths){
//                         if(path.role() == erExternalPerimeter){
//                             isExternal = true;
//                             break;
//                         }
//                     }
//                 }
//
//                 if (need_reverse && !isExternal) {
//                     eloop->make_clockwise();
//                 }
//             }
//         }
//     }
// }

static void group_region_by_fuzzify(PerimeterGenerator& g)
{
    g.regions_by_fuzzify.clear();
    g.has_fuzzy_skin = false;
    g.has_fuzzy_hole = false;

    std::unordered_map<FuzzySkinConfig, SurfacesPtr> regions;
    for (auto region : *g.compatible_regions) {
        const auto& region_config = region->region().config();
        const FuzzySkinConfig cfg{
            region_config.fuzzy_skin,
            scaled<coord_t>(region_config.fuzzy_skin_thickness.value),
            scaled<coord_t>(region_config.fuzzy_skin_point_distance.value),
            region_config.fuzzy_skin_first_layer,
            region_config.fuzzy_skin_noise_type,
            region_config.fuzzy_skin_scale,
            region_config.fuzzy_skin_octaves,
            region_config.fuzzy_skin_persistence
        };
        auto& surfaces = regions[cfg];
        for (const auto& surface : region->slices.surfaces) {
            surfaces.push_back(&surface);
        }

        if (cfg.type != FuzzySkinType::None) {
            g.has_fuzzy_skin = true;
            if (cfg.type != FuzzySkinType::External) {
                g.has_fuzzy_hole = true;
            }
        }
    }

    if (regions.size() == 1) { // optimization
        g.regions_by_fuzzify[regions.begin()->first] = {};
        return;
    }

    for (auto& it : regions) {
        g.regions_by_fuzzify[it.first] = offset_ex(it.second, ClipperSafetyOffset);
    }
}

// void PerimeterGenerator::process_classic()
// {
//     group_region_by_fuzzify(*this);
//
//     // other perimeters
//     m_mm3_per_mm               		= this->perimeter_flow.mm3_per_mm();
//     coord_t perimeter_width         = this->perimeter_flow.scaled_width();
//     coord_t perimeter_spacing       = this->perimeter_flow.scaled_spacing();
//
//     // external perimeters
//     m_ext_mm3_per_mm           		= this->ext_perimeter_flow.mm3_per_mm();
//     coord_t ext_perimeter_width     = this->ext_perimeter_flow.scaled_width();
//
//     coord_t ext_perimeter_spacing   = this->ext_perimeter_flow.scaled_spacing();
//     coord_t ext_perimeter_spacing2;
//     // Orca: ignore precise_outer_wall if wall_sequence is not InnerOuter
//     if(config->precise_outer_wall)
//         ext_perimeter_spacing2 = scaled<coord_t>(0.5f * (this->ext_perimeter_flow.width() + this->perimeter_flow.width()));
//     else
//         ext_perimeter_spacing2 = scaled<coord_t>(0.5f * (this->ext_perimeter_flow.spacing() + this->perimeter_flow.spacing()));
//
//     // overhang perimeters
//     m_mm3_per_mm_overhang      		= this->overhang_flow.mm3_per_mm();
//
//     // solid infill
//     coord_t solid_infill_spacing    = this->solid_infill_flow.scaled_spacing();
//
//     // prepare grown lower layer slices for overhang detection
//     if (this->lower_slices != nullptr && this->config->detect_overhang_wall) {
//         // We consider overhang any part where the entire nozzle diameter is not supported by the
//         // lower layer, so we take lower slices and offset them by half the nozzle diameter used
//         // in the current layer
//         double nozzle_diameter = this->print_config->nozzle_diameter.get_at(this->config->wall_filament - 1);
//         m_lower_slices_polygons = offset(*this->lower_slices, float(scale_(+nozzle_diameter / 2)));
//     }
//
//     // Calculate the minimum required spacing between two adjacent traces.
//     // This should be equal to the nominal flow spacing but we experiment
//     // with some tolerance in order to avoid triggering medial axis when
//     // some squishing might work. Loops are still spaced by the entire
//     // flow spacing; this only applies to collapsing parts.
//     // For ext_min_spacing we use the ext_perimeter_spacing calculated for two adjacent
//     // external loops (which is the correct way) instead of using ext_perimeter_spacing2
//     // which is the spacing between external and internal, which is not correct
//     // and would make the collapsing (thus the details resolution) dependent on
//     // internal flow which is unrelated.
//     coord_t min_spacing         = coord_t(perimeter_spacing      * (1 - INSET_OVERLAP_TOLERANCE));
//     coord_t ext_min_spacing     = coord_t(ext_perimeter_spacing  * (1 - INSET_OVERLAP_TOLERANCE));
//     bool    has_gap_fill 		= this->config->gap_infill_speed.value > 0;
//
//     // BBS: this flow is for smaller external perimeter for small area
//     coord_t ext_min_spacing_smaller = coord_t(ext_perimeter_spacing * (1 - SMALLER_EXT_INSET_OVERLAP_TOLERANCE));
//     this->smaller_ext_perimeter_flow = this->ext_perimeter_flow;
//     // BBS: to be checked
//     this->smaller_ext_perimeter_flow = this->smaller_ext_perimeter_flow.with_width(SCALING_FACTOR *
//         (ext_perimeter_width - 0.5 * SMALLER_EXT_INSET_OVERLAP_TOLERANCE * ext_perimeter_spacing));
//     m_ext_mm3_per_mm_smaller_width = this->smaller_ext_perimeter_flow.mm3_per_mm();
//
//     // prepare grown lower layer slices for overhang detection
//     m_lower_polygons_series = generate_lower_polygons_series(this->perimeter_flow.width());
//     m_lower_overhang_dist_boundary = dist_boundary(this->perimeter_flow.width());
//     if (ext_perimeter_width == perimeter_width){
//         m_external_lower_polygons_series = m_lower_polygons_series;
//         m_external_overhang_dist_boundary=m_lower_overhang_dist_boundary;
//     } else {
//         m_external_lower_polygons_series = generate_lower_polygons_series(this->ext_perimeter_flow.width());
//         m_external_overhang_dist_boundary = dist_boundary(this->ext_perimeter_flow.width());
//     }
//     m_smaller_external_lower_polygons_series = generate_lower_polygons_series(this->smaller_ext_perimeter_flow.width());
//     m_smaller_external_overhang_dist_boundary = dist_boundary(this->smaller_ext_perimeter_flow.width());
//     // we need to process each island separately because we might have different
//     // extra perimeters for each one
//     Surfaces all_surfaces = this->slices->surfaces;
//
//     process_no_bridge(all_surfaces, perimeter_spacing, ext_perimeter_width);
//     // BBS: don't simplify too much which influence arc fitting when export gcode if arc_fitting is enabled
//     double surface_simplify_resolution = (print_config->enable_arc_fitting && !this->has_fuzzy_skin) ? 0.2 * m_scaled_resolution : m_scaled_resolution;
//     //BBS: reorder the surface to reduce the travel time
//     ExPolygons surface_exp;
//     for (const Surface &surface : all_surfaces)
//         surface_exp.push_back(surface.expolygon);
//     std::vector<size_t> surface_order = chain_expolygons(surface_exp);
//     for (size_t order_idx = 0; order_idx < surface_order.size(); order_idx++) {
//         const Surface &surface = all_surfaces[surface_order[order_idx]];
//         // detect how many perimeters must be generated for this island
//         int loop_number = this->config->wall_loops + surface.extra_perimeters - 1;  // 0-indexed loops
//         int sparse_infill_density = this->config->sparse_infill_density.value;
//         if (this->config->alternate_extra_wall && this->layer_id % 2 == 1 && !m_spiral_vase && sparse_infill_density > 0) // add alternating extra wall
//             loop_number++;
//         if (this->layer_id == object_config->raft_layers && this->config->only_one_wall_first_layer)
//             loop_number = 0;
//         // Set the topmost layer to be one wall
//         if (loop_number > 0 && config->only_one_wall_top && this->upper_slices == nullptr)
//             loop_number = 0;
//
//         ExPolygons last        = union_ex(surface.expolygon.simplify_p(surface_simplify_resolution));
//         ExPolygons gaps;
//         ExPolygons top_fills;
//         ExPolygons fill_clip;
//         if (loop_number >= 0) {
//             // In case no perimeters are to be generated, loop_number will equal to -1.
//             std::vector<PerimeterGeneratorLoops> contours(loop_number+1);    // depth => loops
//             std::vector<PerimeterGeneratorLoops> holes(loop_number+1);       // depth => loops
//             ThickPolylines thin_walls;
//             // we loop one time more than needed in order to find gaps after the last perimeter was applied
//             for (int i = 0;; ++ i) {  // outer loop is 0
//                 // Calculate next onion shell of perimeters.
//                 ExPolygons offsets;
//                 ExPolygons offsets_with_smaller_width;
//                 if (i == 0) {
//                     // look for thin walls
//                     if (this->config->detect_thin_wall) {
//                         // the minimum thickness of a single loop is:
//                         // ext_width/2 + ext_spacing/2 + spacing/2 + width/2
//                         offsets = offset2_ex(last,
//                             -float(ext_perimeter_width / 2. + ext_min_spacing / 2. - 1),
//                             +float(ext_min_spacing / 2. - 1));
//                         // the following offset2 ensures almost nothing in @thin_walls is narrower than $min_width
//                         // (actually, something larger than that still may exist due to mitering or other causes)
//                         coord_t min_width = coord_t(scale_(this->ext_perimeter_flow.nozzle_diameter() / 3));
//                         ExPolygons expp = opening_ex(
//                             // medial axis requires non-overlapping geometry
//                             diff_ex(last, offset(offsets, float(ext_perimeter_width / 2.) + ClipperSafetyOffset)),
//                             float(min_width / 2.));
//                         // the maximum thickness of our thin wall area is equal to the minimum thickness of a single loop
//                         for (ExPolygon &ex : expp)
//                             ex.medial_axis(min_width, ext_perimeter_width + ext_perimeter_spacing2, &thin_walls);
//                     } else {
//                         coord_t ext_perimeter_smaller_width = this->smaller_ext_perimeter_flow.scaled_width();
//                         for (const ExPolygon& expolygon : last) {
//                             // BBS: judge whether it's narrow but not too long island which is hard to place two line
//                             ExPolygons expolys;
//                             expolys.push_back(expolygon);
//                             ExPolygons offset_result = offset2_ex(expolys,
//                                 -float(ext_perimeter_width / 2. + ext_min_spacing_smaller / 2.),
//                                 +float(ext_min_spacing_smaller / 2.));
//                             if (offset_result.empty() &&
//                                 expolygon.area() < (double)(ext_perimeter_width + ext_min_spacing_smaller) * scale_(narrow_loop_length_threshold)) {
//                                 // BBS: for narrow external loop, use smaller line width
//                                 ExPolygons temp_result = offset_ex(expolygon, -float(ext_perimeter_smaller_width / 2.));
//                                 offsets_with_smaller_width.insert(offsets_with_smaller_width.end(), temp_result.begin(), temp_result.end());
//                             }
//                             else {
//                                 //BBS: for not narrow loop, use normal external perimeter line width
//                                 ExPolygons temp_result = offset_ex(expolygon, -float(ext_perimeter_width / 2.));
//                                 offsets.insert(offsets.end(), temp_result.begin(), temp_result.end());
//                             }
//                         }
//                     }
//                     if (m_spiral_vase && (offsets.size() > 1 || offsets_with_smaller_width.size() > 1)) {
//                         // Remove all but the largest area polygon.
//                         keep_largest_contour_only(offsets);
//                         //BBS
//                         if (offsets.empty())
//                             //BBS: only have small width loop, then keep the largest in spiral vase mode
//                             keep_largest_contour_only(offsets_with_smaller_width);
//                         else
//                             //BBS: have large area, clean the small width loop
//                             offsets_with_smaller_width.clear();
//                     }
//                 } else {
//                     //FIXME Is this offset correct if the line width of the inner perimeters differs
//                     // from the line width of the infill?
//                     coord_t distance = (i == 1) ? ext_perimeter_spacing2 : perimeter_spacing;
//                     //BBS
//                     //offsets = this->config->thin_walls ?
//                         // This path will ensure, that the perimeters do not overfill, as in
//                         // prusa3d/Slic3r GH #32, but with the cost of rounding the perimeters
//                         // excessively, creating gaps, which then need to be filled in by the not very
//                         // reliable gap fill algorithm.
//                         // Also the offset2(perimeter, -x, x) may sometimes lead to a perimeter, which is larger than
//                         // the original.
//                         //offset2_ex(last,
//                         //        - float(distance + min_spacing / 2. - 1.),
//                         //        float(min_spacing / 2. - 1.)) :
//                         // If "detect thin walls" is not enabled, this paths will be entered, which
//                         // leads to overflows, as in prusa3d/Slic3r GH #32
//                         //offset_ex(last, - float(distance));
//
//                     //BBS: For internal perimeter, we should "enable" thin wall strategy in which offset2 is used to
//                     // remove too closed line, so that gap fill can be used for such internal narrow area in following
//                     // handling.
//                     offsets = offset2_ex(last,
//                         -float(distance + min_spacing / 2. - 1.),
//                         float(min_spacing / 2. - 1.));
//                     // look for gaps
//                     if (has_gap_fill)
//                         // not using safety offset here would "detect" very narrow gaps
//                         // (but still long enough to escape the area threshold) that gap fill
//                         // won't be able to fill but we'd still remove from infill area
//                         append(gaps, diff_ex(
//                             offset(last,    - float(0.5 * distance)),
//                             offset(offsets,   float(0.5 * distance + 10))));  // safety offset
//                 }
//                 if (offsets.empty() && offsets_with_smaller_width.empty()) {
//                     // Store the number of loops actually generated.
//                     loop_number = i - 1;
//                     // No region left to be filled in.
//                     last.clear();
//                     break;
//                 } else if (i > loop_number) {
//                     // If i > loop_number, we were looking just for gaps.
//                     break;
//                 }
//                 {
//                     for (const ExPolygon& expolygon : offsets) {
//                         // Outer contour may overlap with an inner contour,
//                         // inner contour may overlap with another inner contour,
//                         // outer contour may overlap with itself.
//                         //FIXME evaluate the overlaps, annotate each point with an overlap depth,
//                         // compensate for the depth of intersection.
//                         contours[i].emplace_back(expolygon.contour, i, true);
//
//                         if (!expolygon.holes.empty()) {
//                             holes[i].reserve(holes[i].size() + expolygon.holes.size());
//                             for (const Polygon& hole : expolygon.holes)
//                                 holes[i].emplace_back(hole, i, false);
//                         }
//                     }
//
//                     //BBS: save perimeter loop which use smaller width
//                     if (i == 0) {
//                         for (const ExPolygon& expolygon : offsets_with_smaller_width) {
//                             contours[i].emplace_back(PerimeterGeneratorLoop(expolygon.contour, i, true, true));
//                             if (!expolygon.holes.empty()) {
//                                 holes[i].reserve(holes[i].size() + expolygon.holes.size());
//                                 for (const Polygon& hole : expolygon.holes)
//                                     holes[i].emplace_back(PerimeterGeneratorLoop(hole, i, false, true));
//                             }
//                         }
//                     }
//                 }
//
//                 last = std::move(offsets);
//
//                 //BBS: refer to superslicer
//                 //store surface for top infill if only_one_wall_top
//                 if (i == 0 && i!=loop_number && config->only_one_wall_top && !surface.is_bridge() && this->upper_slices != NULL) {
//                     this->split_top_surfaces(last, top_fills, last, fill_clip);
//                 }
//
//                 if (i == loop_number && (! has_gap_fill || this->config->sparse_infill_density.value == 0)) {
//                 	// The last run of this loop is executed to collect gaps for gap fill.
//                 	// As the gap fill is either disabled or not
//                 	break;
//                 }
//             }
//
//             // nest loops: holes first
//             for (int d = 0; d <= loop_number; ++ d) {
//                 PerimeterGeneratorLoops &holes_d = holes[d];
//                 // loop through all holes having depth == d
//                 for (int i = 0; i < (int)holes_d.size(); ++ i) {
//                     const PerimeterGeneratorLoop &loop = holes_d[i];
//                     // find the hole loop that contains this one, if any
//                     for (int t = d + 1; t <= loop_number; ++ t) {
//                         for (int j = 0; j < (int)holes[t].size(); ++ j) {
//                             PerimeterGeneratorLoop &candidate_parent = holes[t][j];
//                             if (candidate_parent.polygon.contains(loop.polygon.first_point())) {
//                                 candidate_parent.children.push_back(loop);
//                                 holes_d.erase(holes_d.begin() + i);
//                                 -- i;
//                                 goto NEXT_LOOP;
//                             }
//                         }
//                     }
//                     // if no hole contains this hole, find the contour loop that contains it
//                     for (int t = loop_number; t >= 0; -- t) {
//                         for (int j = 0; j < (int)contours[t].size(); ++ j) {
//                             PerimeterGeneratorLoop &candidate_parent = contours[t][j];
//                             if (candidate_parent.polygon.contains(loop.polygon.first_point())) {
//                                 candidate_parent.children.push_back(loop);
//                                 holes_d.erase(holes_d.begin() + i);
//                                 -- i;
//                                 goto NEXT_LOOP;
//                             }
//                         }
//                     }
//                     NEXT_LOOP: ;
//                 }
//             }
//             // nest contour loops
//             for (int d = loop_number; d >= 1; -- d) {
//                 PerimeterGeneratorLoops &contours_d = contours[d];
//                 // loop through all contours having depth == d
//                 for (int i = 0; i < (int)contours_d.size(); ++ i) {
//                     const PerimeterGeneratorLoop &loop = contours_d[i];
//                     // find the contour loop that contains it
//                     for (int t = d - 1; t >= 0; -- t) {
//                         for (size_t j = 0; j < contours[t].size(); ++ j) {
//                             PerimeterGeneratorLoop &candidate_parent = contours[t][j];
//                             if (candidate_parent.polygon.contains(loop.polygon.first_point())) {
//                                 candidate_parent.children.push_back(loop);
//                                 contours_d.erase(contours_d.begin() + i);
//                                 -- i;
//                                 goto NEXT_CONTOUR;
//                             }
//                         }
//                     }
//                     NEXT_CONTOUR: ;
//                 }
//             }
//             // at this point, all loops should be in contours[0]
//             bool steep_overhang_contour = false;
//             bool steep_overhang_hole    = false;
//             const WallDirection wall_direction = config->wall_direction;
//             if (wall_direction != WallDirection::Auto) {
//                 // Skip steep overhang detection if wall direction is specified
//                 steep_overhang_contour = true;
//                 steep_overhang_hole    = true;
//             }
//             ExtrusionEntityCollection entities = traverse_loops(*this, contours.front(), thin_walls, steep_overhang_contour, steep_overhang_hole);
//             // All walls are counter-clockwise initially, so we don't need to reorient it if that's what we want
//             if (wall_direction != WallDirection::CounterClockwise) {
//                 reorient_perimeters(entities, steep_overhang_contour, steep_overhang_hole,
//                                     // Reverse internal only if the wall direction is auto
//                                     this->config->overhang_reverse_internal_only && wall_direction == WallDirection::Auto);
//             }
//
//             // if brim will be printed, reverse the order of perimeters so that
//             // we continue inwards after having finished the brim
//             // TODO: add test for perimeter order
//             bool is_outer_wall_first = this->config->wall_sequence == WallSequence::OuterInner;
//             if (is_outer_wall_first ||
//                 //BBS: always print outer wall first when there indeed has brim.
//                 (this->layer_id == 0 &&
//                     this->object_config->brim_type == BrimType::btOuterOnly &&
//                     this->object_config->brim_width.value > 0))
//                 entities.reverse();
//             // Orca: sandwich mode. Apply after 1st layer.
//             else if ((this->config->wall_sequence == WallSequence::InnerOuterInner) && layer_id > 0){
//                 entities.reverse(); // reverse all entities - order them from external to internal
//                 if(entities.entities.size()>2){ // 3 walls minimum needed to do inner outer inner ordering
//                     int position = 0; // index to run the re-ordering for multiple external perimeters in a single island.
//                     int arr_i, arr_j = 0;    // indexes to run through the walls in the for loops
//                     int outer, first_internal, second_internal, max_internal, current_perimeter; // allocate index values
//
//                     // Initiate reorder sequence to bring any index 1 (first internal) perimeters ahead of any second internal perimeters
//                     // Leaving these out of order will result in print defects on the external wall as they will be extruded prior to any
//                     // external wall. To do the re-ordering, we are creating two extrusion arrays - reordered_extrusions which will contain
//                     // the reordered extrusions and skipped_extrusions will contain the ones that were skipped in the scan
//                     ExtrusionEntityCollection reordered_extrusions, skipped_extrusions;
//                     bool found_second_internal = false; // helper variable to indicate the start of a new island
//
//                     for(auto extrusion_to_reorder : entities.entities){ //scan the perimeters to reorder
//                         switch (extrusion_to_reorder->inset_idx) {
//                             case 0: // external perimeter
//                                 if(found_second_internal){ //new island - move skipped extrusions to reordered array
//                                     for(auto extrusion_skipped : skipped_extrusions)
//                                         reordered_extrusions.append(*extrusion_skipped);
//                                     skipped_extrusions.clear();
//                                 }
//                                 reordered_extrusions.append(*extrusion_to_reorder);
//                                 break;
//                             case 1: // first internal perimeter
//                                 reordered_extrusions.append(*extrusion_to_reorder);
//                                 break;
//                             default: // second internal+ perimeter -> put them in the skipped extrusions array
//                                 skipped_extrusions.append(*extrusion_to_reorder);
//                                 found_second_internal = true;
//                                 break;
//                         }
//                     }
//                     if(entities.entities.size()>reordered_extrusions.size()){
//                         // we didnt find any more islands, so lets move the remaining skipped perimeters to the reordered extrusions list.
//                         for(auto extrusion_skipped : skipped_extrusions)
//                             reordered_extrusions.append(*extrusion_skipped);
//                         skipped_extrusions.clear();
//                     }
//
//                     // Now start the sandwich mode wall re-ordering using the reordered_extrusions as the basis
//                     // scan to find the external perimeter, first internal, second internal and last perimeter in the island.
//                     // We then advance the position index to move to the second "island" and continue until there are no more
//                     // perimeters left.
//                     while (position < reordered_extrusions.size()) {
//                         outer = first_internal = second_internal = current_perimeter = -1; // initialise all index values to -1
//                         max_internal = reordered_extrusions.size()-1; // initialise the maximum internal perimeter to the last perimeter on the extrusion list
//                         // run through the walls to get the index values that need re-ordering until the first one for each
//                         // is found. Start at "position" index to enable the for loop to iterate for multiple external
//                         // perimeters in a single island
//                         for (arr_i = position; arr_i < reordered_extrusions.size(); ++arr_i) {
//                             switch (reordered_extrusions.entities[arr_i]->inset_idx) {
//                                 case 0: // external perimeter
//                                     if (outer == -1)
//                                         outer = arr_i;
//                                     break;
//                                 case 1: // first internal wall
//                                     if (first_internal==-1 && arr_i>outer && outer!=-1){
//                                         first_internal = arr_i;
//                                     }
//                                     break;
//                                 case 2: // second internal wall
//                                     if (second_internal == -1 && arr_i > first_internal && outer!=-1){
//                                         second_internal = arr_i;
//                                     }
//                                     break;
//                             }
//                             if(outer >-1 && first_internal>-1 && second_internal>-1 && reordered_extrusions.entities[arr_i]->inset_idx == 0){ // found a new external perimeter after we've found all three perimeters to re-order -> this means we entered a new island.
//                                 arr_i=arr_i-1; //step back one perimeter
//                                 max_internal = arr_i; // new maximum internal perimeter is now this as we have found a new external perimeter, hence a new island.
//                                 break; // exit the for loop
//                             }
//                         }
//
//                         if (outer > -1 && first_internal > -1 && second_internal > -1) { // found perimeters to re-order?
//                             ExtrusionEntityCollection inner_outer_extrusions; // temporary collection to hold extrusions for reordering
//
//                             for (arr_j = max_internal; arr_j >=position; --arr_j){ // go inside out towards the external perimeter (perimeters in reverse order) and store all internal perimeters until the first one identified with inset index 2
//                                 if(arr_j >= second_internal){
//                                     inner_outer_extrusions.append(*reordered_extrusions.entities[arr_j]);
//                                     current_perimeter++;
//                                 }
//                             }
//
//                             for (arr_j = position; arr_j < second_internal; ++arr_j){ // go outside in and map the remaining perimeters (external and first internal wall(s)) using the outside in wall order
//                                 inner_outer_extrusions.append(*reordered_extrusions.entities[arr_j]);
//                             }
//
//                             for(arr_j = position; arr_j <= max_internal; ++arr_j) // replace perimeter array with the new re-ordered array
//                                 entities.replace(arr_j, *inner_outer_extrusions.entities[arr_j-position]);
//                         } else
//                             break;
//                         // go to the next perimeter from the current position to continue scanning for external walls in the same island
//                         position = arr_i + 1;
//                     }
//                 }
//             }
//
//             // append perimeters for this slice as a collection
//             if (! entities.empty())
//                 this->loops->append(entities);
//
//         } // for each loop of an island
//
//         // fill gaps
//         if (! gaps.empty()) {
//             // collapse
//             double min = 0.2 * perimeter_width * (1 - INSET_OVERLAP_TOLERANCE);
//             double max = 2. * perimeter_spacing;
//             ExPolygons gaps_ex = diff_ex(
//                 //FIXME offset2 would be enough and cheaper.
//                 opening_ex(gaps, float(min / 2.)),
//                 offset2_ex(gaps, - float(max / 2.), float(max / 2. + ClipperSafetyOffset)));
//             ThickPolylines polylines;
//             for (ExPolygon& ex : gaps_ex) {
//                 //BBS: Use DP simplify to avoid duplicated points and accelerate medial-axis calculation as well.
//                 ex.douglas_peucker(surface_simplify_resolution);
//                 ex.medial_axis(min, max, &polylines);
//             }
//
// #ifdef GAPS_OF_PERIMETER_DEBUG_TO_SVG
//             {
//                 static int irun = 0;
//                 BoundingBox bbox_svg;
//                 bbox_svg.merge(get_extents(gaps_ex));
//                 {
//                     std::stringstream stri;
//                     stri << "debug_gaps_ex_" << irun << ".svg";
//                     SVG svg(stri.str(), bbox_svg);
//                     svg.draw(to_polylines(gaps_ex), "blue", 0.5);
//                     svg.Close();
//                 }
//                 ++ irun;
//             }
// #endif
//             // SoftFever: filter out tiny gap fills
//             polylines.erase(std::remove_if(polylines.begin(), polylines.end(),
//                 [&](const ThickPolyline& p) {
//                     return p.length() < scale_(config->filter_out_gap_fill.value);
//                 }), polylines.end());
//
//
//             if (! polylines.empty()) {
// 				ExtrusionEntityCollection gap_fill;
// 				variable_width(polylines, erGapFill, this->solid_infill_flow, gap_fill.entities);
//                 /*  Make sure we don't infill narrow parts that are already gap-filled
//                     (we only consider this surface's gaps to reduce the diff() complexity).
//                     Growing actual extrusions ensures that gaps not filled by medial axis
//                     are not subtracted from fill surfaces (they might be too short gaps
//                     that medial axis skips but infill might join with other infill regions
//                     and use zigzag).  */
//                 //FIXME Vojtech: This grows by a rounded extrusion width, not by line spacing,
//                 // therefore it may cover the area, but no the volume.
//                 last = diff_ex(last, gap_fill.polygons_covered_by_width(10.f));
//                 this->gap_fill->append(std::move(gap_fill.entities));
//
// 			}
//         }
//
//         // create one more offset to be used as boundary for fill
//         // we offset by half the perimeter spacing (to get to the actual infill boundary)
//         // and then we offset back and forth by half the infill spacing to only consider the
//         // non-collapsing regions
//         coord_t inset =
//             (loop_number < 0) ? 0 :
//             (loop_number == 0) ?
//                 // one loop
//                 ext_perimeter_spacing / 2 :
//                 // two or more loops?
//                 perimeter_spacing / 2;
//
//         // only apply infill overlap if we actually have one perimeter
//         coord_t infill_peri_overlap = 0;
//         coord_t top_infill_peri_overlap = 0;
//         if (inset > 0) {
//             if(this->layer_id == 0 || this->upper_slices == nullptr){
//                 infill_peri_overlap = coord_t(scale_(this->config->top_bottom_infill_wall_overlap.get_abs_value(unscale<double>(inset + solid_infill_spacing / 2))));
//             }else{
//                 infill_peri_overlap = coord_t(scale_(this->config->infill_wall_overlap.get_abs_value(unscale<double>(inset + solid_infill_spacing / 2))));
//                 top_infill_peri_overlap = coord_t(scale_(this->config->top_bottom_infill_wall_overlap.get_abs_value(unscale<double>(inset + solid_infill_spacing / 2))));
//             }
//             inset -= infill_peri_overlap;
//         }
//         // simplify infill contours according to resolution
//         Polygons pp;
//         for (ExPolygon &ex : last)
//             ex.simplify_p(m_scaled_resolution, &pp);
//         ExPolygons not_filled_exp = union_ex(pp);
//         // collapse too narrow infill areas
//         coord_t min_perimeter_infill_spacing = coord_t(solid_infill_spacing * (1. - INSET_OVERLAP_TOLERANCE));
//
//         ExPolygons infill_exp = offset2_ex(
//             not_filled_exp,
//             float(-inset - min_perimeter_infill_spacing / 2.),
//             float(min_perimeter_infill_spacing / 2.));
//         // append infill areas to fill_surfaces
//         //if any top_fills, grow them by ext_perimeter_spacing/2 to have the real un-anchored fill
//         ExPolygons top_infill_exp = intersection_ex(fill_clip, offset_ex(top_fills, double(ext_perimeter_spacing / 2)));
//         if (!top_fills.empty()) {
//             infill_exp = union_ex(infill_exp, offset_ex(top_infill_exp, double(top_infill_peri_overlap)));
//         }
//         this->fill_surfaces->append(infill_exp, stInternal);
//
//       //  apply_extra_perimeters(infill_exp);
//
//         // BBS: get the no-overlap infill expolygons
//         {
//             ExPolygons polyWithoutOverlap;
//             if (min_perimeter_infill_spacing / 2 > infill_peri_overlap)
//                 polyWithoutOverlap = offset2_ex(
//                     not_filled_exp,
//                     float(-inset - min_perimeter_infill_spacing / 2.),
//                     float(min_perimeter_infill_spacing / 2 - infill_peri_overlap));
//             else
//                 polyWithoutOverlap = offset_ex(
//                     not_filled_exp,
//                     double(-inset - infill_peri_overlap));
//             if (!top_fills.empty())
//                 polyWithoutOverlap = union_ex(polyWithoutOverlap, top_infill_exp);
//             this->fill_no_overlap->insert(this->fill_no_overlap->end(), polyWithoutOverlap.begin(), polyWithoutOverlap.end());
//         }
//
//     } // for each island
// }

//BBS:
void PerimeterGenerator::add_infill_contour_for_arachne( ExPolygons        infill_contour,
                                                         int                loops,
                                                         coord_t            ext_perimeter_spacing,
                                                         coord_t            perimeter_spacing,
                                                         coord_t            min_perimeter_infill_spacing,
                                                         coord_t            spacing,
                                                         bool               is_inner_part)
{
    if( offset_ex(infill_contour, -float(spacing / 2.)).empty() )
    {
        infill_contour.clear(); // Infill region is too small, so let's filter it out.
    }

    // create one more offset to be used as boundary for fill
    // we offset by half the perimeter spacing (to get to the actual infill boundary)
    // and then we offset back and forth by half the infill spacing to only consider the
    // non-collapsing regions
    coord_t insert = (loops < 0) ? 0: ext_perimeter_spacing;
    if (is_inner_part || loops > 0)
        insert = perimeter_spacing;

    insert = coord_t(scale_(this->config->infill_wall_overlap.get_abs_value(unscale<double>(insert))));

    Polygons inner_pp;
    for (ExPolygon &ex : infill_contour)
        ex.simplify_p(m_scaled_resolution, &inner_pp);

    //this->fill_surfaces->append(offset2_ex(union_ex(inner_pp), float(-min_perimeter_infill_spacing / 2.), float(insert + min_perimeter_infill_spacing / 2.)), stInternal);

    //append(*this->fill_no_overlap, offset2_ex(union_ex(inner_pp), float(-min_perimeter_infill_spacing / 2.), float(+min_perimeter_infill_spacing / 2.)));
}

// Orca: sacrificial bridge layer algorithm ported from SuperSlicer
void PerimeterGenerator::process_no_bridge(Surfaces& all_surfaces, coord_t perimeter_spacing, coord_t ext_perimeter_width)
{
    //store surface for bridge infill to avoid unsupported perimeters (but the first one, this one is always good)
    if (this->config->counterbore_hole_bridging != chbNone
        && this->lower_slices != NULL && !this->lower_slices->empty()) {
        const coordf_t bridged_infill_margin = scale_(BRIDGE_INFILL_MARGIN);

        for (size_t surface_idx = 0; surface_idx < all_surfaces.size(); surface_idx++) {
            Surface* surface = &all_surfaces[surface_idx];
            ExPolygons last = { surface->expolygon };
            //compute our unsupported surface
            ExPolygons unsupported = diff_ex(last, *this->lower_slices, ApplySafetyOffset::Yes);
            if (!unsupported.empty()) {
                //remove small overhangs
                ExPolygons unsupported_filtered = offset2_ex(unsupported, double(-perimeter_spacing), double(perimeter_spacing));
                if (!unsupported_filtered.empty()) {
                    //to_draw.insert(to_draw.end(), last.begin(), last.end());
                    //extract only the useful part of the lower layer. The safety offset is really needed here.
                    ExPolygons support = diff_ex(last, unsupported, ApplySafetyOffset::Yes);
                    if (!unsupported.empty()) {
                        //only consider the part that can be bridged (really, by the bridge algorithm)
                        //first, separate into islands (ie, each ExPlolygon)
                        int numploy = 0;
                        //only consider the bottom layer that intersect unsupported, to be sure it's only on our island.
                        ExPolygonCollection lower_island(support);
                        //a detector per island
                        ExPolygons bridgeable;
                        // for (ExPolygon unsupported : unsupported_filtered) {
                        //     BridgeDetector detector{ unsupported,
                        //                             lower_island.expolygons,
                        //                             perimeter_spacing };
                        //     if (detector.detect_angle(Geometry::deg2rad(this->config->bridge_angle.value)))
                        //         expolygons_append(bridgeable, union_ex(detector.coverage(-1, true)));
                        // }
                        if (!bridgeable.empty()) {
                            //check if we get everything or just the bridgeable area
                            if (/*this->config->counterbore_hole_bridging.value == chbNoPeri || */this->config->counterbore_hole_bridging.value == chbFilled) {
                                //we bridge everything, even the not-bridgeable bits
                                for (size_t i = 0; i < unsupported_filtered.size();) {
                                    ExPolygon& poly_unsupp = *(unsupported_filtered.begin() + i);
                                    Polygons contour_simplified = poly_unsupp.contour.simplify(perimeter_spacing);
                                    ExPolygon poly_unsupp_bigger = poly_unsupp;
                                    Polygons contour_bigger = offset(poly_unsupp_bigger.contour, bridged_infill_margin);
                                    if (contour_bigger.size() == 1) poly_unsupp_bigger.contour = contour_bigger[0];

                                    //check convex, has some bridge, not overhang
                                    if (contour_simplified.size() == 1 && contour_bigger.size() == 1 && contour_simplified[0].concave_points().size() == 0
                                        && intersection_ex(bridgeable, ExPolygons{ poly_unsupp }).size() > 0
                                        && diff_ex(ExPolygons{ poly_unsupp_bigger }, union_ex(last, offset_ex(bridgeable, bridged_infill_margin + perimeter_spacing / 2)), ApplySafetyOffset::Yes).size() == 0
                                    ) {
                                        //ok, keep it
                                        i++;
                                    } else {
                                        unsupported_filtered.erase(unsupported_filtered.begin() + i);
                                    }
                                }
                                unsupported_filtered = intersection_ex(last,
                                                                       offset2_ex(unsupported_filtered, double(-perimeter_spacing / 2), double(bridged_infill_margin + perimeter_spacing / 2)));
                                if (this->config->counterbore_hole_bridging.value == chbFilled) {
                                    for (ExPolygon& expol : unsupported_filtered) {
                                        //check if the holes won't be covered by the upper layer
                                        //TODO: if we want to do that, we must modify the geometry before making perimeters.
                                        //if (this->upper_slices != nullptr && !this->upper_slices->expolygons.empty()) {
                                        //    for (Polygon &poly : expol.holes) poly.make_counter_clockwise();
                                        //    float perimeterwidth = this->config->perimeters == 0 ? 0 : (this->ext_perimeter_flow.scaled_width() + (this->config->perimeters - 1) + this->perimeter_flow.scaled_spacing());
                                        //    std::cout << "test upper slices with perimeterwidth=" << perimeterwidth << "=>" << offset_ex(this->upper_slices->expolygons, -perimeterwidth).size();
                                        //    if (intersection(Polygons() = { expol.holes }, to_polygons(offset_ex(this->upper_slices->expolygons, -this->ext_perimeter_flow.scaled_width() / 2))).empty()) {
                                        //        std::cout << " EMPTY";
                                        //        expol.holes.clear();
                                        //    } else {
                                        //    }
                                        //    std::cout << "\n";
                                        //} else {
                                        expol.holes.clear();
                                        //}

                                        //detect inside volume
                                        for (size_t surface_idx_other = 0; surface_idx_other < all_surfaces.size(); surface_idx_other++) {
                                            if (surface_idx == surface_idx_other) continue;
                                            if (intersection_ex(ExPolygons() = { expol }, ExPolygons() = { all_surfaces[surface_idx_other].expolygon }).size() > 0) {
                                                //this means that other_surf was inside an expol holes
                                                //as we removed them, we need to add a new one
                                                ExPolygons new_poly = offset2_ex(ExPolygons{ all_surfaces[surface_idx_other].expolygon }, double(-bridged_infill_margin - perimeter_spacing), double(perimeter_spacing));
                                                if (new_poly.size() == 1) {
                                                    all_surfaces[surface_idx_other].expolygon = new_poly[0];
                                                    expol.holes.push_back(new_poly[0].contour);
                                                    expol.holes.back().make_clockwise();
                                                } else {
                                                    for (size_t idx = 0; idx < new_poly.size(); idx++) {
                                                        Surface new_surf = all_surfaces[surface_idx_other];
                                                        new_surf.expolygon = new_poly[idx];
                                                        all_surfaces.push_back(new_surf);
                                                        expol.holes.push_back(new_poly[idx].contour);
                                                        expol.holes.back().make_clockwise();
                                                    }
                                                    all_surfaces.erase(all_surfaces.begin() + surface_idx_other);
                                                    if (surface_idx_other < surface_idx) {
                                                        surface_idx--;
                                                        surface = &all_surfaces[surface_idx];
                                                    }
                                                    surface_idx_other--;
                                                }
                                            }
                                        }
                                    }

                                }
                                //TODO: add other polys as holes inside this one (-margin)
                            } else if (/*this->config->counterbore_hole_bridging.value == chbBridgesOverhangs || */this->config->counterbore_hole_bridging.value == chbBridges) {
                                //simplify to avoid most of artefacts from printing lines.
                                ExPolygons bridgeable_simplified;
                                for (ExPolygon& poly : bridgeable) {
                                    poly.simplify(perimeter_spacing, &bridgeable_simplified);
                                }
                                bridgeable_simplified = offset2_ex(bridgeable_simplified, -ext_perimeter_width, ext_perimeter_width);
                                //bridgeable_simplified = intersection_ex(bridgeable_simplified, unsupported_filtered);
                                //offset by perimeter spacing because the simplify may have reduced it a bit.
                                //it's not dangerous as it will be intersected by 'unsupported' later
                                //FIXME: add overlap in this->fill_surfaces->append
                                //FIXME: it overlap inside unsuppported not-bridgeable area!

                                //bridgeable_simplified = offset2_ex(bridgeable_simplified, (double)-perimeter_spacing, (double)perimeter_spacing * 2);
                                //ExPolygons unbridgeable = offset_ex(diff_ex(unsupported, bridgeable_simplified), perimeter_spacing * 3 / 2);
                                //ExPolygons unbridgeable = intersection_ex(unsupported, diff_ex(unsupported_filtered, offset_ex(bridgeable_simplified, ext_perimeter_width / 2)));
                                //unbridgeable = offset2_ex(unbridgeable, -ext_perimeter_width, ext_perimeter_width);


                                // if (this->config->counterbore_hole_bridging.value == chbBridges) {
                                    ExPolygons unbridgeable = unsupported_filtered;
                                    for (ExPolygon& expol : unbridgeable)
                                        expol.holes.clear();
                                    unbridgeable = diff_ex(unbridgeable, bridgeable_simplified);
                                    unbridgeable = offset2_ex(unbridgeable, -ext_perimeter_width * 2, ext_perimeter_width * 2);
                                    ExPolygons bridges_temp = offset2_ex(intersection_ex(last, diff_ex(unsupported_filtered, unbridgeable), ApplySafetyOffset::Yes), -ext_perimeter_width / 4, ext_perimeter_width / 4);
                                    //remove the overhangs section from the surface polygons
                                    ExPolygons reference = last;
                                    last = diff_ex(last, unsupported_filtered);
                                    //ExPolygons no_bridge = diff_ex(offset_ex(unbridgeable, ext_perimeter_width * 3 / 2), last);
                                    //bridges_temp = diff_ex(bridges_temp, no_bridge);
                                    coordf_t offset_to_do = bridged_infill_margin;
                                    bool first = true;
                                    unbridgeable = diff_ex(unbridgeable, offset_ex(bridges_temp, ext_perimeter_width));
                                    while (offset_to_do > ext_perimeter_width * 1.5) {
                                        unbridgeable = offset2_ex(unbridgeable, -ext_perimeter_width / 4, ext_perimeter_width * 2.25, ClipperLib::jtSquare);
                                        bridges_temp = diff_ex(bridges_temp, unbridgeable);
                                        bridges_temp = offset_ex(bridges_temp, ext_perimeter_width, ClipperLib::jtMiter, 6.);
                                        unbridgeable = diff_ex(unbridgeable, offset_ex(bridges_temp, ext_perimeter_width));
                                        offset_to_do -= ext_perimeter_width;
                                        first = false;
                                    }
                                    unbridgeable = offset_ex(unbridgeable, ext_perimeter_width + offset_to_do, ClipperLib::jtSquare);
                                    bridges_temp = diff_ex(bridges_temp, unbridgeable);
                                    unsupported_filtered = offset_ex(bridges_temp, offset_to_do);
                                    unsupported_filtered = intersection_ex(unsupported_filtered, reference);
                                // } else {
                                //     ExPolygons unbridgeable = intersection_ex(unsupported, diff_ex(unsupported_filtered, offset_ex(bridgeable_simplified, ext_perimeter_width / 2)));
                                //     unbridgeable = offset2_ex(unbridgeable, -ext_perimeter_width, ext_perimeter_width);
                                //     unsupported_filtered = unbridgeable;

                                //     ////put the bridge area inside the unsupported_filtered variable
                                //     //unsupported_filtered = intersection_ex(last,
                                //     //    diff_ex(
                                //     //    offset_ex(bridgeable_simplified, (double)perimeter_spacing / 2),
                                //     //    unbridgeable
                                //     //    )
                                //     //    );
                                // }
                            } else {
                                unsupported_filtered.clear();
                            }
                        } else {
                            unsupported_filtered.clear();
                        }
                    }

                    if (!unsupported_filtered.empty()) {

                        //add this directly to the infill list.
                        // this will avoid to throw wrong offsets into a good polygons
                        // this->fill_surfaces->append(
                        //     unsupported_filtered,
                        //     stInternal);

                        // store the results
                        last = diff_ex(last, unsupported_filtered, ApplySafetyOffset::Yes);
                        //remove "thin air" polygons (note: it assumes that all polygons below will be extruded)
                        for (int i = 0; i < last.size(); i++) {
                            if (intersection_ex(support, ExPolygons() = { last[i] }).empty()) {
                                // this->fill_surfaces->append(
                                //     ExPolygons() = { last[i] },
                                //     stInternal);
                                last.erase(last.begin() + i);
                                i--;
                            }
                        }
                    }
                }
            }
            if (last.size() == 0) {
                all_surfaces.erase(all_surfaces.begin() + surface_idx);
                surface_idx--;
            } else {
                surface->expolygon = last[0];
                for (size_t idx = 1; idx < last.size(); idx++) {
                    all_surfaces.emplace_back(*surface, last[idx]);
                }
            }
        }
    }
}

// Thanks, Cura developers, for implementing an algorithm for generating perimeters with variable width (Arachne) that is based on the paper
// "A framework for adaptive width control of dense contour-parallel toolpaths in fused deposition modeling"


bool PerimeterGeneratorLoop::is_internal_contour() const
{
    // An internal contour is a contour containing no other contours
    if (! this->is_contour)
        return false;
    for (const PerimeterGeneratorLoop &loop : this->children)
        if (loop.is_contour)
            return false;
    return true;
}

std::vector<Polygons> PerimeterGenerator::generate_lower_polygons_series(float width)
{
    float nozzle_diameter = print_config->nozzle_diameter.get_at(config->wall_filament - 1);
    float start_offset = -0.5 * width;
    float end_offset = 0.5 * nozzle_diameter;

    assert(overhang_sampling_number >= 3);
    // generate offsets
    std::vector<float> offset_series;
    offset_series.reserve(2);

     offset_series.push_back(start_offset + 0.5 * (end_offset - start_offset) / (overhang_sampling_number - 1));
     offset_series.push_back(end_offset);
    std::vector<Polygons> lower_polygons_series;
    if (this->lower_slices == NULL) {
        return lower_polygons_series;
    }

    // offset expolygon to generate series of polygons
    for (int i = 0; i < offset_series.size(); i++) {
        lower_polygons_series.emplace_back(offset(*this->lower_slices, float(scale_(offset_series[i]))));
    }
    return lower_polygons_series;
}

}
