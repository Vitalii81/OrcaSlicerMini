#include "BoundingBox.hpp"
#include "ExPolygon.hpp"
#include "Exception.hpp"
//#include "Geometry/MedialAxis.hpp"
#include "Polygon.hpp"
#include "Line.hpp"
#include "ClipperUtils.hpp"
//#include "SVG.hpp"
#include <algorithm>
#include <cassert>
#include <list>

namespace Slic3r {

void ExPolygon::scale(double factor)
{
    contour.scale(factor);
    for (Polygon &hole : holes)
        hole.scale(factor);
}

void ExPolygon::scale(double factor_x, double factor_y)
{
    contour.scale(factor_x, factor_y);
    for (Polygon &hole : holes)
        hole.scale(factor_x, factor_y);
}

void ExPolygon::translate(const Point &p)
{
    contour.translate(p);
    for (Polygon &hole : holes)
        hole.translate(p);
}

void ExPolygon::rotate(double angle)
{
    contour.rotate(angle);
    for (Polygon &hole : holes)
        hole.rotate(angle);
}

void ExPolygon::rotate(double angle, const Point &center)
{
    contour.rotate(angle, center);
    for (Polygon &hole : holes)
        hole.rotate(angle, center);
}

double ExPolygon::area() const
{
    double a = this->contour.area();
    for (const Polygon &hole : holes)
        a -= - hole.area();  // holes have negative area
    return a;
}

bool ExPolygon::is_valid() const
{
    if (!this->contour.is_valid() || !this->contour.is_counter_clockwise()) return false;
    for (Polygons::const_iterator it = this->holes.begin(); it != this->holes.end(); ++it) {
        if (!(*it).is_valid() || (*it).is_counter_clockwise()) return false;
    }
    return true;
}

void ExPolygon::douglas_peucker(double tolerance)
{
    this->contour.douglas_peucker(tolerance);
    for (Polygon &poly : this->holes)
        poly.douglas_peucker(tolerance);
}

bool ExPolygon::contains(const Line &line) const
{
    return this->contains(Polyline(line.a, line.b));
}

bool ExPolygon::contains(const Polyline &polyline) const
{
    BoundingBox bbox1 = get_extents(*this);
    BoundingBox bbox2 = get_extents(polyline);
    bbox2.inflated(1);
    if (!bbox1.overlap(bbox2))
        return false;

    return diff_pl(polyline, *this).empty();
}

bool ExPolygon::contains(const Polylines &polylines) const
{
    #if 0
    BoundingBox bbox = get_extents(polylines);
    bbox.merge(get_extents(*this));
    SVG svg(debug_out_path("ExPolygon_contains.svg"), bbox);
    svg.draw(*this);
    svg.draw_outline(*this);
    svg.draw(polylines, "blue");
    #endif
    Polylines pl_out = diff_pl(polylines, *this);
    #if 0
    svg.draw(pl_out, "red");
    #endif
    return pl_out.empty();
}

bool ExPolygon::contains(const Point &point, bool border_result /* = true */) const
{
    if (! Slic3r::contains(contour, point, border_result))
        // Outside the outer contour, not on the contour boundary.
        return false;
    for (const Polygon &hole : this->holes)
        if (Slic3r::contains(hole, point, ! border_result))
            // Inside a hole, not on the hole boundary.
            return false;
    return true;
}

bool ExPolygon::on_boundary(const Point &point, double eps) const
{
    if (this->contour.on_boundary(point, eps))
        return true;
    for (const Polygon &hole : this->holes)
        if (hole.on_boundary(point, eps))
            return true;
    return false;
}

// Projection of a point onto the polygon.
Point ExPolygon::point_projection(const Point &point) const
{
    if (this->holes.empty()) {
        return this->contour.point_projection(point);
    } else {
        double dist_min2 = std::numeric_limits<double>::max();
        Point  closest_pt_min;
        for (size_t i = 0; i < this->num_contours(); ++ i) {
            Point closest_pt = this->contour_or_hole(i).point_projection(point);
            double d2 = (closest_pt - point).cast<double>().squaredNorm();
            if (d2 < dist_min2) {
                dist_min2      = d2;
                closest_pt_min = closest_pt;
            }
        }
        return closest_pt_min;
    }
}

bool ExPolygon::overlaps(const ExPolygon &other) const
{
    if (this->empty() || other.empty())
        return false;

    #if 0
    BoundingBox bbox = get_extents(other);
    bbox.merge(get_extents(*this));
    static int iRun = 0;
    SVG svg(debug_out_path("ExPolygon_overlaps-%d.svg", iRun ++), bbox);
    svg.draw(*this);
    svg.draw_outline(*this);
    svg.draw_outline(other, "blue");
    #endif

    Polylines pl_out = intersection_pl(to_polylines(other), *this);

    #if 0
    svg.draw(pl_out, "red");
    #endif

    // See unit test SCENARIO("Clipper diff with polyline", "[Clipper]")
    // for in which case the intersection_pl produces any intersection.
    return ! pl_out.empty() ||
           // If *this is completely inside other, then pl_out is empty, but the expolygons overlap. Test for that situation.
           other.contains(this->contour.points.front());
}

bool overlaps(const ExPolygons& expolys1, const ExPolygons& expolys2)
{
    for (const ExPolygon& expoly1 : expolys1) {
        for (const ExPolygon& expoly2 : expolys2) {
            if (expoly1.overlaps(expoly2))
                return true;
        }
    }
    return false;
}

bool overlaps(const ExPolygons& expolys, const ExPolygon& expoly)
{
    for (const ExPolygon& el : expolys) {
        if (el.overlaps(expoly))
                return true;
    }
    return false;
}

Point projection_onto(const ExPolygons& polygons, const Point& from)
{
    Point projected_pt;
    double min_dist = std::numeric_limits<double>::max();

    for (const auto& poly : polygons) {
        for (int i = 0; i < poly.num_contours(); i++) {
            Point p = from.projection_onto(poly.contour_or_hole(i));
            double dist = (from - p).cast<double>().squaredNorm();
            if (dist < min_dist) {
                projected_pt = p;
                min_dist = dist;
            }
        }
    }

    return projected_pt;
}

void ExPolygon::simplify_p(double tolerance, Polygons* polygons) const
{
    Polygons pp = this->simplify_p(tolerance);
    polygons->insert(polygons->end(), pp.begin(), pp.end());
}

Polygons ExPolygon::simplify_p(double tolerance) const
{
    Polygons pp;
    pp.reserve(this->holes.size() + 1);
    // contour
    {
        Polygon p = this->contour;
        p.points.push_back(p.points.front());
        p.points = MultiPoint::_douglas_peucker(p.points, tolerance);
        p.points.pop_back();
        pp.emplace_back(std::move(p));
    }
    // holes
    for (Polygon p : this->holes) {
        p.points.push_back(p.points.front());
        p.points = MultiPoint::_douglas_peucker(p.points, tolerance);
        p.points.pop_back();
        pp.emplace_back(std::move(p));
    }
    return simplify_polygons(pp);
}

ExPolygons ExPolygon::simplify(double tolerance) const
{
    return union_ex(this->simplify_p(tolerance));
}

void ExPolygon::simplify(double tolerance, ExPolygons* expolygons) const
{
    append(*expolygons, this->simplify(tolerance));
}

// void ExPolygon::medial_axis(double min_width, double max_width, Polylines* polylines) const
// {
//     ThickPolylines tp;
//     this->medial_axis(min_width, max_width, &tp);
//     polylines->reserve(polylines->size() + tp.size());
//     for (auto &pl : tp)
//         polylines->emplace_back(pl.points);
// }

Lines ExPolygon::lines() const
{
    Lines lines = this->contour.lines();
    for (Polygons::const_iterator h = this->holes.begin(); h != this->holes.end(); ++h) {
        Lines hole_lines = h->lines();
        lines.insert(lines.end(), hole_lines.begin(), hole_lines.end());
    }
    return lines;
}

// Do expolygons match? If they match, they must have the same topology,
// however their contours may be rotated.
bool expolygons_match(const ExPolygon &l, const ExPolygon &r)
{
    if (l.holes.size() != r.holes.size() || ! polygons_match(l.contour, r.contour))
        return false;
    for (size_t hole_idx = 0; hole_idx < l.holes.size(); ++ hole_idx)
        if (! polygons_match(l.holes[hole_idx], r.holes[hole_idx]))
            return false;
    return true;
}

BoundingBox get_extents(const ExPolygon &expolygon)
{
    return get_extents(expolygon.contour);
}

BoundingBox get_extents(const ExPolygons &expolygons)
{
    BoundingBox bbox;
    if (! expolygons.empty()) {
        for (size_t i = 0; i < expolygons.size(); ++ i)
			if (! expolygons[i].contour.points.empty())
				bbox.merge(get_extents(expolygons[i]));
    }
    return bbox;
}

BoundingBox get_extents_rotated(const ExPolygon &expolygon, double angle)
{
    return get_extents_rotated(expolygon.contour, angle);
}

BoundingBox get_extents_rotated(const ExPolygons &expolygons, double angle)
{
    BoundingBox bbox;
    if (! expolygons.empty()) {
        bbox = get_extents_rotated(expolygons.front().contour, angle);
        for (size_t i = 1; i < expolygons.size(); ++ i)
            bbox.merge(get_extents_rotated(expolygons[i].contour, angle));
    }
    return bbox;
}

extern std::vector<BoundingBox> get_extents_vector(const ExPolygons &polygons)
{
    std::vector<BoundingBox> out;
    out.reserve(polygons.size());
    for (ExPolygons::const_iterator it = polygons.begin(); it != polygons.end(); ++ it)
        out.push_back(get_extents(*it));
    return out;
}

bool has_duplicate_points(const ExPolygon &expoly)
{
#if 1
    // Check globally.
    size_t cnt = expoly.contour.points.size();
    for (const Polygon &hole : expoly.holes)
        cnt += hole.points.size();
    std::vector<Point> allpts;
    allpts.reserve(cnt);
    allpts.insert(allpts.begin(), expoly.contour.points.begin(), expoly.contour.points.end());
    for (const Polygon &hole : expoly.holes)
        allpts.insert(allpts.end(), hole.points.begin(), hole.points.end());
    return has_duplicate_points(std::move(allpts));
#else
    // Check per contour.
    if (has_duplicate_points(expoly.contour))
        return true;
    for (const Polygon &hole : expoly.holes)
        if (has_duplicate_points(hole))
            return true;
    return false;
#endif
}

bool has_duplicate_points(const ExPolygons &expolys)
{
#if 1
    // Check globally.
    size_t cnt = 0;
    for (const ExPolygon &expoly : expolys) {
        cnt += expoly.contour.points.size();
        for (const Polygon &hole : expoly.holes)
            cnt += hole.points.size();
    }
    std::vector<Point> allpts;
    allpts.reserve(cnt);
    for (const ExPolygon &expoly : expolys) {
        allpts.insert(allpts.begin(), expoly.contour.points.begin(), expoly.contour.points.end());
        for (const Polygon &hole : expoly.holes)
            allpts.insert(allpts.end(), hole.points.begin(), hole.points.end());
    }
    return has_duplicate_points(std::move(allpts));
#else
    // Check per contour.
    for (const ExPolygon &expoly : expolys)
        if (has_duplicate_points(expoly))
            return true;
    return false;
#endif
}

bool remove_same_neighbor(ExPolygons &expolygons)
{
    if (expolygons.empty())
        return false;
    bool remove_from_holes   = false;
    bool remove_from_contour = false;
    for (ExPolygon &expoly : expolygons) {
        remove_from_contour |= remove_same_neighbor(expoly.contour);
        remove_from_holes |= remove_same_neighbor(expoly.holes);
    }
    // Removing of expolygons without contour
    if (remove_from_contour)
        expolygons.erase(std::remove_if(expolygons.begin(), expolygons.end(),
                                        [](const ExPolygon &p) { return p.contour.points.size() <= 2; }),
                         expolygons.end());
    return remove_from_holes || remove_from_contour;
}

bool remove_sticks(ExPolygon &poly)
{
    return remove_sticks(poly.contour) || remove_sticks(poly.holes);
}

bool remove_small_and_small_holes(ExPolygons &expolygons, double min_area)
{
    bool   modified = false;
    size_t free_idx = 0;
    for (size_t expoly_idx = 0; expoly_idx < expolygons.size(); ++expoly_idx) {
        if (std::abs(expolygons[expoly_idx].area()) >= min_area) {
            // Expolygon is big enough, so also check all its holes
            modified |= remove_small(expolygons[expoly_idx].holes, min_area);
            if (free_idx < expoly_idx) {
                std::swap(expolygons[expoly_idx].contour, expolygons[free_idx].contour);
                std::swap(expolygons[expoly_idx].holes, expolygons[free_idx].holes);
            }
            ++free_idx;
        } else
            modified = true;
    }
    if (free_idx < expolygons.size())
        expolygons.erase(expolygons.begin() + free_idx, expolygons.end());
    return modified;
}

void keep_largest_contour_only(ExPolygons &polygons)
{
	if (polygons.size() > 1) {
	    double     max_area = 0.;
	    ExPolygon* max_area_polygon = nullptr;
	    for (ExPolygon& p : polygons) {
	        double a = p.contour.area();
	        if (a > max_area) {
	            max_area         = a;
	            max_area_polygon = &p;
	        }
	    }
	    assert(max_area_polygon != nullptr);
	    ExPolygon p(std::move(*max_area_polygon));
	    polygons.clear();
	    polygons.emplace_back(std::move(p));
	}
}

} // namespace Slic3r
