// Test file for Arachne wall generation
// 
// Tests for duplicate/coinciding wall segment detection in Arachne output.
// 
// This test reproduces an issue where Arachne generates duplicate extrusion
// segments at certain min_bead_width settings. The test uses a polygon with
// an outer rectangle (0,0)-(20,20) and an inner cutout (2,10)-(18,19.5).
//
// With precise_outer_wall enabled and min_bead_width at 55% (0.22mm), Arachne
// generates two separate closed contours that share a coinciding edge at y=19.75.
// At 56% (0.224mm), Arachne handles this differently and avoids the duplicate.
//
// Parameters are based on "0.28mm Extra Draft @BBL X1C" profile with:
// - 0.4mm nozzle, 0.28mm layer height
// - outer_wall_line_width: 0.42mm, inner_wall_line_width: 0.45mm
// - wall_loops: 2, precise_outer_wall: enabled

#include <catch2/catch_all.hpp>

#include "libslic3r/Arachne/WallToolPaths.hpp"
#include "libslic3r/Arachne/utils/ExtrusionLine.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Point.hpp"

#include <set>
#include <cmath>
#include <iostream>
#include <fstream>

#include "libslic3r/SVG.hpp"

using namespace Slic3r;
using namespace Slic3r::Arachne;

namespace {

// Represents a segment with direction-independent comparison
struct Segment {
    Point from;
    Point to;
    size_t inset_idx;
    
    // Normalize segment so that the "smaller" point comes first
    // This allows direction-independent comparison
    Segment normalized() const {
        if (from < to || (from.x() == to.x() && from.y() < to.y())) {
            return *this;
        }
        return {to, from, inset_idx};
    }
    
    bool operator<(const Segment& other) const {
        auto a = normalized();
        auto b = other.normalized();
        if (a.inset_idx != b.inset_idx) return a.inset_idx < b.inset_idx;
        if (a.from != b.from) return a.from < b.from;
        return a.to < b.to;
    }
    
    bool operator==(const Segment& other) const {
        auto a = normalized();
        auto b = other.normalized();
        return a.inset_idx == b.inset_idx && a.from == b.from && a.to == b.to;
    }
};

// Check if two points are approximately equal within tolerance
bool points_approx_equal(const Point& a, const Point& b, coord_t tolerance) {
    return std::abs(a.x() - b.x()) <= tolerance && std::abs(a.y() - b.y()) <= tolerance;
}

// Check if two segments are approximately equal (direction-independent)
bool segments_approx_equal(const Segment& a, const Segment& b, coord_t tolerance) {
    if (a.inset_idx != b.inset_idx) return false;
    
    // Check both directions
    bool same_dir = points_approx_equal(a.from, b.from, tolerance) && 
                    points_approx_equal(a.to, b.to, tolerance);
    bool reverse_dir = points_approx_equal(a.from, b.to, tolerance) && 
                       points_approx_equal(a.to, b.from, tolerance);
    return same_dir || reverse_dir;
}

// Extract all segments from toolpaths (all inset indices)
std::vector<Segment> extract_all_segments(const std::vector<VariableWidthLines>& toolpaths) {
    std::vector<Segment> segments;
    
    for (const auto& inset : toolpaths) {
        for (const auto& line : inset) {
            if (line.junctions.size() < 2) continue;
            
            for (size_t i = 0; i + 1 < line.junctions.size(); ++i) {
                segments.push_back({
                    line.junctions[i].p,
                    line.junctions[i + 1].p,
                    line.inset_idx
                });
            }
        }
    }
    
    return segments;
}

// Find duplicate segments within tolerance
std::vector<std::pair<Segment, Segment>> find_duplicate_segments(
    const std::vector<Segment>& segments, 
    coord_t tolerance) 
{
    std::vector<std::pair<Segment, Segment>> duplicates;
    
    for (size_t i = 0; i < segments.size(); ++i) {
        for (size_t j = i + 1; j < segments.size(); ++j) {
            if (segments_approx_equal(segments[i], segments[j], tolerance)) {
                duplicates.emplace_back(segments[i], segments[j]);
            }
        }
    }
    
    return duplicates;
}

// Create params matching "0.28mm Extra Draft @BBL X1C" profile
// For layer 2+ (not first layer)
// min_bead_width_percent: percentage of nozzle diameter for min_bead_width
WallToolPathsParams make_bbl_x1c_028_params(int min_bead_width_percent) {
    constexpr double nozzle_diameter = 0.4;  // 0.4mm nozzle
    
    WallToolPathsParams params;
    params.min_bead_width = float(min_bead_width_percent / 100.0 * nozzle_diameter);
    // 25% of nozzle = 0.1mm (default min_feature_size)
    params.min_feature_size = float(0.25 * nozzle_diameter);  // 0.1mm
    // 25% of nozzle = 0.1mm (default wall_transition_filter_deviation)
    params.wall_transition_filter_deviation = float(0.25 * nozzle_diameter);  // 0.1mm
    // 100% of nozzle = 0.4mm (default wall_transition_length)
    params.wall_transition_length = float(1.0 * nozzle_diameter);  // 0.4mm
    // 10 degrees (default)
    params.wall_transition_angle = 10.0f;
    // 1 (default)
    params.wall_distribution_count = 1;
    params.min_length_factor = 0.5f;
    // Layer 2 is NOT a top or bottom layer
    params.is_top_or_bottom_layer = false;
    return params;
}

// Export toolpaths to SVG for visualization
void export_toolpaths_to_svg(const std::string& filename,
                              const Polygons& outline,
                              const std::vector<VariableWidthLines>& toolpaths) {
    BoundingBox bbox = get_extents(outline);
    bbox.offset(scaled<coord_t>(1.0));  // Add 1mm margin
    
    SVG svg(filename, bbox);
    
    // Draw input outline in gray
    for (const auto& poly : outline) {
        svg.draw_outline(poly, "gray", scaled<coord_t>(0.05));
    }
    
    // Colors for different inset indices
    const char* colors[] = {"red", "blue", "green", "orange", "purple", "cyan"};
    const int num_colors = sizeof(colors) / sizeof(colors[0]);
    
    // Draw toolpaths
    for (const auto& inset : toolpaths) {
        for (const auto& line : inset) {
            int color_idx = line.inset_idx % num_colors;
            
            // Draw segments
            for (size_t i = 0; i + 1 < line.junctions.size(); ++i) {
                svg.draw(Line(line.junctions[i].p, line.junctions[i + 1].p), 
                        colors[color_idx], scaled<coord_t>(0.03));
            }
            
            // Draw junction points
            for (const auto& junc : line.junctions) {
                svg.draw(junc.p, "black", scaled<coord_t>(0.02));
            }
        }
    }
}

// Run Arachne wall generation test with specified min_bead_width percentage
// Returns the number of duplicate segments found
size_t run_arachne_test(int min_bead_width_percent, const std::string& svg_suffix = "") {
    // Parameters from "0.28mm Extra Draft @BBL X1C" profile with Arachne
    // This simulates layer 2 (not first layer, not top/bottom)
    
    // Layer height from profile
    constexpr double layer_height = 0.28;
    
    // Line widths from profile
    constexpr double ext_perimeter_width_mm = 0.42;   // outer_wall_line_width
    constexpr double perimeter_width_mm = 0.45;       // inner_wall_line_width
    
    // Spacing calculation: width - height * (1 - PI/4)
    // This is Flow::rounded_rectangle_extrusion_spacing()
    constexpr double spacing_factor = 1.0 - 0.25 * M_PI;  // ≈ 0.2146
    double ext_perimeter_spacing_mm = ext_perimeter_width_mm - layer_height * spacing_factor;
    double perimeter_spacing_mm = perimeter_width_mm - layer_height * spacing_factor;
    
    coord_t ext_perimeter_width = scaled<coord_t>(ext_perimeter_width_mm);
    coord_t ext_perimeter_spacing = scaled<coord_t>(ext_perimeter_spacing_mm);
    coord_t perimeter_spacing = scaled<coord_t>(perimeter_spacing_mm);
    
    // WallToolPaths uses bead_width_0 (outer) and bead_width_x (inner)
    // Per PerimeterGenerator line 2161 and 2204
    coord_t bead_width_0 = ext_perimeter_spacing;  // outer wall spacing
    coord_t bead_width_x = perimeter_spacing;      // inner wall spacing
    size_t  inset_count  = 2;                       // wall_loops from profile
    
    // precise_outer_wall enabled:
    // 1. Polygon is pre-offset by -(ext_perimeter_width - ext_perimeter_spacing)
    // 2. wall_0_inset = -(ext_perimeter_width / 2 - ext_perimeter_spacing / 2)
    float precise_offset = -float(ext_perimeter_width - ext_perimeter_spacing);
    coord_t wall_0_inset = -coord_t(ext_perimeter_width / 2 - ext_perimeter_spacing / 2);
    
    auto params = make_bbl_x1c_028_params(min_bead_width_percent);
    
    // User-provided polygon: outer rectangle with inner cutout
    // Outer rectangle: 0,0 to 20,20
    Polygon outer_raw;
    outer_raw.points.emplace_back(Point::new_scale(0.0, 0.0));
    outer_raw.points.emplace_back(Point::new_scale(20.0, 0.0));
    outer_raw.points.emplace_back(Point::new_scale(20.0, 20.0));
    outer_raw.points.emplace_back(Point::new_scale(0.0, 20.0));
    
    // Inner cutout (hole): 0.5,0.5 to 19.5,19.5 (0.5mm inset from outer)
    // Note: holes are typically wound in opposite direction
    Polygon inner_raw;
    inner_raw.points.emplace_back(Point::new_scale(0.5, 0.5));
    inner_raw.points.emplace_back(Point::new_scale(0.5, 19.5));
    inner_raw.points.emplace_back(Point::new_scale(19.5, 19.5));
    inner_raw.points.emplace_back(Point::new_scale(19.5, 0.5));
    
    // Apply precise_outer_wall offset to the polygon (as PerimeterGenerator does)
    ExPolygon input_expolygon;
    input_expolygon.contour = outer_raw;
    input_expolygon.holes.push_back(inner_raw);
    
    ExPolygons offset_result = offset_ex(input_expolygon, precise_offset);
    Polygons outline;
    for (const auto& expoly : offset_result) {
        outline.push_back(expoly.contour);
        for (const auto& hole : expoly.holes) {
            outline.push_back(hole);
        }
    }
    
    std::cout << "\n=== Test Parameters (Layer 2, precise_outer_wall enabled) ===" << std::endl;
    std::cout << "min_bead_width: " << min_bead_width_percent << "% of 0.4mm nozzle = " 
              << params.min_bead_width << "mm" << std::endl;
    std::cout << "layer_height: " << layer_height << "mm" << std::endl;
    std::cout << "ext_perimeter_width: " << ext_perimeter_width_mm << "mm" << std::endl;
    std::cout << "ext_perimeter_spacing: " << ext_perimeter_spacing_mm << "mm" << std::endl;
    std::cout << "perimeter_width (inner): " << perimeter_width_mm << "mm" << std::endl;
    std::cout << "perimeter_spacing (inner): " << perimeter_spacing_mm << "mm" << std::endl;
    std::cout << "bead_width_0 (outer): " << unscaled(bead_width_0) << "mm" << std::endl;
    std::cout << "bead_width_x (inner): " << unscaled(bead_width_x) << "mm" << std::endl;
    std::cout << "precise_offset: " << unscaled(coord_t(precise_offset)) << "mm" << std::endl;
    std::cout << "wall_0_inset: " << unscaled(wall_0_inset) << "mm" << std::endl;
    std::cout << "inset_count: " << inset_count << std::endl;
    std::cout << "outline polygons after offset: " << outline.size() << std::endl;
    
    WallToolPaths wallToolPaths(outline, bead_width_0, bead_width_x, 
                                 inset_count, wall_0_inset, 
                                 layer_height, params);
    auto toolpaths = wallToolPaths.getToolPaths();
    
    // Print toolpath structure
    std::cout << "\n=== WallToolPaths Output ===" << std::endl;
    std::cout << "Number of inset groups: " << toolpaths.size() << std::endl;
    
    for (size_t i = 0; i < toolpaths.size(); ++i) {
        std::cout << "\nInset group " << i << ": " << toolpaths[i].size() << " lines" << std::endl;
        for (size_t j = 0; j < toolpaths[i].size(); ++j) {
            const auto& line = toolpaths[i][j];
            std::cout << "  Line " << j << ": inset_idx=" << line.inset_idx 
                      << ", is_odd=" << line.is_odd 
                      << ", is_closed=" << line.is_closed
                      << ", junctions=" << line.junctions.size() << std::endl;
            for (size_t k = 0; k < line.junctions.size(); ++k) {
                const auto& junc = line.junctions[k];
                std::cout << "    [" << k << "] (" << unscaled(junc.p.x()) << ", " << unscaled(junc.p.y()) 
                          << ") w=" << unscaled(junc.w) << std::endl;
            }
        }
    }
    
    // Extract all wall segments (all inset indices)
    auto all_segments = extract_all_segments(toolpaths);
    
    std::cout << "\n=== Extracted Segments ===" << std::endl;
    std::cout << "Total segments: " << all_segments.size() << std::endl;
    
    // Check for duplicates with 0.1mm (100 micron) tolerance
    auto duplicates = find_duplicate_segments(all_segments, scaled<coord_t>(0.1));
    
    std::cout << "\n=== Duplicate Check (0.1mm tolerance) ===" << std::endl;
    std::cout << "Number of duplicate segment pairs: " << duplicates.size() << std::endl;
    for (const auto& dup : duplicates) {
        std::cout << "Duplicate: inset_idx=" << dup.first.inset_idx 
                  << " (" << unscaled(dup.first.from.x()) << "," << unscaled(dup.first.from.y()) 
                  << ")->(" << unscaled(dup.first.to.x()) << "," << unscaled(dup.first.to.y()) << ")" << std::endl;
    }
    
    // Export SVG for visualization
    std::string svg_path = "/tmp/opencode/arachne_walls_test" + svg_suffix + ".svg";
    export_toolpaths_to_svg(svg_path, outline, toolpaths);
    std::cout << "\nSVG exported to: " << svg_path << std::endl;
    std::cout << std::endl;
    
    return duplicates.size();
}

} // anonymous namespace

TEST_CASE("Arachne wall generation - 55% min_bead_width", "[Arachne]") {
    size_t duplicates = run_arachne_test(55, "_55pct");
    REQUIRE(duplicates == 0);
}

TEST_CASE("Arachne wall generation - 56% min_bead_width", "[Arachne]") {
    size_t duplicates = run_arachne_test(56, "_56pct");
    REQUIRE(duplicates == 0);
}


