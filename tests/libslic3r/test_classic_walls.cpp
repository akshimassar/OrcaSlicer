// Test file for Classic wall generation
//
// Tests for duplicate/coinciding wall segment detection in Classic perimeter output.
//
// This test reproduces an issue where the Classic perimeter generator creates
// duplicate extrusion segments when the wall thickness equals or is close to
// the line width. The offset2_ex() operation can produce split/overlapping
// contours that result in nearly-identical extrusion paths.
//
// Parameters based on gcode "wall issue_PLA_3m38s.gcode":
// - 0.6mm nozzle, 0.25mm layer height
// - outer_wall_line_width: 0.6mm, inner_wall_line_width: 0.6mm
// - wall_loops: 6, detect_thin_wall: false, gap_fill: disabled
// - Frame geometry with 0.6mm wall thickness

#include <catch2/catch_all.hpp>

#include "libslic3r/PerimeterGenerator.hpp"
#include "libslic3r/SurfaceCollection.hpp"
#include "libslic3r/Surface.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/SVG.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Layer.hpp"

#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace Slic3r;

namespace {

// Represents a segment with direction-independent comparison
struct Segment {
    Point from;
    Point to;
    
    // Normalize segment so that the "smaller" point comes first
    // This allows direction-independent comparison
    Segment normalized() const {
        if (from < to || (from.x() == to.x() && from.y() < to.y())) {
            return *this;
        }
        return {to, from};
    }
    
    bool operator<(const Segment& other) const {
        auto a = normalized();
        auto b = other.normalized();
        if (a.from != b.from) return a.from < b.from;
        return a.to < b.to;
    }
    
    bool operator==(const Segment& other) const {
        auto a = normalized();
        auto b = other.normalized();
        return a.from == b.from && a.to == b.to;
    }
};

// Check if two points are approximately equal within tolerance
bool points_approx_equal(const Point& a, const Point& b, coord_t tolerance) {
    return std::abs(a.x() - b.x()) <= tolerance && std::abs(a.y() - b.y()) <= tolerance;
}

// Check if two segments are approximately equal (direction-independent)
bool segments_approx_equal(const Segment& a, const Segment& b, coord_t tolerance) {
    // Check both directions
    bool same_dir = points_approx_equal(a.from, b.from, tolerance) && 
                    points_approx_equal(a.to, b.to, tolerance);
    bool reverse_dir = points_approx_equal(a.from, b.to, tolerance) && 
                       points_approx_equal(a.to, b.from, tolerance);
    return same_dir || reverse_dir;
}

// Extract segments from a single ExtrusionPath
void extract_segments_from_path(const ExtrusionPath& path, std::vector<Segment>& segments) {
    const Points3& points = path.polyline.points;
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        // Convert Point3 to Point (2D) for segment comparison
        segments.push_back({points[i].to_point(), points[i + 1].to_point()});
    }
}

// Extract segments from ExtrusionLoop
void extract_segments_from_loop(const ExtrusionLoop& loop, std::vector<Segment>& segments) {
    for (const ExtrusionPath& path : loop.paths) {
        extract_segments_from_path(path, segments);
    }
}

// Extract segments from ExtrusionMultiPath
void extract_segments_from_multipath(const ExtrusionMultiPath& mp, std::vector<Segment>& segments) {
    for (const ExtrusionPath& path : mp.paths) {
        extract_segments_from_path(path, segments);
    }
}

// Forward declaration for recursive extraction
void extract_segments_from_collection(const ExtrusionEntityCollection& collection, std::vector<Segment>& segments);

// Extract segments from any ExtrusionEntity
void extract_segments_from_entity(const ExtrusionEntity* entity, std::vector<Segment>& segments) {
    if (const auto* loop = dynamic_cast<const ExtrusionLoop*>(entity)) {
        extract_segments_from_loop(*loop, segments);
    } else if (const auto* path = dynamic_cast<const ExtrusionPath*>(entity)) {
        extract_segments_from_path(*path, segments);
    } else if (const auto* mp = dynamic_cast<const ExtrusionMultiPath*>(entity)) {
        extract_segments_from_multipath(*mp, segments);
    } else if (const auto* coll = dynamic_cast<const ExtrusionEntityCollection*>(entity)) {
        extract_segments_from_collection(*coll, segments);
    }
}

// Extract all segments from ExtrusionEntityCollection
void extract_segments_from_collection(const ExtrusionEntityCollection& collection, std::vector<Segment>& segments) {
    for (const ExtrusionEntity* entity : collection.entities) {
        extract_segments_from_entity(entity, segments);
    }
}

// Calculate segment length
coord_t segment_length(const Segment& s) {
    coord_t dx = s.to.x() - s.from.x();
    coord_t dy = s.to.y() - s.from.y();
    return static_cast<coord_t>(std::sqrt(double(dx) * dx + double(dy) * dy));
}

// Find duplicate segments within tolerance
// Only considers segments longer than min_length (default 1mm)
std::vector<std::pair<Segment, Segment>> find_duplicate_segments(
    const std::vector<Segment>& segments, 
    coord_t tolerance,
    coord_t min_length = scaled<coord_t>(1.0)) 
{
    std::vector<std::pair<Segment, Segment>> duplicates;
    
    for (size_t i = 0; i < segments.size(); ++i) {
        // Skip short segments
        if (segment_length(segments[i]) < min_length) continue;
        
        for (size_t j = i + 1; j < segments.size(); ++j) {
            // Skip short segments
            if (segment_length(segments[j]) < min_length) continue;
            
            if (segments_approx_equal(segments[i], segments[j], tolerance)) {
                duplicates.emplace_back(segments[i], segments[j]);
            }
        }
    }
    
    return duplicates;
}

// Export segments to SVG for visualization
void export_segments_to_svg(const std::string& filename,
                            const ExPolygon& outline,
                            const std::vector<Segment>& segments) {
    BoundingBox bbox = get_extents(outline);
    bbox.offset(scaled<coord_t>(1.0));  // Add 1mm margin
    
    SVG svg(filename, bbox);
    
    // Draw input outline in gray
    svg.draw_outline(outline, "gray", "gray", 0.05);
    
    // Draw each segment with alternating colors and arrows
    const char* colors[] = {"red", "blue", "green", "orange", "purple", "cyan"};
    const int num_colors = sizeof(colors) / sizeof(colors[0]);
    
    for (size_t i = 0; i < segments.size(); ++i) {
        const Segment& seg = segments[i];
        svg.draw(Line(seg.from, seg.to), colors[i % num_colors], scaled<coord_t>(0.03));
        // Draw small circle at start point to show direction
        svg.draw(seg.from, colors[i % num_colors], scaled<coord_t>(0.08));
    }
}

// Run classic wall generation test
// Returns the number of duplicate segment pairs found
size_t run_classic_test(float frame_thickness_mm = 0.6f, bool detect_thin_wall = false) {
    // Parameters from gcode "wall issue_PLA_3m38s.gcode"
    constexpr float nozzle_diameter = 0.6f;
    constexpr float layer_height = 0.25f;
    constexpr float line_width = 0.6f;
    constexpr int wall_loops = 6;
    
    std::cout << "\n=== Classic Wall Generation Test ===" << std::endl;
    std::cout << "nozzle_diameter: " << nozzle_diameter << "mm" << std::endl;
    std::cout << "layer_height: " << layer_height << "mm" << std::endl;
    std::cout << "line_width: " << line_width << "mm" << std::endl;
    std::cout << "wall_loops: " << wall_loops << std::endl;
    std::cout << "frame_thickness: " << frame_thickness_mm << "mm" << std::endl;
    std::cout << "detect_thin_wall: " << (detect_thin_wall ? "true" : "false") << std::endl;
    
    // Create configs
    PrintRegionConfig region_config;
    region_config.wall_loops.value = wall_loops;
    region_config.detect_thin_wall.value = detect_thin_wall;
    region_config.gap_infill_speed.value = 0;  // Disables gap fill
    region_config.precise_outer_wall.value = false;
    region_config.wall_sequence.value = WallSequence::InnerOuter;
    region_config.detect_overhang_wall.value = false;
    region_config.overhang_reverse.value = false;
    region_config.only_one_wall_top.value = false;
    region_config.only_one_wall_first_layer.value = false;
    region_config.extra_perimeters_on_overhangs.value = false;
    region_config.alternate_extra_wall.value = false;
    region_config.wall_direction.value = WallDirection::CounterClockwise;
    region_config.outer_wall_line_width.value = line_width;
    region_config.inner_wall_line_width.value = line_width;
    region_config.wall_filament.value = 1;
    region_config.sparse_infill_density.value = 0;  // No infill
    
    PrintObjectConfig object_config;
    object_config.raft_layers.value = 0;
    
    PrintConfig print_config;
    print_config.nozzle_diameter.values = {nozzle_diameter};
    print_config.resolution.value = 0.0125;  // Default resolution
    
    // Create flows
    Flow perimeter_flow(line_width, layer_height, nozzle_diameter);
    Flow ext_perimeter_flow(line_width, layer_height, nozzle_diameter);
    
    std::cout << "perimeter_flow width: " << perimeter_flow.width() << "mm" << std::endl;
    std::cout << "perimeter_flow spacing: " << perimeter_flow.spacing() << "mm" << std::endl;
    
    // Create geometry: 20mm square with frame_thickness_mm frame
    ExPolygon frame;
    frame.contour.points = {
        Point::new_scale(0.0, 0.0),
        Point::new_scale(20.0, 0.0),
        Point::new_scale(20.0, 20.0),
        Point::new_scale(0.0, 20.0)
    };
    
    // Inner hole (CCW winding for holes in Clipper)
    frame.holes.emplace_back();
    frame.holes.back().points = {
        Point::new_scale(frame_thickness_mm, frame_thickness_mm),
        Point::new_scale(frame_thickness_mm, 20.0 - frame_thickness_mm),
        Point::new_scale(20.0 - frame_thickness_mm, 20.0 - frame_thickness_mm),
        Point::new_scale(20.0 - frame_thickness_mm, frame_thickness_mm)
    };
    
    std::cout << "Frame outer: 20mm x 20mm" << std::endl;
    std::cout << "Frame inner: " << (20.0 - 2*frame_thickness_mm) << "mm x " 
              << (20.0 - 2*frame_thickness_mm) << "mm" << std::endl;
    
    // Create SurfaceCollection with a single internal surface
    SurfaceCollection slices;
    Surface surface(stInternal, frame);
    slices.surfaces.push_back(surface);
    
    // Output collections
    ExtrusionEntityCollection loops;
    ExtrusionEntityCollection gap_fill;
    SurfaceCollection fill_surfaces;
    ExPolygons fill_no_overlap;
    
    // Empty compatible_regions (required for fuzzy skin code, cannot be nullptr)
    LayerRegionPtrs compatible_regions;
    
    // Create PerimeterGenerator
    PerimeterGenerator g(
        &slices,
        &compatible_regions,  // must not be nullptr (fuzzy skin code dereferences it)
        layer_height,
        layer_height,  // slice_z (layer 2)
        perimeter_flow,
        &region_config,
        &object_config,
        &print_config,
        false,  // spiral_mode
        &loops,
        &gap_fill,
        &fill_surfaces,
        &fill_no_overlap
    );
    
    g.layer_id = 1;  // Layer 2 (0-indexed)
    g.ext_perimeter_flow = ext_perimeter_flow;
    g.overhang_flow = perimeter_flow;
    g.solid_infill_flow = perimeter_flow;
    // upper_slices and lower_slices remain nullptr (set in constructor)
    
    std::cout << "\nRunning process_classic()..." << std::endl;
    
    // Run classic wall generation
    g.process_classic();
    
    std::cout << "Generated " << loops.entities.size() << " loop entities" << std::endl;
    
    // Print loop details
    int loop_count = 0;
    for (const ExtrusionEntity* entity : loops.entities) {
        if (const auto* loop = dynamic_cast<const ExtrusionLoop*>(entity)) {
            std::cout << "  Loop " << loop_count << ": " << loop->paths.size() << " paths, "
                      << "role=" << static_cast<int>(loop->role()) << std::endl;
            loop_count++;
        } else if (const auto* coll = dynamic_cast<const ExtrusionEntityCollection*>(entity)) {
            std::cout << "  Collection with " << coll->entities.size() << " entities" << std::endl;
        }
    }
    
    // Extract all segments
    std::vector<Segment> segments;
    extract_segments_from_collection(loops, segments);
    
    std::cout << "\n=== Extracted Segments ===" << std::endl;
    std::cout << "Total segments: " << segments.size() << std::endl;
    for (size_t i = 0; i < segments.size(); ++i) {
        double len_mm = unscaled(segment_length(segments[i]));
        std::cout << "  [" << i << "] (" 
                  << unscaled(segments[i].from.x()) << "," << unscaled(segments[i].from.y()) 
                  << ")->(" << unscaled(segments[i].to.x()) << "," << unscaled(segments[i].to.y()) 
                  << ") len=" << len_mm << "mm" << std::endl;
    }
    
    // Check for duplicates with 0.1mm (100 micron) tolerance
    auto duplicates = find_duplicate_segments(segments, scaled<coord_t>(0.1));
    
    std::cout << "\n=== Duplicate Check (0.1mm tolerance) ===" << std::endl;
    std::cout << "Number of duplicate segment pairs: " << duplicates.size() << std::endl;
    
    // Print first few duplicates for debugging
    int shown = 0;
    for (const auto& dup : duplicates) {
        if (shown++ < 5) {
            std::cout << "  Duplicate: (" 
                      << unscaled(dup.first.from.x()) << "," << unscaled(dup.first.from.y()) 
                      << ")->(" << unscaled(dup.first.to.x()) << "," << unscaled(dup.first.to.y()) << ")"
                      << " vs ("
                      << unscaled(dup.second.from.x()) << "," << unscaled(dup.second.from.y()) 
                      << ")->(" << unscaled(dup.second.to.x()) << "," << unscaled(dup.second.to.y()) << ")"
                      << std::endl;
        }
    }
    if (duplicates.size() > 5) {
        std::cout << "  ... and " << (duplicates.size() - 5) << " more" << std::endl;
    }
    
    // Export SVG for visualization
    std::string svg_path = "/tmp/opencode/classic_walls_test.svg";
    export_segments_to_svg(svg_path, frame, segments);
    std::cout << "\nSVG exported to: " << svg_path << std::endl;
    
    std::cout << "\nResult: " << duplicates.size() << " duplicates" << std::endl;
    
    return duplicates.size();
}

} // anonymous namespace

TEST_CASE("Classic wall generation - 0.61mm frame with 6 walls", "[Classic]") {
    // This test reproduces the duplicate extrusion bug in classic mode.
    // With 0.61mm frame (slightly larger than 0.6mm line width), the classic
    // generator creates two perimeters that are only ~0.01mm apart:
    //   - Inner perimeter at 0.31mm inset
    //   - Outer perimeter at 0.30mm inset
    // These should be a single perimeter, not two nearly-identical ones.
    
    size_t duplicates = run_classic_test(0.61f);
    REQUIRE(duplicates == 0);
}
