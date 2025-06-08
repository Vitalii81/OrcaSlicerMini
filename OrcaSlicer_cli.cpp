#include <libslic3r/Print.hpp>
#include <libslic3r/Model.hpp>
#include <libslic3r/Config.hpp>
#include <libslic3r/TriangleMesh.hpp>
#include <libslic3r/PrintConfig.hpp>
#include <libslic3r/Point.hpp>
#include <libslic3r/Layer.hpp>
#include "OrcaSlicer_cli.hpp"

#include <iostream>
#include <fstream>
#include <csignal>
#include <cstdlib>
#include <string>
#include <algorithm>
#include <filesystem>
#include <tuple>
#include <vector>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <sys/types.h>
#include <signal.h>

using namespace Slic3r;
using namespace std;

/**
 * Logs a processing step message to stdout
 * @param msg The message to log
 */
void log(const char* msg) { std::cout << "=== " << msg << " ===" << std::endl; }

/**
 * Signal handler for critical errors
 * @param signum The signal number caught
 */
void signal_handler(int signum) { std::cerr << "Signal " << signum << " caught!" << std::endl; std::exit(signum); }


#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>

void render_slicingdata_png(const SlicingData& data, const std::string& filename, int width = 1024, int height = 1024) {
    std::vector<unsigned char> img(width * height * 3, 240); // Light gray background

    float angle_rad = 0.6f;
    float fov = 100.0f;     // For perspective
    float z_scale = 1.0f;
    float scale = 1.0f;

    // Calculate bounds in perspective projection at an angle
    float min_px = 1e9f, max_px = -1e9f;
    float min_py = 1e9f, max_py = -1e9f;
    for (const auto& layer : data.layers) {
        float z = layer.z_height;
        float depth = fov / (fov + z * z_scale);
        for (const auto& c : layer.perimeters) {
            for (const auto& pt : c.points) {
                float px = (pt.x + z * std::cos(angle_rad)) * depth;
                float py = (pt.y - z * std::sin(angle_rad)) * depth;
                min_px = std::min(min_px, px);
                max_px = std::max(max_px, px);
                min_py = std::min(min_py, py);
                max_py = std::max(max_py, py);
            }
        }
    }

    float dx = max_px - min_px;
    float dy = max_py - min_py;
    scale = std::min(width / dx, height / dy) * 0.9f;

    float offset_x = width / 2.0f - ((min_px + max_px) / 2.0f) * scale;
    float offset_y = height / 2.0f + ((min_py + max_py) / 2.0f) * scale;

    auto put_pixel = [&](int x, int y, unsigned char r, unsigned char g, unsigned char b) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        int idx = (y * width + x) * 3;
        img[idx + 0] = r;
        img[idx + 1] = g;
        img[idx + 2] = b;
    };

    auto draw_line = [&](int x0, int y0, int x1, int y1, unsigned char r, unsigned char g, unsigned char b) {
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;
        while (true) {
            put_pixel(x0, y0, r, g, b);
            if (x0 == x1 && y0 == y1) break;
            e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    };

    size_t total_layers = data.layers.size();
    for (size_t li = 0; li < total_layers; ++li) {
        const auto& layer = data.layers[li];
        float z = layer.z_height;
        float depth = fov / (fov + z * z_scale);

        // Color gradient
        float t = static_cast<float>(li) / std::max(total_layers - 1, size_t(1));
        unsigned char r = static_cast<unsigned char>(255 * t);
        unsigned char g = static_cast<unsigned char>(100 * (1.0f - t));
        unsigned char b = static_cast<unsigned char>(255 * (1.0f - t));

        for (const auto& contour : layer.perimeters) {
            const auto& pts = contour.points;
            for (size_t i = 0; i < pts.size(); ++i) {
                const auto& p1 = pts[i];
                const auto& p2 = pts[(i + 1) % pts.size()];

                float px1 = (p1.x + z * std::cos(angle_rad)) * depth * scale + offset_x;
                float py1 = (p1.y - z * std::sin(angle_rad)) * depth * scale;
                float px2 = (p2.x + z * std::cos(angle_rad)) * depth * scale + offset_x;
                float py2 = (p2.y - z * std::sin(angle_rad)) * depth * scale;

                draw_line(static_cast<int>(px1), static_cast<int>(offset_y - py1),
                          static_cast<int>(px2), static_cast<int>(offset_y - py2),
                          r, g, b);
            }
        }
    }

    stbi_write_png(filename.c_str(), width, height, 3, img.data(), width * 3);
    std::cout << "✅ Perspective image at an angle with colors saved: " << filename << std::endl;
}


#include <fstream>
#include <cmath>
#include <iostream>
#include <algorithm>

static void hsv_to_rgb(float h, float s, float v, float& r, float& g, float& b) {
    float c = v * s;
    float x = c * (1 - std::fabs(std::fmod(h * 6, 2) - 1));
    float m = v - c;

    if (h < 1.0f / 6.0f)      { r = c; g = x; b = 0; }
    else if (h < 2.0f / 6.0f) { r = x; g = c; b = 0; }
    else if (h < 3.0f / 6.0f) { r = 0; g = c; b = x; }
    else if (h < 4.0f / 6.0f) { r = 0; g = x; b = c; }
    else if (h < 5.0f / 6.0f) { r = x; g = 0; b = c; }
    else                     { r = c; g = 0; b = x; }

    r += m;
    g += m;
    b += m;
}

#include <cstdlib>

void openObjFile(const std::string& path) {
    std::string command = "xdg-open \"" + path + "\"";
    std::system(command.c_str());
}

void export_slicingdata_to_obj(const SlicingData& data, const std::string& filename) {
    std::ofstream out(filename);
    if (!out) { std::cerr << "Failed to open OBJ file\n"; return; }

    std::string base = filename.substr(0, filename.find_last_of('.'));
    std::string mtl_filename = base + ".mtl";
    out << "mtllib " << mtl_filename.substr(mtl_filename.find_last_of("/\\") + 1) << "\n";

    std::ofstream mtl(mtl_filename);
    if (!mtl) { std::cerr << "Failed to open MTL file\n"; return; }

    // 🌈 Rainbow colors
    size_t total_layers = data.layers.size();
    for (size_t li = 0; li < total_layers; ++li) {
        float hue = static_cast<float>(li) / std::max(total_layers - 1, size_t(1)); // 0..1
        float r, g, b;
        hsv_to_rgb(hue, 1.0f, 1.0f, r, g, b); // s = 1, v = 1 → full saturation

        mtl << "newmtl layer" << li << "\n";
        mtl << "Kd " << r << " " << g << " " << b << "\n";
        mtl << "d 1.0\n\n";
    }

    // 🟩 Objects: vertices + lines
    int vertex_id = 1;
    for (size_t li = 0; li < total_layers; ++li) {
        const auto& layer = data.layers[li];
        out << "usemtl layer" << li << "\n";
        for (const auto& contour : layer.perimeters) {
            const auto& pts = contour.points;
            for (size_t i = 0; i < pts.size(); ++i) {
                const auto& p1 = pts[i];
                const auto& p2 = pts[(i + 1) % pts.size()];
                out << "v " << p1.x << " " << p1.y << " " << layer.z_height << "\n";
                out << "v " << p2.x << " " << p2.y << " " << layer.z_height << "\n";
                out << "l " << vertex_id << " " << (vertex_id + 1) << "\n";
                vertex_id += 2;
            }
        }
    }

    out.close();
    mtl.close();
    std::cout << "✅ Saved with rainbow layers: " << filename << " + " << mtl_filename << std::endl;

    openObjFile(filename);
}

///////////////////////////////////////////////////////////////////////////

TypeFile get_file_type(const std::string& path) {
    std::string ext = path.substr(path.find_last_of(".") + 1);
    if (ext == "stl") return TypeSTL;
    if (ext == "step" || ext == "stp") return TypeSTEP;
    if (ext == "3mf") return Type3MF;
    if (ext == "obj") return TypeOBJ;
    return TypeUnknown;
}

TriangleMesh load_model(const std::string& path) {
    TypeFile type = get_file_type(path);
    TriangleMesh mesh;
    Model model;

    DynamicPrintConfig config;
    ConfigSubstitutionContext config_substitutions(ForwardCompatibilitySubstitutionRule::Disable);
    Semver file_version;

    bool repair = true;
    bool is_bbs_3mf = false; // Should be declared at function level as it's used in multiple places

    try {
        switch (type) {

            case TypeSTL:
                if (!mesh.ReadSTLFile(path.c_str(), repair)) {
                    std::cerr << "Failed to read STL file";
                    throw std::runtime_error("STL read error");
                }
                break;

            case TypeSTEP:
                model = Model::read_from_step(
                    path,
                    LoadStrategy::Default,
                    nullptr, nullptr, nullptr,
                    0.01, 20.0, false
                );
                if (model.objects.empty())
                    throw std::runtime_error("Empty STEP model");

                mesh = model.objects.front()->raw_mesh();
                break;

            case Type3MF:
                model = Model::read_from_file(
                    path,
                    &config,
                    &config_substitutions,
                    LoadStrategy::AddDefaultInstances,
                    nullptr, // plate_data
                    nullptr, // project_presets
                    &is_bbs_3mf,
                    &file_version,
                    nullptr, // Import3mfProgressFn
                    nullptr, // ImportstlProgressFn
                    nullptr, // BBLProject*
                    0,       // plate_id
                    nullptr  // ObjImportColorFn
                );

                if (model.objects.empty())
                    throw std::runtime_error("Empty 3MF model");

                mesh = model.objects.front()->raw_mesh();
                break;

            case TypeOBJ:
                model = Model::read_from_file(
                    path,
                    &config,
                    &config_substitutions,
                    LoadStrategy::AddDefaultInstances,
                    nullptr, // plate_data
                    nullptr, // project_presets
                    &is_bbs_3mf,
                    &file_version,
                    nullptr, // Import3mfProgressFn
                    nullptr, // ImportstlProgressFn
                    nullptr, // BBLProject*
                    0,       // plate_id
                    nullptr  // ObjImportColorFn
                );

                if (model.objects.empty())
                    throw std::runtime_error("Empty OBJ model");

                mesh = model.objects.front()->raw_mesh();
                break;

            default:
                throw std::runtime_error("Unsupported file format");
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << " (" << path << ")" << std::endl;
        return TriangleMesh(); // Return empty mesh in case of error
    }

    return mesh;
}

/**
 * Prints detailed information about a single layer
 * @param layer The layer data to display
 */
void print_slicing_data(const LayerData &layer) {
    std::cout << " (ID: " << layer.layer_id << ") "
              << "Z: " << layer.z_height << endl;

    for (size_t j = 0; j < layer.perimeters.size(); ++j) {
        const auto& contour = layer.perimeters[j];
        std::cout << "    Contour " << j
                  << (contour.is_outer ? " (outer)" : " (hole)")  // <<< added here!
                  << " points: " << contour.points.size() << std::endl;
        for (const auto& pt : contour.points) {
            std::cout << "      (" << pt.x << ", " << pt.y << ")" << std::endl;
        }
    }

}

/**
 * Prints information about all layers in the slicing data
 * @param data The complete slicing data to display
 */
void print_slicing_datas(const SlicingData& data) {
    std::cout << "Number of layers: " << data.layers.size() << endl;
    for (size_t i = 0; i < data.layers.size(); ++i) {
        print_slicing_data(data.layers[i]);
    }
}

/**
 * Creates a directory if it doesn't exist, or clears it if it exists
 * @param path Directory path to create/clear
 * @throws std::runtime_error if operation fails
 */
void create_directory_if_not_exists(const std::string& path) {
    namespace fs = std::filesystem;

    try {
        if (fs::exists(path)) {
            // Clear directory
            for (const auto& entry : fs::directory_iterator(path)) {
                fs::remove_all(entry.path());
            }
            std::cout << "Cleared existing directory: " << path << std::endl;
        } else {
            // Create new directory
            if (!fs::create_directories(path)) {
                throw std::runtime_error("Failed to create directory");
            }
            std::cout << "Created new directory: " << path << std::endl;
        }
    } catch (const fs::filesystem_error& e) {
        throw std::runtime_error("Directory operation failed for '" + path + "': " + e.what());
    }
}

/**
 * Writes a single layer's data to a binary stream
 * @param layer The layer data to write
 * @param out Output stream to write to
 */
void write_layer_to_stream(const LayerData& layer, std::ostream& out) {
    std::ostringstream buffer(std::ios::binary);

    // Write layer metadata
    buffer.write(reinterpret_cast<const char*>(&layer.layer_id), sizeof(layer.layer_id));
    buffer.write(reinterpret_cast<const char*>(&layer.z_height), sizeof(layer.z_height));

    // Write perimeter count
    uint32_t perimeters_count = layer.perimeters.size();
    buffer.write(reinterpret_cast<const char*>(&perimeters_count), sizeof(perimeters_count));

    for (const auto& contour : layer.perimeters) {
        uint32_t points_count = contour.points.size();
        buffer.write(reinterpret_cast<const char*>(&points_count), sizeof(points_count));

        // Write is_outer flag (1 byte)
        uint8_t is_outer_flag = contour.is_outer ? 1 : 0;
        buffer.write(reinterpret_cast<const char*>(&is_outer_flag), sizeof(is_outer_flag));

        // Write points
        buffer.write(reinterpret_cast<const char*>(contour.points.data()), points_count * sizeof(Point_));
    }

    std::string buffer_str = buffer.str();
    uint32_t layer_size = buffer_str.size();

    // Write layer size followed by the actual data
    out.write(reinterpret_cast<const char*>(&layer_size), sizeof(layer_size));
    out.write(buffer_str.data(), buffer_str.size());
}


/**
 * Writes a single layer to a binary file
 * @param layer The layer data to write
 * @param filename Path to the output file
 * @throws std::runtime_error if file cannot be opened
 */
void write_single_layer(const LayerData& layer, const std::string& filename) {
    std::ofstream out(filename, std::ios::binary);
    if (!out) throw std::runtime_error("Cannot open file: " + filename);
    write_layer_to_stream(layer, out);
}

/**
 * Writes layer data to either multiple files or a single file
 * @param data The complete slicing data to write
 * @param path Output directory (for separate files) or file path
 * @param separate_files Whether to write each layer to a separate file
 * @throws std::runtime_error if file operations fail
 */
void write_layers(const SlicingData& data, const std::string& path, bool separate_files = false) {
    if (separate_files) {
        auto layers = data.layers;

        for (const auto& layer : layers) {
            std::ostringstream filename;
            filename << path << "/layer_" << layer.layer_id << ".bin";
            write_single_layer(layer, filename.str());
        }
    } else {
        std::ofstream out(path, std::ios::binary);
        if (!out) throw std::runtime_error("Cannot open file: " + path);

        uint32_t layers_total = data.layers.size();
        out.write(reinterpret_cast<const char*>(&layers_total), sizeof(layers_total));
        for (const auto& layer : data.layers)
            write_layer_to_stream(layer, out);
    }
}

/**
 * Reads a single layer from a binary stream
 * @param in Input stream to read from
 * @return The parsed LayerData structure
 * @throws std::runtime_error if layer size mismatch is detected
 */
LayerData read_single_layer(std::istream& in) {
    LayerData layer;

    // Read layer size (can be used for verification or skipping)
    uint32_t layer_size = 0;
    in.read(reinterpret_cast<char*>(&layer_size), sizeof(layer_size));

    // Create buffer for reading layer
    std::vector<char> buffer(layer_size);
    in.read(buffer.data(), layer_size);
    std::istringstream layer_stream(std::string(buffer.data(), layer_size), std::ios::binary);

    // Read metadata
    layer_stream.read(reinterpret_cast<char*>(&layer.layer_id), sizeof(layer.layer_id));
    layer_stream.read(reinterpret_cast<char*>(&layer.z_height), sizeof(layer.z_height));

    // Read perimeters count
    uint32_t perimeters_count = 0;
    layer_stream.read(reinterpret_cast<char*>(&perimeters_count), sizeof(perimeters_count));

    for (uint32_t i = 0; i < perimeters_count; ++i)
    {
        uint32_t points_count = 0;
        layer_stream.read(reinterpret_cast<char*>(&points_count), sizeof(points_count));

        // Read is_outer flag (1 byte)
        uint8_t is_outer_flag = 0;
        layer_stream.read(reinterpret_cast<char*>(&is_outer_flag), sizeof(is_outer_flag));

        Contour contour;
        contour.is_outer = (is_outer_flag != 0);
        contour.points.resize(points_count);
        if (points_count > 0)
        {
            layer_stream.read(reinterpret_cast<char*>(contour.points.data()), points_count * sizeof(Point_));
        }

        layer.perimeters.push_back(std::move(contour));
    }

    return layer;
}

/**
 * Reads layer data from either multiple files or a single file
 * @param path Input directory (for separate files) or file path
 * @param separate_files Whether to read from separate files
 * @param layer_index Specific layer to read (-1 for all layers)
 * @return The parsed SlicingData structure
 * @throws std::runtime_error if file operations fail or layer index is invalid
 */
SlicingData read_layers(const std::string& path, bool separate_files = false, int layer_index = -1) {
    SlicingData data;

    if (separate_files) {
        std::vector<std::string> layer_files;
        for (const auto& entry : fs::recursive_directory_iterator(path)) {
            if (entry.is_regular_file()) {
                const std::string filename = entry.path().filename().string();
                if (filename.find("layer_") == 0 && filename.find(".bin") != std::string::npos)
                    layer_files.push_back(entry.path().string());
            }
        }

        // Sort files by numerical layer ID
        std::sort(layer_files.begin(), layer_files.end(), [](const std::string& a, const std::string& b) {
            auto extract_id = [](const std::string& filename) -> int {
                size_t start = filename.find("layer_") + 6;
                size_t end = filename.find(".bin");
                return std::stoi(filename.substr(start, end - start));
            };
            return extract_id(a) < extract_id(b);
        });

        if (layer_index >= 0) {
            if (layer_index >= static_cast<int>(layer_files.size()))
                throw std::runtime_error("Requested layer index out of range.");
            std::ifstream in(layer_files[layer_index], std::ios::binary);
            if (!in) throw std::runtime_error("Cannot open file: " + layer_files[layer_index]);
            data.layers.push_back(read_single_layer(in));
        } else {
            for (const auto& filename : layer_files) {
                std::ifstream in(filename, std::ios::binary);
                if (!in) throw std::runtime_error("Cannot open file: " + filename);
                cout << filename << ": ";
                auto lr = read_single_layer(in); print_slicing_data(lr);
                data.layers.push_back(lr);
            }
        }
    } else {
        std::ifstream in(path, std::ios::binary);
        if (!in) throw std::runtime_error("Cannot open file: " + path);

        uint32_t layers_total = 0;
        in.read(reinterpret_cast<char*>(&layers_total), sizeof(layers_total));
        for (uint32_t i = 0; i < layers_total; ++i)
            data.layers.push_back(read_single_layer(in));
    }

    return data;
}

//------------------------------------------------------------------

void remove_close_duplicates(Contour& contour, double tolerance = 1e-5)
{
    auto& pts = contour.points;
    pts.erase(std::unique(pts.begin(), pts.end(), [tolerance](const Point_& a, const Point_& b) {
                  return std::abs(a.x - b.x) < tolerance && std::abs(a.y - b.y) < tolerance;
              }), pts.end());

    // Check first and last point (if closed)
    if (!pts.empty()) {
        const auto& first = pts.front();
        const auto& last  = pts.back();
        double dx = first.x - last.x;
        double dy = first.y - last.y;
        if (std::sqrt(dx * dx + dy * dy) < tolerance) {
            pts.pop_back();  // Remove last if it's a duplicate of first
        }
    }
}


SlicingData collect_layers_data(const Slic3r::Print& print)
{
    SlicingData data;

    for (const auto* obj : print.objects())
    {
        const auto& layers = obj->layers();
        for (const auto* layer : layers)
        {
            LayerData layer_info;
            layer_info.layer_id = layer->id();
            layer_info.z_height = layer->print_z;

            const auto& regions = layer->regions();
            for (const auto* region : regions)
            {
                const auto& slices = region->get_slices();

                for (const auto& surface : slices)
                {
                    const auto& expoly = surface.expolygon;

                    // Outer contour
                    Contour outer;
                    outer.is_outer = true;   // <<< here!
                    for (const auto& pt : expoly.contour.points)
                    {
                        Slic3r::Vec2d unscaled_pt = Slic3r::unscale(pt);
                        outer.points.push_back(Point_{
                            static_cast<float>(unscaled_pt.x()),
                            static_cast<float>(unscaled_pt.y())
                        });
                    }
                    layer_info.perimeters.push_back(std::move(outer));

                    // Holes
                    for (const auto& hole : expoly.holes)
                    {
                        std::cout << "Hole contour points: " << hole.points.size() << std::endl;
                        Contour hole_contour;
                        hole_contour.is_outer = false;   // <<< here!
                        for (const auto& pt : hole.points)
                        {
                            Slic3r::Vec2d unscaled_pt = Slic3r::unscale(pt);
                            hole_contour.points.push_back(Point_{
                                static_cast<float>(unscaled_pt.x()),
                                static_cast<float>(unscaled_pt.y())
                            });
                        }
                        layer_info.perimeters.push_back(std::move(hole_contour));
                    }
                }
            }
            data.layers.push_back(std::move(layer_info));
        }
    }

    return data;
}


/**
 * Performs STL file slicing with given parameters
 * @param path Path to the STL file
 * @param layer_height Layer height in mm
 * @return Unique pointer to the Print object or nullptr on failure`
 */
std::unique_ptr<Slic3r::Print> slicing_file(const char* path, float layer_height)
{
    log("Loading model");
    Slic3r::TriangleMesh    mesh = load_model( path );

    auto print = std::make_unique<Slic3r::Print>();
    Slic3r::Model           model;

    Slic3r::ModelObject*    obj = model.add_object("Object1", path, std::move(mesh));
    Slic3r::ModelInstance*  instance = obj->add_instance();

    const auto bbox = obj->raw_bounding_box();
    const auto center = bbox.center();
    const auto size = bbox.max - bbox.min;

    std::cout << "Center: (" << center.x() << ", " << center.y() << ", " << center.z() << ")\n";
    std::cout << "Size (W x H x D): "
              << size.x() << " x " << size.y() << " x " << size.z() << " mm\n";

    // Apply Z offset to ensure model sits on build plate
    double z_shift = -bbox.min(2);
    instance->set_offset(instance->get_offset() + Vec3d(0, 0, z_shift));
    std::cout << "Applied Z shift: " << z_shift << " mm\n";

    if (model.objects.empty()) {
        std::cerr << "ERROR: Model has no objects!" << std::endl;
        return nullptr;
    }

    log("Setting up config");
    Slic3r::DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();
    config.set_key_value("layer_height", new Slic3r::ConfigOptionFloat(layer_height));
    config.set_key_value("first_layer_height", new Slic3r::ConfigOptionFloatOrPercent(layer_height, false));

    // 2. Contour settings (mandatory!)
    config.set_key_value("perimeters", new Slic3r::ConfigOptionInt(1));       // Outer contour
    config.set_key_value("thin_walls", new Slic3r::ConfigOptionBool(true));  // For precise contours
    config.set_key_value("detect_thin_walls", new Slic3r::ConfigOptionBool(true));  // For precise contours

    // 3. Disable all unnecessary features
    config.set_key_value("fill_density", new Slic3r::ConfigOptionPercent(0));    // No infill
    config.set_key_value("top_solid_layers", new Slic3r::ConfigOptionInt(0));   // No top layers
    config.set_key_value("bottom_solid_layers", new Slic3r::ConfigOptionInt(0));// No bottom layers
    config.set_key_value("skirts", new Slic3r::ConfigOptionInt(0));             // No skirts

    // 4. Critical parameters to avoid errors
    config.set_key_value("extrusion_width", new Slic3r::ConfigOptionFloat(0));  // Disable extrusion
    config.set_key_value("perimeter_extrusion_width", new Slic3r::ConfigOptionFloat(0));

    // 5. OrcaSlicer 2.3+ specific
    config.set_key_value("slice_closing_radius", new Slic3r::ConfigOptionFloat(0.1f)); // For clean contours
    config.set_key_value("gap_fill_enabled", new Slic3r::ConfigOptionBool(false));

    if (print->apply(model, config) == Slic3r::Print::APPLY_STATUS_INVALIDATED) {
        std::cerr << "ERROR: Config apply resulted in invalidated state" << std::endl;
        return nullptr;
    }

    log("Slicing (slice, perimeter, infill)");
    try {
        print->process();
    } catch (const std::exception& e) {
        std::cerr << "FATAL ERROR during slicing: " << e.what() << std::endl;
        return nullptr;
    }

    std::cout << "=== Slicing completed successfully ===" << std::endl;
    return print;
}

/**
 * Splits a file path into directory, filename, and extension components
 * @param full_path The complete file path
 * @return Tuple containing: (directory path, filename without extension, file extension)
 */
std::tuple<std::string, std::string, std::string> split_file_path(const std::string& full_path) {
    namespace fs = std::filesystem;

    fs::path path_obj(full_path);
    std::string directory = path_obj.parent_path().string();
    std::string filename = path_obj.stem().string();
    std::string extension = path_obj.extension().string();

    if (!extension.empty() && extension[0] == '.')
        extension.erase(0, 1);

    return std::make_tuple(directory, filename, extension);
}

int orca_slicer(int argc, char** argv)
{
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <path_to_stl> [layer_height]\n";
        return EXIT_FAILURE;
    }

    string path = argv[1];

    auto [dir, name, ext] = split_file_path(path);

    float layer_height = (argc >= 3) ? stof(argv[2]) : 0.2f;

    cout << "layer_height: " << layer_height << endl;

    // Set up signal handlers for critical errors
    signal(SIGSEGV, signal_handler);
    signal(SIGABRT, signal_handler);

    // Perform the slicing operation
    auto print =  slicing_file(path.c_str(), layer_height );

    if (!print) return EXIT_FAILURE;

    // Prepare output directory
    string path_n = dir + "/" + name;
    cout << "Path Out: " << path_n << std::endl;
    create_directory_if_not_exists(path_n);

    // Collect and write layer data
    auto layers = collect_layers_data(*print);
    write_layers(layers, path_n, true);

    export_slicingdata_to_obj(layers, (path_n + "/layers.obj") );
   // render_slicingdata_png(layers, (path_n + "/layers.png") );

    // Verify the written data by reading it back
    cout << "Read Bin:\n";
    auto rx = read_layers(path_n, true);
    return EXIT_SUCCESS;

}

/**
 * Main entry point for the slicing application
 * @param argc Argument count
 * @param argv Argument values
 * @return Exit status code
 */

using namespace std;
//namespace fs = std::filesystem;

int main(int argc, char** argv)
{
    cout << "--- Started Orca Slicer App ---" << endl;

    if (argc >= 2)
    {
        if (argc == 2 && strcmp(argv[1], "test") == 0) {
            cout << "--- Usage: " << argv[0] << " <path_to_file> [layer_height] ---" << endl;
            cout << "Test slicing model cube.stl" << endl;

            fs::path exe_path = fs::canonical(argv[0]);
            fs::path base_dir = exe_path.parent_path().parent_path().parent_path();
            static std::string model_str = (base_dir / "3dModel" / "cube.stl").string();
            const char* c_model = model_str.c_str();

            char* _argv[] = {
                (char*)"program_name",
                (char*)c_model,
                (char*)"0.4"
            };

            int _argc = sizeof(_argv) / sizeof(_argv[0]);
            return orca_slicer(_argc, _argv);

        } else if (argc == 2 && strcmp(argv[1], "-h") == 0) {
            cout << "Usage:\n";
            cout << "  ./run.sh <path_to_file> [layer_height]    Slice specified model with optional layer height\n";
            cout << "  ./run.sh test                             Slice default test model (cube.stl) with 0.4mm layer height\n";
            cout << "  ./run.sh -h                               Show this help message\n";
            cout << "\nAlternatively:\n";
            cout << "  " << argv[0] << " <path_to_file> [layer_height]\n";
            cout << "  " << argv[0] << " test\n";
            cout << "  " << argv[0] << " -h\n";


            return 0;

        } else {
            return orca_slicer(argc, argv);
        }
    }

    cout << "  " << argv[0] << " -h                                 Show this help message\n";

    return -1;
}

