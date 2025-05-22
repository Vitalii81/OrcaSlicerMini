#include <libslic3r/Print.hpp>
#include <libslic3r/Model.hpp>
#include <libslic3r/Config.hpp>
#include <libslic3r/TriangleMesh.hpp>
#include <libslic3r/PrintConfig.hpp>
#include <libslic3r/Point.hpp>
#include <libslic3r/Layer.hpp>

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
void log_step(const char* msg) { std::cout << "=== " << msg << " ===" << std::endl; }

/**
 * Signal handler for critical errors
 * @param signum The signal number caught
 */
void signal_handler(int signum) { std::cerr << "Signal " << signum << " caught!" << std::endl; std::exit(signum); }

/**
 * Structure representing a 2D point with float coordinates
 */
struct Point_ {
    float x, y;  // X and Y coordinates
};

/**
 * Structure representing a contour/polygon composed of points
 */
struct Contour {
    std::vector<Point_> points;  // Collection of points forming the contour
    bool is_outer = true;  // true — outer perimeter, false — hole (inner)
};

/**
 * Structure representing data for a single layer
 */
struct LayerData {
    uint32_t __size;             // Size of layer data in bytes (for serialization)
    uint32_t layer_id;           // Identifier of the layer
    double z_height;             // Z-coordinate/height of the layer
    std::vector<Contour> perimeters;  // Collection of contours (outer perimeter and holes)
};

/**
 * Structure containing all slicing data for a model
 */
struct SlicingData {
    std::vector<LayerData> layers;  // Collection of all layers
};

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
                  << (contour.is_outer ? " (outer)" : " (hole)")  // <<< тут додаємо!
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
            //
            for (const auto& entry : fs::directory_iterator(path)) {
                fs::remove_all(entry.path());
            }
            std::cout << "Cleared existing directory: " << path << std::endl;
        } else {
            //
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

    // Read layer size (можеш використовувати для перевірки або пропуску)
    uint32_t layer_size = 0;
    in.read(reinterpret_cast<char*>(&layer_size), sizeof(layer_size));

    // Створюємо буфер для читання шару
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

    // Перевірити першу і останню точку (чи замкнено)
    if (!pts.empty()) {
        const auto& first = pts.front();
        const auto& last  = pts.back();
        double dx = first.x - last.x;
        double dy = first.y - last.y;
        if (std::sqrt(dx * dx + dy * dy) < tolerance) {
            pts.pop_back();  // Прибрати останню, якщо це дублікат першої
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
                    outer.is_outer = true;   // <<< тут!
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
                        hole_contour.is_outer = false;   // <<< тут!
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
 * @return Unique pointer to the Print object or nullptr on failure
 */
std::unique_ptr<Slic3r::Print> slicing_stl(const char* path, float layer_height = 0.1)
{
    auto print = std::make_unique<Slic3r::Print>();

    log_step("Loading model");
    Slic3r::TriangleMesh mesh;
    if (!mesh.ReadSTLFile(path, true, nullptr)) {
        std::cerr << "ERROR: Failed to read STL file!" << std::endl;
        return nullptr;
    }

    Slic3r::Model model;
    Slic3r::ModelObject* obj = model.add_object("Object1", path, std::move(mesh));
    Slic3r::ModelInstance* instance = obj->add_instance();

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

    log_step("Setting up config");
    Slic3r::DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();
    config.set_key_value("layer_height", new Slic3r::ConfigOptionFloat(layer_height));
    config.set_key_value("first_layer_height", new Slic3r::ConfigOptionFloatOrPercent(layer_height, false));

    // 2. Налаштування контурів (обов'язково!)
    config.set_key_value("perimeters", new Slic3r::ConfigOptionInt(1));       // Зовнішній контур
    config.set_key_value("thin_walls", new Slic3r::ConfigOptionBool(true));  // Для точних контурів
    config.set_key_value("detect_thin_walls", new Slic3r::ConfigOptionBool(true));  // Для точних контурів

    // 3. Вимкнення всього зайвого
    config.set_key_value("fill_density", new Slic3r::ConfigOptionPercent(0));    // Ніякого заповнення
    config.set_key_value("top_solid_layers", new Slic3r::ConfigOptionInt(0));   // Без верхніх шарів
    config.set_key_value("bottom_solid_layers", new Slic3r::ConfigOptionInt(0));// Без нижніх шарів
    config.set_key_value("skirts", new Slic3r::ConfigOptionInt(0));             // Без юбок

    // 4. Критичні параметри для уникнення помилок
    config.set_key_value("extrusion_width", new Slic3r::ConfigOptionFloat(0));  // Вимкнення екструзії
    config.set_key_value("perimeter_extrusion_width", new Slic3r::ConfigOptionFloat(0));

    // 5. Особливість OrcaSlicer 2.3+
    config.set_key_value("slice_closing_radius", new Slic3r::ConfigOptionFloat(0.1f)); // Для чистих контурів
    config.set_key_value("gap_fill_enabled", new Slic3r::ConfigOptionBool(false));

    if (print->apply(model, config) == Slic3r::Print::APPLY_STATUS_INVALIDATED) {
        std::cerr << "ERROR: Config apply resulted in invalidated state" << std::endl;
        return nullptr;
    }

    log_step("Slicing (slice, perimeter, infill)");
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
    string stl_path = argv[1];
    auto [dir, name, ext] = split_file_path(stl_path);

    string    out_path = (argc >= 4) ? dir:"./";
    float layer_height = (argc >= 3) ? stof(argv[2]) : 0.2f;

    cout << "layer_height: " << layer_height << endl;

    // Set up signal handlers for critical errors
    signal(SIGSEGV, signal_handler);
    signal(SIGABRT, signal_handler);

    // Perform the slicing operation
    auto print = slicing_stl(stl_path.c_str(), layer_height );

    if (!print) return EXIT_FAILURE;

    // Prepare output directory
    string path_n = dir + "/" + name;
    cout << "Path Out: " << path_n << std::endl;
    create_directory_if_not_exists(path_n);

    // Collect and write layer data
    auto layers = collect_layers_data(*print);
    write_layers(layers, path_n, true);

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
int main(int argc, char** argv) {
    cout << "--- Stated Orca Slicer App ---" << endl;
    if (argc  == 1) // When no argument
    {
        cout << "--- Usage: " << argv[0] << " <path_to_stl> [layer_height] ---\n";
        cout << "Test slicing model cube.stl\n";
        char* _argv[] = {
            "program_name",              // argv[0] — умовна назва програми
            "/home/vitalii/Desktop/Slice3rCore/3dModel/cube.stl",
            "0.4"
        };
        int _argc = sizeof(_argv) / sizeof(_argv[0]);

        return orca_slicer( _argc, _argv);
    }else
    {
        return orca_slicer( argc, argv);
    }

}
