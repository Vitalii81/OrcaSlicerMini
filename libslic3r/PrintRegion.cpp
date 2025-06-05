#include "Exception.hpp"
#include "Print.hpp"

namespace Slic3r {

coordf_t PrintRegion::nozzle_dmr_avg(const PrintConfig &print_config) const
{
    return (print_config.nozzle_diameter.get_at(m_config.wall_filament.value    - 1) + 
            print_config.nozzle_diameter.get_at(m_config.sparse_infill_filament.value       - 1) + 
            print_config.nozzle_diameter.get_at(m_config.solid_infill_filament.value - 1)) / 3.;
}

coordf_t PrintRegion::bridging_height_avg(const PrintConfig &print_config) const
{
    return this->nozzle_dmr_avg(print_config) * sqrt(m_config.bridge_flow.value);
}

void PrintRegion::collect_object_printing_extruders(const PrintConfig &print_config, const PrintRegionConfig &region_config, const bool has_brim, std::vector<unsigned int> &object_extruders)
{
    // These checks reflect the same logic used in the GUI for enabling/disabling extruder selection fields.
    // BBS
    auto num_extruders = (int)print_config.filament_diameter.size();
    auto emplace_extruder = [num_extruders, &object_extruders](int extruder_id) {
    	int i = std::max(0, extruder_id - 1);
        object_extruders.emplace_back((i >= num_extruders) ? 0 : i);
    };
    if (region_config.wall_loops.value > 0 || has_brim)
    	emplace_extruder(region_config.wall_filament);
    if (region_config.sparse_infill_density.value > 0)
    	emplace_extruder(region_config.sparse_infill_filament);
    if (region_config.top_shell_layers.value > 0 || region_config.bottom_shell_layers.value > 0)
    	emplace_extruder(region_config.solid_infill_filament);
}

void PrintRegion::collect_object_printing_extruders(const Print &print, std::vector<unsigned int> &object_extruders) const
{
    // PrintRegion, if used by some PrintObject, shall have all the extruders set to an existing printer extruder.
    // If not, then there must be something wrong with the Print::apply() function.
#ifndef NDEBUG
    // BBS
    auto num_extruders = int(print.config().filament_diameter.size());
    assert(this->config().wall_filament    <= num_extruders);
    assert(this->config().sparse_infill_filament       <= num_extruders);
    assert(this->config().solid_infill_filament <= num_extruders);
#endif
    collect_object_printing_extruders(print.config(), this->config(), print.has_brim(), object_extruders);
}

}
