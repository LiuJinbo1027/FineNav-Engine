/*
 * Terrain parameter file reader for the retap tooling (S1/S2).
 *
 * retap_core::TerrainConfig is the single semantic source of the terrain
 * parameters; this file only maps YAML scalars onto it.  The same key names
 * are later used by the FineNav plugin's generate_parameter_library file, so a
 * calibration done here transfers verbatim.
 */
#pragma once

#include <optional>
#include <string>

#include "retap_core/terrain_types.h"

namespace finenav::retap {

struct TerrainConfigFile {
  retap_core::TerrainConfig terrain;
  std::optional<double> resolution;
};

// Reads a flat YAML mapping.  An optional `retap_terrain:` (or `terrain:`)
// wrapper is unwrapped first, so the file can also be embedded in a ROS
// parameter file without duplicating keys.
bool loadTerrainConfigFile(const std::string &path, TerrainConfigFile *out,
                           std::string *error);

}  // namespace finenav::retap
