#include "terrain_config.h"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <exception>

namespace finenav::retap {
namespace {

template <typename T>
void readScalar(const YAML::Node &root, const char *key, T *out) {
  const YAML::Node value = root[key];
  if (!value) return;
  try {
    *out = value.as<T>();
  } catch (const YAML::Exception &) {
    throw std::runtime_error(std::string("invalid value for '") + key + "'");
  }
}

}  // namespace

bool loadTerrainConfigFile(const std::string &path, TerrainConfigFile *out,
                           std::string *error) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception &e) {
    if (error) *error = "failed to read " + path + ": " + e.what();
    return false;
  }
  if (!root.IsMap()) {
    if (error) *error = path + " is not a YAML mapping";
    return false;
  }
  if (root["retap_terrain"]) root = root["retap_terrain"];
  else if (root["terrain"]) root = root["terrain"];

  TerrainConfigFile result;
  retap_core::TerrainConfig &cfg = result.terrain;
  try {
    double resolution = 0.0;
    if (root["resolution"]) {
      resolution = root["resolution"].as<double>();
      result.resolution = resolution;
    }
    readScalar(root, "slope_threshold", &cfg.slope_threshold);
    readScalar(root, "roughness_threshold", &cfg.roughness_threshold);
    readScalar(root, "sparsity_threshold", &cfg.sparsity_threshold);
    readScalar(root, "traversability_threshold",
               &cfg.traversability_threshold);
    readScalar(root, "slope_weight", &cfg.slope_weight);
    readScalar(root, "roughness_weight", &cfg.roughness_weight);
    readScalar(root, "sparsity_weight", &cfg.sparsity_weight);
    readScalar(root, "traversability_weight", &cfg.traversability_weight);
    readScalar(root, "points_number_threshold", &cfg.points_number_threshold);
    readScalar(root, "collision_neighbor", &cfg.collision_neighbor);
    readScalar(root, "frame_height", &cfg.frame_height);
    readScalar(root, "filling_radius", &cfg.filling_radius);
    readScalar(root, "update_free", &cfg.update_free);
    readScalar(root, "terrain_update_period_sec",
               &cfg.terrain_update_period_sec);
    readScalar(root, "terrain_update_batch", &cfg.terrain_update_batch);
  } catch (const std::exception &e) {
    if (error) *error = e.what();
    return false;
  }

  const int neighbor = static_cast<int>(cfg.collision_neighbor);
  if (neighbor != 9 && neighbor != 17 && neighbor != 25) {
    if (error) {
      *error = "collision_neighbor must be 9, 17 or 25 (got " +
               std::to_string(neighbor) + ")";
    }
    return false;
  }
  if (cfg.update_free < 0 || cfg.update_free > 2) {
    if (error) {
      *error = "update_free must be 0, 1 or 2 (got " +
               std::to_string(cfg.update_free) + ")";
    }
    return false;
  }
  const float non_negative[] = {cfg.slope_threshold,
                                cfg.roughness_threshold,
                                cfg.sparsity_threshold,
                                cfg.traversability_threshold,
                                cfg.slope_weight,
                                cfg.roughness_weight,
                                cfg.sparsity_weight,
                                cfg.traversability_weight};
  for (const float v : non_negative) {
    if (!std::isfinite(v) || v < 0.0f) {
      if (error) *error = "terrain thresholds and weights must be finite and >= 0";
      return false;
    }
  }

  if (out) *out = result;
  return true;
}

}  // namespace finenav::retap
