/*
 * retap_map_tool - offline driver for retap_core (migration stage S1).
 *
 *   analyze : PCD -> IgTree -> terrain attributes -> .bt + diagnostic PCDs
 *   info    : inspect a processed .bt
 *   plan    : A* self-check on a processed .bt (no ROS involved)
 *
 * The .bt written here is the octomap ASCII container ReTAP itself uses
 * (WorldRepresentation::saveOctomapServerCallback calls IgTree::write): the
 * binary format only stores occupancy bits and would drop every terrain
 * attribute.
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "retap_core/astar_3d/astar_3d.h"
#include "retap_core/map_io.h"
#include "terrain_config.h"

namespace {

struct Args {
  std::map<std::string, std::string> values;

  bool has(const std::string &key) const {
    return values.find(key) != values.end();
  }
  std::string get(const std::string &key,
                  const std::string &fallback = "") const {
    const auto it = values.find(key);
    return it == values.end() ? fallback : it->second;
  }
};

void usage() {
  std::cout
      << "retap_map_tool - offline ReTAP terrain analysis\n"
         "\n"
         "usage:\n"
         "  retap_map_tool analyze --pcd <map.pcd> [--resolution 0.2]\n"
         "                         [--config <terrain.yaml>] [--output-dir DIR]\n"
         "                         [--center x,y,z]\n"
         "  retap_map_tool info    --bt <map_terrain.bt>\n"
         "                         [--trajectory poses.txt] [--histogram]\n"
         "  retap_map_tool plan    --bt <map_terrain.bt>\n"
         "                         --start x,y,z --goal x,y,z\n"
         "                         [--heuristic-weight 0.5]\n"
         "                         [--traversability-weight 0.5]\n"
         "                         [--snap-radius 1.0] [--path-pcd out.pcd]\n"
         "\n"
         "analyze writes <stem>_terrain.bt, <stem>_{slope,roughness,\n"
         "sparsity,traversability,collision,drivable,connected}.pcd and\n"
         "<stem>_stats.json into --output-dir (default ./retap_<stem>).\n"
         "`drivable` is 1 - collision; `connected` is 6-connectivity to the\n"
         "insertion point, which is a different (and much smaller) set.\n"
         "\n"
         "info --trajectory scores a recorded pose track (one `x y z` per\n"
         "line, `#` comments allowed) against the map: for every distinct\n"
         "voxel a pose falls in, the first occupied voxel down its column is\n"
         "the surface, and the surface's collision flag decides.  This is the\n"
         "metric the config/ experiments are compared with.\n"
         "info --histogram prints how many points the occupied voxels hold,\n"
         "which is what points_number_threshold / min_voxel_points act on.\n";
}

Args parseArgs(int argc, char **argv) {
  Args args;
  for (int i = 2; i < argc; ++i) {
    std::string token = argv[i];
    if (token.rfind("--", 0) != 0) {
      std::cerr << "ignoring unexpected argument: " << token << "\n";
      continue;
    }
    token = token.substr(2);
    const auto eq = token.find('=');
    if (eq != std::string::npos) {
      args.values[token.substr(0, eq)] = token.substr(eq + 1);
    } else if (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) {
      args.values[token] = argv[++i];
    } else {
      args.values[token] = "";  // flag
    }
  }
  return args;
}

bool parseVec3(const std::string &text, Eigen::Vector3f *out) {
  std::stringstream ss(text);
  std::string item;
  std::vector<float> values;
  while (std::getline(ss, item, ',')) {
    try {
      values.push_back(std::stof(item));
    } catch (const std::exception &) {
      return false;
    }
  }
  if (values.size() != 3) return false;
  *out = Eigen::Vector3f(values[0], values[1], values[2]);
  return true;
}

bool parseDouble(const std::string &text, double *out) {
  try {
    *out = std::stod(text);
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

std::string jsonEscape(const std::string &text) {
  std::string out;
  out.reserve(text.size());
  for (const char c : text) {
    if (c == '"' || c == '\\') out += '\\';
    out += c;
  }
  return out;
}

bool writeStatsJson(const std::filesystem::path &path,
                    const std::string &pcd_path, double resolution,
                    const retap_core::TerrainConfig &cfg,
                    const retap_core::TerrainStats &s,
                    const retap_core::TerrainBuildResult &build,
                    std::string *error) {
  std::ofstream f(path);
  if (!f.is_open()) {
    if (error) *error = "failed to open " + path.string();
    return false;
  }
  f << std::fixed << std::setprecision(6);
  f << "{\n";
  f << "  \"map\": \"" << jsonEscape(pcd_path) << "\",\n";
  f << "  \"resolution\": " << resolution << ",\n";
  f << "  \"center\": [" << build.center.x() << ", " << build.center.y()
    << ", " << build.center.z() << "],\n";
  f << "  \"input_points\": " << build.input_points << ",\n";
  f << "  \"occupied_voxels\": " << s.occupied_voxels << ",\n";
  f << "  \"drivable_voxels\": " << s.drivable_voxels << ",\n";
  f << "  \"connected_voxels\": " << s.connected_voxels << ",\n";
  f << "  \"drivable_ratio\": "
    << (s.occupied_voxels == 0
            ? 0.0
            : static_cast<double>(s.drivable_voxels) /
                  static_cast<double>(s.occupied_voxels))
    << ",\n";
  f << "  \"slope\": {\"min\": " << s.slope_min << ", \"mean\": "
    << s.slope_mean << ", \"max\": " << s.slope_max << "},\n";
  f << "  \"roughness\": {\"min\": " << s.roughness_min << ", \"mean\": "
    << s.roughness_mean << ", \"max\": " << s.roughness_max << "},\n";
  f << "  \"sparsity\": {\"min\": " << s.sparsity_min << ", \"mean\": "
    << s.sparsity_mean << ", \"max\": " << s.sparsity_max << "},\n";
  f << "  \"traversability\": {\"min\": " << s.traversability_min
    << ", \"mean\": " << s.traversability_mean << ", \"max\": "
    << s.traversability_max << "},\n";
  f << "  \"timing_ms\": {\"load\": " << build.load_ms << ", \"insert\": "
    << build.insert_ms << ", \"analysis\": " << build.analysis_ms << "},\n";
  f << "  \"config\": {\n";
  f << "    \"slope_threshold\": " << cfg.slope_threshold << ",\n";
  f << "    \"roughness_threshold\": " << cfg.roughness_threshold << ",\n";
  f << "    \"sparsity_threshold\": " << cfg.sparsity_threshold << ",\n";
  f << "    \"traversability_threshold\": " << cfg.traversability_threshold
    << ",\n";
  f << "    \"slope_weight\": " << cfg.slope_weight << ",\n";
  f << "    \"roughness_weight\": " << cfg.roughness_weight << ",\n";
  f << "    \"sparsity_weight\": " << cfg.sparsity_weight << ",\n";
  f << "    \"traversability_weight\": " << cfg.traversability_weight << ",\n";
  f << "    \"points_number_threshold\": " << cfg.points_number_threshold
    << ",\n";
  f << "    \"collision_neighbor\": "
    << static_cast<int>(cfg.collision_neighbor) << ",\n";
  f << "    \"clearance_voxels\": " << cfg.clearance_voxels << ",\n";
  f << "    \"support_voxels\": " << cfg.support_voxels << ",\n";
  f << "    \"min_voxel_points\": " << cfg.min_voxel_points << ",\n";
  f << "    \"frame_height\": " << cfg.frame_height << ",\n";
  f << "    \"filling_radius\": " << cfg.filling_radius << ",\n";
  f << "    \"update_free\": " << cfg.update_free << "\n";
  f << "  }\n";
  f << "}\n";
  if (!f.good()) {
    if (error) *error = "failed to write " + path.string();
    return false;
  }
  return true;
}

void printStats(const retap_core::TerrainStats &s) {
  const double ratio =
      s.occupied_voxels == 0
          ? 0.0
          : static_cast<double>(s.drivable_voxels) /
                static_cast<double>(s.occupied_voxels);
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "  occupied=" << s.occupied_voxels
            << " drivable=" << s.drivable_voxels << " (" << ratio * 100.0
            << "%) connected=" << s.connected_voxels << "\n";
  std::cout << "  slope          [" << s.slope_min << ", " << s.slope_mean
            << ", " << s.slope_max << "]\n";
  std::cout << "  roughness      [" << s.roughness_min << ", "
            << s.roughness_mean << ", " << s.roughness_max << "]\n";
  std::cout << "  sparsity       [" << s.sparsity_min << ", "
            << s.sparsity_mean << ", " << s.sparsity_max << "]\n";
  std::cout << "  traversability [" << s.traversability_min << ", "
            << s.traversability_mean << ", " << s.traversability_max << "]\n";
}

int runAnalyze(const Args &args) {
  const std::string pcd_path = args.get("pcd");
  if (pcd_path.empty()) {
    std::cerr << "analyze: --pcd is required\n";
    return 1;
  }

  retap_core::TerrainConfig cfg;
  std::optional<double> resolution;
  if (args.has("config")) {
    finenav::retap::TerrainConfigFile file;
    std::string error;
    if (!finenav::retap::loadTerrainConfigFile(args.get("config"), &file,
                                               &error)) {
      std::cerr << "analyze: " << error << "\n";
      return 1;
    }
    cfg = file.terrain;
    resolution = file.resolution;
  }
  if (args.has("resolution")) {
    double value = 0.0;
    if (!parseDouble(args.get("resolution"), &value) || value <= 0.0) {
      std::cerr << "analyze: invalid --resolution\n";
      return 1;
    }
    resolution = value;
  }
  const double res = resolution.value_or(0.2);

  std::optional<Eigen::Vector3f> center;
  if (args.has("center")) {
    Eigen::Vector3f value;
    if (!parseVec3(args.get("center"), &value)) {
      std::cerr << "analyze: --center expects x,y,z\n";
      return 1;
    }
    center = value;
  }

  const std::string stem = std::filesystem::path(pcd_path).stem().string();
  std::filesystem::path out_dir = args.get("output-dir");
  if (out_dir.empty()) out_dir = "retap_" + stem;
  std::error_code ec;
  std::filesystem::create_directories(out_dir, ec);
  if (ec) {
    std::cerr << "analyze: cannot create " << out_dir << ": " << ec.message()
              << "\n";
    return 1;
  }

  std::cout << "[analyze] " << pcd_path << "\n"
            << "  output dir: " << out_dir << "\n"
            << "  resolution: " << res << " m\n";

  retap_core::TerrainBuildResult build;
  std::string error;
  std::unique_ptr<retap_core::TerrainModel> model =
      retap_core::buildTerrainFromPcd(pcd_path, res, cfg, &build, &error,
                                      center ? &*center : nullptr);
  if (!model) {
    std::cerr << "analyze: " << error << "\n";
    return 1;
  }

  int exit_code = 0;
  struct FieldOut {
    retap_core::VoxelField field;
    const char *suffix;
  };
  const FieldOut fields[] = {
      {retap_core::VoxelField::kSlope, "_slope.pcd"},
      {retap_core::VoxelField::kRoughness, "_roughness.pcd"},
      {retap_core::VoxelField::kSparsity, "_sparsity.pcd"},
      {retap_core::VoxelField::kTraversability, "_traversability.pcd"},
      {retap_core::VoxelField::kCollision, "_collision.pcd"},
      // `_drivable` used to carry the connected flag -- a different question
      // ("same 6-connected component as the robot") answered with a number
      // under a name that says "drivable".  They are now separate files.
      {retap_core::VoxelField::kDrivable, "_drivable.pcd"},
      {retap_core::VoxelField::kConnected, "_connected.pcd"},
  };
  for (const auto &field : fields) {
    const std::filesystem::path path = out_dir / (stem + field.suffix);
    if (!retap_core::writeVoxelCloud(model->tree(), path.string(),
                                     field.field, &error)) {
      std::cerr << "analyze: " << error << "\n";
      exit_code = 1;
    }
  }

  const std::filesystem::path bt_path = out_dir / (stem + "_terrain.bt");
  if (!retap_core::saveTerrainBt(model->tree(), bt_path.string(), &error)) {
    std::cerr << "analyze: " << error << "\n";
    return 1;
  }

  const retap_core::TerrainStats stats =
      retap_core::computeStats(model->tree());
  const std::filesystem::path json_path = out_dir / (stem + "_stats.json");
  if (!writeStatsJson(json_path, pcd_path, res, cfg, stats, build, &error)) {
    std::cerr << "analyze: " << error << "\n";
    return 1;
  }

  std::cout << "  points=" << build.input_points << "\n";
  printStats(stats);
  std::cout << std::fixed << std::setprecision(1) << "  timing: load "
            << build.load_ms << " ms, insert " << build.insert_ms
            << " ms, analysis " << build.analysis_ms << " ms\n";
  std::cout << "  wrote " << bt_path << "\n  wrote " << json_path << "\n";

  if (stats.drivable_voxels == 0) {
    // collision_ defaults to "blocked", so this is the classic symptom of a
    // map that never went through terrain analysis.
    std::cerr << "analyze: no drivable voxel produced; check the thresholds "
                 "and that update_free matches the input (0 for static PCD)\n";
    return 3;
  }
  return exit_code;
}

// ── trajectory scoring and point-count histogram (analysis aids) ───────────
//
// RetAP has no notion of "is this recorded path passable", but the config/
// experiments are compared with exactly that number, so it lives here where
// it uses the same tree, the same keys and the same collision flag as the
// planner does.

struct TrajectoryReport {
  size_t poses = 0;         // lines read
  size_t cells = 0;         // distinct voxels the poses fall in
  size_t with_surface = 0;  // cells with an occupied voxel in the column
  size_t drivable = 0;      // ... whose surface is collision-free
  size_t connected = 0;     // ... of those, connected to the root
  size_t blocked = 0;       // ... whose surface is a collision
  size_t no_surface = 0;    // nothing occupied above/below the pose
};

bool readTrajectory(const std::string &path, std::vector<Eigen::Vector3f> *out,
                    std::string *error) {
  std::ifstream f(path);
  if (!f.is_open()) {
    if (error) *error = "cannot open " + path;
    return false;
  }
  std::string line;
  int line_no = 0;
  while (std::getline(f, line)) {
    ++line_no;
    const auto hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    std::istringstream ss(line);
    Eigen::Vector3f p;
    if (!(ss >> p.x() >> p.y() >> p.z())) continue;  // blank or incomplete
    (void)line_no;
    out->push_back(p);
  }
  if (out->empty()) {
    if (error) *error = path + " holds no `x y z` line";
    return false;
  }
  return true;
}

// The surface of a pose is the first occupied voxel found scanning down its
// column from 0.2 m above the pose to --column-depth metres below it.  The
// lidar sits above the floor and cannot see straight down, so a pose whose
// column is empty is reported as no_surface rather than as blocked.
TrajectoryReport scoreTrajectory(octomap::IgTree &tree,
                                 const std::vector<Eigen::Vector3f> &poses,
                                 double column_depth_m = 0.6) {
  const double res = tree.getResolution();
  const int up = static_cast<int>(std::ceil(0.2 / res));
  const int down = static_cast<int>(std::ceil(column_depth_m / res));
  std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> seen;

  TrajectoryReport report;
  report.poses = poses.size();
  for (const auto &p : poses) {
    octomap::OcTreeKey key;
    if (!tree.coordToKeyChecked(octomap::point3d(p.x(), p.y(), p.z()), key))
      continue;
    if (!seen.insert(key).second) continue;
    report.cells++;

    octomap::IgTreeNode *surface = nullptr;
    for (int dz = up; dz >= -down && !surface; --dz) {
      // OcTreeKey::k is uint16_t (the tree's max key is 65535), so the guard
      // must be against that range, not against 32767.
      const int z = static_cast<int>(key.k[2]) + dz;
      if (z < 0 || z > 65535) continue;
      octomap::OcTreeKey k = key;
      k.k[2] = static_cast<octomap::key_type>(z);
      octomap::IgTreeNode *node = tree.search(k);
      if (node && tree.isNodeOccupied(node)) surface = node;
    }
    if (!surface) {
      report.no_surface++;
    } else if (surface->getCollision()) {
      report.with_surface++;
      report.blocked++;
    } else {
      report.with_surface++;
      report.drivable++;
      if (surface->getConnected()) report.connected++;
    }
  }
  return report;
}

void printTrajectoryReport(const TrajectoryReport &r, double resolution) {
  const auto pct = [](size_t a, size_t b) {
    return b == 0 ? 0.0 : 100.0 * static_cast<double>(a) /
                               static_cast<double>(b);
  };
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "  trajectory: " << r.poses << " poses -> " << r.cells
            << " voxels (res " << resolution << " m)\n";
  std::cout << "    surface found " << r.with_surface << " ("
            << pct(r.with_surface, r.cells) << "%), none " << r.no_surface
            << " (" << pct(r.no_surface, r.cells) << "%)\n";
  std::cout << "    passable " << r.drivable << " ("
            << pct(r.drivable, r.with_surface) << "% of judged, "
            << pct(r.drivable, r.cells) << "% of all)  blocked " << r.blocked
            << "  connected-passable " << r.connected << "\n";
}

void printPointHistogram(octomap::IgTree &tree) {
  // Buckets of IgTreeNode::pointCount() over occupied voxels.  The NDT
  // statistics (and therefore slope / roughness / sparsity) are only
  // meaningful in the upper buckets, which is why this is printed next to
  // points_number_threshold and min_voxel_points.
  const int bounds[] = {1, 2, 5, 10, 19};  // upper bounds of the first buckets
  // 0 (never used: zeros are counted separately), <=1, <=2, <=5, <=10, <=19,
  // >19.  The last bucket must exist on its own: merging it into <=19 made
  // the line print both "<=19 100%" and ">=20 x%" from the same counter.
  size_t counts[7] = {0, 0, 0, 0, 0, 0, 0};
  size_t zeros = 0;
  size_t total = 0;
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
    if (!tree.isNodeOccupied(&*it)) continue;
    const int n = it->pointCount();
    total++;
    if (n <= 0) {
      zeros++;
      continue;
    }
    int bucket = 6;
    for (int b = 0; b < 5; ++b) {
      if (n <= bounds[b]) {
        bucket = b + 1;
        break;
      }
    }
    counts[bucket]++;
  }
  const auto pct = [total](size_t a) {
    return total == 0 ? 0.0 : 100.0 * static_cast<double>(a) /
                                   static_cast<double>(total);
  };
  std::cout << std::fixed << std::setprecision(1);
  std::cout << "  points/voxel over " << total << " occupied voxels:\n"
            << "    =1 " << pct(counts[1]) << "%  <=2 " << pct(counts[1] + counts[2])
            << "%  <=5 " << pct(counts[1] + counts[2] + counts[3])
            << "%  <=10 " << pct(counts[1] + counts[2] + counts[3] + counts[4])
            << "%  <=19 "
            << pct(counts[1] + counts[2] + counts[3] + counts[4] + counts[5])
            << "%  >19 " << pct(counts[6]) << "%";
  if (zeros) std::cout << "  (with 0 points: " << zeros << ")";
  std::cout << "\n";
}

int runInfo(const Args &args) {
  const std::string bt_path = args.get("bt");
  if (bt_path.empty()) {
    std::cerr << "info: --bt is required\n";
    return 1;
  }
  std::string error;
  std::unique_ptr<octomap::IgTree> tree =
      retap_core::loadTerrainBt(bt_path, &error);
  if (!tree) {
    std::cerr << "info: " << error << "\n";
    return 1;
  }
  const retap_core::TerrainStats stats = retap_core::computeStats(*tree);
  double min_x = 0, min_y = 0, min_z = 0, max_x = 0, max_y = 0, max_z = 0;
  tree->getMetricMin(min_x, min_y, min_z);
  tree->getMetricMax(max_x, max_y, max_z);
  std::cout << "[info] " << bt_path << "\n"
            << "  type=" << tree->getTreeType()
            << " resolution=" << tree->getResolution()
            << " nodes=" << tree->size() << "\n";
  std::cout << std::fixed << std::setprecision(2) << "  bounds min ["
            << min_x << ", " << min_y << ", " << min_z << "] max [" << max_x
            << ", " << max_y << ", " << max_z << "]\n";
  printStats(stats);

  if (args.has("histogram")) printPointHistogram(*tree);

  if (args.has("trajectory")) {
    std::vector<Eigen::Vector3f> poses;
    if (!readTrajectory(args.get("trajectory"), &poses, &error)) {
      std::cerr << "info: " << error << "\n";
      return 1;
    }
    printTrajectoryReport(scoreTrajectory(*tree, poses), tree->getResolution());
  }
  return 0;
}

bool writePathPcd(const std::string &path,
                  const std::vector<Eigen::Vector3f> &points,
                  std::string *error) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.reserve(points.size());
  for (const auto &p : points) cloud.emplace_back(p.x(), p.y(), p.z());
  cloud.width = static_cast<uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
  if (pcl::io::savePCDFileBinary(path, cloud) != 0) {
    if (error) *error = "failed to write " + path;
    return false;
  }
  return true;
}

int runPlan(const Args &args) {
  const std::string bt_path = args.get("bt");
  if (bt_path.empty()) {
    std::cerr << "plan: --bt is required\n";
    return 1;
  }
  Eigen::Vector3f start;
  Eigen::Vector3f goal;
  if (!parseVec3(args.get("start"), &start) ||
      !parseVec3(args.get("goal"), &goal)) {
    std::cerr << "plan: --start and --goal expect x,y,z\n";
    return 1;
  }

  std::string error;
  std::unique_ptr<octomap::IgTree> tree =
      retap_core::loadTerrainBt(bt_path, &error);
  if (!tree) {
    std::cerr << "plan: " << error << "\n";
    return 1;
  }

  double snap_radius = 1.0;
  if (args.has("snap-radius") &&
      !parseDouble(args.get("snap-radius"), &snap_radius)) {
    std::cerr << "plan: invalid --snap-radius\n";
    return 1;
  }
  Eigen::Vector3f start_snapped;
  Eigen::Vector3f goal_snapped;
  if (!retap_core::nearestDrivableVoxel(*tree, start, static_cast<float>(snap_radius),
                      &start_snapped)) {
    std::cerr << "plan: no drivable voxel within " << snap_radius
              << " m of the start\n";
    return 2;
  }
  if (!retap_core::nearestDrivableVoxel(*tree, goal, static_cast<float>(snap_radius),
                      &goal_snapped)) {
    std::cerr << "plan: no drivable voxel within " << snap_radius
              << " m of the goal\n";
    return 2;
  }
  std::cout << "[plan] " << bt_path << "\n"
            << "  start " << start.transpose() << " -> "
            << start_snapped.transpose() << "\n"
            << "  goal  " << goal.transpose() << " -> "
            << goal_snapped.transpose() << "\n";

  double heuristic_weight = 0.5;
  double traversability_weight = 0.5;
  if (args.has("heuristic-weight") &&
      !parseDouble(args.get("heuristic-weight"), &heuristic_weight)) {
    std::cerr << "plan: invalid --heuristic-weight\n";
    return 1;
  }
  if (args.has("traversability-weight") &&
      !parseDouble(args.get("traversability-weight"),
                   &traversability_weight)) {
    std::cerr << "plan: invalid --traversability-weight\n";
    return 1;
  }

  Astar astar(tree.get(), static_cast<float>(heuristic_weight),
              static_cast<float>(traversability_weight), 1.0f);
  astar.SetStartAndGoal(start_snapped, goal_snapped);
  const std::vector<Eigen::Vector3f> path = astar.startSearch();
  if (path.empty()) {
    std::cerr << "plan: A* found no path\n";
    return 2;
  }

  double length = 0.0;
  double max_step = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    const double step = (path[i] - path[i - 1]).norm();
    length += step;
    max_step = std::max(max_step, step);
  }
  const double res = tree->getResolution();
  std::cout << std::fixed << std::setprecision(3)
            << "  path: " << path.size() << " waypoints, length " << length
            << " m, max step " << max_step << " m (limit "
            << std::sqrt(3.0) * res << " m)\n";

  // The path must stay on occupied, collision-free voxels.
  size_t blocked = 0;
  for (const auto &p : path) {
    octomap::OcTreeKey key;
    if (!tree->coordToKeyChecked(octomap::point3d(p.x(), p.y(), p.z()), key)) {
      ++blocked;
      continue;
    }
    octomap::IgTreeNode *node = tree->search(key);
    if (node == nullptr || !tree->isNodeOccupied(node) ||
        node->getCollision()) {
      ++blocked;
    }
  }
  if (blocked != 0) {
    std::cerr << "plan: " << blocked << " path voxels are not drivable\n";
    return 4;
  }

  if (args.has("path-pcd")) {
    if (!writePathPcd(args.get("path-pcd"), path, &error)) {
      std::cerr << "plan: " << error << "\n";
      return 1;
    }
    std::cout << "  wrote " << args.get("path-pcd") << "\n";
  }
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    usage();
    return 1;
  }
  const std::string command = argv[1];
  if (command == "-h" || command == "--help" || command == "help") {
    usage();
    return 0;
  }
  const Args args = parseArgs(argc, argv);
  if (command == "analyze") return runAnalyze(args);
  if (command == "info") return runInfo(args);
  if (command == "plan") return runPlan(args);

  std::cerr << "unknown command: " << command << "\n";
  usage();
  return 1;
}
