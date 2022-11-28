#include <algorithm>
#include <assert.h>
#include <iostream>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

enum class LatDecision { IGNORE, LEFT, RIGHT };

struct SLBoundary {
  double start_s = 0.0;
  double end_s = 0.0;
  double start_l = 0.0;
  double end_l = 0.0;
};

struct Obstacle {
  bool is_static = false;
  std::string id = "0";
  SLBoundary sl_bound;
};

struct PathBound {
  double start_s;
  double resolution;
  std::vector<std::pair<double, double>> bound;
};

namespace {
constexpr double FLAGS_obstacle_lon_start_buffer = 1.0;
constexpr double FLAGS_obstacle_lon_end_buffer = 0.5;
constexpr double FLAGS_obstacle_lat_buffer = 0.3;
constexpr double adc_width = 1.8;
constexpr double adc_s = 0.0;
} // namespace

using ObstacleEdge = std::tuple<int, double, double, double, std::string>;

std::vector<ObstacleEdge> SortObstaclesForSweepLine(
    const std::unordered_map<std::string, Obstacle> &obstacles,
    const double adc_s) {
  std::vector<ObstacleEdge> sorted_obstacles;

  // Go through every obstacle and preprocess it.
  for (const auto &obs_pair : obstacles) {
    const auto &obs = obs_pair.second;
    // Only focus on those within-scope obstacles.
    if (!obs.is_static) {
      continue;
    }
    // Only focus on obstacles that are ahead of ADC.
    if (obs.sl_bound.end_s < adc_s) {
      continue;
    }
    // Decompose each obstacle's rectangle into two edges: one at
    // start_s; the other at end_s.
    const auto &obstacle_sl = obs.sl_bound;
    sorted_obstacles.emplace_back(
        1, obstacle_sl.start_s - FLAGS_obstacle_lon_start_buffer,
        obstacle_sl.start_l - FLAGS_obstacle_lat_buffer,
        obstacle_sl.end_l + FLAGS_obstacle_lat_buffer, obs.id);
    sorted_obstacles.emplace_back(
        0, obstacle_sl.end_s + FLAGS_obstacle_lon_end_buffer,
        obstacle_sl.start_l - FLAGS_obstacle_lat_buffer,
        obstacle_sl.end_l + FLAGS_obstacle_lat_buffer, obs.id);
  }

  // Sort.
  std::sort(sorted_obstacles.begin(), sorted_obstacles.end(),
            [](const ObstacleEdge &lhs, const ObstacleEdge &rhs) {
              if (std::get<1>(lhs) != std::get<1>(rhs)) {
                return std::get<1>(lhs) < std::get<1>(rhs);
              } else {
                return std::get<0>(lhs) > std::get<0>(rhs);
              }
            });

  return sorted_obstacles;
}

bool UpdatePathBoundary(size_t idx, double left_bound, double right_bound,
                        PathBound *const path_boundary) {
  // Update the right bound (l_min):
  double new_l_min = std::max(path_boundary->bound[idx].first, right_bound);
  // Update the left bound (l_max):
  double new_l_max = std::min(path_boundary->bound[idx].second, left_bound);

  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (new_l_min > new_l_max) {
    std::cerr << "Path is blocked at idx = " << idx << "\n";
    return false;
  }
  // Otherwise, update path_boundaries and center_line; then return true.
  path_boundary->bound[idx].first = new_l_min;
  path_boundary->bound[idx].second = new_l_max;
  return true;
}

bool UpdatePathBoundaryWithBuffer(size_t idx, double left_bound,
                                  double right_bound,
                                  PathBound *const path_boundary) {
  // substract vehicle width when bound does not come from the lane boundary
  const double default_adc_buffer_coeff = 1.0;
  double left_adc_buffer_coeff = default_adc_buffer_coeff;
  double right_adc_buffer_coeff = default_adc_buffer_coeff;

  // Update the right bound (l_min):
  double new_l_min =
      std::max(path_boundary->bound[idx].first,
               right_bound + right_adc_buffer_coeff * 0.5 * adc_width);
  // Update the left bound (l_max):
  double new_l_max =
      std::min(path_boundary->bound[idx].second,
               left_bound - left_adc_buffer_coeff * 0.5 * adc_width);

  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (new_l_min > new_l_max) {
    std::cerr << "Path is blocked at idx = " << idx << "\n";
    return false;
  }
  // Otherwise, update path_boundaries and center_line; then return true.
  path_boundary->bound[idx].first = new_l_min;
  path_boundary->bound[idx].second = new_l_max;
  return true;
}

void TrimPathBounds(const int path_blocked_idx,
                    PathBound *const path_boundary) {
  if (path_blocked_idx != -1) {
    if (path_blocked_idx == 0) {
      std::cerr << "Completely blocked. Cannot move at all.\n";
    }
    int range =
        static_cast<int>(path_boundary->bound.size()) - path_blocked_idx;
    for (int i = 0; i < range; ++i) {
      path_boundary->bound.pop_back();
    }
  }
}

bool GetBoundaryFromStaticObstacles(
    const double adc_s,
    const std::unordered_map<std::string, LatDecision> &lat_decisions,
    const std::unordered_map<std::string, Obstacle> &obstacles,
    PathBound *const path_boundary, std::string *const blocking_obstacle_id) {
  // Preprocessing.
  auto sorted_obstacles = SortObstaclesForSweepLine(obstacles, adc_s);
  std::cout << "There are " << sorted_obstacles.size() << " obstacles.\n";
  size_t obs_idx = 0;
  int path_blocked_idx = -1;
  std::multiset<double, std::greater<double>> right_bounds;
  right_bounds.insert(std::numeric_limits<double>::lowest());
  std::multiset<double> left_bounds;
  left_bounds.insert(std::numeric_limits<double>::max());
  // Maps obstacle ID's to the decided ADC pass direction, if ADC should
  // pass from left, then true; otherwise, false.
  std::unordered_map<std::string, bool> obs_id_to_direction;

  // Step through every path point.
  for (size_t i = 1; i < path_boundary->bound.size(); ++i) {
    double curr_s = path_boundary->start_s + i * path_boundary->resolution;
    // Check and see if there is any obstacle change:
    if (obs_idx < sorted_obstacles.size() &&
        std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
      while (obs_idx < sorted_obstacles.size() &&
             std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
        const auto &curr_obstacle = sorted_obstacles[obs_idx];
        const double curr_obstacle_s = std::get<1>(curr_obstacle);
        const double curr_obstacle_l_min = std::get<2>(curr_obstacle);
        const double curr_obstacle_l_max = std::get<3>(curr_obstacle);
        const std::string curr_obstacle_id = std::get<4>(curr_obstacle);
        auto iter = lat_decisions.find(curr_obstacle_id);
        assert(iter != lat_decisions.end());
        std::cout << "id[" << curr_obstacle_id << "] s[" << curr_obstacle_s
                  << "] curr_obstacle_l_min[" << curr_obstacle_l_min
                  << "] curr_obstacle_l_max[" << curr_obstacle_l_max
                  << "] lat decision[" << static_cast<int>(iter->second)
                  << "]\n";
        if (std::get<0>(curr_obstacle) == 1) {
          // A new obstacle enters into our scope:
          //   - Decide which direction for the ADC to pass.
          //   - Update the left/right bound accordingly.
          //   - If boundaries blocked, then decide whether can side-pass.
          //   - If yes, then borrow neighbor lane to side-pass.
          if (iter->second == LatDecision::LEFT) {
            // Obstacle is to the right of center-line, should pass from left.
            obs_id_to_direction[curr_obstacle_id] = true;
            right_bounds.insert(curr_obstacle_l_max);
          } else {
            // Obstacle is to the left of center-line, should pass from right.
            obs_id_to_direction[curr_obstacle_id] = false;
            left_bounds.insert(curr_obstacle_l_min);
          }
          if (!UpdatePathBoundaryWithBuffer(i, *left_bounds.begin(),
                                            *right_bounds.begin(),
                                            path_boundary)) {
            path_blocked_idx = static_cast<int>(i);
            *blocking_obstacle_id = curr_obstacle_id;
            break;
          }
        } else {
          // An existing obstacle exits our scope.
          if (obs_id_to_direction[curr_obstacle_id]) {
            right_bounds.erase(right_bounds.find(curr_obstacle_l_max));
          } else {
            left_bounds.erase(left_bounds.find(curr_obstacle_l_min));
          }
          obs_id_to_direction.erase(curr_obstacle_id);
        }
        // Update the bounds
        path_boundary->bound[i].first =
            std::max(path_boundary->bound[i].first,
                     *right_bounds.begin() + 0.5 * adc_width);
        path_boundary->bound[i].second =
            std::min(path_boundary->bound[i].second,
                     *left_bounds.begin() - 0.5 * adc_width);
        if (path_boundary->bound[i].first > path_boundary->bound[i].second) {
          std::cerr << "Path is blocked at s = " << curr_s << "\n";
          path_blocked_idx = static_cast<int>(i);
          if (!obs_id_to_direction.empty()) {
            *blocking_obstacle_id = obs_id_to_direction.begin()->first;
          }
          break;
        }
        ++obs_idx;
      }
    } else {
      // If no obstacle change, update the bounds and center_line.
      path_boundary->bound[i].first =
          std::max(path_boundary->bound[i].first,
                   *right_bounds.begin() + 0.5 * adc_width);
      path_boundary->bound[i].second =
          std::min(path_boundary->bound[i].second,
                   *left_bounds.begin() - 0.5 * adc_width);
      if (path_boundary->bound[i].first > path_boundary->bound[i].second) {
        std::cerr << "Path is blocked at s = " << curr_s << "\n";
        path_blocked_idx = static_cast<int>(i);
        if (!obs_id_to_direction.empty()) {
          *blocking_obstacle_id = obs_id_to_direction.begin()->first;
        }
      }
    }

    // Early exit if path is blocked.
    if (path_blocked_idx != -1) {
      break;
    }
  }

  TrimPathBounds(path_blocked_idx, path_boundary);

  return true;
}

void PrintBoundary(const PathBound &boundary) {
  const double start_s = boundary.start_s;
  const double resolution = boundary.resolution;
  for (size_t i = 0; i < boundary.bound.size(); ++i) {
    const double curr_s = start_s + resolution * i;
    std::cout << "s[" << curr_s << "] right[" << boundary.bound[i].first
              << "] left[" << boundary.bound[i].second << "]\n";
  }
}

int main() {
  // create obstacles
  std::unordered_map<std::string, Obstacle> obstacles;
  std::unordered_map<std::string, LatDecision> lat_decisions;
  {
    Obstacle obs;
    obs.id = "10001";
    obs.is_static = true;
    obs.sl_bound.start_s = adc_s + 5.0;
    obs.sl_bound.end_s = obs.sl_bound.start_s + 1.0;
    obs.sl_bound.start_l = -2.0;
    obs.sl_bound.end_l = -0.5;
    obstacles.insert(std::make_pair(obs.id, obs));
    lat_decisions.insert(std::make_pair(obs.id, LatDecision::LEFT));
  }

  {
    Obstacle obs;
    obs.id = "10002";
    obs.is_static = true;
    obs.sl_bound.start_s = adc_s + 10.0;
    obs.sl_bound.end_s = obs.sl_bound.start_s + 1.0;
    obs.sl_bound.start_l = 0.5;
    obs.sl_bound.end_l = 2.0;
    obstacles.insert(std::make_pair(obs.id, obs));
    lat_decisions.insert(std::make_pair(obs.id, LatDecision::RIGHT));
  }
  // Build Lane Boundary
  PathBound boundary;
  boundary.start_s = adc_s;
  boundary.resolution = 0.5;
  for (int i = 0; i < 40; ++i) {
    boundary.bound.emplace_back(std::make_pair(-2.0, 2.0));
  }

  std::string blocking_obstacle_id;
  GetBoundaryFromStaticObstacles(adc_s, lat_decisions, obstacles, &boundary,
                                 &blocking_obstacle_id);
  PrintBoundary(boundary);
  return 0;
}
