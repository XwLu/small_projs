#include <cmath>
#include <iostream>
#include <vector>

namespace {
constexpr double kEgoHalfWidth = 0.0;
}

struct Vec2d {
  double x;
  double y;
  Vec2d(double _x, double _y) {
    x = _x;
    y = _y;
  }

  Vec2d operator+(const Vec2d &pt) const { return Vec2d(x + pt.x, y + pt.y); }

  Vec2d operator-(const Vec2d &pt) const { return Vec2d(x - pt.x, y - pt.y); }

  double norm2() const { return x * x + y * y; }
  double norm() const { return std::sqrt(norm2()); }

  double proj(const Vec2d &v) const { return x * v.x + y * v.y; }
  double prod(const Vec2d &v) const { return x * v.y - y * v.x; }
};

struct Polygon2d {
  std::vector<Vec2d> pts;
};

struct State {
  double x;
  double y;
  double theta;
};

double calLatMagnitude(const Polygon2d &poly, const State &ego, bool left) {
  Vec2d ego_pt(ego.x, ego.y);
  Vec2d ego_vec(std::cos(ego.theta), std::sin(ego.theta));
  double lat_magnitude = 0.0;
  for (const auto &pt : poly.pts) {
    const auto &v1 = pt - ego_pt;
    double proj = v1.proj(ego_vec);
    if (proj <= 0) {
      if (v1.prod(ego_vec) > 0.0) {
        if (!left) {
          std::cerr << "no way to right nudge\n";
          return -1.0;
        }
      } else {
        if (left) {
          std::cerr << "no way to left nudge\n";
          return -1.0;
        }
      }
      continue;
    }
    double dist = proj * std::sqrt(v1.norm2() - proj * proj) / v1.norm();
    if (v1.prod(ego_vec) > 0.0) {
      dist *= -1;
    }
    if (left) {
      dist += kEgoHalfWidth;
      lat_magnitude = std::max(lat_magnitude, dist);
    } else {
      dist -= kEgoHalfWidth;
      lat_magnitude = std::min(lat_magnitude, dist);
    }
  }
  return std::abs(lat_magnitude);
}

int main() {
  State ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.theta = M_PI_2;

  Polygon2d poly;
  poly.pts.emplace_back(Vec2d(1.0, 1.0));
  poly.pts.emplace_back(Vec2d(1.0, -1.0));
  poly.pts.emplace_back(Vec2d(3.0, 0.0));

  std::cout << calLatMagnitude(poly, ego, false) << std::endl;
  std::cout << calLatMagnitude(poly, ego, true) << std::endl;
  return 0;
}
