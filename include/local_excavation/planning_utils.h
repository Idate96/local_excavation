//
// Created by lorenzo on 15.02.22.
//

#ifndef M545_COVERAGE_SIM_PLANNING_UTILS_H
#define M545_COVERAGE_SIM_PLANNING_UTILS_H

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/grid_map_core.hpp>

namespace planning_utils {

inline grid_map::Polygon toPolygon(std::vector<Eigen::Vector2d> points) {
  ROS_INFO_STREAM("[PolygonConvertion]: points size: " << points.size());
  grid_map::Polygon polygon;
  polygon.setFrameId("map");
  for (auto& point : points) {
    polygon.addVertex(point);
  }
  return polygon;
}

}
#endif  // M545_COVERAGE_SIM_PLANNING_UTILS_H
