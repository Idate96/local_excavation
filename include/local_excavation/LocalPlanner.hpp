#pragma once

#include "excavation_mapping/ExcavationMapping.hpp"
#include <grid_map_msgs/GridMap.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <eigen3/Eigen/Dense>

namespace local_excavation {

class LocalPlanner {

 public:
  LocalPlanner(std::unique_ptr<excavation_mapping::ExcavationMapping> excavationMapping);
  // ros
  ros::NodeHandle nh_;
  ros::Publisher markerPublisher_;
  // transform listener
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;


  bool initialize(std::string designMapBag);
  bool loadParameters();
  /*!
   * Update the submap and the global map using the measured elevation map.
   * @return
   */
  bool updateLocalMap();
  void publishLocalMap();
  void publishMarker(grid_map::Position3 position, std::string frameId) const;

  // planning
  grid_map::Position3 findExcavationPoint();
  grid_map::Position3 findRandomExcavationPoint();
  grid_map::Position3 findDumpPoint();
  bool completedWorkspace();

private:
  std::unique_ptr<excavation_mapping::ExcavationMapping> excavationMappingPtr_;
  // sub-map representing the reachable workspace of the robot
  grid_map::GridMap localMap_ = grid_map::GridMap();
  grid_map::Position3 excavationPoint_;
  grid_map::Index excavationPointIndex_;
  grid_map::Position3 dumpPoint_;
  grid_map::Index dumpPointIndex_;

  // ros
  ros::Publisher localMapPublisher_;

  // update local map
  bool addDataFrom(grid_map::GridMap& map, const grid_map::GridMap& other, std::vector<std::string> layers);

  // parameters
  double localMapResolution_;
  std::vector<double> localMapSize_;
  std::vector<double> world_localMapPosition_;
  std::vector<double> base_localMapPosition_;

};

} // namespace local_excavation