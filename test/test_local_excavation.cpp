//
// Created by lorenzo on 28.10.21.
//
#include "local_excavation/LocalPlanner.hpp"
#include "excavation_mapping/ExcavationMapping.hpp"
#include "ros/ros.h"

// create gtest main function
int main(int argc, char **argv) {
  ros::init(argc, argv, "test_local_excavation");
  ros::NodeHandle nh;
//  excavation_mapping::ExcavationMapping mapping = excavation_mapping::ExcavationMapping(nh);
  // make a unique pointer to the instance of ExcavationMapping
  std::unique_ptr<excavation_mapping::ExcavationMapping> mapping_ptr =
      std::make_unique<excavation_mapping::ExcavationMapping>(nh);
  local_excavation::LocalPlanner planning = local_excavation::LocalPlanner(std::move(mapping_ptr));
  std::string desiredMapPath;
  nh.param<std::string>("desired_map_path", desiredMapPath, "/home");
  planning.initialize(desiredMapPath);
  planning.findDiggingPointLocalMap();
  Eigen::Vector3d diggingPoint = planning.getDiggingPoint();
  ROS_INFO("Digging point: %f, %f, %f", diggingPoint(0), diggingPoint(1), diggingPoint(2));
  ros::spin();
}