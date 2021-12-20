//
// Created by lorenzo on 28.10.21.
//
#include "local_excavation/LocalPlanner.hpp"
#include <tf2_eigen/tf2_eigen.h>

namespace local_excavation {

LocalPlanner::LocalPlanner(std::unique_ptr<excavation_mapping::ExcavationMapping> excavationMapping)
    : excavationMappingPtr_(std::move(excavationMapping)) {
  // initialize the buffer and the listener
  tfBuffer_ = std::make_shared<tf2_ros::Buffer>();
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  // intialize the markerPublisher
  markerPublisher_ = nh_.advertise<visualization_msgs::Marker>("/local_excavation/marker", 1);
  // publishers
  localMapPublisher_ = nh_.advertise<grid_map_msgs::GridMap>("/local_excavation/submap", 1, true);
  // load the parameters
  this->loadParameters();
};

bool LocalPlanner::loadParameters() {
  // load the parameters
  if (!nh_.getParam("/local_excavation/local_map_size", localMapSize_)) {
    ROS_ERROR("Could not load the submap size");
    return false;
  }
  if (!nh_.getParam("/local_excavation/local_map_resolution", localMapResolution_)) {
    ROS_ERROR("Could not load the submap resolution");
    return false;
  }
  if (!nh_.getParam("/local_excavation/local_map_position", base_localMapPosition_)) {
    ROS_ERROR("Could not load the submap frame");
    return false;
  }
};

bool LocalPlanner::initialize(std::string designMapBag) {
  ROS_INFO_STREAM("[LocalPlanner] Initializing the local planner");
  ROS_INFO_STREAM("[LocalPlanner]: Loaded the following parameters:" << localMapSize_.at(0) << " " << localMapSize_.at(1));
  excavationMappingPtr_->initialize(designMapBag);
  localMap_.setFrameId("BASE");
  // convert std::vector<double> to Eigen::Vector2d
  localMap_.add("elevation");
  localMap_.add("desired_elevation");
  // print layers
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer_->lookupTransform("BASE", "BASE", ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }
  Eigen::Affine3d transform = tf2::transformToEigen(transformStamped);
  // print rotation
  Eigen::Vector3d rotation = transform.rotation().eulerAngles(0, 1, 2);
  ROS_INFO_STREAM("[LocalPlanner]: Loaded the following parameters:" << rotation.transpose());
  Eigen::Vector3d world_localMapPosition = transform * Eigen::Vector3d(base_localMapPosition_[0], base_localMapPosition_[1], 0);
  localMap_.setGeometry(grid_map::Length(localMapSize_.at(0), localMapSize_.at(1)), localMapResolution_,
                        world_localMapPosition.head(2));
  // rotate local map
  this->updateLocalMap();
  return true;
};

bool LocalPlanner::updateLocalMap() {
  // get the current world position from the base frame position using the tf2
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer_->lookupTransform("BASE", "BASE", ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }
  Eigen::Affine3d transform = tf2::transformToEigen(transformStamped);
  Eigen::Vector3d world_localMapPosition = transform * Eigen::Vector3d(base_localMapPosition_[0], base_localMapPosition_[1], 0);
  // translate local map to the current position
  localMap_.move(world_localMapPosition.head(2));
  bool extendMap = false;
  bool overwriteMap = true;
  bool copyAllLayers = false;
  this->addDataFrom(localMap_, excavationMappingPtr_->gridMap_, {"elevation", "desired_elevation"});
  this->publishLocalMap();
  return true;
};

bool LocalPlanner::addDataFrom(grid_map::GridMap& map, const grid_map::GridMap& other, std::vector<std::string> layers) {
  // Check if all layers to copy exist and add missing layers.
  for (const auto& layer : layers) {
    auto layers_ = map.getLayers();
    if (std::find(layers_.begin(), layers_.end(), layer) == layers_.end()) {
      map.add(layer);
    }
  }

  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer_->lookupTransform(other.getFrameId(), map.getFrameId(), ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }
  Eigen::Affine3d transform = tf2::transformToEigen(transformStamped);

  // Copy data.
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    if (map.isValid(*iterator)) continue;
    grid_map::Position position;
    map.getPosition(*iterator, position);
    Eigen::Vector3d otherPosition3 = transform * Eigen::Vector3d(position.x(), position.y(), 0);
    grid_map::Position otherPosition = grid_map::Position(otherPosition3.x(), otherPosition3.y());
    grid_map::Index index;
    if (!other.isInside(otherPosition)) continue;
    other.getIndex(otherPosition, index);
    for (const auto& layer : layers) {
      if (!other.isValid(index, layer)) continue;
      map.at(layer, *iterator) = other.at(layer, index) - otherPosition3.z();
    }
  }
  return true;
}

void LocalPlanner::publishLocalMap(){
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(localMap_, message);
  localMapPublisher_.publish(message);
}

grid_map::Position3 LocalPlanner::findRandomExcavationPoint() {
  // sample a (3, 1) vector from a gaussian with mean mu and std sigma
  // this is the mean of the gaussian
  // maybe use ros?
//  this->updateLocalMap();
  Eigen::Vector3d mu(6, 0, 0);
  // this is the standard deviation of the gaussian
  Eigen::Vector3d sigma(2, 2, 0);
  // sample a point from the gaussian
  Eigen::Vector3d base_pos_exc = mu + (sigma.array() * Eigen::Vector3d::Random().array()).matrix();
  Eigen::Vector3d map_pos_exc;
  // transform b_pos_exc from map to BASE frame using tfListener_ using the latest transform
  tf2::doTransform(base_pos_exc, map_pos_exc, tfBuffer_->lookupTransform("map", "BASE", ros::Time(0)));
//  ROS_INFO_STREAM("[LocalPlanner]: base_pos_exc: " << base_pos_exc.transpose());
  publishMarker(map_pos_exc, "map");
//  publishMarker(base_pos_exc, "BASE");
  ROS_INFO_STREAM("[LocalPlanner]: base_pos_exc: " << base_pos_exc.transpose());
  return grid_map::Position3(base_pos_exc(0), base_pos_exc(1), base_pos_exc(2));
};

//std::vector<Eigen::Vector3d> LocalPlanner::digTrajectory(Eigen::Vector3d base_digPos) {
//    // create straight line from origin to start in base frame
//    Eigen::Vector3d start_pos(0, 0, 0);
//    Eigen::Vector3d end_pos(base_digPos(0), base_digPos(1), base_digPos(2));
//    Eigen::Vector3d dir = end_pos - start_pos;
//    dir.normalize();
//    // create a vector of points along the line
//    std::vector<Eigen::Vector3d> trajectory;
//    for (int i = 0; i < 10; i++) {
//        trajectory.push_back(start_pos + dir * i);
//    }
//
//    // create line iterator for grid mpa that follow the trajectory
//    grid_map::LineIterator it(excavationMappingPtr_->gridMap_(), start_pos, end_pos);
//
//}


void LocalPlanner::publishMarker(grid_map::Position3 position, std::string frameId) const{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frameId;
  marker.header.stamp = ros::Time::now();
  marker.ns = "local_excavation";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = position.x();
  marker.pose.position.y = position.y();
  marker.pose.position.z = position.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  markerPublisher_.publish(marker);
}

} // namespace local_excavation