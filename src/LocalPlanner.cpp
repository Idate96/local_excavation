//
// Created by lorenzo on 28.10.21.
//
#include "local_excavation/LocalPlanner.hpp"
#include <tf2_eigen/tf2_eigen.h>
#include "tf2/LinearMath/Transform.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
// include geometry_msgs
#include <geometry_msgs/PoseStamped.h>
#include "local_excavation/planning_utils.h"
#include <eigen3/Eigen/Dense>

namespace local_excavation {

LocalPlanner::LocalPlanner(std::unique_ptr<excavation_mapping::ExcavationMapping> excavationMapping)
    : excavationMappingPtr_(std::move(excavationMapping)) {
  // initialize the buffer and the listener
  tfBuffer_ = std::make_shared<tf2_ros::Buffer>();
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  // intialize the markerPublisher
  markerPublisher_ = nh_.advertise<visualization_msgs::Marker>("/local_excavation/marker", 1);
  markersTrajectoryPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/local_excavation/markers_trajectory", 1);
  penetrationDirectionPublisher_ = nh_.advertise<visualization_msgs::Marker>("/local_excavation/penetration_direction", 1, true);
  normalPublisher_ = nh_.advertise<visualization_msgs::Marker>("/local_excavation/normal", 1, true);
  projectedVectorPublisher_ = nh_.advertise<visualization_msgs::Marker>("/local_excavation/projected_vector", 1, true);
  diggingDirectionPublisher_ = nh_.advertise<visualization_msgs::Marker>("/local_excavation/digging_direction", 1, true);
  jointVectorPublisher_ = nh_.advertise<visualization_msgs::Marker>("/local_excavation/joint_vector", 1, true);
  desiredShovelOrientationPublisher_ = nh_.advertise<visualization_msgs::Marker>("/local_excavation/desired_shovel_orientation", 1, true);
  planningMapPublisher_ = nh_.advertise<grid_map_msgs::GridMap>("/local_excavation/planning_map", 1, true);
  desiredPosePublisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/local_excavation/se3_pose", 1, true);
  vectorsPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/local_excavation/vectors", 1, true);
  trajectoryPosesPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/local_excavation/trajectory_poses", 1, true);
  shovelPointsPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/local_excavation/shovel_points", 1, true);
  workspacePtsPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/local_excavation/workspace_points", 1, true);
  headingPublisher_ = nh_.advertise<visualization_msgs::Marker>("/local_excavation/heading", 1, true);
  polygonPublisher_ = nh_.advertise<geometry_msgs::PolygonStamped>("/local_excavation/polygon", 1, true);
  shovelFilterPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/local_excavation/shovel_filter", 1, true);
  // load the parameters
  this->loadParameters();
};

bool LocalPlanner::loadParameters() {
  bool loaded = nh_.param<double>("/local_excavation/height_precision", heightPrecision_, 0.1) &&
                nh_.param<double>("/local_excavation/inner_digging_radius", circularWorkspaceOuterRadius_, 6) &&
                nh_.param<double>("/local_excavation/outer_digging_radius", circularWorkspaceInnerRadius_, 8) &&
                nh_.param<double>("/local_excavation/inner_dumping_radius", dumpingZoneInnerRadius_ , 4.5) &&
                nh_.param<double>("/local_excavation/outer_dumping_radius", dumpingZoneOuterRadius_, 8) &&
                nh_.param<double>("/local_excavation/min_distance_shovel_to_base", minDistanceShovelToBase_, 3.5) &&
                nh_.param<double>("/local_excavation/workspace_angle", circularWorkspaceAngle_, M_PI / 2) &&
                nh_.param<double>("/local_excavation/max_volume", maxVolume_, 0.5) &&
                nh_.param<double>("/local_excavation/volume_weight", volumeWeight_, 1) &&
                nh_.param<double>("/local_excavation/distance_weight", distanceWeight_, 0.005) &&
                nh_.param<double>("/local_excavation/heading_weight", headingWeight_, 0.05) &&
                nh_.param<double>("/local_excavation/dump_height", dumpAtHeight_, 0.9) &&
                nh_.param<double>("/local_excavation/volume_dirt_threshold", volumeDirtThreshold_, 0.1) &&
                nh_.param<double>("/local_excavation/height_dirt_threshold", heightDirtThreshold_, 0.5) &&
                nh_.param<double>("/local_excavation/volume_threshold", volumeThreshold_, 0.5) &&
                nh_.param<double>("/local_excavation/height_threshold", heightThreshold_, 0.5);

  if (!loaded) {
    ROS_ERROR("[LocalPlanner]: Could not load parameters for local planner");
    return false;
  }
  return true;
//  // load the parameters
//  if (!nh_.getParam("/local_excavation/local_map_size", localMapSize_)) {
//    ROS_ERROR("Could not load the submap size");
//    return false;
//  }
//  if (!nh_.getParam("/local_excavation/local_map_resolution", localMapResolution_)) {
//    ROS_ERROR("Could not load the submap resolution");
//    return false;
//  }
//  if (!nh_.getParam("/local_excavation/local_map_position", base_localMapPosition_)) {
//    ROS_ERROR("Could not load the submap frame");
//    return false;
//  }
//  if (!nh_.getParam("/local_excavation/height_precision", heightPrecision_)) {
//    ROS_ERROR("Could not load height precision");
//    return false;
//  }

  //  if (!nh_.getParam("/local_excavation/inner_radius", circularWorkspaceInnerRadius_)) {
  //    ROS_ERROR("Could not load the inner radius");
  //    return false;
  //  }
  //  // this is not loaded now!!!!! che palle
  //  if (!nh_.getParam("/local_excavation/outer_radius", circularWorkspaceOuterRadius_)) {
  //    ROS_ERROR("Could not load the outer radius");
  //    return false;
  //  }
};

bool LocalPlanner::initialize(std::string designMapBag) {
  excavationMappingPtr_->initialize(designMapBag);
//  excavationMappingPtr_->gridMap_.add("planning_zones", 0);
//  excavationMappingPtr_->gridMap_.add("cost", 0);
  // start a new thread to create the planning zones and join it to the main thread
  unique_lock lock(mapMutex_);
  planningMap_.setFrameId(excavationMappingPtr_->gridMap_.getFrameId());
  planningMap_.setGeometry(excavationMappingPtr_->gridMap_.getLength(),
                           excavationMappingPtr_->gridMap_.getResolution(),
                           excavationMappingPtr_->gridMap_.getPosition());
  planningMap_.setTimestamp(excavationMappingPtr_->gridMap_.getTimestamp());
  planningMap_.add("elevation");
  planningMap_.add("desired_elevation", 0);
  planningMap_.add("planning_elevation", 0);
  planningMap_.add("original_elevation", 0);
  planningMap_.add("planning_zones", 0);
  planningMap_.add("cost", 0);
  planningMap_["elevation"] = excavationMappingPtr_->gridMap_["elevation"];
  planningMap_["original_elevation"] = excavationMappingPtr_->gridMap_["elevation"];
  planningMap_["desired_elevation"] = excavationMappingPtr_->gridMap_["desired_elevation"];
  planningMap_["planning_elevation"] = excavationMappingPtr_->gridMap_["elevation"];

  lock.unlock();

  this->createPlanningZones();
  this->createShovelFilter();
//    std::thread planningThread_ = std::thread(&LocalPlanner::createPlanningZones, this);
  // detach the thread
//  planningThread_.detach();
  this->updatePlanningMap();
  return true;
};

Eigen::Vector3d LocalPlanner::getDiggingPointBaseFrame() const {
  Eigen::Vector3d diggingPointBaseFrame;
  tf2::doTransform(diggingPoint_, diggingPointBaseFrame, tfBuffer_->lookupTransform("map", "BASE", ros::Time(0)));
  return diggingPointBaseFrame;
}


bool LocalPlanner::updatePlanningMap() {
//  this->addDataFrom(planningMap_, excavationMappingPtr_->gridMap_, {"elevation"});
  unique_lock lock(mapMutex_);
  planningMap_["elevation"] = excavationMappingPtr_->gridMap_["elevation"];
  lock.unlock();
  this->publishPlanningMap();
  this->excavationMappingPtr_->publishGridMap();
  return true;
};

void LocalPlanner::setLocalMap(grid_map::GridMap& localMap) {
  localMap_ = localMap;
  this->publishPlanningMap();
};

bool LocalPlanner::addDataFrom(grid_map::GridMap& map, const grid_map::GridMap& other, std::vector<std::string> layers) {
  // this function allow to copy over data from one map to another even if they don't have the same reference frame
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

void LocalPlanner::publishPlanningMap(){
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(planningMap_, message);
  planningMapPublisher_.publish(message);
}

bool LocalPlanner::findRandomDiggingPoint() {
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
  tf2::doTransform(base_pos_exc, map_pos_exc, tfBuffer_->lookupTransform("BASE", "map", ros::Time(0)));
//  ROS_INFO_STREAM("[LocalPlanner]: base_pos_exc: " << base_pos_exc.transpose());
  publishMarker(map_pos_exc, "map");
//  publishMarker(base_pos_exc, "BASE");
//  ROS_INFO_STREAM("[LocalPlanner]: base_pos_exc: " << base_pos_exc.transpose());
  // fill excavationPoint_
  diggingPoint_ = grid_map::Position3(map_pos_exc(0), map_pos_exc(1), map_pos_exc(2));
  return true;
};

double LocalPlanner::objectiveDistanceAndElevation(grid_map::Position& base_diggingPoint) {
  // get index of the digging point
  grid_map::Index index;
  if (!planningMap_.getIndex(base_diggingPoint, index)) {
    ROS_WARN("[LocalPlanner]: digging point is not inside local map");
    return -1;
  }
  // get the elevation of the digging point
  double diggingPointElevation = planningMap_.at("elevation", index);
  // get the desired elevation of the digging point
  double diggingPointDesiredElevation = planningMap_.at("desired_elevation", index);
  // get the distance between the digging point and the origin
  double distance = (base_diggingPoint).norm();
  // get the distance between the digging point and the desired elevation
  double distanceToDesiredElevation = diggingPointElevation - diggingPointDesiredElevation;
  double objective = -1;
  if (distanceToDesiredElevation > heightPrecision_) {
    // get the objective function
    double weightDistance = 0.01;
    double weightHeight = 10;
    objective = weightDistance * distance + weightHeight * distanceToDesiredElevation;
  }
  // print elevation
//  ROS_INFO_STREAM("[LocalPlanner]: digging point elevation: " << distanceToDesiredElevation);
  planningMap_.at("cost", index) = objective;
//  ROS_INFO_STREAM("[LocalPlanner]: point " << base_diggingPoint.transpose() << " has objective " << objective);
  return objective;
}

Eigen::Vector3d LocalPlanner::findDiggingPointLocalMap() {
  // find the 3d coordinates of the digging point in the base frame
  // we do an exhaustive search for the point in the local map that maximizes an objective function
  // iterate over all points in the local map and find the point that maximizes the objective function
  double maxObjective = -1;
  grid_map::Position bestDiggingPoint;

  for (grid_map::PolygonIterator iterator(planningMap_, digZone_); !iterator.isPastEnd(); ++iterator) {
      // get the position of the point
      grid_map::Position diggingPoint;
      planningMap_.getPosition(*iterator, diggingPoint);
//      ROS_INFO_STREAM("[LocalPlanner]: iterator index " << *iterator << " digging point " << diggingPoint.transpose());
      // get the elevation of the point
      double objective = this->objectiveDistanceAndElevation(diggingPoint);
      if (objective > maxObjective) {
        maxObjective = objective;
        bestDiggingPoint = diggingPoint;
      }
  }
  grid_map::Index index;
  if (!planningMap_.getIndex(bestDiggingPoint, index)) {
    ROS_WARN("[LocalPlanner]: digging point is not inside planning map");
    return Eigen::Vector3d(0, 0, 0);
  }
  ROS_INFO_STREAM("[LocalPlanner]: best digging point is " << bestDiggingPoint.transpose());
  double diggingPointElevation = planningMap_.at("elevation", index);
  double diggingPointDesiredElevation = planningMap_.at("desired_elevation", index);
  ROS_INFO_STREAM("[LocalPlanner]: best objective is " << maxObjective);
  ROS_INFO_STREAM("[LocalPlanner]: digging point elevation: " << diggingPointElevation);
  ROS_INFO_STREAM("[LocalPlanner]: digging point desired elevation: " << diggingPointDesiredElevation);
  // get height of digging point

  // show the map with the updated costs
//  this->publishPlanningMap();
//  Eigen::Vector3d diggingPoint3dBase(bestDiggingPoint.x(), bestDiggingPoint.y(), diggingPointElevation);
  Eigen::Vector3d diggingPoint3dMap(bestDiggingPoint.x(), bestDiggingPoint.y(), diggingPointElevation);
//   transform digging point from map to base frame using tfListener_ using the latest transform
//  tf2::doTransform(diggingPoint3dBase, diggingPoint3dMap, tfBuffer_->lookupTransform("map", "BASE", ros::Time(0)));
  this->publishMarker(diggingPoint3dMap, "map");
  // set digging point
  diggingPoint_ = diggingPoint3dMap;
  return diggingPoint3dMap;
};

loco_m545::RotationQuaternion LocalPlanner::findOrientationWorldToShovel(double shovelRollAngle, double shovelPitchAngle,
                                                                           double shovelYawAngle) {
  return loco_m545::RotationQuaternion(loco_m545::AngleAxis(shovelPitchAngle, 0, 1, 0) * loco_m545::AngleAxis(shovelRollAngle, 1, 0, 0) *
                                       loco_m545::AngleAxis(shovelYawAngle, 0, 0, 1));
}



Trajectory LocalPlanner::computeTrajectory(Eigen::Vector3d& w_P_wd){
  // w_P_wd is the digging point
  // point below the surface
  grid_map::Position wg_P_wd(w_P_wd(0), w_P_wd(1));
  double elevation = excavationMappingPtr_->getElevation(wg_P_wd);
  double desiredElevation = excavationMappingPtr_->getDesiredElevation(wg_P_wd);
  if (elevation - desiredElevation < heightPrecision_) {
//    ROS_WARN("[LocalPlanner]: digging point is not below the surface");
    return Trajectory();
  }
  // from w_P_wd we want compute the trajectory that scoops up most volume
  // we procede in this way:
  // 1. start tracing the trajectory from w_P_wd
  // 2. while tracing the trajectory:
  //    - if the trajectory is not valid, we stop tracing it i.e we check if shovel is full and if we reached the boundary of the working space
  //    - if the trajectory is valid, we compute the next point in the trajectory and the current volume for the trajectory
  // 3. when trajectory is not valid we stop and close the trajector
  Eigen::Vector3d w_P_wb;
  // get the transform between the ENDEFFECTOR_CONTACT frame and the BOOM frame using tf2
  geometry_msgs::TransformStamped T_mba;
  // get transform from base to cabin frame
  try {
    T_mba = tfBuffer_->lookupTransform("map", "BASE", ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  // get the position of the BASE in the map frame
  Eigen::Vector3d w_P_wba = Eigen::Vector3d(T_mba.transform.translation.x, T_mba.transform.translation.y, T_mba.transform.translation.z);
//  ROS_INFO_STREAM("Base origin in map frame: " << w_P_wba.transpose());
  double roll_b, pitch_b, yaw_b;
  tf2::Quaternion R_mba_q = tf2::Quaternion(T_mba.transform.rotation.x, T_mba.transform.rotation.y, T_mba.transform.rotation.z, T_mba.transform.rotation.w);
  tf2::Matrix3x3(R_mba_q).getRPY(roll_b, pitch_b, yaw_b);
  // base to digging point in world frame
  Eigen::Vector3d w_P_bad = w_P_wd - w_P_wba;
//  ROS_INFO_STREAM("[LocalPlanner]: digging point wrt base in world frame: " << w_P_bad.transpose());
  // transform from world to base frame using R_mba
  // convert R_mba to eigen quaternion
  Eigen::Quaterniond R_mba_qe(R_mba_q.w(), R_mba_q.x(), R_mba_q.y(), R_mba_q.z());
  // transform w_P_bad from world to base frame using R_mba
  Eigen::Vector3d ba_P_bad = R_mba_qe.inverse() * w_P_bad;

  // relative heading
//  ROS_INFO_STREAM("[LocalPlanner]: digging point wrt base in base frame: " << ba_P_bad.transpose());
  double relativeHeading = atan2(ba_P_bad(1), ba_P_bad(0));
//  ROS_INFO_STREAM("Base heading in map frame: " <<  yaw_b);
//  ROS_INFO_STREAM("[LocalPlanner]: opt traj relative heading is " << relativeHeading);
  double heading =  - yaw_b - relativeHeading;
//  ROS_INFO_STREAM("[LocalPlanner]: opt traj heading " << heading);

//  ROS_INFO_STREAM("[LocalPlanner]: True boom heading " << shovelYaw);
  // transform the yaw angle into a direction vector
  // get perpendicular vector in 2D
  Eigen::Vector3d w_P_dba = (-w_P_wd + w_P_wba).normalized();
  this->publishHeading(w_P_wd, w_P_dba, "map");


  // this takes care of the fact that we have a penetration phase
  Eigen::Vector3d w_P_wd_off = w_P_wd - radialOffset_ * w_P_dba;
  // penetration vector
  Eigen::Vector3d w_P_dd1 = Eigen::Vector3d(0, 0, 0);
  // vertical displacement is the difference between the desired elevation and the elevation of the digging point
  w_P_dd1(2) = desiredElevation - elevation;
  // these angles are all wrt the digging direction
  // for flat ground the desired attitude angle corresponds does not
  double slopeAngle = 0;
  double desiredLocalAttitudeAngle = 45 * M_PI / 180;
  double attitudeAngle = desiredLocalAttitudeAngle + slopeAngle;
  Eigen::Quaterniond R_ws_d = this->get_R_sw(0, -attitudeAngle, heading);

  // the projection in 2d of the penetration vector is parallel to the digging vector and has length
  // angle of attack, if 0 shovel moves along the local z axis of the end effector
  double alpha = 0;
  double diggingPathAngle = desiredLocalAttitudeAngle - alpha;
  double horizontalDisplacement = desiredElevation/tan(diggingPathAngle);
  w_P_dd1.head(2) = w_P_dba * horizontalDisplacement ;
  //  ROS_INFO_STREAM("[LocalPlanner]: digging vector in world frame " << w_P_dd1.transpose());
  //  ROS_INFO_STREAM("[LocalPlanner]: horizontal displacement " << horizontalDisplyaacement);

  Eigen::Vector3d w_P_wd1 = w_P_wd_off + w_P_dd1;
  Eigen::Quaterniond R_ws_d1 = this->get_R_sw(0, -attitudeAngle, heading);

  this->publishVector(w_P_wd, w_P_dd1, "map");

  bool valid = true;
  std::vector<Eigen::Vector3d> digPoints;
  std::vector<Eigen::Quaterniond> digOrientations;
  double targetAttitude = M_PI / 2;
  Eigen::Vector3d w_P_wd_current = w_P_wd1;
  double stepSize = planningMap_.getResolution() / 2;
  double volume = 0;
  double workspaceVolume = 0;
  grid_map::Matrix& data = planningMap_["planning_elevation"];
  grid_map::Matrix& workspace = planningMap_["planning_zones"];

  Eigen::Vector3d s_posLeftShovel_cl(0.0, 0.75, 0.0);
  Eigen::Vector3d s_posRightShovel_cr(0.0, -0.75, 0.0);
  // now we march with step size of planningMap resolution / 2 in the direction of the boom direction until the trajectory is not valid anymore
  int numSteps = 0;
  while (valid){
    Eigen::Vector3d w_P_next = w_P_wd_current + stepSize * w_P_dba;
    // position of the left point of the shovel (l) in world frame
    Eigen::Vector3d w_posLeftShovel_wl = w_P_next + R_ws_d1.inverse() * s_posLeftShovel_cl;
    Eigen::Vector3d w_posRightShovel_wr = w_P_next + R_ws_d1.inverse() * s_posRightShovel_cr;
    double elevationChange = 0;

    // update planning layer of the planning map by setting the height to the new point
    for (grid_map::LineIterator iterator(planningMap_, Eigen::Vector2d(w_posRightShovel_wr.block<2, 1>(0, 0)),
                                         Eigen::Vector2d(w_posLeftShovel_wl.block<2, 1>(0, 0)));
         !iterator.isPastEnd(); ++iterator) {
      grid_map::Position iteratorPosition;
      planningMap_.getPosition(*iterator, iteratorPosition);
      const grid_map::Index index(*iterator);
      double currentPointElevation =
          w_posRightShovel_wr(2) + (w_posLeftShovel_wl(2) - w_posRightShovel_wr(2)) /
                                           (w_posLeftShovel_wl.block<2, 1>(0, 0) - w_posRightShovel_wr.block<2, 1>(0, 0)).norm() *
                                           (iteratorPosition - w_posRightShovel_wr.block<2, 1>(0, 0)).norm();
      double terrainElevation = planningMap_.atPosition("elevation", iteratorPosition);
      // set the value if the elevation at the current position in layer elevation is lower than the current value
      if (currentPointElevation < terrainElevation) {
        data(index(0), index(1)) = currentPointElevation;
        elevationChange += terrainElevation - currentPointElevation;
        if (workspace(index(0), index(1)) < 0) {
          workspaceVolume += planningMap_.getResolution() * planningMap_.getResolution() * (terrainElevation - currentPointElevation);
        }
      }
    }
    // append point
    digPoints.push_back(w_P_next);
    // get volume
    double lineVolume = elevationChange * planningMap_.getResolution() * planningMap_.getResolution();
    volume += lineVolume;
    // check if the trajectory is valid
//    ROS_INFO_STREAM("[LocalPlanner]: line volume " << lineVolume);
//    ROS_INFO_STREAM("[LocalPlanner]: volume " << volume);
//    ROS_INFO_STREAM("[LocalPlanner]: workspace volume " << workspaceVolume);
    if (volume > maxVolume_) {
      valid = false;
      break;
    }
    // compute distance from the base
    double distanceFromBase = (w_P_next - w_P_wba).norm();
//    ROS_INFO_STREAM("[LocalPlanner]: distance from base " << distanceFromBase);
    if (distanceFromBase < minDistanceShovelToBase_) {
      valid = false;
      break;
    }
    w_P_wd_current = w_P_next;
    numSteps++;
//    ROS_INFO_STREAM("[LocalPlanner]: step " << numSteps << " volume " << volume);
  }
  // interpolate the orientation of the shovel
  double stepSizePitchOrientation = (targetAttitude - attitudeAngle) / (digPoints.size() - 1);
  double currentPitchOrientation = attitudeAngle;
  for (size_t i = 0; i < digPoints.size(); i++) {
    Eigen::Quaterniond R_ws_d = this->get_R_sw(0, -currentPitchOrientation, heading);
    digOrientations.push_back(R_ws_d);
    currentPitchOrientation += stepSizePitchOrientation;
  }
  // get last point of the trajectory
  Eigen::Vector3d w_P_wd_last = digPoints.back();
  // get the orientation of the shovel at the last point
  Eigen::Quaterniond R_ws_d_last = digOrientations.back();

  Eigen::Vector3d closingOffset(0.1, 0, 0.7);
  Eigen::Vector3d w_P_d2d3 = w_P_dba.normalized() * closingOffset(0);
  // get desired height at the end of the trajectory
  grid_map::Position wg_P_wd2(w_P_wd_last(0), w_P_wd_last(1));
  double elevation2 = excavationMappingPtr_->getElevation(wg_P_wd2);
  double desiredElevation2 = excavationMappingPtr_->getDesiredElevation(wg_P_wd2);
  // vertical displacement is the difference between the desired elevation and the elevation of the digging point
  w_P_d2d3(2) = elevation2 - desiredElevation + closingOffset(2);
  // transfrom to world frame by rotating yaw angle around the z axis
  Eigen::AngleAxisd R_wc(heading, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d w_P_wd3 = w_P_wd_last + w_P_d2d3;
  double theta = M_PI/2 - M_PI / 2; // last quadrant of the circle
                                       //  Eigen::Vector3d w_P_wd3 =
                                       //      w_P_wd2 +
  Eigen::Quaterniond R_ws_d3 = this->get_R_sw(0, - M_PI * 3 / 4, heading);
//  ROS_INFO_STREAM("[LocalPlanner]: Euler angles 3 " << R_ws_d3.toRotationMatrix().eulerAngles(0, 1, 2).transpose());

  // fuse together the two trajectories
  std::vector<Eigen::Vector3d> digPointsFused;
  std::vector<Eigen::Quaterniond> digOrientationsFused;
  digPointsFused.push_back(w_P_wd_off);
  digOrientationsFused.push_back(R_ws_d);
  digPointsFused.push_back(w_P_wd1);
  digOrientationsFused.push_back(R_ws_d1);
//  digPointsFused.insert(digPointsFused.end(), digPoints.begin(), digPoints.end());
//  digOrientationsFused.insert(digOrientationsFused.end(), digOrientations.begin(), digOrientations.end());
  digPointsFused.push_back(w_P_wd_last);
  digOrientationsFused.push_back(R_ws_d_last);
  digPointsFused.push_back(w_P_wd3);
  digOrientationsFused.push_back(R_ws_d3);
  // get the trajectory
  Trajectory trajectory;
  trajectory.positions = digPointsFused;
  trajectory.orientations = digOrientationsFused;
  trajectory.scoopedVolume = volume;
  trajectory.workspaceVolume = workspaceVolume;
  trajectory.distanceFromBase = (w_P_wd_off - w_P_wba).norm();
  trajectory.relativeHeading = abs(relativeHeading);

  // set the trajectory
//  this->publishDesiredShovelPose(w_P_wd, R_ws_d1);
//  this->publishDesiredShovelPose(w_P_wd3, R_ws_d3);
//  this->publishTrajectoryPoses(digPoints, digOrientations);
//  this->publishMarkersTrajectory(digPointsFused, "map");
  // reset elevation mapping
  unique_lock lock(mapMutex_);
  planningMap_["planning_elevation"] = planningMap_["elevation"];
  lock.unlock();
  return trajectory;
}

double LocalPlanner::volumeObjective(Trajectory trajectory){
  double objective = volumeWeight_ * trajectory.workspaceVolume + distanceWeight_ * trajectory.distanceFromBase + headingWeight_ * trajectory.relativeHeading;
  return objective;
}

void LocalPlanner::optimizeTrajectory() {
  // iterate over all the points belonging to the workspace polygon and compute a feasable trajectory starting from each point
  // and find the point that maximizes the objective function
  Trajectory bestTrajectory;
  double maxObjective = 0;
  for (grid_map::PolygonIterator iterator(planningMap_, digZone_); !iterator.isPastEnd(); ++iterator) {
    // get the position of the point
    grid_map::Position diggingPoint;
    planningMap_.getPosition(*iterator, diggingPoint);
    // get the elevation of the point
    double elevation = excavationMappingPtr_->getElevation(diggingPoint);
    // create the point in 3d
    Eigen::Vector3d w_P_wd(diggingPoint.x(), diggingPoint.y(), elevation);
    Trajectory trajectory = this->computeTrajectory(w_P_wd);
    double objective = this->volumeObjective(trajectory);
    if (objective > maxObjective) {
      bestTrajectory = trajectory;
      maxObjective = objective;
    }
  }
  ROS_INFO_STREAM("[LocalPlanner]: Best trajectory has volume " << maxObjective);
  optimalDigTrajectory_ = bestTrajectory;
}

bool LocalPlanner::isDigZoneComplete(){
  // sum up all the remaining volume in the digZone and check two conditions:
  // 1. the sum is smaller than the volumeTreshold
  // 2. difference in target height in each cell is lower than the heightTreshold
  double volume = 0;
  for (grid_map::PolygonIterator iterator(planningMap_, digZone_); !iterator.isPastEnd(); ++iterator) {
    // get the position of the point
    grid_map::Index index(*iterator);
    // get the elevation of the point
    double elevation = planningMap_.at("elevation", index);
    // get the desired elevation of the point
    double desiredElevation = planningMap_.at("desired_elevation", index);
    // get soil volume remaining
    double heightDifference = std::max(0.0, elevation - desiredElevation);
    if (heightDifference > heightThreshold_) {
      return false;
    }
    double soilVolume = heightDifference * planningMap_.getResolution() * planningMap_.getResolution();
    volume += soilVolume;
  }
  return volume < volumeThreshold_;
}

bool LocalPlanner::isLateralFrontZoneComplete(int zoneId){
  // check that the lateral front zones are clear of dirt and that the elevation of the terrain matches the original elevation of the terrain
  // assert zoneId is 1 or 2
  assert(zoneId == 1 || zoneId == 2);
  // iterate over the lateral front zone
  for (grid_map::PolygonIterator iterator(planningMap_, planningZones_.at(zoneId)); !iterator.isPastEnd(); ++iterator) {
    // get the position of the point
    grid_map::Index index(*iterator);
    // get the elevation of the point
    double elevation = planningMap_.at("elevation", index);
    // get the desired elevation of the point
    double desiredElevation = planningMap_.at("desired_elevation", index);
    // get soil volume remaining
    double heightDifference = std::max(0.0, elevation - desiredElevation);
    if (heightDifference > heightDirtThreshold_) {
      return false;
    }
  }
  return true;
}

void LocalPlanner::createShovelFilter(){
  // create a shovel filter by using grid map polygon
  // the filter is has the dimension of shovel is centered at the origin, length is shovelLength_ and width is shovelWidth_
  // find the 4 corners of the shovel, the center is at (0, 0)
  Eigen::Vector2d shovelCorner1(-shovelLength_ / 2, -shovelWidth_ / 2);
  Eigen::Vector2d shovelCorner2(-shovelLength_ / 2, shovelWidth_ / 2);
  Eigen::Vector2d shovelCorner3(shovelLength_ / 2, shovelWidth_ / 2);
  Eigen::Vector2d shovelCorner4(shovelLength_ / 2, -shovelWidth_ / 2);


  shovelFilter_.push_back(shovelCorner1);
  shovelFilter_.push_back(shovelCorner2);
  shovelFilter_.push_back(shovelCorner3);
  shovelFilter_.push_back(shovelCorner4);
  this->publishShovelFilter(shovelFilter_, "map");
}

void LocalPlanner::createPlanningZones(){
  // create the planning zones
  planningZones_.clear();
  // get vertices for the zones
  std::vector<Eigen::Vector2d> b_PosDigVertex_bd = getDiggingPatchVertices();
  std::vector<Eigen::Vector2d> w_PosDigVertex_wd;
  std::vector<Eigen::Vector2d> b_PosDumpFrontLeft_bdu = getLeftCircularFrontSegmentPatch();
  std::vector<Eigen::Vector2d> w_PosDumpFrontLeft_wdu;
  std::vector<Eigen::Vector2d> b_PosDumpFrontRight_bdu = getRightCircularFrontSegmentPatch();
  std::vector<Eigen::Vector2d> w_PosDumpFrontRight_wdu;
  std::vector<Eigen::Vector2d> b_PosDumpBackLeft_bdu = getLeftCircularBackSegmentPatch();
  std::vector<Eigen::Vector2d> w_PosDumpBackLeft_wdu;
  std::vector<Eigen::Vector2d> b_PosDumpBackRight_bdu = getRightCircularBackSegmentPatch();
  std::vector<Eigen::Vector2d> w_PosDumpBackRight_wdu;

  // transform the vertices from base frame to world frame
  geometry_msgs::TransformStamped T_bw;
  // get transform from world to base frame
  // wait until transform is available
  //  while (ros::ok() && !tfBuffer_->canTransform("BASE", "map", ros::Time(0))) {
  //    ROS_INFO_ONCE("[LocalPlanner]: waiting for transform from world to base_link");
  //    ros::Duration(0.1).sleep();
  //  }
  std::vector<Eigen::Vector2d> w_PosDigVertex_bd;
  try {
    T_bw = tfBuffer_->lookupTransform("map", "BASE", ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("[LocalPlanner]: %s", ex.what());
    ros::Duration(1.0).sleep();
  }
  // transform the vertices
    for (Eigen::Vector2d vertex : b_PosDigVertex_bd) {
      Eigen::Vector3d vertex_eigen(vertex(0), vertex(1), 0);
      Eigen::Vector3d vertex_w_eigen;
      tf2::doTransform(vertex_eigen, vertex_w_eigen, T_bw);
      Eigen::Vector2d vertex_w(vertex_w_eigen(0), vertex_w_eigen(1));
      w_PosDigVertex_bd.push_back(vertex_w);
    }
    for (Eigen::Vector2d vertex : b_PosDumpFrontLeft_bdu) {
      Eigen::Vector3d vertex_eigen(vertex(0), vertex(1), 0);
      Eigen::Vector3d vertex_w_eigen;
      tf2::doTransform(vertex_eigen, vertex_w_eigen, T_bw);
      Eigen::Vector2d vertex_w(vertex_w_eigen(0), vertex_w_eigen(1));
      w_PosDumpFrontLeft_wdu.push_back(vertex_w);
    }
    for (Eigen::Vector2d vertex : b_PosDumpFrontRight_bdu) {
      Eigen::Vector3d vertex_eigen(vertex(0), vertex(1), 0);
      Eigen::Vector3d vertex_w_eigen;
      tf2::doTransform(vertex_eigen, vertex_w_eigen, T_bw);
      Eigen::Vector2d vertex_w(vertex_w_eigen(0), vertex_w_eigen(1));
      w_PosDumpFrontRight_wdu.push_back(vertex_w);
    }
    for (Eigen::Vector2d vertex : b_PosDumpBackLeft_bdu) {
      Eigen::Vector3d vertex_eigen(vertex(0), vertex(1), 0);
      Eigen::Vector3d vertex_w_eigen;
      tf2::doTransform(vertex_eigen, vertex_w_eigen, T_bw);
      Eigen::Vector2d vertex_w(vertex_w_eigen(0), vertex_w_eigen(1));
      w_PosDumpBackLeft_wdu.push_back(vertex_w);
    }
    for (Eigen::Vector2d vertex : b_PosDumpBackRight_bdu) {
      Eigen::Vector3d vertex_eigen(vertex(0), vertex(1), 0);
      Eigen::Vector3d vertex_w_eigen;
      tf2::doTransform(vertex_eigen, vertex_w_eigen, T_bw);
      Eigen::Vector2d vertex_w(vertex_w_eigen(0), vertex_w_eigen(1));
      w_PosDumpBackRight_wdu.push_back(vertex_w);
    }


//
//  // transform the vertices from base frame to world frame
//  for (Eigen::Vector2d vertex : b_PosDigVertex_bd) {
//    Eigen::Vector3d vertex_eigen(vertex(0), vertex(1), 0);
//    Eigen::Vector3d vertex_w_eigen;
//    tf2::doTransform(vertex_eigen, vertex_w_eigen, T_bw);
//    Eigen::Vector2d vertex_w(vertex_w_eigen(0), vertex_w_eigen(1));
//    w_PosDigVertex_bd.push_back(vertex_w);
//  }

  // create the polygon for the digging zone
  this->publishWorkspacePts(w_PosDigVertex_bd, "map");
  digZone_ = planning_utils::toPolygon(w_PosDigVertex_bd); // zone 0
  dumpingLeftFrontZone_ = planning_utils::toPolygon(w_PosDumpFrontLeft_wdu); // zone 1
  dumpingRightFrontZone_ = planning_utils::toPolygon(w_PosDumpFrontRight_wdu); // zone 2
  dumpingLeftBackZone_ = planning_utils::toPolygon(w_PosDumpBackLeft_wdu); // zone 3
  dumpingRightBackZone_ = planning_utils::toPolygon(w_PosDumpBackRight_wdu); // zone 4

  // print the polygon
  //  for (grid_map::PolygonIterator iterator(excavationMappingPtr_->gridMap_, digZone); !iterator.isPastEnd(); ++iterator) {
  //    try {
  ////      ROS_INFO_STREAM("[LocalPlanner]: Adding dig zone to map at " << *iterator);
  //      excavationMappingPtr_->gridMap_.at("planning_zones", *iterator) = 1;
  //    } catch (std::out_of_range& e) {
  //      ROS_ERROR_STREAM("[LocalPlanner]: " << e.what());
  //    }
  //  }

  planningZones_.push_back(digZone_);
  planningZones_.push_back(dumpingLeftFrontZone_);
  planningZones_.push_back(dumpingRightFrontZone_);
  planningZones_.push_back(dumpingLeftBackZone_);
  planningZones_.push_back(dumpingRightBackZone_);

  std::vector<double> zoneValues = {-1, .2, .2, 1, 1};
  this->addPlanningZonesToMap(zoneValues);
  ROS_INFO_STREAM("[LocalPlanner]: planning zones created");
}


Eigen::Vector3d LocalPlanner::projectVectorOntoSubspace(Eigen::Vector3d& vector, Eigen::Matrix3Xd& subspaceBasis){
  // each column of subspaceBasis is a subspace basis vector, normalize it
//  for (int i = 0; i < subspaceBasis.cols(); i++) {
//    subspaceBasis.col(i) = subspaceBasis.col(i).normalized();
//  }
  // projection matrix
  Eigen::Matrix3d projectionMatrix = subspaceBasis * (subspaceBasis.transpose() * subspaceBasis).inverse() * subspaceBasis.transpose();
  // project vector onto subspace
  Eigen::Vector3d projectedVector = projectionMatrix * vector;
  return projectedVector;
};

Eigen::Vector3d LocalPlanner::findDiggingPointTrack(){
  // find the 3d coordinates of the digging point in the base frame
  // get number of tracks by diving the arc length by the track width
    double arcLength = circularWorkspaceOuterRadius_ * circularWorkspaceAngle_;
    int numTracks = arcLength / trackWidth_;
    return Eigen::Vector3d(0, 0, 0);
}

Eigen::Vector3d LocalPlanner::findShovelDesiredOrientation(Eigen::Vector3d& world_diggingPoint, Eigen::Vector3d& diggingDirection){
  // wanna find the orientation of the shovel that will be used to dig the digging point
  // the angle is selected such that the angle between the shovel and the normal of the digging point is minimized is fixed to a desired local attitude

  // get normal vector at the digging point
  Eigen::Vector3d normalVector = this->excavationMappingPtr_->calculateSurfaceNormalAtPosition(world_diggingPoint.head(2), 0.3, planningMap_, "elevation");
  this->publishNormal(world_diggingPoint, normalVector, "map");
  ROS_INFO_STREAM("[LocalPlanner]: normal vector at digging point " << normalVector.transpose());

  // vector that connects the end effector contact with the shovel
  Eigen::Vector3d world_P_es;
  // get the transform between the ENDEFFECTOR_CONTACT frame and the BOOM frame using tf2
  geometry_msgs::TransformStamped contactToJoint;
  // get transform from base to cabin frame
  try {
    contactToJoint = tfBuffer_->lookupTransform("SHOVEL", "ENDEFFECTOR_CONTACT", ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  // get the position of the boom in the map frame
  world_P_es = Eigen::Vector3d(contactToJoint.transform.translation.x, contactToJoint.transform.translation.y,
                                     contactToJoint.transform.translation.z);
  // project the normal vector onto the shovel plane span by the world_P_es and the diggingDirection vector
  Eigen::Vector3d shovelPlaneNormal = normalVector.cross(world_P_es);
  shovelPlaneNormal.normalize();
  ROS_INFO_STREAM("[LocalPlanner]: shovel plane normal " << shovelPlaneNormal.transpose());
  // get the angle between the shovel plane and the normal vector
  double shovelPlaneAngle = acos(shovelPlaneNormal.dot(normalVector));
  ROS_INFO_STREAM("[LocalPlanner]: shovel plane angle " << shovelPlaneAngle);

  // get the angle between the shovel and the normal vector
  double localAttitude = - M_PI / 4.; // local attitude angle
  double shovelNormalAngle = atan2(normalVector.y(), normalVector.x());
  double angleDifference = shovelNormalAngle - localAttitude;
  // get the desired shovel orientation
  Eigen::Vector3d shovelDesiredOrientation(cos(angleDifference), sin(angleDifference), 0);
  // find the basis of the subspace that contains the two vectors: digging direction and the world_P_es vector
  Eigen::Matrix3Xd subspaceBasis(3, 2);
  subspaceBasis.col(0) = world_P_es;
  this->publishJointVector(world_diggingPoint, world_P_es, "map");
  subspaceBasis.col(1) = diggingDirection;
  // test if projection works correctly
  Eigen::Vector3d projectedNormal = this->projectVectorOntoSubspace(shovelDesiredOrientation, subspaceBasis);
  this->publishProjectedVector(world_diggingPoint, projectedNormal, "map");

  this->publishDesiredShovelOrientation(world_diggingPoint, shovelDesiredOrientation, "map");
  return shovelDesiredOrientation;
}

Trajectory LocalPlanner::getDigTrajectoryWorldFrame(Eigen::Vector3d& w_P_wd){
  // I need to obtain the position vector form the end effector to the boom expressed in map frame
  // express the end effector in world frame
  Eigen::Vector3d w_P_wb;
  // get the transform between the ENDEFFECTOR_CONTACT frame and the BOOM frame using tf2
  geometry_msgs::TransformStamped T_mb;
  // get transform from base to cabin frame
  try {
    T_mb = tfBuffer_->lookupTransform("map", "BOOM", ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  // get the position of the boom in the map frame
  w_P_wb = Eigen::Vector3d(T_mb.transform.translation.x, T_mb.transform.translation.y, T_mb.transform.translation.z);
  ROS_INFO_STREAM("[LocalPlanner]: digging point in world frame " << w_P_wd.transpose());
  ROS_INFO_STREAM("[LocalPlanner]: boom origin in world frame " << w_P_wb.transpose());


  // positions 2d direction vector w_P_bd = w_P_bw - w_P_dw
  Eigen::Vector3d w_P_bd = - w_P_wb + w_P_wd;
  // z coordinate is set to 0
  w_P_bd(2) = 0;
  // make it unit vector
  w_P_bd.normalize();

  // initial orientation of the shovel
  double heading = - atan2(w_P_bd(1), w_P_bd(0));
//  ROS_INFO_STREAM("[LocalPlanner]: heading " << heading);
  // this must be wrt the worldf frame event though its weird if the y and x axis are not aligned with the shovel axis
//  loco_m545::RotationQuaternion shovelOrientation_loco = this->get_R_sw(0, -M_PI/2 , heading);
  // get boom heading

  // get the transform between the ENDEFFECTOR_CONTACT frame and the map frame using tf2
  geometry_msgs::TransformStamped T_sw;
  // get transform from base to cabin frame
  try {
    T_sw = tfBuffer_->lookupTransform("CABIN", "map", ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  // get rpy
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf2::Quaternion(T_sw.transform.rotation.x, T_sw.transform.rotation.y, T_sw.transform.rotation.z, T_sw.transform.rotation.w)).getRPY(roll, pitch, yaw);
//  ROS_INFO_STREAM("[LocalPlanner]: roll " << roll);
//  ROS_INFO_STREAM("[LocalPlanner]: pitch " << pitch);
//  ROS_INFO_STREAM("[LocalPlanner]: yaw " << yaw);

  // transform quaternion to euler angles
  Eigen::Quaterniond R_sw(T_sw.transform.rotation.w, T_sw.transform.rotation.x, T_sw.transform.rotation.y,
                          T_sw.transform.rotation.z);
  Eigen::Vector3d R_sw_euler = R_sw.toRotationMatrix().eulerAngles(0, 1, 2);
//  ROS_INFO_STREAM("[LocalPlanner]: Euler angles " << R_sw_euler.transpose());
  double shovelYaw = R_sw_euler(2);
//  ROS_INFO_STREAM("[LocalPlanner]: boom heading " << shovelYaw);


  Eigen::Quaterniond R_ws_0 = get_R_sw(0, -M_PI / 4, yaw);
//  ROS_INFO_STREAM("[LocalPlanner]: Euler angles 0 " << R_ws_0.toRotationMatrix().eulerAngles(0, 1, 2).transpose());
//  ROS_INFO_STREAM("[LocalPlanner]: orientation of the shovel " << R_ws_0.x() << " " << R_ws_0.y() << " " << R_ws_0.z() << " " << R_ws_0.w());
  // transform quaternion orientation into a 3d vector

  // shovel orientation in world frame
//  Eigen::Vector3d C_ws = this->findShovelDesiredOrientation(w_P_wd, w_P_bd);

//  ROS_INFO_STREAM("[LocalPlanner]: digging vector in world frame " << w_P_bd.transpose());
  // get heading from the vector, this is useful if you wanna go from distance to coordinates
  // get desired height
  grid_map::Position wg_P_wd(w_P_wd(0), w_P_wd(1));
  double elevation = excavationMappingPtr_->getElevation(wg_P_wd);
  double desiredElevation = excavationMappingPtr_->getDesiredElevation(wg_P_wd);

  // penetration vector
  Eigen::Vector3d w_P_dd1 = Eigen::Vector3d(0, 0, 0);
  // vertical displacement is the difference between the desired elevation and the elevation of the digging point
  w_P_dd1(2) = desiredElevation - elevation;
  // these angles are all wrt the digging direction
  // for flat ground the desired attitude angle corresponds does not
  double slopeAngle = 0;
  double desiredLocalAttitudeAngle = 45 * M_PI / 180;
  double attitudeAngle = desiredLocalAttitudeAngle + slopeAngle;
  // the projection in 2d of the penetration vector is parallel to the digging vector and has length
  // angle of attack, if 0 shovel moves along the local z axis of the end effector
  double alpha = 0;
  double diggingPathAngle = desiredLocalAttitudeAngle - alpha;
  double horizontalDisplacement = desiredElevation/tan(diggingPathAngle);
  w_P_dd1.head(2) = - wg_P_wd.normalized() * horizontalDisplacement;
//  ROS_INFO_STREAM("[LocalPlanner]: digging vector in world frame " << w_P_dd1.transpose());
//  ROS_INFO_STREAM("[LocalPlanner]: horizontal displacement " << horizontalDisplacement);

  Eigen::Vector3d w_P_wd1 = w_P_wd + w_P_dd1;
  Eigen::Quaterniond R_ws_d1 = this->get_R_sw(0, -M_PI / 4, yaw);
//  ROS_INFO_STREAM("[LocalPlanner]: Euler angles 1 " << R_ws_d1.toRotationMatrix().eulerAngles(0, 1, 2).transpose());

//  this->publishVector(w_P_wd, w_P_dd1, "map");

  double trajectoryLength = 2;
  Eigen::Vector3d w_P_d1d2 = - w_P_bd * trajectoryLength;
  // displace the w_P_wd1 along the digging vector for the desired length
  Eigen::Vector3d w_P_wd2 = w_P_wd1 + w_P_d1d2;
  Eigen::Quaterniond R_ws_d2 = this->get_R_sw(0, - M_PI / 2., yaw);
//  ROS_INFO_STREAM("[LocalPlanner]: Euler angles 2 " << R_ws_d2.toRotationMatrix().eulerAngles(0, 1, 2).transpose());


//  this->publishVector(w_P_wd1, w_P_bd, "map");
  // closing offset in cabin frame
  Eigen::Vector3d closingOffset(0.1, 0, 0.7);
  Eigen::Vector3d w_P_d2d3 = - w_P_bd.normalized() * closingOffset(0);
  // get desired height at the end of the trajectory
  grid_map::Position wg_P_wd2(w_P_wd2(0), w_P_wd2(1));
  double elevation2 = excavationMappingPtr_->getElevation(wg_P_wd2);
  double desiredElevation2 = excavationMappingPtr_->getDesiredElevation(wg_P_wd2);
  // vertical displacement is the difference between the desired elevation and the elevation of the digging point
  w_P_d2d3(2) = elevation2 - desiredElevation + closingOffset(2);
  // transfrom to world frame by rotating yaw angle around the z axis
  Eigen::AngleAxisd R_wc(shovelYaw, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d w_P_wd3 = w_P_wd2 + w_P_d2d3;
  double theta = M_PI/2 - M_PI / 2; // last quadrant of the circle
//  Eigen::Vector3d w_P_wd3 =
//      w_P_wd2 +
  Eigen::Quaterniond R_ws_d3 = this->get_R_sw(0, - M_PI * 3 / 4, yaw);
//  ROS_INFO_STREAM("[LocalPlanner]: Euler angles 3 " << R_ws_d3.toRotationMatrix().eulerAngles(0, 1, 2).transpose());

  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Quaterniond> orientations;

  positions.push_back(w_P_wd);
  orientations.push_back(R_ws_0);
//  this->publishDesiredShovelPose(w_P_wd, R_ws_0);
  positions.push_back(w_P_wd1);
  orientations.push_back(R_ws_d1);
//  this->publishDesiredShovelPose(w_P_wd1, R_ws_d1);
  positions.push_back(w_P_wd2);
  orientations.push_back(R_ws_d2);
//  this->publishDesiredShovelPose(w_P_wd2, R_ws_d2);
  positions.push_back(w_P_wd3);
  orientations.push_back(R_ws_d3);
//  this->publishDesiredShovelPose(w_P_wd3, R_ws_d3);
//  this->publishTrajectoryPoses(positions, orientations);
  Trajectory trajectory;
  trajectory.positions = positions;
  trajectory.orientations = orientations;
//  this->publishMarkersTrajectory(positions, "map");
  return trajectory;
}

//Eigen::Quaterniond LocalPlanner::C_sw(double shovelRollAngle, double shovelPitchAngle, double shovelYawAngle) {
//  // assume that only the cabin is flat (zero roll and pitch)
//  // TODO: make it more general C_{SW} = C_{SB} * C_{BW}
//  Eigen::Matrix3d C_ws;
//  C_ws = Eigen::AngleAxisd(shovelPitchAngle, Eigen::Vector3d::UnitY())  *   Eigen::AngleAxisd(shovelRollAngle, Eigen::Vector3d::UnitX()) *           // shovel roll
//                                       Eigen::AngleAxisd(shovelYawAngle, Eigen::Vector3d::UnitZ()) ;  // world to cabin yaw
//  // transform the rotation matrix to quaternion
//  Eigen::Quaterniond C_sw_quat(C_ws);
//  return C_sw_quat;
//}

void LocalPlanner::findDumpPoint(int zoneId) {
  ROS_INFO("[LocalPlanner]: findDumpPoint");
  geometry_msgs::TransformStamped T_mba;
  // get transform from base to cabin frame
  try {
    T_mba = tfBuffer_->lookupTransform("map", "BASE", ros::Time(0));
    // get also invese transform
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  // from geometric message to tf2 transform
  tf2::Transform T_mba_tf2 = tf2::Transform(tf2::Quaternion(T_mba.transform.rotation.x, T_mba.transform.rotation.y, T_mba.transform.rotation.z, T_mba.transform.rotation.w),
                                            tf2::Vector3(T_mba.transform.translation.x, T_mba.transform.translation.y, T_mba.transform.translation.z));
  tf2::Transform T_bam_tf2 = T_mba_tf2.inverse();

  Eigen::Quaterniond R_mba_q(T_mba.transform.rotation.w, T_mba.transform.rotation.x, T_mba.transform.rotation.y, T_mba.transform.rotation.z);
  // select a point to dump the soil contained in the shovel
  // the dump point belongs to the planning zone with id zoneId
  // the dump point should be one with least elevation and further in the back of the base frame
  double minSignedBaseDistance = 0; // dump area start here
  grid_map::PolygonIterator iterator(planningMap_, planningZones_.at(zoneId));
  for (iterator; !iterator.isPastEnd(); ++iterator) {
    ++iterator;
    grid_map::Index index(*iterator);
    double elevation = planningMap_.at("elevation", index);
    double originalElevation = planningMap_.at("original_elevation", index);
    // get position
    grid_map::Position positionZone;
    planningMap_.getPosition(*iterator, positionZone);
    // convert to 3d point for transformation
    tf2::Vector3 w_P_d(positionZone.x(), positionZone.y(), elevation);
    // transform to base frame
    tf2::Vector3 b_P_d = T_bam_tf2 * w_P_d;
    // get position in base frame, used for the optimzation
    // convert T_bam_Tf2 into transform stamped;
    // get the y coordinates
    double x_base = b_P_d.x();

    std::vector<grid_map::Position> w_shovelFilter_sv;
    // rotate filter to have same orientation as base frame
     // transform the vertices to the world frame using the current position as center of the shovel\
    std::vector<Eigen::Vector3d> s_shovelFilter_wv;
    for (int i = 0; i < shovelFilter_.size(); i++) {
      // transform to world frame
      Eigen::Vector3d s_shovelFilter_sv_3d =
          Eigen::Vector3d(shovelFilter_.at(i).x(), shovelFilter_.at(i).y(), 0);
      Eigen::Vector3d currentPosition_3d = Eigen::Vector3d(positionZone.x(), positionZone.y(), 0);
      Eigen::Vector3d s_shovelFilter_wv_i = R_mba_q * s_shovelFilter_sv_3d + currentPosition_3d;
      w_shovelFilter_sv.push_back(s_shovelFilter_wv_i.head(2));
    }

    // this is empty
    // create polygon from new vertices
    grid_map::Polygon polygon_w;
    for (int i = 0; i < w_shovelFilter_sv.size(); i++) {
      polygon_w.addVertex(w_shovelFilter_sv.at(i));
    }
    bool valid = true;
    // check that all vertices are inside the zone else skip this point
    for (int i = 0; i < w_shovelFilter_sv.size(); i++) {
      if (!planningZones_.at(zoneId).isInside(w_shovelFilter_sv.at(i))) {
        valid = false;
      }
    }
    // if not valid skip this point
    if (!valid) {
      continue;
    }
    // check the amount of volume inside the polygon
    double maxHeightDiff = 0;
    double filterVolume = 0;
    for (grid_map::PolygonIterator iterator(planningMap_, polygon_w); !iterator.isPastEnd(); ++iterator) {
      grid_map::Index index(*iterator);
      double elevation = planningMap_.at("elevation", index);
      double originalElevation = planningMap_.at("original_elevation", index);
      double heightDiff = elevation - originalElevation;
      if (heightDiff > maxHeightDiff) {
        maxHeightDiff = heightDiff;
      }
      filterVolume += heightDiff * planningMap_.getResolution() * planningMap_.getResolution();
    }
    // check if volume is below the threshold
    // if volume is below the threshold, then the dump point is the current position
    if (filterVolume < volumeDirtThreshold_ && maxHeightDiff < heightDirtThreshold_) {
      ROS_INFO("[LocalPlanner]: findDumpPoint: found dump point at %f, %f", positionZone.x(), positionZone.y());
      this->publishShovelFilter(w_shovelFilter_sv, "map");
      // pause for 0.3 seconds
      ros::Duration(0.3).sleep();
      if (x_base < minSignedBaseDistance) {
        minSignedBaseDistance = x_base;
        dumpPointIndex_ = index;
      }
    }
  }
  // raise a warning  if the dump point is not found
  if (iterator.isPastEnd() && minSignedBaseDistance == 0) {
    ROS_WARN("Dump point not found");
  }
}

Eigen::Vector3d LocalPlanner::getDumpPoint(){
  // return the position using the dumpPointIterator_
  grid_map::Position dumpPoint;
  planningMap_.getPosition(dumpPointIndex_, dumpPoint);
  // vertical offset is introduced to avoid having to introduce a more complex planner for the arm
  Eigen::Vector3d dumpPoint_3d(dumpPoint.x(), dumpPoint.y(), dumpAtHeight_ + planningMap_.at("elevation", dumpPointIndex_));
  return dumpPoint_3d;
}

void LocalPlanner::syncLayers(){
  excavationMappingPtr_->syncElevationLayer();
  unique_lock lock(mapMutex_);
  planningMap_["elevation"] = excavationMappingPtr_->gridMap_["elevation"];
  planningMap_["planning_elevation"] = planningMap_["elevation"];
  lock.unlock();
}

Eigen::Quaterniond LocalPlanner::get_R_sw(double shovelRollAngle, double shovelPitchAngle,
                                                                           double shovelYawAngle) {
//  ROS_INFO_STREAM("[LocalPlanner] yaw angle: " << shovelYawAngle);
  return loco_m545::RotationQuaternion(loco_m545::AngleAxis(shovelPitchAngle, 0, 1, 0) * loco_m545::AngleAxis(shovelRollAngle, 1, 0, 0) *
                                       loco_m545::AngleAxis(shovelYawAngle, 0, 0, 1)).toImplementation();
}

std::vector<Eigen::Vector2d> LocalPlanner::getDiggingPatchVertices(){
  // the digging patch is approximated by a semicircle with radius 0.5
  // that spans from the angle -pi/5 to 1/5 pi
  // the circle is sampled at 10 points
  std::vector<Eigen::Vector2d> vertices;

  for (int i = 0; i < 15; i++) {
    double angle = -M_PI / 5 + i * M_PI / 25;
    Eigen::Vector2d vertex(circularWorkspaceOuterRadius_ * cos(angle), circularWorkspaceOuterRadius_ * sin(angle));
    vertices.push_back(vertex);
  }
  // add vertices belonging to another arc with radius circularWorkspaceInnerRadius_
  // add them in the opposite order
  for (int i = 9; i >= 0; i--) {
    double angle = -M_PI / 5 + i * M_PI / 25;
    Eigen::Vector2d vertex(circularWorkspaceInnerRadius_ * cos(angle), circularWorkspaceInnerRadius_ * sin(angle));
    vertices.push_back(vertex);
  }
  return vertices;
}

std::vector<Eigen::Vector2d> LocalPlanner::getLeftFrontPatch() {
  // the dumping patch is approximated by a semicircle with radius dumpingOuterRadius_;
  // that spans from the angle pi 1/5 to 1/2 pi
  // the circle is sampled at 10 points
  std::vector<Eigen::Vector2d> vertices;
  for (int i = 0; i < 10; i++) {
    double angle = M_PI / 5 + i * M_PI / 25;
    Eigen::Vector2d vertex(dumpingZoneOuterRadius_ * cos(angle), dumpingZoneOuterRadius_ * sin(angle));
    vertices.push_back(vertex);
  }
  // add vertices belonging to another arc with radius dumpingInnerRadius_
  // add them in the opposite order
  for (int i = 9; i >= 0; i--) {
    double angle = M_PI / 5 + i * M_PI / 25;
    Eigen::Vector2d vertex(dumpingZoneInnerRadius_ * cos(angle), dumpingZoneInnerRadius_ * sin(angle));
    vertices.push_back(vertex);
  }
  return vertices;
}


std::vector<Eigen::Vector2d> LocalPlanner::getLeftCircularFrontSegmentPatch() {
  // the dumping patch is approximated by a circular segment with  radius dumpingOuterRadius_;
  // that spans from the angle 1/5 pi to 1/2 pi
  // the circle is sampled at 10 points
  double startAngle = M_PI / 5;
  std::vector<Eigen::Vector2d> vertices;
  for (int i = 0; i < 10; i++) {
    double angle = startAngle + i * M_PI / 25;
    Eigen::Vector2d vertex(dumpingZoneOuterRadius_ * cos(angle), dumpingZoneOuterRadius_ * sin(angle));
    vertices.push_back(vertex);
  }
  // define a vertical line segment
  Eigen::Vector2d vertex(dumpingZoneInnerRadius_ * cos(startAngle), dumpingZoneInnerRadius_ * sin(startAngle));
  Eigen::Vector2d vertex2(0, dumpingZoneInnerRadius_ * sin(startAngle));

  vertices.push_back(vertex2);
  vertices.push_back(vertex);
  return vertices;
}


std::vector<Eigen::Vector2d> LocalPlanner::getRightFrontPatch(){
  // the dumping patch is approximated by a semicircle with radius dumpingOuterRadius_;
  // that spans from the angle -1/5 pi to -1/2 pi
  // the circle is sampled at 10 points
  std::vector<Eigen::Vector2d> vertices;
  for (int i = 0; i < 10; i++) {
    double angle = -M_PI / 5 - i * M_PI / 25;
    Eigen::Vector2d vertex(dumpingZoneOuterRadius_ * cos(angle), dumpingZoneOuterRadius_ * sin(angle));
    vertices.push_back(vertex);
  }
  // add vertices belonging to another arc with radius dumpingInnerRadius_
  // add them in the opposite order
  for (int i = 9; i >= 0; i--) {
    double angle = -M_PI / 5 - i * M_PI / 25;
    Eigen::Vector2d vertex(dumpingZoneInnerRadius_ * cos(angle), dumpingZoneInnerRadius_ * sin(angle));
    vertices.push_back(vertex);
  }
  return vertices;
}

std::vector<Eigen::Vector2d> LocalPlanner::getRightCircularFrontSegmentPatch(){
  // the dumping patch is approximated by a circular segment with  radius dumpingOuterRadius_;
  // that spans from the angle -1/5 pi to -1/2 pi
  // the circle is sampled at 10 points
  double startAngle = -M_PI / 5;
  std::vector<Eigen::Vector2d> vertices;
  for (int i = 0; i < 10; i++) {
    double angle = startAngle - i * M_PI / 25;
    Eigen::Vector2d vertex(dumpingZoneOuterRadius_ * cos(angle), dumpingZoneOuterRadius_ * sin(angle));
    vertices.push_back(vertex);
  }
  // define a vertical line segment
  Eigen::Vector2d vertex(dumpingZoneInnerRadius_ * cos(startAngle), dumpingZoneInnerRadius_ * sin(startAngle));
  Eigen::Vector2d vertex2(0, dumpingZoneInnerRadius_ * sin(startAngle));

  vertices.push_back(vertex2);
  vertices.push_back(vertex);
  return vertices;
}


std::vector<Eigen::Vector2d> LocalPlanner::getLeftBackPatch(){
  // the dumping patch is approximated by a semicircle with radius dumpingOuterRadius_;
  // that spans from the angle 1/2 pi to 4/5 pi
  // the circle is sampled at 10 points
  std::vector<Eigen::Vector2d> vertices;
  for (int i = 0; i < 10; i++) {
    double angle = M_PI / 2 + i * M_PI / 25;
    Eigen::Vector2d vertex(dumpingZoneOuterRadius_ * cos(angle), dumpingZoneOuterRadius_ * sin(angle));
    vertices.push_back(vertex);
  }
  // add vertices belonging to another arc with radius dumpingInnerRadius_
  // add them in the opposite order
  for (int i = 9; i >= 0; i--) {
    double angle =  M_PI / 2 + i * M_PI / 25;
    Eigen::Vector2d vertex(dumpingZoneInnerRadius_ * cos(angle), dumpingZoneInnerRadius_ * sin(angle));
    vertices.push_back(vertex);
  }
  return vertices;
}


std::vector<Eigen::Vector2d> LocalPlanner::getLeftCircularBackSegmentPatch(){
  // the dumping patch is approximated by a circular segment with  radius dumpingOuterRadius_;
  // that spans from the angle 1/2 pi to 4/5 pi
  // the circle is sampled at 10 points
  double startAngle = M_PI / 2;
  std::vector<Eigen::Vector2d> vertices;
  for (int i = 0; i < 10; i++) {
    double angle = startAngle + i * M_PI / 25;
    Eigen::Vector2d vertex(dumpingZoneOuterRadius_ * cos(angle), dumpingZoneOuterRadius_ * sin(angle));
    vertices.push_back(vertex);
  }
  // define a vertical line segment
  double endAngle = 4./5 * M_PI;
  Eigen::Vector2d vertex(dumpingZoneInnerRadius_ * cos(endAngle), dumpingZoneInnerRadius_ * sin(endAngle));
  Eigen::Vector2d vertex2(0, dumpingZoneInnerRadius_ * sin(endAngle));

  vertices.push_back(vertex);
  vertices.push_back(vertex2);
  return vertices;
}


std::vector<Eigen::Vector2d> LocalPlanner::getRightBackPatch(){
  // the dumping patch is approximated by a semicircle with radius dumpingOuterRadius_;
  // that spans from the angle -1/2 pi to -4/5 pi
  // the circle is sampled at 10 points
  std::vector<Eigen::Vector2d> vertices;
  for (int i = 0; i < 10; i++) {
    double angle = M_PI * 4 / 5 - i * M_PI / 25;
    Eigen::Vector2d vertex(dumpingZoneOuterRadius_ * cos(angle), dumpingZoneOuterRadius_ * sin(angle));
    vertices.push_back(vertex);
  }
  // add vertices belonging to another arc with radius dumpingInnerRadius_
  // add them in the opposite order
  for (int i = 9; i >= 0; i--) {
    double angle = M_PI * 4 / 5 - i * M_PI / 25;
    Eigen::Vector2d vertex(dumpingZoneInnerRadius_ * cos(angle), dumpingZoneInnerRadius_ * sin(angle));
    vertices.push_back(vertex);
  }
  return vertices;
}

std::vector<Eigen::Vector2d> LocalPlanner::getRightCircularBackSegmentPatch(){
  // the dumping patch is approximated by a circular segment with  radius dumpingOuterRadius_;
  // that spans from the angle -1/2 pi to -4/5 pi
  // the circle is sampled at 10 points
  double startAngle = - 1./2 * M_PI;
  std::vector<Eigen::Vector2d> vertices;
  for (int i = 0; i < 10; i++) {
    double angle = startAngle - i * M_PI / 25;
    Eigen::Vector2d vertex(dumpingZoneOuterRadius_ * cos(angle), dumpingZoneOuterRadius_ * sin(angle));
    vertices.push_back(vertex);
  }
  // define a vertical line segment
  double endAngle = - 4./5 * M_PI;
  Eigen::Vector2d vertex(dumpingZoneInnerRadius_ * cos(endAngle), dumpingZoneInnerRadius_ * sin(endAngle));
  Eigen::Vector2d vertex2(0, dumpingZoneInnerRadius_ * sin(endAngle));

  vertices.push_back(vertex);
  vertices.push_back(vertex2);
  return vertices;
}


void LocalPlanner::addPlanningZonesToMap(std::vector<double> values){
  for (int i = 0; i < planningZones_.size(); i++) {
    grid_map::Polygon polygon = planningZones_.at(i);
    grid_map::PolygonIterator iterator(planningMap_, polygon);
    while (!iterator.isPastEnd()) {
      // get iterator index
      grid_map::Index index(*iterator);
      planningMap_.at("planning_zones", index) = values.at(i);
      ++iterator;
    }
  }
}


//
//std::vector<Eigen::Vector3d> LocalPlanner::getDigStraightTrajectory(Eigen::Vector3d& diggingPoint) {
//  std::vector<Eigen::Vector3d> trajectoryCabinFrame;
//  std::vector<Eigen::Vector3d> trajectoryBaseFrame;
//  // get the desired height from the local map
//  // get index of digging point
//  grid_map::Index index;
//  // local map is in base frame too
//  localMap_.getIndex(grid_map::Position(diggingPoint(0), diggingPoint(1)), index);
//  double currentHeight = localMap_.at("elevation", index);
//  // start dig position in base frame
//  Eigen::Vector3d b_P_s = diggingPoint;
//  ROS_INFO_STREAM("Digging Point coordinates in base frame: " << b_P_s.transpose());
//  double desiredHeight = localMap_.at("desired_elevation", index);
//
//  // transform the b_P_s from base to cabin frame
//  Eigen::Vector3d c_P_s;
//  geometry_msgs::TransformStamped transformStamped;
//  // get transform from base to cabin frame
//  try {
//    transformStamped = tfBuffer_->lookupTransform("BASE", "CABIN", ros::Time(0));
//  } catch (tf2::TransformException &ex) {
//    ROS_WARN("%s", ex.what());
//    ros::Duration(1.0).sleep();
//  }
//
//  // find the transform from SHOVEL TO CABIN frame
//  geometry_msgs::TransformStamped transformStampedShovelToCabin;
//  // get transform from base to cabin frame
//  try {
//    transformStampedShovelToCabin = tfBuffer_->lookupTransform("CABIN", "ENDEFFECTOR_CONTACT", ros::Time(0));
//  } catch (tf2::TransformException &ex) {
//    ROS_WARN("%s", ex.what());
//    ros::Duration(1.0).sleep();
//  }
//  // end effector origin in cabin frame
//  Eigen::Vector3d c_P_e = Eigen::Vector3d(transformStampedShovelToCabin.transform.translation.x,
//                                          transformStampedShovelToCabin.transform.translation.y,
//                                          transformStampedShovelToCabin.transform.translation.z);
//  // print out the end effector origin in cabin frame
//  ROS_INFO_STREAM("End effector origin in cabin frame: " << c_P_e.transpose());
//  tf2::doTransform(b_P_s, c_P_s, transformStamped);
////  tf2::doTransform(b_P_s, c_P_s, tfBuffer_->lookupTransform("BASE", "CABIN", ros::Time(0)));
//  // local attitude angle with the soil
//  // print transform
//  ROS_INFO_STREAM("[LocalPlanner] transformStamped.transform.translation.x: " << transformStamped.transform.translation.x <<
//  " transformStamped.transform.translation.y: " << transformStamped.transform.translation.y <<
//  " transformStamped.transform.translation.z: " << transformStamped.transform.translation.z <<
//  " transformStamped.transform.rotation.x: " << transformStamped.transform.rotation.x <<
//  " transformStamped.transform.rotation.y: " << transformStamped.transform.rotation.y <<
//  " transformStamped.transform.rotation.z: " << transformStamped.transform.rotation.z <<
//  " transformStamped.transform.rotation.w: " << transformStamped.transform.rotation.w);
//  // get the local attitude angle with the soil
//
//  ROS_INFO_STREAM("Digging Point coordinates in cabin frame: " << c_P_e.transpose());
//  double beta = 70 * M_PI / 180;
//  // angle of attack of the shovel
//  double alpha = 0;
//  // terrain slope along the cabin direction (in radians)
//  double gamma = 0;
//  // attitude angle wrt world frame
//  double theta = beta + gamma;
//  double heightDiff = desiredHeight - currentHeight;
//  // position dig at desired height in cabin frame
//  Eigen::Vector3d c_P_ds = c_P_e + Eigen::Vector3d(heightDiff/tan(beta), 0, heightDiff);
//  // horizontal length of the dig
//  double minDistanceShovelToCabin = 5;
//  // calculate dig length at which the shovel is full
//  // rought estimate for the moment to be done properly using height map
//  // approximate as trapezoid with height 1 and bases 0.5 and 1;
//  // volume ~ 0.5
//  double shovelCrossSectionArea = (1. * 1) / 2. * 1;
//  // assume revomed soil is a rectangle
//  double digLengthShovelFull = shovelCrossSectionArea / abs(heightDiff);
//  ROS_INFO_STREAM("Abs height difference between current and desired height: " << abs(heightDiff));
//  ROS_INFO_STREAM("Length to make shovel full at: " << digLengthShovelFull);
//  double distanceDigLengthShovelFull = c_P_ds(0) - digLengthShovelFull;
//  // pick most conservative option for now
//  double stopDigPositionX = std::max(minDistanceShovelToCabin, distanceDigLengthShovelFull);
//  // cabin frame is very convenient to formulate the trajectory
//  grid_map::Position c_P_endDig(stopDigPositionX, c_P_ds(1)); // cabin frame
//  // transform to cabin frame
//
//  Eigen::Vector3d cabin_endDigPosition = Eigen::Vector3d(c_P_endDig(0), c_P_endDig(1), desiredHeight);
//  // add an arc of 90 degrees to the trajectory starting from the current position, radius of the circle is the distance between
//  // the contact point and the joint of the shovel
//  double radius = sqrt(pow(c_P_s(0), 2) + pow(c_P_s(1), 2));
//  ROS_INFO_STREAM("Radius of the circle: " << radius);
//  double arcLength = M_PI / 2;
//  ROS_INFO_STREAM("Arc length: " << arcLength);
//  Eigen::Vector3d c_P_arc = c_P_s + Eigen::Vector3d(radius * cos(theta), 0, radius * sin(theta));
//  // add all points to the trajectory vector
//  trajectoryCabinFrame.push_back(c_P_e);
//  trajectoryCabinFrame.push_back(c_P_ds);
//  trajectoryCabinFrame.push_back(cabin_endDigPosition);
//  trajectoryCabinFrame.push_back(c_P_arc);
//  // print trajectory
//  ROS_INFO_STREAM("[LocalPlanner]: trajectory in cabin frame: " << trajectoryCabinFrame.size());
//  for (int i = 0; i < trajectoryCabinFrame.size(); i++) {
//    ROS_INFO_STREAM("[LocalPlanner]: trajectory point " << i << ": " << trajectoryCabinFrame[i].transpose());
//  }
//  this->publishMarkersTrajectory(trajectoryCabinFrame, "CABIN");
//  // transform the trajectory from cabin to base frame
//  for (auto& point : trajectoryCabinFrame) {
//    Eigen::Vector3d base_point;
//    tf2::doTransform(point, base_point, tfBuffer_->lookupTransform("CABIN", "BASE", ros::Time(0)));
//    trajectoryBaseFrame.push_back(base_point);
//  }
//  // print trajectory
//  ROS_INFO_STREAM("[LocalPlanner]: trajectory in base frame: " << trajectoryBaseFrame.size());
//  for (int i = 0; i < trajectoryBaseFrame.size(); i++) {
//    ROS_INFO_STREAM("[LocalPlanner]: trajectory point " << i << ": " << trajectoryBaseFrame[i].transpose());
//  }
//  return trajectoryBaseFrame;
//}


//std::vector<Eigen::Vector3d> LocalPlanner::digTrajectory(Eigen::Vector3d& base_digPosition) {
//    // create straight line from origin to start in base frame
//    Eigen::Vector3d start_pos(0, 0, 0);
//    Eigen::Vector3d end_pos(base_digPosition(0), base_digPosition(1), base_digPosition(2));
//    Eigen::Vector3d dir = end_pos - start_pos;
//    dir.normalize();
//    // create a vector of points along the line
//    std::vector<Eigen::Vector3d> trajectory;
//    for (int i = 0; i < 10; i++) {
//        trajectory.push_back(start_pos + dir * i);
//    }
//    // create line iterator for grid mpa that follow the trajectory
//    grid_map::LineIterator it(excavationMappingPtr_->gridMap_, start_pos, end_pos);
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

void LocalPlanner::publishShovelPoints(Eigen::Vector3d& shovelLeftContactPoint, Eigen::Vector3d& shovelRightContactPoint){
  // publish shovel points for visualization using markersArray
  visualization_msgs::MarkerArray markerArray;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "shovel_points";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = shovelRightContactPoint(0);
  marker.pose.position.y = shovelRightContactPoint(1);
  marker.pose.position.z = shovelRightContactPoint(2);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  // set marker scale and color
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;


  visualization_msgs::Marker marker2;
  marker2.header.frame_id = "map";
  marker2.header.stamp = ros::Time::now();
  marker2.ns = "shovel_points";
  marker2.id = 1;
  marker2.type = visualization_msgs::Marker::SPHERE;
  marker2.action = visualization_msgs::Marker::ADD;
  marker2.pose.position.x = shovelLeftContactPoint(0);
  marker2.pose.position.y = shovelLeftContactPoint(1);
  marker2.pose.position.z = shovelLeftContactPoint(2);
  marker2.pose.orientation.x = 0.0;
  marker2.pose.orientation.y = 0.0;
  marker2.pose.orientation.z = 0.0;
  marker2.pose.orientation.w = 1.0;
  // set marker scale and color
  marker2.scale.x = 0.1;
  marker2.scale.y = 0.1;
  marker2.scale.z = 0.1;
  marker2.color.a = 1.0;
  marker2.color.r = 0.0;
  marker2.color.g = 0.0;
  marker2.color.b = 1.0;

  markerArray.markers.push_back(marker);
  markerArray.markers.push_back(marker2);
  shovelPointsPublisher_.publish(markerArray);
}

void LocalPlanner::publishWorkspacePts(std::vector<Eigen::Vector2d> workspacePts, std::string frameId){
  // ros info stream the workspace points
  std::cout << "workspace points: " << std::endl;
  for (int i = 0; i < workspacePts.size(); i++) {
    std::cout << workspacePts[i].transpose() << std::endl;
  }

  // iterate over the points and publish them as spheres
  visualization_msgs::MarkerArray markerArray;
  for (int i = 0; i < workspacePts.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frameId;
    marker.header.stamp = ros::Time::now();
    marker.ns = "workspace_points";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = workspacePts[i](0);
    marker.pose.position.y = workspacePts[i](1);
    marker.pose.position.z = 0.0;
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
    markerArray.markers.push_back(marker);
  }
  // wait until you have one subscriber
//  while (workspacePtsPublisher_.getNumSubscribers() < 1) {
//    ROS_WARN_ONCE("No subscribers to workspace points yet");
//    ros::Duration(0.5).sleep();
//  }
  workspacePtsPublisher_.publish(markerArray);
  ROS_INFO("[LocalPlanner]: Published workspace points");
}


void LocalPlanner::publishShovelFilter(std::vector<Eigen::Vector2d> workspacePts, std::string frameId){
  // ros info stream the workspace points
  std::cout << "workspace points: " << std::endl;
  for (int i = 0; i < workspacePts.size(); i++) {
    std::cout << workspacePts[i].transpose() << std::endl;
  }

  // iterate over the points and publish them as spheres
  visualization_msgs::MarkerArray markerArray;
  for (int i = 0; i < workspacePts.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frameId;
    marker.header.stamp = ros::Time::now();
    marker.ns = "workspace_points";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = workspacePts[i](0);
    marker.pose.position.y = workspacePts[i](1);
    marker.pose.position.z = 0.0;
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
    markerArray.markers.push_back(marker);
  }
  // wait until you have one subscriber
  //  while (workspacePtsPublisher_.getNumSubscribers() < 1) {
  //    ROS_WARN_ONCE("No subscribers to workspace points yet");
  //    ros::Duration(0.5).sleep();
  //  }
  shovelFilterPublisher_.publish(markerArray);
  ROS_INFO("[LocalPlanner]: Published shovel filter points");
}

void LocalPlanner::publishMarkersTrajectory(std::vector<grid_map::Position3> positions, std::string frameId) const{
  visualization_msgs::MarkerArray markerArray;
  for (int i = 0; i < positions.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frameId;
    marker.header.stamp = ros::Time::now();
    marker.ns = "local_excavation";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = positions[i].x();
    marker.pose.position.y = positions[i].y();
    marker.pose.position.z = positions[i].z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    markerArray.markers.push_back(marker);
  }
  markersTrajectoryPublisher_.publish(markerArray);
}

void LocalPlanner::publishVector(Eigen::Vector3d position, Eigen::Vector3d direction, std::string frameId) const{
  // publish an arrow located at position and with the direction of direction
//  ROS_INFO_STREAM("publishing vector");
  visualization_msgs::Marker marker;
  marker.header.frame_id = frameId;
  marker.header.stamp = ros::Time::now();
  marker.ns = "local_excavation";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  // convert eigen vector3d into geometry_msgs
  geometry_msgs::Point position_msg;
  position_msg.x = position(0);
  position_msg.y = position(1);
  position_msg.z = position(2);
  marker.points.push_back(position_msg);
  // convert eigen vector3d into geometry_msgs
  geometry_msgs::Point direction_msg;
  direction_msg.x = position(0) + direction(0);
  direction_msg.y = position(1) + direction(1);
  direction_msg.z = position(2) + direction(2);
  marker.points.push_back(direction_msg);
  marker.scale.x = 0.05;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  penetrationDirectionPublisher_.publish(marker);
}

void LocalPlanner::publishHeading(Eigen::Vector3d position, Eigen::Vector3d direction, std::string frameId) const{
  // publish an arrow located at position and with the direction of direction
//  ROS_INFO_STREAM("publishing heading");
  visualization_msgs::Marker marker;
  marker.header.frame_id = frameId;
  marker.header.stamp = ros::Time::now();
  marker.ns = "local_excavation";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  // convert eigen vector3d into geometry_msgs
  geometry_msgs::Point position_msg;
  position_msg.x = position(0);
  position_msg.y = position(1);
  position_msg.z = position(2);
  marker.points.push_back(position_msg);
  // convert eigen vector3d into geometry_msgs
  geometry_msgs::Point direction_msg;
  direction_msg.x = position(0) + direction(0);
  direction_msg.y = position(1) + direction(1);
  direction_msg.z = position(2) + direction(2);
  marker.points.push_back(direction_msg);
  marker.scale.x = 0.05;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  headingPublisher_.publish(marker);
}


void LocalPlanner::publishNormal(Eigen::Vector3d position, Eigen::Vector3d direction, std::string frameId) const{
  // publish an arrow located at position and with the direction of direction
  ROS_INFO_STREAM("publishing normal");
  visualization_msgs::Marker marker;
  marker.header.frame_id = frameId;
  marker.header.stamp = ros::Time::now();
  marker.ns = "local_excavation";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  // convert eigen vector3d into geometry_msgs
  geometry_msgs::Point position_msg;
  position_msg.x = position(0);
  position_msg.y = position(1);
  position_msg.z = position(2);
  marker.points.push_back(position_msg);
  // convert eigen vector3d into geometry_msgs
  geometry_msgs::Point direction_msg;
  direction_msg.x = position(0) + direction(0);
  direction_msg.y = position(1) + direction(1);
  direction_msg.z = position(2) + direction(2);
  marker.points.push_back(direction_msg);
  marker.scale.x = 0.05;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  normalPublisher_.publish(marker);
}

void LocalPlanner::publishVectors(std::vector<Eigen::Vector3d> positions, std::vector<Eigen::Vector3d> directions, std::string frameId) const{
  // publish an arrow located at position and with the direction of direction
  ROS_INFO_STREAM("publishing vectors");
  visualization_msgs::MarkerArray markerArray;
  for (int i = 0; i < positions.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frameId;
    marker.header.stamp = ros::Time::now();
    marker.ns = "local_excavation";
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    // convert eigen vector3d into geometry_msgs
    geometry_msgs::Point position_msg;
    position_msg.x = positions[i](0);
    position_msg.y = positions[i](1);
    position_msg.z = positions[i](2);
    marker.points.push_back(position_msg);
    // convert eigen vector3d into geometry_msgs
    geometry_msgs::Point direction_msg;
    direction_msg.x = positions[i](0) + directions[i](0);
    direction_msg.y = positions[i](1) + directions[i](1);
    direction_msg.z = positions[i](2) + directions[i](2);
    marker.points.push_back(direction_msg);
    marker.scale.x = 0.05;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  }
  vectorsPublisher_.publish(markerArray);
}

void LocalPlanner::publishDesiredShovelOrientation(Eigen::Vector3d position, Eigen::Vector3d direction, std::string frameId) const{
  // publish an arrow located at position and with the direction of direction
  ROS_INFO_STREAM("publishing desired shovel orientation");
  visualization_msgs::Marker marker;
  marker.header.frame_id = frameId;
  marker.header.stamp = ros::Time::now();
  marker.ns = "local_excavation";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  // convert eigen vector3d into geometry_msgs
  geometry_msgs::Point position_msg;
  position_msg.x = position(0);
  position_msg.y = position(1);
  position_msg.z = position(2);
  marker.points.push_back(position_msg);
  // convert eigen vector3d into geometry_msgs
  geometry_msgs::Point direction_msg;
  direction_msg.x = position(0) + direction(0);
  direction_msg.y = position(1) + direction(1);
  direction_msg.z = position(2) + direction(2);
  marker.points.push_back(direction_msg);
  marker.scale.x = 0.05;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  desiredShovelOrientationPublisher_.publish(marker);
}

void LocalPlanner::publishProjectedVector(Eigen::Vector3d position, Eigen::Vector3d direction, std::string frameId) const{
  // publish an arrow located at position and with the direction of direction
  ROS_INFO_STREAM("publishing projected vector");
  visualization_msgs::Marker marker;
  marker.header.frame_id = frameId;
  marker.header.stamp = ros::Time::now();
  marker.ns = "local_excavation";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  // convert eigen vector3d into geometry_msgs
  geometry_msgs::Point position_msg;
  position_msg.x = position(0);
  position_msg.y = position(1);
  position_msg.z = position(2);
  marker.points.push_back(position_msg);
  // convert eigen vector3d into geometry_msgs
  geometry_msgs::Point direction_msg;
  direction_msg.x = position(0) + direction(0);
  direction_msg.y = position(1) + direction(1);
  direction_msg.z = position(2) + direction(2);
  marker.points.push_back(direction_msg);
  marker.scale.x = 0.05;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  projectedVectorPublisher_.publish(marker);
}

void LocalPlanner::publishJointVector(Eigen::Vector3d position, Eigen::Vector3d direction, std::string frameId) const{
  // publish an arrow located at position and with the direction of direction
  ROS_INFO_STREAM("publishing joint vector");
  visualization_msgs::Marker marker;
  marker.header.frame_id = frameId;
  marker.header.stamp = ros::Time::now();
  marker.ns = "local_excavation";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  // convert eigen vector3d into geometry_msgs
  geometry_msgs::Point position_msg;
  position_msg.x = position(0);
  position_msg.y = position(1);
  position_msg.z = position(2);
  marker.points.push_back(position_msg);
  // convert eigen vector3d into geometry_msgs
  geometry_msgs::Point direction_msg;
  direction_msg.x = position(0) + direction(0);
  direction_msg.y = position(1) + direction(1);
  direction_msg.z = position(2) + direction(2);
  marker.points.push_back(direction_msg);
  marker.scale.x = 0.05;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  jointVectorPublisher_.publish(marker);
}

void LocalPlanner::publishTrajectoryPoses(std::vector<Eigen::Vector3d> positions, std::vector<Eigen::Quaterniond> orientations) const{
  visualization_msgs::MarkerArray markerArray;
  // publish one arrow per pose
  for (int i = 0; i < positions.size(); i++) {
    Eigen::Quaterniond orientation = orientations[i].inverse();
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "local_excavation";
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    // set the market pose
    marker.pose.position.x = positions[i](0);
    marker.pose.position.y = positions[i](1);
    marker.pose.position.z = positions[i](2);
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();
    // set the arrow scale
    marker.scale.x = 0.5;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    // set the arrow color
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    markerArray.markers.push_back(marker);
    // add to marker array
  }
  trajectoryPosesPublisher_.publish(markerArray);
}


void LocalPlanner::publishDesiredShovelPose(Eigen::Vector3d position, Eigen::Quaterniond orientation) const{
  // publish pose to rviz
  Eigen::Quaterniond orientation_inverse = orientation.inverse();
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = position(0);
  pose.pose.position.y = position(1);
  pose.pose.position.z = position(2);
  pose.pose.orientation.x = orientation_inverse.x();
  pose.pose.orientation.y = orientation_inverse.y();
  pose.pose.orientation.z = orientation_inverse.z();
  pose.pose.orientation.w = orientation_inverse.w();
  desiredPosePublisher_.publish(pose);
}



} // namespace local_excavation