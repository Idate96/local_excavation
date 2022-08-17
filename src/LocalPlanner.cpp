//
// Created by lorenzo on 28.10.21.
//
#include "local_excavation/LocalPlanner.hpp"
#include <tf2_eigen/tf2_eigen.h>
#include "geometry_msgs/Point.h"
#include "tf2/LinearMath/Transform.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "grid_map_sdf/SignedDistance2d.hpp"


// include geometry_msgs
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <m545_description/m545_description.hpp>
#include "grid_map_sdf/SignedDistanceField.hpp"
#include "local_excavation/planning_utils.h"

namespace local_excavation {

  LocalPlanner::LocalPlanner(std::unique_ptr<excavation_mapping::ExcavationMapping> excavationMapping)
      : excavationMappingPtr_(std::move(excavationMapping)), model_(0.01) {
    // make a copy of the excavator model (not a reference)
    // initialize the buffer and the listener
    // load the parameters
    this->loadParameters();
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>();
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    // intialize the markerPublisher
    markerPublisher_ = nh_.advertise<visualization_msgs::Marker>("/local_excavation/marker", 1);
    markersTrajectoryPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/local_excavation/markers_trajectory", 1);
    penetrationDirectionPublisher_ = nh_.advertise<visualization_msgs::Marker>(
        "/local_excavation/penetration_direction", 1, true);
    normalPublisher_ = nh_.advertise<visualization_msgs::Marker>("/local_excavation/normal", 1, true);
    projectedVectorPublisher_ = nh_.advertise<visualization_msgs::Marker>("/local_excavation/projected_vector", 1,
                                                                          true);
    diggingDirectionPublisher_ = nh_.advertise<visualization_msgs::Marker>("/local_excavation/digging_direction", 1,
                                                                           true);
    jointVectorPublisher_ = nh_.advertise<visualization_msgs::Marker>("/local_excavation/joint_vector", 1, true);
    desiredShovelOrientationPublisher_ = nh_.advertise<visualization_msgs::Marker>(
        "/local_excavation/desired_shovel_orientation", 1, true);
    planningMapPublisher_ = nh_.advertise<grid_map_msgs::GridMap>("/local_excavation/planning_map", 1, true);
    desiredPosePublisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/local_excavation/se3_pose", 1, true);
    vectorsPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/local_excavation/vectors", 1, true);
    trajectoryPosesPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/local_excavation/trajectory_poses",
                                                                               1, true);
    shovelPointsPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/local_excavation/shovel_points", 1,
                                                                            true);
    workspacePtsPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/local_excavation/workspace_points", 1,
                                                                            true);
    headingPublisher_ = nh_.advertise<visualization_msgs::Marker>("/local_excavation/heading", 1, true);
    polygonPublisher_ = nh_.advertise<geometry_msgs::PolygonStamped>("/local_excavation/polygon", 1, true);
    shovelFilterPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/local_excavation/shovel_filter", 1,
                                                                            true);
    workingAreaPublisher_ = nh_.advertise<grid_map_msgs::GridMap>("/local_excavation/traversable_area", 1, true);
    // initialize the service
    digAndDumpService_ = nh_.advertiseService("/local_excavation/dig_and_dump",
                                              &LocalPlanner::digAndDumpServiceCallback, this);
    // subs
    footprintSubscriber_ = nh_.subscribe(footprintTopic_, 1, &LocalPlanner::footprintCallback, this);
    globalPathSub_ = nh_.subscribe(pathTopic_, 1, &LocalPlanner::globalPathCallback, this);

    // model for collisions checks
    std::string urdfDescription;
    nh_.getParam("/romo_mm_description", urdfDescription);
    model_.initModelFromUrdf(urdfDescription);
    //  excavator_model::ExcavatorState excavatorState = model_.getState();
    //  int numDofBoom_ = 7;
    //  // get the last numDofBoom joints of the model
    //  //  Eigen::VectorXd boomState = excavatorState.getJointPositions().toImplementation().tail(numDofBoom_);
    //  // desired end effector pose in base frame
    //  Eigen::VectorXd boomState = Eigen::VectorXd::Zero(numDofBoom_);
    //  kindr::HomTransformQuatD desiredEndEffectorPose_(kindr::Position3D(7, 0, 0), kindr::RotationQuaternionD(0, 0, 0, 1));
    //  ROS_INFO("[LocalPlanner]: number of dof boom " << excavator_model::RD::getNumDofLimb(m545_description::enums::LimbEnum::BOOM));
    //  model_.getLimbJointPositionsFromPoseEndEffectorToBaseIteratively(boomState, desiredEndEffectorPose_,
    //                                                                   m545_description::enums::LimbEnum::BOOM);
    //  // print boom state vector
    //  ROS_INFO("[LocalPlanner]: boom state vector " << boomState);
    // Set up collision checks
    const romo::RigidBodyShPtrContainer<excavator_model::ConcreteDescription> &ptr_container = model_.getBodyContainer();
    using CollisionBodyRomo = romo::CollisionBodyRomo<excavator_model::ConcreteDescription>;

    //  ptr_container[loco_m545::BodyEnum::ENDEFFECTOR]->collision_geometry.get()->getType()
    // create a new box collision geometry
    Eigen::Vector3d size = Eigen::Vector3d(0.75, 1.5, 0.75);
    // ec = end effector contact, g = geometry
    kindr::Position3D ec_P_ecg(-0.325, 0, 0.325);
    kindr::RotationMatrixD C_gec(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    auto shovelGeo = std::make_shared<collisions_geometry::CollisionGeometryBox>(size(0), size(1), size(2),
                                                                                 ec_P_ecg, C_gec);
    // create a body from using the shovelBoxGeo
    // for the moment dummy values
    kindr::Position3D w_P_wec = kindr::Position3D(0., 0., 0.);
    kindr::RotationMatrixD P_ecw = kindr::RotationMatrixD(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    shovelBodyPtr_ = std::make_shared<collisions::CollisionBodySimple>(shovelGeo, w_P_wec, P_ecw, 124);
    armCollisionGroup_.addCollisionBody(shovelBodyPtr_);

    legCollisionGroup_.clear();
    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::LF_BEAM]));
    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::LF_BEARING]));
    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::LF_KNUCKLE]));
    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::LF_PARALLEL]));
    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::LF_SWIVEL]));
    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::LF_WHEEL]));

    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::RF_BEAM]));
    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::RF_BEARING]));
    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::RF_KNUCKLE]));
    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::RF_PARALLEL]));
    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::RF_SWIVEL]));
    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::RF_WHEEL]));

    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::LH_BEAM]));
    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::LH_OUTRIGGER]));
    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::LH_KNUCKLE]));
    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::LH_ROTATOR]));
    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::LH_WHEEL]));

    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::RH_BEAM]));
    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::RH_OUTRIGGER]));
    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::RH_KNUCKLE]));
    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::RH_ROTATOR]));
    legCollisionGroup_.addCollisionBody(
        std::make_shared<CollisionBodyRomo>(ptr_container[loco_m545::BodyEnum::RH_WHEEL]));


  };

  bool LocalPlanner::loadParameters() {
    bool loaded = nh_.param<double>("/local_excavation/height_precision", heightPrecision_, 0.1) &&
                  nh_.param<double>("/local_excavation/inner_digging_radius", circularWorkspaceInnerRadius_, 6) &&
                  nh_.param<double>("/local_excavation/outer_digging_radius", circularWorkspaceOuterRadius_, 8) &&
                  nh_.param<double>("/local_excavation/inner_dumping_radius", dumpingZoneInnerRadius_, 4.5) &&
                  nh_.param<double>("/local_excavation/outer_dumping_radius", dumpingZoneOuterRadius_, 8) &&
                  nh_.param<double>("/local_excavation/min_distance_shovel_to_base", minDistanceShovelToBase_,
                                    3.5) &&
                  nh_.param<double>("/local_excavation/workspace_angle", circularWorkspaceAngle_, M_PI / 2) &&
                  nh_.param<double>("/local_excavation/max_volume", maxVolume_, 0.5) &&
                  nh_.param<double>("/local_excavation/volume_weight", volumeWeight_, 1) &&
                  nh_.param<double>("/local_excavation/distance_weight", distanceWeight_, 0.005) &&
                  nh_.param<double>("/local_excavation/heading_weight", headingWeight_, 0.05) &&
                  nh_.param<double>("/local_excavation/dump_height", dumpAtHeight_, 0.9) &&
                  nh_.param<double>("/local_excavation/volume_dirt_threshold", volumeDirtThreshold_, 0.1) &&
                  nh_.param<double>("/local_excavation/height_dirt_threshold", heightDirtThreshold_, 0.5) &&
                  nh_.param<double>("/local_excavation/volume_threshold", volumeThreshold_, 0.5) &&
                  nh_.param<double>("/local_excavation/height_threshold", heightThreshold_, 0.5) &&
                  nh_.param<double>("/local_excavation/inactive_area_ratio", inactiveAreaRatio_, .5) &&
                  nh_.param<double>("/local_excavation/dumping_distance_weight", dumpingZoneDistanceWeight_, .5) &&
                  nh_.param<double>("/local_excavation/dig_dump_distance_weight", digDumpDistanceWeight_, 1.) &&
                  nh_.param<double>("/local_excavation/working_dir_weight", workingDirWeight_, 1.) &&
                  nh_.param<double>("/local_excavation/excavation_area_ratio", excavationAreaRatio_, .1) &&
                  nh_.param<double>("/local_excavation/x_dump_weight", xDumpWeight_, -1.) &&
                  nh_.param<double>("/local_excavation/y_dump_weight", yDumpWeight_, 1.) &&
                  nh_.param<double>("/local_excavation/dump_cells_weight", dumpCellsWeight_, 0.1) &&
                  nh_.param<double>("/local_excavation/height_traversable_threshold", heightTraversableThreshold_,
                                    0.5) &&
                  nh_.param<double>("/local_excavation/bonus_volume", shovelVolumeBonus_, 0.05) &&
                  nh_.param<double>("/local_excavation/missing_cells_threshold", missingCellsThreshold_, 0.04) &&
                  nh_.param<double>("/local_excavation/height_dig_area_threshold", heightDigAreaThreshold_, 0.05) &&
                  nh_.param<double>("/local_excavation/volume_dirt_weight", volumeDirtWeight_, -0.5) &&
                  nh_.param<double>("/local_excavation/height_dump_threshold", heightDumpThreshold_, 1.1) &&
                  nh_.param<double>("/local_excavation/radial_offset", radialOffset_, 0.3) &&
                  nh_.param<double>("/local_excavation/vertical_offset", verticalOffset_, 0.3) &&
                  nh_.param<double>("/local_excavation/max_dig_depth", maxDigDepth_, 0.3) &&
                  nh_.param<double>("/local_excavation/max_dig_dirt_depth", maxDigDirtDepth_, 0.6) &&
                  nh_.param<double>("/local_excavation/dragging_distance", draggingDistance_, 1.5) &&
                  nh_.param<double>("/local_excavation/dragging_angle", draggingAngle_, 0.5) &&
                  nh_.param<double>("/local_excavation/closing_z_translation", closingZTranslation_, 0.7) &&
                  nh_.param<double>("/local_excavation/min_distance_collision", minDistanceCollision_, 1.0) &&
                  nh_.param<double>("/local_excavation/target_dig_attitude", targetDigAttitude_, 1.73) &&
                  nh_.param<double>("/local_excavation/target_dig_dirt_attitude", targetDigDirtAttitude_, 0.3) &&
                  nh_.param<double>("/local_excavation/radial_dirt_offset", radialDirtOffset_, 0.3) &&
                  nh_.param<double>("/local_excavation/dragging_dirt_distance", draggingDirtDistance_, 0.3) &&
                  nh_.param<std::string>("/m545_planner_node/footprint_topic", footprintTopic_, "/footprint") &&
                  nh_.param<std::string>("path_topic", pathTopic_, "/coverage/poses") &&
                  nh_.param<double>("/local_excavation/target_height_diff_threshold", targetHeightDiffThreshold_,
                                    0.3) &&
                  nh_.param<double>("/local_excavation/min_scoop_volume", minScoopVolume_, 0.1) &&
                  nh_.param<int>("/local_excavation/low_volume_scoop_attempts", lowVolumeScoopAttempts_, 3);
    nh_.param<std::string>("/local_excavation/save_map_path", saveMapPath_,
                           ros::package::getPath("local_excavation") + "/maps/latest.bag");
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

  void LocalPlanner::globalPathCallback(const geometry_msgs::PoseArray& msg) {
    std::vector<geometry_msgs::Pose> path;
    for (size_t i = 0; i < msg.poses.size(); i++) {
      geometry_msgs::Pose pose;
      pose.position.x = msg.poses[i].position.x;
      pose.position.y = msg.poses[i].position.y;
      pose.position.z = msg.poses[i].position.z;
      pose.orientation.x = msg.poses[i].orientation.x;
      pose.orientation.y = msg.poses[i].orientation.y;
      pose.orientation.z = msg.poses[i].orientation.z;
      pose.orientation.w = msg.poses[i].orientation.w;
      path.push_back(pose);
    }
//    ROS_INFO_STREAM("[LocalPlanner]: Received global path with " << path.size() << " poses");
    globalPath_ = path;
  }

  void LocalPlanner::footprintCallback(const m545_planner_msgs::M545FootprintRos& msg){
    unique_lock writelock(footprintMutex_);
    // extract the data
    for (int i = 0; i < msg.wheelPoses.size(); i++) {
      footprint_(i, 0) = msg.wheelPoses[i].position.x;
      footprint_(i, 1) = msg.wheelPoses[i].position.y;
    }
    writelock.unlock();
  }

  void LocalPlanner::setExcavationMaskAtFutureStates() {
    // reset current excavation mask
    ROS_INFO_STREAM("[LocalPlanner]: Reset excavation mask");
    planningMap_["current_excavation_mask"] = planningMap_["excavation_mask"];
    // print current index
    ROS_INFO_STREAM("[LocalPlanner]: Current index: " << waypointIndex_);
    if (waypointIndex_ + 1 >= globalPath_.size()) {
      return;
    }
    // print footprint (eigen::vector2d)
    ROS_INFO_STREAM("[LocalPlanner]: Footprint: " << footprint_);
    for (auto waypointIdx = waypointIndex_ + 1; waypointIdx < globalPath_.size(); waypointIdx++) {
      ROS_INFO_STREAM("[LocalPlanner]: Set excavation mask at waypoint " << waypointIdx);
      // extract x, y and yaw from the pose
      geometry_msgs::Pose nextWaypoint;
      try {
        nextWaypoint = globalPath_.at(waypointIdx);
      } catch (const std::out_of_range& oor) {
        ROS_ERROR_STREAM("[LocalPlanner]: Out of range exception: " << oor.what());
        return;
      }
      double roll_b, pitch_b, yaw_b;
      tf2::Quaternion R_mba_q =
          tf2::Quaternion(nextWaypoint.orientation.x, nextWaypoint.orientation.y, nextWaypoint.orientation.z,
                          nextWaypoint.orientation.w);
      tf2::Matrix3x3(R_mba_q).getRPY(roll_b, pitch_b, yaw_b);
      se2_planning::SE2state state(nextWaypoint.position.x, nextWaypoint.position.y,
                                   yaw_b);
      // get the footprint expressed in the world frame
      // the inflation factor is useful to prevent the dirt from accumulating close to the tracks
      double inflationFactor = 1.2;
      auto w_footprint = footprintAtState(state, footprint_, inflationFactor);
      // make a gridmap poligon iterator and mark the points inside the footprint in the "current_excavation_mask" with value 0
      grid_map::Polygon polygon;
      for (int i = 0; i < 4; i++) {
        polygon.addVertex(w_footprint.row(i));
      }
      // set the value of the mask to 0 unless it's -1 in the excavation_mask layer
      for (auto polygonIterator = grid_map::PolygonIterator(planningMap_, polygon);
           !polygonIterator.isPastEnd(); ++polygonIterator) {
        if (planningMap_.at("current_excavation_mask", *polygonIterator) != -1) {
          planningMap_.at("current_excavation_mask", *polygonIterator) = 0;
        }
      }
    }
  }

  bool LocalPlanner::digAndDumpServiceCallback(DigDumpSrv::Request &request, DigDumpSrv::Response &response) {
    ROS_INFO_STREAM("[LocalPlanner]: Dig and dump service called with dump zone " << request.dump_zone << " and "
                                                                                  << request.dig_zone);
    this->setDigZone(request.dig_zone);
    this->setDumpZone(request.dump_zone);
    // if both are -1 set the planner mode to automatic
    if (request.dig_zone == -1 && request.dump_zone == -1) {
      autoZoneSelection_ = true;
    } else {
      autoZoneSelection_ = false;
    }
    return true;
  }

  bool LocalPlanner::initialize(std::string designMapBag) {
    excavationMappingPtr_->initialize(designMapBag);
    ROS_INFO_STREAM("[LocalPlanner] loaded map into excavation mapping");
    //  excavationMappingPtr_->gridMap_.add("planning_zones", 0);
    //  excavationMappingPtr_->gridMap_.add("cost", 0);
    // start a new thread to create the planning zones and join it to the main thread
    unique_lock lock(mapMutex_);
    // print all available layers in the map
    planningMap_.setFrameId(excavationMappingPtr_->gridMap_.getFrameId());
    planningMap_.setGeometry(excavationMappingPtr_->gridMap_.getLength(),
                             excavationMappingPtr_->gridMap_.getResolution(),
                             excavationMappingPtr_->gridMap_.getPosition());
    planningMap_.setTimestamp(excavationMappingPtr_->gridMap_.getTimestamp());
    // copy over the existing layers
    planningMap_ = excavationMappingPtr_->gridMap_;
    // add the following layers if they don't exit: "planning elevation", "dumping_distance", "planning_zones", "dug_area", "working_area"
    if (!planningMap_.exists("planning_zones")) {
      planningMap_.add("planning_zones");
    }
    if (!planningMap_.exists("dumping_distance")) {
      planningMap_.add("dumping_distance", 0);
    }
    if (!planningMap_.exists("planning_elevation")) {
      planningMap_.add("planning_elevation");
      planningMap_["planning_elevation"] = excavationMappingPtr_->gridMap_["elevation"];
    }
    if (!planningMap_.exists("dug_area")) {
      planningMap_.add("dug_area", 0);
    }
    if (!planningMap_.exists("working_area")) {
      if (planningMap_.exists("occupancy")) {
        planningMap_.add("working_area", 0);
        planningMap_["working_area"] = planningMap_["occupancy"];
      } else {
        planningMap_.add("working_area", 0);
      }
    }
    if (!planningMap_.exists("predig_elevation")) {
      planningMap_.add("predig_elevation", 0);
      planningMap_["predig_elevation"] = excavationMappingPtr_->gridMap_["original_elevation"];
    }
    
    if (!planningMap_.exists("current_excavation_mask")) {
      planningMap_.add("current_excavation_mask", 0);
      planningMap_["current_excavation_mask"] = excavationMappingPtr_->gridMap_["excavation_mask"];
    }

    //  planningMap_.add("working_area", 0);
    // if occupancy layer is available initialize the working area equal to it
    // the working area layer is used by the path planner to determine the area that is free to move into
    //  if (excavationMappingPtr_->gridMap_.exists("occupancy")) {
    //    ROS_INFO_STREAM("[LocalPlanner]: Initializing working area from occupancy layer");
    //    planningMap_["working_area"] = excavationMappingPtr_->gridMap_["occupancy"];
    //  }
    //  if (excavationMappingPtr_->gridMap_.exists("current_excavation_mask")) {
    //    planningMap_["current_excavation_mask"] = excavationMappingPtr_->gridMap_["current_excavation_mask"];
    //  }
    // mostly for debugging
    planningMap_.add("active_dig_zones", 0);
    planningMap_.add("active_dump_zones", 0);
    planningMap_.add("cost", 0);
    //  planningMap_["elevation"] = excavationMappingPtr_->gridMap_["elevation"];
    //  planningMap_["original_elevation"] = excavationMappingPtr_->gridMap_["original_elevation"];
    //  planningMap_["desired_elevation"] = excavationMappingPtr_->gridMap_["desired_elevation"];
    //  planningMap_["planning_elevation"] = excavationMappingPtr_->gridMap_["elevation"];

    lock.unlock();
    ROS_INFO_STREAM("[LocalPlanner]: Initialized planning map");
    this->createPlanningZones();
    createNewZones_ = true;  // initial zones are based on current pase position
    //  this->updateDugZones();
    //  this->choosePlanningZones();
    this->createShovelFilter();
    //    std::thread planningThread_ = std::thread(&LocalPlanner::createPlanningZones, this);
    // detach the thread
    //  planningThread_.detach();
    this->updateWorkingArea();
    //  this->choosePlanningZones();
    this->updateDugZones();
    //  this->updatePlanningMap();
    return true;
  };

  void LocalPlanner::reset() {
    this->createPlanningZones();
    this->updateWorkingArea();
    this->updateDugZones();
    // reset the dig and dump zones
    digZoneId_ = -1;
    previousDigZoneId_ = -1;
    dumpZoneId_ = -1;
    previousDumpZoneId_ = -1;
    workspaceVolume_ = 0;
  }

  void LocalPlanner::updateRobotState(excavator_model::ExcavatorState &excavatorState) {
    // listen to the m545_state message and copy the state into the excavator model
    model_.setState(excavatorState, true, false, false);
  }

  double LocalPlanner::computeWorkspaceVolume(int zoneId, std::string targetLayer) {
    // compute the volume of the workspace of the zone
    // by subtracking the elevation layer from the target layer in the area of the zone
    // obtain the volume by summing the difference and multiplying by the resolution^2
    if (zoneId == -1) {
      return 0;
    }
    double workspaceVolume = 0;
    ROS_INFO_STREAM("[LocalPlanner]: Computing workspace volume for zone " << zoneId);
    // iterate over the zone cells
    grid_map::Polygon zonePolygon = planningZones_.at(zoneId);
    for (grid_map::PolygonIterator iterator(planningMap_, zonePolygon); !iterator.isPastEnd(); ++iterator) {
      // get the cell coordinates
      grid_map::Index index(*iterator);
      // get the elevation value of the cell
      double elevation = excavationMappingPtr_->gridMap_.at("elevation", index);
      // get the target value of the cell
      double target = excavationMappingPtr_->gridMap_.at(targetLayer, index);
      // subtract the elevation from the target value min 0
      double difference = std::max(0.0, elevation - target);
      // if difference is nan skip
      if (std::isnan(difference)) {
        continue;
      }
      // multiply the difference by the resolution^2
      double volume = difference * planningMap_.getResolution() * planningMap_.getResolution();
      // add the volume to the total volume
      workspaceVolume += volume;
    }
    ROS_INFO_STREAM("[LocalPlanner]: Workspace volume for zone " << zoneId << " is " << workspaceVolume_);
    return workspaceVolume;
  }

  std::string LocalPlanner::getTargetDigLayer(int zoneId) {
    // target elevation if zone id is 0, original elevation if zone id is 1 or 2
    if (zoneId == 0) {
      return "desired_elevation";
    } else if (zoneId == 1) {
      return "original_elevation";
    } else if (zoneId == 2) {
      return "original_elevation";
    } else {
      ROS_ERROR("[LocalPlanner]: Invalid zone id");
      return "elevation";
    }
  }

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
    // create a separate thread to publish the maps
    std::thread mapThread_ = std::thread(&LocalPlanner::publishMaps, this);
    // detach the thread
    mapThread_.detach();
    return true;
  };

  void LocalPlanner::setLocalMap(grid_map::GridMap &localMap) {
    localMap_ = localMap;
    this->publishPlanningMap();
  };

  void LocalPlanner::setDigZone(int zoneId) {
    // check that is between 0 and 2
    if (zoneId < 0 || zoneId > 2) {
      ROS_ERROR_STREAM("[LocalPlanner]: Invalid zone id: " << zoneId);
      return;
    }
    digZoneId_ = zoneId;
    //  this->publishPlanningMap();
  };

  void LocalPlanner::setDumpZone(int zoneId) {
    // check that is between 1 and 4
    if (zoneId < 1 || zoneId > 4) {
      ROS_ERROR_STREAM("[LocalPlanner]: Invalid zone id: " << zoneId);
      return;
    }
    dumpZoneId_ = zoneId;
  }

  bool
  LocalPlanner::addDataFrom(grid_map::GridMap &map, const grid_map::GridMap &other, std::vector<std::string> layers) {
    // this function allow to copy over data from one map to another even if they don't have the same reference frame
    // Check if all layers to copy exist and add missing layers.
    for (const auto &layer: layers) {
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
      for (const auto &layer: layers) {
        if (!other.isValid(index, layer)) continue;
        map.at(layer, *iterator) = other.at(layer, index) - otherPosition3.z();
      }
    }
    return true;
  }

  void LocalPlanner::publishPlanningMap() {
    ROS_INFO_STREAM("[LocalPlanner]: Publishing planning map");
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(planningMap_, message);
    planningMapPublisher_.publish(message);
  }

  void LocalPlanner::publishMaps() {
    this->publishPlanningMap();
    excavationMappingPtr_->publishGridMap();
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

  double LocalPlanner::objectiveDistanceAndElevation(grid_map::Position &base_diggingPoint) {
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
    //  ROS_INFO_STREAM("[LocalPlanner]: best digging point is " << bestDiggingPoint.transpose());
    double diggingPointElevation = planningMap_.at("elevation", index);
    double diggingPointDesiredElevation = planningMap_.at("desired_elevation", index);
    //  ROS_INFO_STREAM("[LocalPlanner]: best objective is " << maxObjective);
    //  ROS_INFO_STREAM("[LocalPlanner]: digging point elevation: " << diggingPointElevation);
    //  ROS_INFO_STREAM("[LocalPlanner]: digging point desired elevation: " << diggingPointDesiredElevation);
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

  loco_m545::RotationQuaternion
  LocalPlanner::findOrientationWorldToShovel(double shovelRollAngle, double shovelPitchAngle,
                                             double shovelYawAngle) {
    return loco_m545::RotationQuaternion(
        loco_m545::AngleAxis(shovelPitchAngle, 0, 1, 0) * loco_m545::AngleAxis(shovelRollAngle, 1, 0, 0) *
        loco_m545::AngleAxis(shovelYawAngle, 0, 0, 1));
  }

  std::tuple<double, double> LocalPlanner::computeVolumeBetweenShovelPoints(Eigen::Vector3d &w_posLeftShovel_wl,
                                                                            Eigen::Vector3d &w_posRightShovel_wr,
                                                                            double previousTerrainElevation) {
    double lineWorkspaceVolume = 0;
    double lineOtherVolume = 0;
    // update planning layer of the planning map by setting the height to the new point
    for (grid_map::LineIterator iterator(planningMap_, Eigen::Vector2d(w_posRightShovel_wr.block<2, 1>(0, 0)),
                                         Eigen::Vector2d(w_posLeftShovel_wl.block<2, 1>(0, 0)));
         !iterator.isPastEnd(); ++iterator) {
      grid_map::Position iteratorPosition;
      planningMap_.getPosition(*iterator, iteratorPosition);
      const grid_map::Index index(*iterator);
      double currentPointElevation =
          w_posRightShovel_wr(2) + (w_posLeftShovel_wl(2) - w_posRightShovel_wr(2)) /
                                   (w_posLeftShovel_wl.block<2, 1>(0, 0) -
                                    w_posRightShovel_wr.block<2, 1>(0, 0)).norm() *
                                   (iteratorPosition - w_posRightShovel_wr.block<2, 1>(0, 0)).norm();
      double terrainElevation = planningMap_.atPosition("planning_elevation", iteratorPosition);
      planningMap_.atPosition("planning_elevation", iteratorPosition) = currentPointElevation;
      // if nan set equal to previous terrain elevation
      if (std::isnan(terrainElevation)) {
        terrainElevation = previousTerrainElevation;
      } else {
        previousTerrainElevation = terrainElevation;
      }
      double volumeSign = 1;
      // we need a volume sign to check weather we are supposed to dig here or not
      if (planningMap_.at("current_excavation_mask", index) != -1) {
        volumeSign = -1;
      }
      // if the value of terrain elevation is nan do not compute the volume
      //      if (std::isnan(terrainElevation)) {
      //        continue;
      //      }
      // set the value if the elevation at the current position in layer elevation is lower than the current value
      if (currentPointElevation < terrainElevation) {
        double cellVolume = planningMap_.getResolution() * planningMap_.getResolution() *
                            (terrainElevation - currentPointElevation);

        if (planningMap_.at("planning_zones", index) == digZoneId_) {
          lineWorkspaceVolume += volumeSign * cellVolume;
          // if workspaceVolume becomes nan raise an error
          if (std::isnan(lineWorkspaceVolume)) {
            ROS_ERROR("[LocalPlanner]: workspaceVolume is nan");
          }
        } else {
          // if the cell is not in the digging zone we do not want to dig here
          lineOtherVolume += volumeSign * cellVolume;
        }
      }
    }
    // return a tuple with the volume of the workspace and the volume of the other layer
    return std::make_tuple(lineWorkspaceVolume, lineOtherVolume);
  }

  Trajectory LocalPlanner::computeTrajectory(Eigen::Vector3d &w_P_wd, std::string targetLayer, int zoneId){
    Trajectory trajectory;
    if (zoneId == 0) {
      trajectory = this->computeDigTrajectory(w_P_wd, targetLayer);
    } else {
      trajectory = this->computeDirtTrajectory(w_P_wd, targetLayer);
    }
    return trajectory;
  }

  Trajectory LocalPlanner::computeDigTrajectory(Eigen::Vector3d &w_P_wd, std::string targetLayer) {
    // w_P_wd is the digging point
    // point below the surface
    //  ROS_INFO_STREAM("Computing trajectory for " << w_P_wd.transpose());
    grid_map::Position wg_P_wd(w_P_wd(0), w_P_wd(1));
    grid_map::Index wg_index;
    planningMap_.getIndex(wg_P_wd, wg_index);
    double elevation = excavationMappingPtr_->getElevation(wg_P_wd);
//    ROS_INFO_STREAM("[LocalPlanner]: target layer: " << targetLayer);
    double desiredElevation = planningMap_.at(targetLayer, wg_index);
    // throw an error if elevation or desired elevation is nan
    if (std::isnan(elevation) || std::isnan(desiredElevation)) {
      ROS_ERROR("[LocalPlannerc::ComputeTrajectory]: elevation or desired elevation is nan");
      return Trajectory();
    }
//      ROS_INFO_STREAM("[LocalPlanner]: elevation: " << elevation);
//       ROS_INFO_STREAM("[LocalPlanner]: desired elevation: " << desiredElevation);
    // print current elevation and desired elevation
    if (elevation - desiredElevation < heightPrecision_) {
      //    ROS_WARN("[LocalPlanner]: digging point is not below the surface");
      return Trajectory();
    }
    // from w_P_wd we want compute the trajectory that scoops up most volume
    // we procede in this way:
    // 1. start tracing the trajectory from w_P_wd
    // 2. while tracing the trajectory:
    //    - if the trajectory is not valid, we stop tracing it i.e we check if shovel is full and if we reached the boundary of the working
    //    space
    //    - if the trajectory is valid, we compute the next point in the trajectory and the current volume for the trajectory
    // 3. when trajectory is not valid we stop and close the trajectory
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
    Eigen::Vector3d w_P_wba = Eigen::Vector3d(T_mba.transform.translation.x, T_mba.transform.translation.y,
                                              T_mba.transform.translation.z);
    //  ROS_INFO_STREAM("Base origin in map frame: " << w_P_wba.transpose());
    double roll_b, pitch_b, yaw_b;
    tf2::Quaternion R_mba_q =
        tf2::Quaternion(T_mba.transform.rotation.x, T_mba.transform.rotation.y, T_mba.transform.rotation.z,
                        T_mba.transform.rotation.w);
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
    //   ROS_INFO_STREAM("[LocalPlanner]: digging point wrt base in base frame: " << ba_P_bad.transpose());
    double relativeHeading = atan2(ba_P_bad(1), ba_P_bad(0));
    //   ROS_INFO_STREAM("Base heading in map frame: " <<  yaw_b);
    //   ROS_INFO_STREAM("[LocalPlanner]: opt traj relative heading is " << relativeHeading);
    double heading = -yaw_b - relativeHeading;
    //   ROS_INFO_STREAM("[LocalPlanner]: opt traj heading " << heading);

    //  ROS_INFO_STREAM("[LocalPlanner]: True boom heading " << shovelYaw);
    // transform the yaw angle into a direction vector
    // get perpendicular vector in 2D (is it 2D?)
    Eigen::Vector3d w_P_dba = (-w_P_wd + w_P_wba).normalized();
    //  this->publishHeading(w_P_wd, w_P_dba, "map");
    //  Eigen::Vector3d C_ws = this->findShovelDesiredOrientation(w_P_wd, w_P_dba);
    double slopeAngle = 0;
    double desiredLocalAttitudeAngle = targetDigAttitude_;
    double attitudeAngle = desiredLocalAttitudeAngle + slopeAngle;
    Eigen::Quaterniond R_ws_d = this->get_R_sw(0, -attitudeAngle, heading);
    // move the initial point along the radial direction of the shovel
    // this allows the shovel to keep the same orientation for a bit even outside of the soil
    // this is convenient in practice because the height map might be imprecise and we might encounter soil before we expect it.
    Eigen::Vector3d w_P_wd_off = w_P_wd - verticalOffset_ * tan(attitudeAngle) * w_P_dba;
    // apply radialOffset_ in the direction dba
    w_P_wd_off = w_P_wd_off - radialOffset_ * w_P_dba;
    w_P_wd_off(2) = w_P_wd(2) + verticalOffset_;

    // this takes care of the fact that we have a penetration phase
    //  Eigen::Vector3d w_P_wd_off = w_P_wd - radialOffset_ * w_P_dba;
    // ROS_INFO_STREAM("[LocalPlanner]: w_P_dba: " << w_P_dba.transpose());
    // ROS_INFO_STREAM("[LocalPlanner]: w_P_wd_off: " << w_P_wd_off.transpose());
    // ROS_INFO_STREAM("[LocalPlanner]: radialOffset_: " << radialOffset_);
    // penetration vector
    Eigen::Vector3d w_P_dd1 = Eigen::Vector3d(0, 0, 0);
    // vertical displacement is the difference between the desired elevation and the elevation of the digging point
    w_P_dd1(2) = std::max(desiredElevation - elevation, -maxDigDepth_);  // fix maximal depth
    //  ROS_INFO_STREAM("[LocalPlanner]: w_P_dd1 depth: " << w_P_dd1(2));
    // these angles are all wrt the digging direction
    // for flat ground the desired attitude angle corresponds does not

    // the projection in 2d of the penetration vector is parallel to the digging vector and has length
    // angle of attack, if 0 shovel moves along the local z axis of the end effector
    double alpha = 0;
    double diggingPathAngle = desiredLocalAttitudeAngle - alpha;
    double heightChange = std::min(elevation - desiredElevation, maxDigDepth_);
    // this is probably the problem
    //  ROS_INFO_STREAM("[LocalPlanner]: heightChange: " << heightChange);
    double horizontalDisplacement = heightChange / tan(diggingPathAngle);
    //  w_P_dd1.head(2) = w_P_dba * horizontalDisplacement;
    w_P_dd1.head(2) = (-w_P_dd1(2) * tan(attitudeAngle)) * w_P_dba;
    //   ROS_INFO_STREAM("[LocalPlanner]: w_P_dd1: " << w_P_dd1.transpose());
    //   ROS_INFO_STREAM("[LocalPlanner]: digging vector in world frame " << w_P_dd1.transpose());
    //   ROS_INFO_STREAM("[LocalPlanner]: horizontal displacement " << horizontalDisplacement);

    Eigen::Vector3d w_P_wd1 = w_P_wd;
    w_P_wd1(2) -= heightChange;
    w_P_wd_off.head(2) = w_P_wd.head(2) - (-w_P_dd1(2) * tan(attitudeAngle)) * w_P_dba;
    Eigen::Quaterniond R_ws_d1 = this->get_R_sw(0, -attitudeAngle, heading);

    //  this->publishVector(w_P_wd, w_P_dd1, "map");

    bool valid = true;
    std::vector<Eigen::Vector3d> digPoints;
    std::vector<Eigen::Quaterniond> digOrientations;
    double targetAttitude = M_PI / 2;
    Eigen::Vector3d w_P_wd_current = w_P_wd1;
    double stepSize = planningMap_.getResolution();
    double volume = 0;
    double volumeLeft = 0;
    double volumeRight = 0;
    std::vector<double> stepVolumes;
    double workspaceVolume = 0;

    Eigen::Vector3d s_posLeftShovel_cl(0.0, 0.75, 0.0);
    Eigen::Vector3d s_posRightShovel_cr(0.0, -0.75, 0.0);
    // now we march with step size of planningMap resolution / 2 in the direction of the boom direction until the trajectory is not valid
    // anymore
    int numSteps = 0;
    //  volume += shovelVolumeBonus_;
    // variable used to handle nans
    double previousElevation = 0;
    double previousLineVolume = 0;
    double previousCellVolume = 0;

    while (valid) {
      Eigen::Vector3d w_P_next = w_P_wd_current + stepSize * w_P_dba;
      // height is overridden
      grid_map::Index nextIndex;
      planningMap_.getIndex(w_P_next.head(2), nextIndex);
      double nextDesiredElevation = planningMap_.at(targetLayer, nextIndex);
      double nextElevation = planningMap_.at("planning_elevation", nextIndex);
      // if nan set nextElevation to previousElevation
      if (std::isnan(nextElevation)) {
        nextElevation = previousElevation;
      } else {
        previousElevation = nextElevation;
      }

      w_P_next(2) =
          nextElevation + std::max(desiredElevation - nextElevation, -maxDigDepth_);  // fix maximal depth

      // position of the left point of the shovel (l) in world frame
      Eigen::Vector3d w_posLeftShovel_wl = w_P_next + R_ws_d1.inverse() * s_posLeftShovel_cl;
      Eigen::Vector3d w_posRightShovel_wr = w_P_next + R_ws_d1.inverse() * s_posRightShovel_cr;
      double previousTerrainElevation = nextElevation;
      // get the tuple lineVolume and otherLineVolume
      // compute the volume for the left part of the shovel
      std::tuple<double, double> lineLeftVolume = this->computeVolumeBetweenShovelPoints(w_posLeftShovel_wl,
                                                                                         w_P_next,
                                                                                         nextDesiredElevation);
      std::tuple<double, double> lineRightVolume =
          this->computeVolumeBetweenShovelPoints(w_P_next, w_posRightShovel_wr, nextDesiredElevation);
      // get the volume inside the workspace and outside
      double workspaceLineVolume = std::get<0>(lineLeftVolume) + std::get<0>(lineRightVolume);
      double otherLineVolume = std::get<1>(lineLeftVolume) + std::get<1>(lineRightVolume);

      //    std::tuple<double, double> lineVolume = this->computeVolumeBetweenShovelPoints(w_posLeftShovel_wl, w_posRightShovel_wr,
      //    nextDesiredElevation); double workspaceLineVolume_ = std::get<0>(lineVolume); double otherLineVolume_ = std::get<1>(lineVolume);
      //    double totalLineVolume = workspaceLineVolume_ + otherLineVolume_;
      // append point
      digPoints.push_back(w_P_next);
      // check if digging outside of the dig area stop
      volume += workspaceLineVolume + otherLineVolume;
      stepVolumes.push_back(workspaceLineVolume + otherLineVolume);
      volumeLeft += std::get<0>(lineLeftVolume) + std::get<1>(lineLeftVolume);
      volumeRight += std::get<0>(lineRightVolume) + std::get<1>(lineRightVolume);

      workspaceVolume += workspaceLineVolume;
      // if volume becomes nan raise an error
      if (std::isnan(volume)) {
        ROS_ERROR("[LocalPlanner]: volume is nan");
      }
      // todo: verify it's useful
      // sometimes the digger could start just before the workspace starts, then the initial volume is less then 0
      if (volume < 0 and numSteps > 5) {
        valid = false;
        break;
      }
      // check if the trajectory is valid
      //  ROS_INFO_STREAM("[LocalPlanner]: line volume " << lineVolume);
      //  ROS_INFO_STREAM("[LocalPlanner]: volume " << volume);
      //  ROS_INFO_STREAM("[LocalPlanner]: workspace volume " << workspaceVolume);
      if (volume > maxVolume_ || volumeLeft > maxVolume_ / 2 || volumeRight > maxVolume_ / 2) {
        valid = false;
        break;
      }
      // compute distance from the base
      double distanceFromBase = (w_P_next - w_P_wba).norm();
      //    ROS_INFO_STREAM("[LocalPlanner]: distance from base " << startDistanceFromBase);
      if (distanceFromBase < minDistanceShovelToBase_) {
        valid = false;
        break;
      }
      w_P_wd_current = w_P_next;
      numSteps++;
      //    ROS_INFO_STREAM("[LocalPlanner]: step " << numSteps << " volume " << volume);
    }
    // reset planning elevation
    planningMap_["planning_elevation"] = planningMap_["elevation"];

    // refine the trajectories by the last n point whose volume is zero
    // and the first n point whose volume is zero
    size_t numPointsToRemove = 0;
    for (size_t i = 0; i < stepVolumes.size(); i++) {
      if (stepVolumes[i] <= 0) {
        numPointsToRemove++;
      } else {
        break;
      }
    }
    // remove some points, later on we run an interpolation on the remaining ones
    digPoints.erase(digPoints.begin(), digPoints.begin() + numPointsToRemove);
    stepVolumes.erase(stepVolumes.begin(), stepVolumes.begin() + numPointsToRemove);
    numSteps -= numPointsToRemove;
    // refine the trajectories by the last n point whose volume is zero
    numPointsToRemove = 0;
    // we start from the end of stepVolumes because we want to remove the last n points
    for (int i = stepVolumes.size() - 1; i >= 0; i--) {
      if (stepVolumes[i] <= 0) {
        numPointsToRemove++;
      } else {
        break;
      }
    }
    digPoints.erase(digPoints.end() - numPointsToRemove, digPoints.end());
    stepVolumes.erase(stepVolumes.end() - numPointsToRemove, stepVolumes.end());
    numSteps -= numPointsToRemove;

    // if no volume in the workspace return empty trajectory
    if (digPoints.size() == 0) {
      return Trajectory();
    }
    // smooth out the z coordinates of DigPoints with filtering
    std::vector<Eigen::Vector3d> smoothedDigPoints = this->smoothZCoordinates(digPoints);
    // interpolate the z coordinates of DigPoints
    //  // since the max depth is fixed and elevation map can be noisy we smooth out the z values
    //  // by interpolating between the first and last point
    //  // print difference in elevation between the first and the last point
    //        double elevationDifference = digPoints.back()(2) - digPoints.front()(2);
    //        ROS_INFO_STREAM("[LocalPlanner]: elevation difference " << elevationDifference);
    //  for (size_t i = 0; i < digPoints.size(); i++) {
    //    digPoints[i](2) = digPoints[0](2) + (digPoints[digPoints.size() - 1](2) - digPoints[0](2)) * (double) i / (digPoints.size() - 1);
    //  }

    // count how many points are necessary to reach a dragging distance of 1.5 meters
    // this is used to control the attitude of the shovel
    // if the trajectory is too long the shovel stays too vertical and soil won't be dragged effectively
    size_t numPointsToReachDraggingDistance = 0;
    for (size_t i = 1; i < smoothedDigPoints.size(); i++) {
      if ((smoothedDigPoints[i] - smoothedDigPoints[0]).norm() < draggingDistance_) {
        numPointsToReachDraggingDistance = i;
      }
    }
    int numPointFromDraggingDistance = smoothedDigPoints.size() - numPointsToReachDraggingDistance + 1;
    double stepSizePitchOrientationAfterDrag = 0;

    if (numPointFromDraggingDistance > 1) {
      double stepSizePitchOrientationAfterDrag =
          (targetAttitude - draggingAngle_) / (numPointFromDraggingDistance - 1);
    }

    // interpolate the orientation of the shovel
    double stepSizePitchOrientation = (draggingAngle_ - attitudeAngle) / (numPointsToReachDraggingDistance - 1);
    double currentPitchOrientation = attitudeAngle;
    for (size_t i = 0; i < smoothedDigPoints.size(); i++) {
      Eigen::Quaterniond R_ws_d = this->get_R_sw(0, -currentPitchOrientation, heading);
      digOrientations.push_back(R_ws_d);
      if (i <= numPointsToReachDraggingDistance) {
        currentPitchOrientation += stepSizePitchOrientation;
      } else {
        currentPitchOrientation += stepSizePitchOrientationAfterDrag;
        // cap the dig points z coordinate to the depth before dragging starts
        // to avoid the shovel pushing against the soil when almost flat
        if (smoothedDigPoints[i](2) < smoothedDigPoints[i - 1](2)) {
          smoothedDigPoints[i](2) = smoothedDigPoints[i - 1](2);
        }
      }
    }

    // check for collisions
    std::vector<Eigen::Vector3d> collisionFreeDigPoints;
    std::vector<Eigen::Quaterniond> collisionFreeOrientations;
    collisions::DistanceOptions distanceOptions;
    for (size_t i = 0; i < smoothedDigPoints.size(); i++) {
      this->updateShovelCollisionBody(smoothedDigPoints[i], digOrientations[i]);
      collisions::DistanceResults results = colliderManager_.checkDistance(armCollisionGroup_, legCollisionGroup_,
                                                                           distanceOptions);
      double minDistance = 100;
      for (const auto &resultMap: results.getResults()) {
        const collisions::DistanceResult &result = resultMap.second;

        // distance between closest points
        const double d = result.distance;
        if (d < minDistance) {
          minDistance = d;
        }
      }
      // print the distance
      if (minDistance > minDistanceCollision_) {
        collisionFreeDigPoints.push_back(smoothedDigPoints[i]);
        collisionFreeOrientations.push_back(digOrientations[i]);
      } else {
        break;
      }
    }
    if (collisionFreeDigPoints.size() == 0) {
      return Trajectory();
    }
    Eigen::Vector3d w_P_wd_last = collisionFreeDigPoints.back();
    // get the orientation of the shovel at the last point
    Eigen::Quaterniond R_ws_d_last = collisionFreeOrientations.back();
    Eigen::Vector3d closingOffset(0.3, 0, 0.7);
    double scaling = 0.3;
    Eigen::Vector3d w_P_d2d3 = w_P_dba.normalized() * scaling;
    // get desired height at the end of the trajectory
    grid_map::Position wg_P_wd2(w_P_wd_last(0), w_P_wd_last(1));
    double elevation2 = excavationMappingPtr_->getElevation(wg_P_wd2);
    grid_map::Index index2;
    planningMap_.getIndex(wg_P_wd2, index2);
    double desiredElevation2 = planningMap_.at(targetLayer, index2);
    // vertical displacement is the difference between the desired elevation and the elevation of the digging point
    w_P_d2d3(2) = elevation2 - desiredElevation + closingOffset(2);
    // transfrom to world frame by rotating yaw angle around the z axis
    Eigen::AngleAxisd R_wc(heading, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d w_P_wd3 = w_P_wd_last + w_P_d2d3;
    w_P_wd3(2) = elevation2;
    Eigen::Vector3d w_P_wd4 = w_P_wd3 + closingZTranslation_ * Eigen::Vector3d::UnitZ();
    double theta = M_PI / 2 - M_PI / 2;  // last quadrant of the circle
    //  Eigen::Vector3d w_P_wd3 =
    //      w_P_wd2 +
    Eigen::Quaterniond R_ws_d3 = this->get_R_sw(0, -M_PI * 3 / 4, heading);
    Eigen::Quaterniond R_ws_d4 = R_ws_d3;
    //  ROS_INFO_STREAM("[LocalPlanner]: Euler angles 3 " << R_ws_d3.toRotationMatrix().eulerAngles(0, 1, 2).transpose());

    // fuse together the two trajectories
    std::vector<Eigen::Vector3d> digPointsFused;
    std::vector<Eigen::Quaterniond> digOrientationsFused;
    digPointsFused.push_back(w_P_wd_off);
    digOrientationsFused.push_back(R_ws_d);
    digPointsFused.push_back(w_P_wd1);
    digOrientationsFused.push_back(R_ws_d1);
    // append digPoints vector to digPointsFused
    digPointsFused.insert(digPointsFused.end(), collisionFreeDigPoints.begin(), collisionFreeDigPoints.end() - 1);
    digOrientationsFused.insert(digOrientationsFused.end(), collisionFreeOrientations.begin(),
                                collisionFreeOrientations.end() - 1);
    //    digPointsFused.push_back(w_P_wd_last);
    //    digOrientationsFused.push_back(R_ws_d_last);
    digPointsFused.push_back(w_P_wd3);
    digOrientationsFused.push_back(R_ws_d3);
    digPointsFused.push_back(w_P_wd4);
    digOrientationsFused.push_back(R_ws_d4);
    // check if digPointFused size is bigger then 0 else raise an warning
    if (digPointsFused.size() == 0) {
      ROS_WARN_STREAM("[LocalPlanner]: could not find a valid trajectory!");
    }
    // get the trajectory
    // print the digPointsFused
    //  ROS_INFO_STREAM("[LocalPlanner]: digPointsFused size " << digPointsFused.size());
    //  for (size_t i = 0; i < digPointsFused.size(); i++) {
    //    ROS_INFO_STREAM("[LocalPlanner]: digPointsFused " << digPointsFused[i].transpose());
    //  }
    Trajectory trajectory;
    trajectory.positions = digPointsFused;
    trajectory.orientations = digOrientationsFused;
    trajectory.scoopedVolume = volume;
    trajectory.workspaceVolume = workspaceVolume;
    trajectory.startDistanceFromBase = (w_P_wd_off.head(2) - w_P_wba.head(2)).norm();
    trajectory.endDistanceFromBase = (w_P_wd_last.head(2) - w_P_wba.head(2)).norm();
    trajectory.relativeHeading = relativeHeading;
    // compute total trajectory length summing the distance between the points of the trajectory
    trajectory.length = 0;
    for (size_t i = 0; i < digPointsFused.size() - 1; i++) {
      trajectory.length += (digPointsFused[i] - digPointsFused[i + 1]).norm();
    }
    trajectory.stepVolumes = stepVolumes;
    //  ROS_INFO_STREAM("[LocalPlanner]: completed trajectory starting at" << w_P_wd_off.transpose() << " with heading " << heading);

    // set the trajectory
    //  this->publishDesiredShovelPose(w_P_wd, R_ws_d1);
    //  this->publishDesiredShovelPose(w_P_wd3, R_ws_d3);
    //  this->publishTrajectoryPoses(digPoints, digOrientations);
    //  this->publishMarkersTrajectory(digPointsFused, "map");
    // reset elevation mapping
    //  unique_lock lock(mapMutex_);
    //  planningMap_["planning_elevation"] = planningMap_["elevation"];
    //  lock.unlock();
    return trajectory;
  }


  Trajectory LocalPlanner::computeDirtTrajectory(Eigen::Vector3d &w_P_wd, std::string targetLayer) {
    // w_P_wd is the digging point
    // point below the surface
    //  ROS_INFO_STREAM("Computing trajectory for " << w_P_wd.transpose());
    grid_map::Position wg_P_wd(w_P_wd(0), w_P_wd(1));
    grid_map::Index wg_index;
    planningMap_.getIndex(wg_P_wd, wg_index);
    double elevation = excavationMappingPtr_->getElevation(wg_P_wd);
//    ROS_INFO_STREAM("[LocalPlanner::ComputeDirtTrajectory]: target layer: " << targetLayer);
    double desiredElevation = planningMap_.at(targetLayer, wg_index);
    // throw an error if elevation or desired elevation is nan
    if (std::isnan(elevation) || std::isnan(desiredElevation)) {
      ROS_WARN("[LocalPlanner::ComputeTrajectory]: elevation or desired elevation is nan");
      return Trajectory();
    }
//    ROS_INFO_STREAM("[LocalPlanner]: elevation: " << elevation);
//    ROS_INFO_STREAM("[LocalPlanner]: desired elevation: " << desiredElevation);
    // print current elevation and desired elevation
    if (elevation - desiredElevation < heightPrecision_) {
      //    ROS_WARN("[LocalPlanner]: digging point is not below the surface");
      return Trajectory();
    }
    // from w_P_wd we want compute the trajectory that scoops up most volume
    // we procede in this way:
    // 1. start tracing the trajectory from w_P_wd
    // 2. while tracing the trajectory:
    //    - if the trajectory is not valid, we stop tracing it i.e we check if shovel is full and if we reached the boundary of the working
    //    space
    //    - if the trajectory is valid, we compute the next point in the trajectory and the current volume for the trajectory
    // 3. when trajectory is not valid we stop and close the trajectory
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
    Eigen::Vector3d w_P_wba = Eigen::Vector3d(T_mba.transform.translation.x, T_mba.transform.translation.y,
                                              T_mba.transform.translation.z);
    //  ROS_INFO_STREAM("Base origin in map frame: " << w_P_wba.transpose());
    double roll_b, pitch_b, yaw_b;
    tf2::Quaternion R_mba_q =
        tf2::Quaternion(T_mba.transform.rotation.x, T_mba.transform.rotation.y, T_mba.transform.rotation.z,
                        T_mba.transform.rotation.w);
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
    //   ROS_INFO_STREAM("[LocalPlanner]: digging point wrt base in base frame: " << ba_P_bad.transpose());
    double relativeHeading = atan2(ba_P_bad(1), ba_P_bad(0));
    //   ROS_INFO_STREAM("Base heading in map frame: " <<  yaw_b);
    //   ROS_INFO_STREAM("[LocalPlanner]: opt traj relative heading is " << relativeHeading);
    double heading = -yaw_b - relativeHeading;
    //   ROS_INFO_STREAM("[LocalPlanner]: opt traj heading " << heading);

    //  ROS_INFO_STREAM("[LocalPlanner]: True boom heading " << shovelYaw);
    // transform the yaw angle into a direction vector
    // get perpendicular vector in 2D (is it 2D?)
    Eigen::Vector3d w_P_dba = (-w_P_wd + w_P_wba).normalized();
    //  this->publishHeading(w_P_wd, w_P_dba, "map");
    //  Eigen::Vector3d C_ws = this->findShovelDesiredOrientation(w_P_wd, w_P_dba);
    double slopeAngle = 0;
    double desiredLocalAttitudeAngle = targetDigDirtAttitude_;
    double attitudeAngle = desiredLocalAttitudeAngle + slopeAngle;
    Eigen::Quaterniond R_ws_d = this->get_R_sw(0, -targetDigAttitude_, heading);
    // move the initial point along the radial direction of the shovel
    // this allows the shovel to keep the same orientation for a bit even outside of the soil
    // this is convenient in practice because the height map might be imprecise and we might encounter soil before we expect it.
    Eigen::Vector3d w_P_wd_off = w_P_wd - verticalOffset_ * w_P_dba;
    // apply radialOffset_ in the direction dba
    w_P_wd_off = w_P_wd_off - radialDirtOffset_ * w_P_dba;
    w_P_wd_off(2) = w_P_wd(2) + verticalOffset_;

    // this takes care of the fact that we have a penetration phase
    //  Eigen::Vector3d w_P_wd_off = w_P_wd - radialOffset_ * w_P_dba;
    // ROS_INFO_STREAM("[LocalPlanner]: w_P_dba: " << w_P_dba.transpose());
    // ROS_INFO_STREAM("[LocalPlanner]: w_P_wd_off: " << w_P_wd_off.transpose());
    // ROS_INFO_STREAM("[LocalPlanner]: radialOffset_: " << radialOffset_);
    // penetration vector
    Eigen::Vector3d w_P_dd1 = Eigen::Vector3d(0, 0, 0);
    // vertical displacement is the difference between the desired elevation and the elevation of the digging point
    w_P_dd1(2) = std::max(desiredElevation - elevation, -maxDigDirtDepth_);  // fix maximal depth
    //  ROS_INFO_STREAM("[LocalPlanner]: w_P_dd1 depth: " << w_P_dd1(2));
    // these angles are all wrt the digging direction
    // for flat ground the desired attitude angle corresponds does not

    // the projection in 2d of the penetration vector is parallel to the digging vector and has length
    // angle of attack, if 0 shovel moves along the local z axis of the end effector
    double alpha = 0;
    double diggingPathAngle = desiredLocalAttitudeAngle - alpha;
    double heightChange = std::min(elevation - desiredElevation, maxDigDirtDepth_);
    // this is probably the problem
    //  ROS_INFO_STREAM("[LocalPlanner]: heightChange: " << heightChange);
    double horizontalDisplacement = 0;
    //  w_P_dd1.head(2) = w_P_dba * horizontalDisplacement;
//    w_P_dd1.head(2) = (-w_P_dd1(2) * tan(attitudeAngle)) * w_P_dba;
    //   ROS_INFO_STREAM("[LocalPlanner]: w_P_dd1: " << w_P_dd1.transpose());
    //   ROS_INFO_STREAM("[LocalPlanner]: digging vector in world frame " << w_P_dd1.transpose());
    //   ROS_INFO_STREAM("[LocalPlanner]: horizontal displacement " << horizontalDisplacement);

    Eigen::Vector3d w_P_wd1 = w_P_wd;
//    w_P_wd1.head(2) -= radialDirtOffset_ * w_P_dba.head(2);
    w_P_wd1(2) -= heightChange;
    w_P_wd_off.head(2) = w_P_wd.head(2) - (-w_P_dd1(2) - horizontalDisplacement) * w_P_dba;
    Eigen::Quaterniond R_ws_d1 = this->get_R_sw(0, -targetDigDirtAttitude_, heading);

    //  this->publishVector(w_P_wd, w_P_dd1, "map");

    bool valid = true;
    std::vector<Eigen::Vector3d> digPoints;
    std::vector<Eigen::Quaterniond> digOrientations;
    double targetAttitude = targetDigDirtAttitude_;
    Eigen::Vector3d w_P_wd_current = w_P_wd1;
    double stepSize = planningMap_.getResolution();
    double volume = 0;
    double volumeLeft = 0;
    double volumeRight = 0;
    std::vector<double> stepVolumes;
    double workspaceVolume = 0;

    Eigen::Vector3d s_posLeftShovel_cl(0.0, 0.75, 0.0);
    Eigen::Vector3d s_posRightShovel_cr(0.0, -0.75, 0.0);
    // now we march with step size of planningMap resolution / 2 in the direction of the boom direction until the trajectory is not valid
    // anymore
    int numSteps = 0;
    //  volume += shovelVolumeBonus_;
    // variable used to handle nans
    double previousElevation = 0;
    double previousLineVolume = 0;
    double previousCellVolume = 0;
    double previousStepVolume = 0;

    while (valid) {
      Eigen::Vector3d w_P_next = w_P_wd_current + stepSize * w_P_dba;
      // height is overridden
      grid_map::Index nextIndex;
      planningMap_.getIndex(w_P_next.head(2), nextIndex);
      double nextDesiredElevation = planningMap_.at(targetLayer, nextIndex);
      double nextElevation = planningMap_.at("planning_elevation", nextIndex);
      // if nan set nextElevation to previousElevation
      if (std::isnan(nextElevation)) {
        nextElevation = previousElevation;
      } else {
        previousElevation = nextElevation;
      }

      w_P_next(2) =
          nextElevation + std::max(desiredElevation - nextElevation, -maxDigDirtDepth_);  // fix maximal depth

      // position of the left point of the shovel (l) in world frame
      Eigen::Vector3d w_posLeftShovel_wl = w_P_next + R_ws_d1.inverse() * s_posLeftShovel_cl;
      Eigen::Vector3d w_posRightShovel_wr = w_P_next + R_ws_d1.inverse() * s_posRightShovel_cr;
      double previousTerrainElevation = nextElevation;
      // get the tuple lineVolume and otherLineVolume
      // compute the volume for the left part of the shovel
      std::tuple<double, double> lineLeftVolume = this->computeVolumeBetweenShovelPoints(w_posLeftShovel_wl,
                                                                                         w_P_next,
                                                                                         nextDesiredElevation);
      std::tuple<double, double> lineRightVolume =
          this->computeVolumeBetweenShovelPoints(w_P_next, w_posRightShovel_wr, nextDesiredElevation);
      // get the volume inside the workspace and outside
      double workspaceLineVolume = std::get<0>(lineLeftVolume) + std::get<0>(lineRightVolume);
      double otherLineVolume = std::get<1>(lineLeftVolume) + std::get<1>(lineRightVolume);

      //    std::tuple<double, double> lineVolume = this->computeVolumeBetweenShovelPoints(w_posLeftShovel_wl, w_posRightShovel_wr,
      //    nextDesiredElevation); double workspaceLineVolume_ = std::get<0>(lineVolume); double otherLineVolume_ = std::get<1>(lineVolume);
      //    double totalLineVolume = workspaceLineVolume_ + otherLineVolume_;
      // append point
      digPoints.push_back(w_P_next);
      // check if digging outside of the dig area stop
      volume += workspaceLineVolume + otherLineVolume;
      double stepVolume = workspaceLineVolume + otherLineVolume;
      if (stepVolume == 0 && previousStepVolume != 0) {
        valid = false;
        break;
      }
      previousStepVolume = stepVolume;
      stepVolumes.push_back(stepVolume);
      volumeLeft += std::get<0>(lineLeftVolume) + std::get<1>(lineLeftVolume);
      volumeRight += std::get<0>(lineRightVolume) + std::get<1>(lineRightVolume);

      workspaceVolume += workspaceLineVolume;
      // if volume becomes nan raise an error
      if (std::isnan(volume)) {
        ROS_ERROR("[LocalPlanner]: volume is nan");
      }
      // todo: verify it's useful
      // sometimes the digger could start just before the workspace starts, then the initial volume is less then 0
      if (volume < 0 and numSteps > 5) {
        valid = false;
        break;
      }
      // check if the trajectory is valid
      //  ROS_INFO_STREAM("[LocalPlanner]: line volume " << lineVolume);
      //  ROS_INFO_STREAM("[LocalPlanner]: volume " << volume);
      //  ROS_INFO_STREAM("[LocalPlanner]: workspace volume " << workspaceVolume);
      if (volume > maxVolume_ || volumeLeft > maxVolume_ / 2 || volumeRight > maxVolume_ / 2) {
        valid = false;
        break;
      }
      // compute distance from the base
      double distanceFromBase = (w_P_next - w_P_wba).norm();
      //    ROS_INFO_STREAM("[LocalPlanner]: distance from base " << startDistanceFromBase);
      if (distanceFromBase < minDistanceShovelToBase_) {
        valid = false;
        break;
      }
      w_P_wd_current = w_P_next;
      numSteps++;
      //    ROS_INFO_STREAM("[LocalPlanner]: step " << numSteps << " volume " << volume);
    }
    // reset planning elevation
    planningMap_["planning_elevation"] = planningMap_["elevation"];
    //
    // refine the trajectories by the last n point whose volume is zero
    // and the first n point whose volume is zero
    size_t numPointsToRemove = 0;
    for (size_t i = 0; i < stepVolumes.size(); i++) {
      if (stepVolumes[i] == 0) {
        numPointsToRemove++;
      } else {
        break;
      }
    }
    // remove some points, later on we run an interpolation on the remaining ones
    digPoints.erase(digPoints.begin(), digPoints.begin() + numPointsToRemove);
    stepVolumes.erase(stepVolumes.begin(), stepVolumes.begin() + numPointsToRemove);
    numSteps -= numPointsToRemove;
    // refine the trajectories by the last n point whose volume is zero
//    numPointsToRemove = stepVolumes.size() - 1;
//    // we start from the end of stepVolumes because we want to remove the last n points
//    for (int i = 0; i < stepVolumes.size(); i++) {
//      if (stepVolumes[i] > 0) {
//        numPointsToRemove--;
//      } else {
//        break;
//      }
//    }
//    digPoints.erase(digPoints.end() - numPointsToRemove, digPoints.end());
//    stepVolumes.erase(stepVolumes.end() - numPointsToRemove, stepVolumes.end());
//    numSteps -= numPointsToRemove;

    // if no volume in the workspace return empty trajectory
    if (digPoints.size() == 0) {
      return Trajectory();
    }
    // smooth out the z coordinates of DigPoints with filtering
    std::vector<Eigen::Vector3d> smoothedDigPoints = this->smoothZCoordinates(digPoints);
    // interpolate the z coordinates of DigPoints
    //  // since the max depth is fixed and elevation map can be noisy we smooth out the z values
    //  // by interpolating between the first and last point
    //  // print difference in elevation between the first and the last point
    //        double elevationDifference = digPoints.back()(2) - digPoints.front()(2);
    //        ROS_INFO_STREAM("[LocalPlanner]: elevation difference " << elevationDifference);
    //  for (size_t i = 0; i < digPoints.size(); i++) {
    //    digPoints[i](2) = digPoints[0](2) + (digPoints[digPoints.size() - 1](2) - digPoints[0](2)) * (double) i / (digPoints.size() - 1);
    //  }

    // count how many points are necessary to reach a dragging distance of 1.5 meters
    // this is used to control the attitude of the shovel
    // if the trajectory is too long the shovel stays too vertical and soil won't be dragged effectively
    size_t numPointsToReachDraggingDistance = 0;
    for (size_t i = 1; i < smoothedDigPoints.size(); i++) {
      if ((smoothedDigPoints[i] - smoothedDigPoints[0]).norm() < draggingDirtDistance_) {
        numPointsToReachDraggingDistance = i;
      }
    }
    int numPointFromDraggingDistance = smoothedDigPoints.size() - numPointsToReachDraggingDistance + 1;
    double stepSizePitchOrientationAfterDrag = 0;

    if (numPointFromDraggingDistance > 1) {
      double stepSizePitchOrientationAfterDrag =
          (targetDigDirtAttitude_ - draggingAngle_) / (numPointFromDraggingDistance - 1);
    }

    // interpolate the orientation of the shovel
    double stepSizePitchOrientation = (draggingAngle_ - attitudeAngle) / (numPointsToReachDraggingDistance - 1);
    double currentPitchOrientation = attitudeAngle;
//    ROS_INFO_STREAM("[LocalPlanner]: **** attitude angle " << attitudeAngle);
    for (size_t i = 0; i < smoothedDigPoints.size(); i++) {
      Eigen::Quaterniond R_ws_d = this->get_R_sw(0, -currentPitchOrientation, heading);
      digOrientations.push_back(R_ws_d);
      if (i <= numPointsToReachDraggingDistance) {
        currentPitchOrientation += stepSizePitchOrientation;
      } else {
        currentPitchOrientation += stepSizePitchOrientationAfterDrag;
        // cap the dig points z coordinate to the depth before dragging starts
        // to avoid the shovel pushing against the soil when almost flat
        if (smoothedDigPoints[i](2) < smoothedDigPoints[i - 1](2)) {
          smoothedDigPoints[i](2) = smoothedDigPoints[i - 1](2);
        }
      }
    }

    // check for collisions
    std::vector<Eigen::Vector3d> collisionFreeDigPoints;
    std::vector<Eigen::Quaterniond> collisionFreeOrientations;
    collisions::DistanceOptions distanceOptions;
    for (size_t i = 0; i < smoothedDigPoints.size(); i++) {
      this->updateShovelCollisionBody(smoothedDigPoints[i], digOrientations[i]);
      collisions::DistanceResults results = colliderManager_.checkDistance(armCollisionGroup_, legCollisionGroup_,
                                                                           distanceOptions);
      double minDistance = 100;
      for (const auto &resultMap: results.getResults()) {
        const collisions::DistanceResult &result = resultMap.second;

        // distance between closest points
        const double d = result.distance;
        if (d < minDistance) {
          minDistance = d;
        }
      }
      // print the distance
      if (minDistance > minDistanceCollision_) {
        collisionFreeDigPoints.push_back(smoothedDigPoints[i]);
        collisionFreeOrientations.push_back(digOrientations[i]);
      } else {
        break;
      }
    }
    if (collisionFreeDigPoints.size() == 0) {
      return Trajectory();
    }
    Eigen::Vector3d w_P_wd_last = collisionFreeDigPoints.back();
    // get the orientation of the shovel at the last point
    Eigen::Quaterniond R_ws_d_last = collisionFreeOrientations.back();
    double horizontalClosingOffset = 0.3;
    double verticalClosingOffset = 0.3;
//    Eigen::Vector3d w_P_d2d3 = w_P_dba.normalized() * scaling;
//    // get desired height at the end of the trajectory
//    grid_map::Position wg_P_wd2(w_P_wd_last(0), w_P_wd_last(1));
//    double elevation2 = excavationMappingPtr_->getElevation(wg_P_wd2);
//    grid_map::Index index2;
//    planningMap_.getIndex(wg_P_wd2, index2);
//    double desiredElevation2 = planningMap_.at(targetLayer, index2);
//    // vertical displacement is the difference between the desired elevation and the elevation of the digging point
//    w_P_d2d3(2) = closingOffset(2);
//    // transfrom to world frame by rotating yaw angle around the z axis
//    Eigen::AngleAxisd R_wc(heading, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d w_P_wd3 = w_P_wd_last;
    w_P_wd3.head(2) -= w_P_dba.head(2).normalized() * horizontalClosingOffset;
    w_P_wd3(2) += verticalClosingOffset;
    double theta = M_PI / 2 - M_PI / 2;  // last quadrant of the circle
    //  Eigen::Vector3d w_P_wd3 =
    //      w_P_wd2 +
    Eigen::Quaterniond R_ws_d3 = this->get_R_sw(0, -M_PI * 3 / 4, heading);
    //  ROS_INFO_STREAM("[LocalPlanner]: Euler angles 3 " << R_ws_d3.toRotationMatrix().eulerAngles(0, 1, 2).transpose());

    // fuse together the two trajectories
    std::vector<Eigen::Vector3d> digPointsFused;
    std::vector<Eigen::Quaterniond> digOrientationsFused;
    digPointsFused.push_back(w_P_wd_off);
    digOrientationsFused.push_back(R_ws_d);
    digPointsFused.push_back(w_P_wd1);
    digOrientationsFused.push_back(R_ws_d);
    // append digPoints vector to digPointsFused
    digPointsFused.insert(digPointsFused.end(), collisionFreeDigPoints.begin(), collisionFreeDigPoints.end());
    digOrientationsFused.insert(digOrientationsFused.end(), collisionFreeOrientations.begin(),
                                collisionFreeOrientations.end());
    //    digPointsFused.push_back(w_P_wd_last);
    //    digOrientationsFused.push_back(R_ws_d_last);
    digPointsFused.push_back(w_P_wd3);
    digOrientationsFused.push_back(R_ws_d3);
//    digPointsFused.push_back(w_P_wd4);
//    digOrientationsFused.push_back(R_ws_d4);
    // check if digPointFused size is bigger then 0 else raise an warning
    if (digPointsFused.size() == 0) {
      ROS_WARN_STREAM("[LocalPlanner]: could not find a valid trajectory!");
    }
    // get the trajectory
    // print the digPointsFused
    //  ROS_INFO_STREAM("[LocalPlanner]: digPointsFused size " << digPointsFused.size());
    //  for (size_t i = 0; i < digPointsFused.size(); i++) {
    //    ROS_INFO_STREAM("[LocalPlanner]: digPointsFused " << digPointsFused[i].transpose());
    //  }
    Trajectory trajectory;
    trajectory.positions = digPointsFused;
    trajectory.orientations = digOrientationsFused;
    trajectory.scoopedVolume = volume;
    trajectory.workspaceVolume = workspaceVolume;
    trajectory.startDistanceFromBase = (w_P_wd_off.head(2) - w_P_wba.head(2)).norm();
    trajectory.endDistanceFromBase = (w_P_wd_last.head(2) - w_P_wba.head(2)).norm();
    trajectory.relativeHeading = relativeHeading;
    // compute total trajectory length summing the distance between the points of the trajectory
    trajectory.length = 0;
    for (size_t i = 0; i < digPointsFused.size() - 1; i++) {
      trajectory.length += (digPointsFused[i] - digPointsFused[i + 1]).norm();
    }
    trajectory.stepVolumes = stepVolumes;
    //  ROS_INFO_STREAM("[LocalPlanner]: completed trajectory starting at" << w_P_wd_off.transpose() << " with heading " << heading);

    // set the trajectory
    //  this->publishDesiredShovelPose(w_P_wd, R_ws_d1);
    //  this->publishDesiredShovelPose(w_P_wd3, R_ws_d3);
    //  this->publishTrajectoryPoses(digPoints, digOrientations);
    //  this->publishMarkersTrajectory(digPointsFused, "map");
    // reset elevation mapping
    //  unique_lock lock(mapMutex_);
    //  planningMap_["planning_elevation"] = planningMap_["elevation"];
    //  lock.unlock();
    return trajectory;
  }


  std::vector<Eigen::Vector3d> LocalPlanner::smoothZCoordinates(std::vector<Eigen::Vector3d> &digPoints) {
    // print dig points
    //  ROS_INFO_STREAM("[LocalPlanner]: digPoints size " << digPoints.size());
    // subsample the trajectory to get a smoother trajectory, pick one every three points
    std::vector<Eigen::Vector3d> digPointsSubsampled;
    for (size_t i = 0; i < digPoints.size(); i += 2) {
      digPointsSubsampled.push_back(digPoints[i]);
      // if final point is not subsampled, add it to the subsampled vector
      if (i == digPoints.size() - 1) {
        digPointsSubsampled.push_back(digPoints[i]);
      }
    }

    // first we filter out outliers based on the previous and next point
    // the z coordinate cannot be higher or lower than both the previous and next point ones
    // in this case take the average of the previous and next point z coordinates
    std::vector<Eigen::Vector3d> digPointsFiltered;
    digPointsFiltered.push_back(digPointsSubsampled[0]);
    for (size_t i = 1; i < digPointsSubsampled.size() - 1; i++) {
      Eigen::Vector3d digPoint = digPointsSubsampled[i];
      Eigen::Vector3d digPointPrevious = digPointsSubsampled[i - 1];
      Eigen::Vector3d digPointNext = digPointsSubsampled[i + 1];
      if (digPoint(2) > digPointPrevious(2) && digPoint(2) > digPointNext(2)) {
        digPoint(2) = (digPointPrevious(2) + digPointNext(2)) / 2;
      } else if (digPoint(2) < digPointPrevious(2) && digPoint(2) < digPointNext(2)) {
        digPoint(2) = (digPointPrevious(2) + digPointNext(2)) / 2;
      }
      digPointsFiltered.push_back(digPoint);
    }
    // use a low pass filter to smooth the z coordinates
    std::vector<Eigen::Vector3d> digPointsSmoothed;
    for (size_t i = 1; i < digPointsFiltered.size(); i++) {
      Eigen::Vector3d &p = digPointsFiltered[i];
      Eigen::Vector3d &p_prev = digPointsFiltered[i - 1];
      Eigen::Vector3d p_smoothed = digPointsFiltered[i];
      p_smoothed(2) = (p(2) + p_prev(2)) / 2;  // integrator
      digPointsSmoothed.push_back(p_smoothed);
    }
    return digPointsSmoothed;
  }

  bool LocalPlanner::updateShovelCollisionBody(Eigen::Vector3d w_P_ws, Eigen::Quaterniond C_ws) {
    // get transform from world to base frame with tf
    geometry_msgs::TransformStamped T_bw;
    try {
      T_bw = tfBuffer_->lookupTransform("map", "BASE", ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("[LocalPlanner]: %s", ex.what());
      ros::Duration(1.0).sleep();
    }
    Eigen::Quaterniond C_wb(T_bw.transform.rotation.w, T_bw.transform.rotation.x, T_bw.transform.rotation.y,
                            T_bw.transform.rotation.z);
    Eigen::Vector3d w_P_wb(T_bw.transform.translation.x, T_bw.transform.translation.y,
                           T_bw.transform.translation.z);
//    ROS_INFO_STREAM("[LocalPlanner]: w_P_wb " << w_P_wb.transpose());
    // transform w_P_ws and C_sw to w_P_bs and C_bs
    Eigen::Vector3d b_P_bs = C_wb.inverse() * (w_P_ws - w_P_wb);
    Eigen::Quaterniond C_bs = C_wb.inverse() * C_ws;

    // print w_P_ws
//      ROS_INFO_STREAM("[LocalPlanner]: b_P_bs " << b_P_bs.transpose());
    shovelBodyPtr_->setPositionInWorld(kindr::Position3D(b_P_bs(0), b_P_bs(1), b_P_bs(2)));
    // transform the eigen quaternion into an eigen rotation matrix
    Eigen::Matrix3d R_bs = C_bs.toRotationMatrix();
    kindr::RotationMatrixD R_bs_k = kindr::RotationMatrixD(R_bs(0, 0), R_bs(0, 1), R_bs(0, 2), R_bs(1, 0),
                                                           R_bs(1, 1), R_bs(1, 2),
                                                           R_bs(2, 0), R_bs(2, 1), R_bs(2, 2));
    shovelBodyPtr_->setOrientationInWorld(R_bs_k);
    return true;
  }


  Trajectory LocalPlanner::getOptimalTrajectory() {
    // this code should not be here
    // this was done to correct the small error caused by assuming the direction of the arm to be from the
    // base to the shovel instead of boom to shovel

    //  this->publishTrajectoryPoses(optimalDigTrajectory_.positions, optimalDigTrajectory_.orientations);
    //  Eigen::Vector3d w_P_wd = optimalDigTrajectory_.positions.at(0);
    //  // get normal at dig point
    //  // w_P_wd is the digging point
    //  // point below the surface
    //  grid_map::Position wg_P_wd(w_P_wd(0), w_P_wd(1));
    //  grid_map::Index wg_index;
    //  planningMap_.getIndex(wg_P_wd, wg_index);
    //  double elevation = excavationMappingPtr_->getElevation(wg_P_wd);
    //  double desiredElevation = planningMap_.at("desired_elevation", wg_index);
    //  // from w_P_wd we want compute the trajectory that scoops up most volume
    //  // we procede in this way:
    //  // 1. start tracing the trajectory from w_P_wd
    //  // 2. while tracing the trajectory:
    //  //    - if the trajectory is not valid, we stop tracing it i.e we check if shovel is full and if we reached the boundary of the
    //  working
    //  //    space
    //  //    - if the trajectory is valid, we compute the next point in the trajectory and the current volume for the trajectory
    //  // 3. when trajectory is not valid we stop and close the trajector
    //  Eigen::Vector3d w_P_wb;
    //  // get the transform between the ENDEFFECTOR_CONTACT frame and the BOOM frame using tf2
    //  geometry_msgs::TransformStamped T_mba;
    //  // get transform from base to cabin frame
    //  try {
    //    T_mba = tfBuffer_->lookupTransform("map", "BASE", ros::Time(0));
    //  } catch (tf2::TransformException& ex) {
    //    ROS_WARN("%s", ex.what());
    //    ros::Duration(1.0).sleep();
    //  }
    //  // get the position of the BASE in the map frame
    //  Eigen::Vector3d w_P_wba = Eigen::Vector3d(T_mba.transform.translation.x, T_mba.transform.translation.y,
    //  T_mba.transform.translation.z);
    //  //  ROS_INFO_STREAM("Base origin in map frame: " << w_P_wba.transpose());
    //  double roll_b, pitch_b, yaw_b;
    //  tf2::Quaternion R_mba_q =
    //      tf2::Quaternion(T_mba.transform.rotation.x, T_mba.transform.rotation.y, T_mba.transform.rotation.z, T_mba.transform.rotation.w);
    //  tf2::Matrix3x3(R_mba_q).getRPY(roll_b, pitch_b, yaw_b);
    //  // base to digging point in world frame
    //  Eigen::Vector3d w_P_bad = w_P_wd - w_P_wba;
    //  //  ROS_INFO_STREAM("[LocalPlanner]: digging point wrt base in world frame: " << w_P_bad.transpose());
    //  // transform from world to base frame using R_mba
    //  // convert R_mba to eigen quaternion
    //  Eigen::Quaterniond R_mba_qe(R_mba_q.w(), R_mba_q.x(), R_mba_q.y(), R_mba_q.z());
    //  // transform w_P_bad from world to base frame using R_mba
    //  Eigen::Vector3d ba_P_bad = R_mba_qe.inverse() * w_P_bad;
    //
    //  // relative heading
    //  //  ROS_INFO_STREAM("[LocalPlanner]: digging point wrt base in base frame: " << ba_P_bad.transpose());
    //  double relativeHeading = atan2(ba_P_bad(1), ba_P_bad(0));
    //  //  ROS_INFO_STREAM("Base heading in map frame: " <<  yaw_b);
    //  //  ROS_INFO_STREAM("[LocalPlanner]: opt traj relative heading is " << relativeHeading);
    //  double heading = -yaw_b - relativeHeading;
    //  //  ROS_INFO_STREAM("[LocalPlanner]: opt traj heading " << heading);
    //
    //  //  ROS_INFO_STREAM("[LocalPlanner]: True boom heading " << shovelYaw);
    //  // transform the yaw angle into a direction vector
    //  // get perpendicular vector in 2D (is it 2D?)
    //  Eigen::Vector3d w_P_dba = (-w_P_wd + w_P_wba).normalized();
    //  //  this->publishHeading(w_P_wd, w_P_dba, "map");
    //  Eigen::Vector3d C_ws = this->findShovelDesiredOrientation(w_P_wd, w_P_dba);
    //  //  ROS_INFO_STREAM("[LocalPlanner]: Shovel desired orientation: " << C_ws.transpose());
    this->publishTrajectoryPoses(optimalDigTrajectory_.positions, optimalDigTrajectory_.orientations);
    return optimalDigTrajectory_;
  }

  double LocalPlanner::volumeObjective(Trajectory trajectory) {
//    ROS_INFO_STREAM("[LocalPlanner]: remaining volume ratio: " << remainingVolumeRatio_);
//    // start distance
//    ROS_INFO_STREAM("[LocalPlanner]: start distance: " << trajectory.startDistanceFromBase);
//    // heading
//    ROS_INFO_STREAM("[LocalPlanner]: heading: " << trajectory.relativeHeading);
    double workspaceCost = volumeWeight_ * trajectory.workspaceVolume;
    // as volume is removed more importance is given to the workspace volume
    double distanceWeight = remainingVolumeRatio_ * distanceWeight_ *
                            (trajectory.startDistanceFromBase - circularWorkspaceInnerRadius_) /
                            (circularWorkspaceOuterRadius_ - circularWorkspaceInnerRadius_);
    double headingWeight =
        remainingVolumeRatio_ * headingWeight_ * trajectory.relativeHeading / (circularWorkspaceAngle_ / 2.0);
    double objective = workspaceCost * (1 + distanceWeight + headingWeight);
    return objective;
  }

  void LocalPlanner::optimizeTrajectory() {
    this->updatePlanningMap();
    ROS_INFO_STREAM("[LocalPlanner]: Optimizing trajectory");
    //  ROS_INFO_STREAM("[LocalPlanner]: Planning map updated");
    // iterate over all the points belonging to the workspace polygon and compute a feasable trajectory starting from each point
    // and find the point that maximizes the objective function
    // check that digZoneId_ is between 0 and and 2
    // print available layers of planningMap_
    //  for (auto layer : planningMap_.getLayers()) {
    //    ROS_INFO_STREAM("[LocalPlanner]: Planning map layer: " << layer);
    //  }
    std::string targetLayer;
    if (digZoneId_ < 0 || digZoneId_ > 2) {
      ROS_ERROR("[LocalPlanner]: cannot compute optimal trajectory, digZoneId_ is not between 0 and 2");
      return;
    }
    if (digZoneId_ == 0) {
      targetLayer = "desired_elevation";
    } else if (digZoneId_ == 1 || digZoneId_ == 2) {
      targetLayer = "original_elevation";
    }
    //  ROS_INFO_STREAM("[LocalPlanner]: Optimizing trajectory for digZoneId_ " << digZoneId_);
    //  ROS_INFO_STREAM("[LocalPlanner]: planning zones " << planningZones_.size());
    Trajectory bestTrajectory;
    double maxObjective = -10;
    for (grid_map::PolygonIterator iterator(planningMap_,
                                            planningZones_.at(digZoneId_)); !iterator.isPastEnd(); ++iterator) {
      // get the position of the point
      grid_map::Position diggingPoint;
      planningMap_.getPosition(*iterator, diggingPoint);
      // get the elevation of the point
      double elevation = excavationMappingPtr_->getElevation(diggingPoint);
      // if the elevation is not nan compute the trajectory
      if (!std::isnan(elevation)) {
        // create the point in 3d
        Eigen::Vector3d w_P_wd(diggingPoint.x(), diggingPoint.y(), elevation);
        Trajectory trajectory = this->computeTrajectory(w_P_wd, targetLayer, digZoneId_);
        // print volume
//        ROS_INFO_STREAM("[LocalPlanner]: Volume " << trajectory.workspaceVolume);
        double objective = this->volumeObjective(trajectory);
//        ROS_INFO_STREAM("[LocalPlanner]: Objective " << objective);
        if (objective > maxObjective && trajectory.workspaceVolume > 0.02) {
          bestTrajectory = trajectory;
          maxObjective = objective;
        }
      }
    }
    ROS_INFO_STREAM("[LocalPlanner]: Best trajectory has volume " << bestTrajectory.workspaceVolume);
    //  // print best trajectory relative heading nad distance from base
    ROS_INFO_STREAM("[LocalPlanner]: Best trajectory has relative heading " << bestTrajectory.relativeHeading);
    ROS_INFO_STREAM("[LocalPlanner]: Best trajectory has distance from base " << bestTrajectory.startDistanceFromBase);
    double workspaceCost = volumeWeight_ * bestTrajectory.workspaceVolume;
    double distanceWeight = remainingVolumeRatio_ * distanceWeight_ * distanceWeight_ *
                            (bestTrajectory.startDistanceFromBase - circularWorkspaceInnerRadius_) /
                            (circularWorkspaceOuterRadius_ - circularWorkspaceInnerRadius_);
    double headingWeight = remainingVolumeRatio_ * headingWeight_ * bestTrajectory.relativeHeading /
                           (circularWorkspaceAngle_ / 2.0);
    ROS_INFO_STREAM("[LocalPlanner]: remainingVolumeRatio_ " << remainingVolumeRatio_);
    ROS_INFO_STREAM("[LocalPlanner]: Best trajectory has volume cost " << workspaceCost);
    ROS_INFO_STREAM("[LocalPlanner]: Best trajectory has distance weight " << distanceWeight);
    ROS_INFO_STREAM("[LocalPlanner]: Best trajectory has heading weight " << headingWeight);
    ROS_INFO_STREAM("[LocalPlanner]: Best trajectory has length " << bestTrajectory.positions.size());
    // raise a warning if the found trajectory is empty
    if (bestTrajectory.positions.size() == 0) {
      completedDigAreas_.at(digZoneId_) = 1;
      ROS_WARN_STREAM("[LocalPlanner]: could not find a valid trajectory in digZoneId_ " << digZoneId_);
    }
    // print expected volume for the best trajectory
    ROS_INFO_STREAM(
        "[LocalPlanner]: ---------------------------- Expected volume " << bestTrajectory.workspaceVolume);
    // print trajectory step volumes
    optimalDigTrajectory_ = bestTrajectory;
  }

  void LocalPlanner::getLayerHeightAlongBoom(std::string layer, std::vector<double> &layerValues,
                                             std::vector<double> &distanceBoom) {
    // get the position of the ENDEFFECTOR_CONTACT (symbol s) in map frame (symbol w) using tf
    geometry_msgs::TransformStamped T_bm;
    try {
      T_bm = tfBuffer_->lookupTransform("map", "BOOM", ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    geometry_msgs::TransformStamped T_sm;
    try {
      T_sm = tfBuffer_->lookupTransform("map", "ENDEFFECTOR_CONTACT", ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    // get vector from the shovel to the base in base map frame
    Eigen::Vector3d w_P_sw = Eigen::Vector3d(T_sm.transform.translation.x, T_sm.transform.translation.y,
                                             T_sm.transform.translation.z);
    Eigen::Vector3d w_P_bw = Eigen::Vector3d(T_bm.transform.translation.x, T_bm.transform.translation.y,
                                             T_bm.transform.translation.z);
    Eigen::Vector3d w_P_sb = w_P_sw - w_P_bw;
    Eigen::Vector2d w_P_sb_projected = w_P_sb.head(2);
    Eigen::Vector2d w_P_bw_projected = w_P_bw.head(2);
    Eigen::Vector2d w_P_sw_projected = w_P_sw.head(2);
    // get the distance from the base to the shovel
    double distance = w_P_sb_projected.norm();
    // normalize to get the direction
    w_P_sb_projected.normalize();
    // sample the distance between the boom and the shovel
    double step = distance / (layerValues.size() - 1);
    // we will publish these points
    std::vector<Eigen::Vector3d> w_P_pts;
    for (int i = 0; i < layerValues.size(); i++) {
      // position to query
      Eigen::Vector2d nextPosition = w_P_bw_projected + i * step * w_P_sb_projected;
      // get the elevation at the position
      double elevation = excavationMappingPtr_->getValueAtLayer(nextPosition, layer);
      // if the elevation is not nan add it to the vector
      if (!std::isnan(elevation)) {
        layerValues.at(i) = elevation;
        distanceBoom.at(i) = i * step;
        w_P_pts.push_back(Eigen::Vector3d(nextPosition(0), nextPosition(1), elevation));
      }
    }
    // publish the points
    this->publishMarkersTrajectory(w_P_pts, "map");
  }

  double LocalPlanner::getShovelHeightFromLayer(std::string layer) {
    geometry_msgs::TransformStamped T_sm;
    try {
      T_sm = tfBuffer_->lookupTransform("ENDEFFECTOR_CONTACT", "map", ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    // get vector from the shovel to the base in base map frame
    Eigen::Vector3d w_P_sw = Eigen::Vector3d(T_sm.transform.translation.x, T_sm.transform.translation.y,
                                             T_sm.transform.translation.z);
    // get the elevation at the position
    double elevation = excavationMappingPtr_->getValueAtLayer(w_P_sw.head(2), layer);
    // if the elevation is not nan add it to the vector
    return w_P_sw(2) - elevation;
  }

  double LocalPlanner::getVolume() {
    return optimalDigTrajectory_.workspaceVolume;
  }

  bool LocalPlanner::isDigZoneComplete(int zoneId) {
    // assert index between 0 and 2
    if (zoneId < 0 || zoneId > 2) {
      ROS_ERROR_STREAM("[LocalPlanner]: Invalid zone id " << zoneId);
      return false;
    }
    if (completedDigAreas_.at(zoneId) == 1) {
      return true;
    }
    // print height theshold
    ROS_INFO_STREAM("[LocalPlanner]: height threshold " << heightThreshold_);
    bool completed = false;
    // sum up all the remaining volume in the digZone and check two conditions:
    // 1. the sum is smaller than the volumeTreshold
    // 2. difference in target height in each cell is lower than the heightTreshold
    double volume = 0;
    double totalVolume = 0;
    int totalNumCells = 0;
    int numMissingCells = 0;
    // print zone id
    // print length of planning zones and the current zone id
    ROS_INFO_STREAM(
        "[LocalPlanner]: length of planning zones " << planningZones_.size() << " current zone id " << zoneId);
    for (grid_map::PolygonIterator iterator(planningMap_,
                                            planningZones_.at(zoneId)); !iterator.isPastEnd(); ++iterator) {
      // get the position of the point
      grid_map::Index index(*iterator);
      // print position
      // grid_map::Position position;
      // planningMap_.getPosition(index, position);
      // // print position
      // ROS_INFO_STREAM("[LocalPlanner]: position " << position);

      // get the elevation of the point
      double elevation = planningMap_.at("elevation", index);  // this updated between one scoop and the next.
      double heightDifference;
      double originalHeightDifference;
      if (planningMap_.at("current_excavation_mask", index) == -1) {
        if (zoneId == 0) {
          // get the desired elevation of the point
          double desiredElevation = planningMap_.at("desired_elevation", index);
          double originalElevation = planningMap_.at("predig_elevation", index);
          // prevents maps inaccuracies from causing the volumeRatio to be bigger than 1
          if (elevation > originalElevation) {
            originalElevation = elevation;
          }

          //  ROS_INFO_STREAM("[LocalPlanner]: desired elevation " << desiredElevation << " elevation " << elevation);
          // get soil volume remaining
          heightDifference = std::max(0.0, elevation - desiredElevation);
          //        ROS_INFO_STREAM("[LocalPlanner]: height difference " << heightDifference);
          originalHeightDifference = std::max(0.0, originalElevation - desiredElevation);
          // if heightDifference or originalHeightDifference is nan skip
          if (std::isnan(heightDifference) || std::isnan(originalHeightDifference)) {
            continue;
          }
          //        ROS_INFO_STREAM("[LocalPlanner]: height difference " << heightDifference);
          //        ROS_INFO_STREAM("[LocalPlanner]: original height difference " << originalHeightDifference);
          if (heightDifference > heightThreshold_) {
            numMissingCells++;
          }
          double deltaOriginalVolume =
              originalHeightDifference * planningMap_.getResolution() * planningMap_.getResolution();
          double deltaSoilVolume =
              heightDifference * planningMap_.getResolution() * planningMap_.getResolution();
          volume += deltaSoilVolume;
          totalVolume += deltaOriginalVolume;
        } else if (zoneId == 1 || zoneId == 2) {
          double desiredElevation = excavationMappingPtr_->gridMap_.at("original_elevation", index);
//          ROS_INFO_STREAM("[LocalPlanner]: desired elevation " << desiredElevation << " elevation " << elevation);
          heightDifference = std::max(0.0, elevation - desiredElevation);
          if (std::isnan(heightDifference)) {
            continue;
          }
//                  ROS_INFO_STREAM("[LocalPlanner]: height difference " << heightDifference);
          //        originalHeightDifference = std::max(0.0, planningMap_.at("original_elevation", index) - desiredElevation);
          if (heightDifference > heightDirtThreshold_) {
            numMissingCells++;
          }
          double deltaSoilVolume =
              heightDifference * planningMap_.getResolution() * planningMap_.getResolution();
          volume += deltaSoilVolume;
        }
        totalNumCells++;
      }
    }

      ROS_INFO_STREAM("[LocalPlanner]: Volume remaining in zone " << zoneId << " is " << volume);
    if (zoneId > 0){
      ROS_INFO_STREAM("[LocalPlanner]: Total volume to remove is " << digZonesVolume_.at(zoneId));
    }
    //  ROS_INFO_STREAM("[LocalPlanner]: Total volume remaining is " << totalVolume);
    //  ROS_INFO_STREAM("[LocalPlanner]: Total number of cells is " << totalNumCells);
    //  ROS_INFO_STREAM("[LocalPlanner]: Number of missing cells is " << numMissingCells);
    //  ROS_INFO_STREAM("[LocalPlanner]: Volume ratio: " << volume / totalVolume);
    //  ROS_INFO_STREAM("[LocalPlanner]: Number of missing cells ratio: " << (double) numMissingCells / totalNumCells);
    if (zoneId == 0) {
      remainingVolumeRatio_ = volume / totalVolume;
    } else {
      remainingVolumeRatio_ = volume / digZonesVolume_.at(zoneId);
    }
    ROS_INFO_STREAM("[LocalPlanner]: Workspace volume " << workspaceVolume_);
    // if nan throw error
    if (std::isnan(remainingVolumeRatio_)) {
      ROS_ERROR_STREAM("[LocalPlanner]: Volume ratio is nan");
      return true;
    } else if (remainingVolumeRatio_ < 0.0) {
      ROS_WARN("[LocalPlanner]: Volume ratio is negative");
      remainingVolumeRatio_ = 0.0;
    } else if (remainingVolumeRatio_ > 1.0) {
      ROS_WARN("[LocalPlanner]: Volume ratio is bigger than 1");
      remainingVolumeRatio_ = 1.0;
    }
    double missingCellsRatio = (double) numMissingCells / totalNumCells;
    ROS_INFO_STREAM(
        "[LocalPlanner]: missing num cells " << numMissingCells << " total num cells " << totalNumCells);
    ROS_INFO_STREAM("#########################");
    ROS_INFO_STREAM("[LocalPlanner]: missing cell ratio: " << missingCellsRatio);
    ROS_INFO_STREAM("[LocalPlanner]: Volume ratio: " << remainingVolumeRatio_);
    ROS_INFO_STREAM("#########################");
    if (digZoneId_ == 0) {
      completed = remainingVolumeRatio_ < volumeThreshold_ && missingCellsRatio < missingCellsThreshold_;
      if (completed) {
        // if the map remainingVolumeRatio_ does not have a value for the current waypointIndex_, set it to the current
        // remainingVolumeRatio_
        if (remainingVolumeRatios_.find(waypointIndex_) == remainingVolumeRatios_.end()) {
          remainingVolumeRatios_.insert(std::pair<int, double>(waypointIndex_, remainingVolumeRatio_));
        }
        if (remainingVolume_.find(waypointIndex_) == remainingVolume_.end()) {
          remainingVolume_.insert(std::pair<int, double>(waypointIndex_, volume));
        }
        if (remainingCellsRatio_.find(waypointIndex_) == remainingCellsRatio_.end()) {
          remainingCellsRatio_.insert(std::pair<int, double>(waypointIndex_, missingCellsRatio));
        }
      }
    } else {
      completed = missingCellsRatio < missingCellsThreshold_;
    }
    //  if (completed) {
    //    ROS_INFO_STREAM("[LocalPlanner]: Dig zone " << zoneId << " is completed!");
    //  }
    return completed;
  }

  void LocalPlanner::checkScoopedVolume(double volume) {
    if (volume < minScoopVolume_){
      ROS_WARN("[LocalPlanner]: Scooped volume is less than minimum threshold volume");
      lowVolumeScoopCounter_ += 1;
    } else {
      lowVolumeScoopCounter_ = 0;
    }
    if (lowVolumeScoopCounter_ > lowVolumeScoopAttempts_){
      ROS_WARN("[LocalPlanner]: Scooped volume is less than minimum threshold volume for too long, dig zone is completed");
      lowVolumeScoopCounter_ = 0;
      completedDigAreas_.at(digZoneId_) = 1;
    }
  }

  bool LocalPlanner::isLocalWorkspaceComplete() {
    // if both dig zone is -1 then the local workspace is complete
    // time each step with chrono
    auto start = std::chrono::high_resolution_clock::now();
    this->updatePlanningMap();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    ROS_INFO_STREAM("[LocalPlanner]: updatePlanningMap took " << duration.count() << " microseconds");
    start = std::chrono::high_resolution_clock::now();
    if (createNewZones_) {
      // empty the planning zones vector
      planningZones_.clear();
      this->createPlanningZones();
    }
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    ROS_INFO_STREAM("[LocalPlanner]: createPlanningZones took " << duration.count() << " microseconds");
    start = std::chrono::high_resolution_clock::now();
    // updating the working ares is necessary to choose the right zone
    this->updateWorkingArea();
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    ROS_INFO_STREAM("[LocalPlanner]: updateWorkingArea took " << duration.count() << " microseconds");
    start = std::chrono::high_resolution_clock::now();
    this->updateDugZones();
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    ROS_INFO_STREAM("[LocalPlanner]: updateDugZones took " << duration.count() << " microseconds");
    start = std::chrono::high_resolution_clock::now();
    this->choosePlanningZones();
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    ROS_INFO_STREAM("[LocalPlanner]: choosePlanningZones took " << duration.count() << " microseconds");
    //  ROS_INFO_STREAM("[LocalPlanner]: Local workspace is not complete!");
    if (digZoneId_ == -1) {
      ROS_INFO_STREAM("[LocalPlanner]: Local workspace is complete!");
      createNewZones_ = true;
      return true;
    } else {
      return false;
    }
    // create a new thread and call the function logWorkspaceData
    std::thread logWorkspaceDataThread(std::bind(&LocalPlanner::logWorkspaceData, this));
    logWorkspaceDataThread.detach();
  }

  bool LocalPlanner::isLateralFrontZoneComplete(int zoneId) {
    // check that the lateral front zones are clear of dirt and that the elevation of the terrain matches the original elevation of the
    // terrain assert zoneId is 1 or 2
    assert(zoneId == 1 || zoneId == 2);
    // iterate over the lateral front zone
    for (grid_map::PolygonIterator iterator(planningMap_,
                                            planningZones_.at(zoneId)); !iterator.isPastEnd(); ++iterator) {
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

  void LocalPlanner::createShovelFilter() {
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

  int LocalPlanner::chooseDigZone() {
    if (autoZoneSelection_) {
      digZoneId_ = -1;
      if (this->isZoneActive(0, true) && !this->isDigZoneComplete(0)) {
        digZoneId_ = 0;
      } else {
        for (int i = 1; i < 3; i++) {
          if (this->isZoneActive(i, true)) {
            ROS_INFO_STREAM("[LocalPlanner]: Digging Zone " << i << " is active!");
            if (!this->isDigZoneComplete(i)) {
              digZoneId_ = i;
              break;
            } else {
              ROS_INFO_STREAM("[LocalPlanner]: Dig zone " << i << " is complete");
            }
          }
        }
      }
    } else {
      if (this->isDigZoneComplete(digZoneId_)) {
        digZoneId_ = -1;
      }
    }
    return digZoneId_;
  }

  int LocalPlanner::chooseDumpZone(int digZoneId) {
    if (autoZoneSelection_) {
      // select the dumping zone we check zones 1, 2, 3, 4 to see if they are active.
      // we then sort the active zones based on their dumpingScore
      // we then select the zone with the highest dumpingScore
      dumpZoneId_ = -1;
      // set dumpingScore to max
      double dumpingScore = std::numeric_limits<double>::max();
      for (int i = 1; i < 5; i++) {
        if (i != digZoneId) {
          bool zoneActive = this->isZoneActive(i, false) && not completedDumpAreas_.at(i - 1);
          ROS_INFO_STREAM("[LocalPlanner]: Dumping Zone " << i << " is active: " << zoneActive);
          if (zoneActive && digZoneId != -1) {
            double zoneDumpingScore = this->getDumpingScore(i);
            ROS_INFO_STREAM("[LocalPlanner]: Dumping Zone " << i << " has dumping score: " << zoneDumpingScore);
            if (zoneDumpingScore < dumpingScore) {
              dumpingScore = zoneDumpingScore;
              dumpZoneId_ = i;
            }
          }
        }
      }
    }
    return dumpZoneId_;
  }

  void LocalPlanner::choosePlanningZones() {
    // this function selects the dig and dump zones.
    // we first select the dig area between 0, 1, 2 zones
    // dig zone 0 has priority if it's not completed
    digZoneId_ = this->chooseDigZone();

    // select the dumping zone we check zones 1, 2, 3, 4 to see if they are active.
    // we then sort the active zones based on their dumpingScore
    // we then select the zone with the highest dumpingScore
    dumpZoneId_ = this->chooseDumpZone(digZoneId_);
    // if the dig zone changed from the previous iteration, then we need to update the current workspace volume
    if (digZoneId_ != previousDigZoneId_) {
      previousDigZoneId_ = digZoneId_;
      workspaceVolume_ = this->computeWorkspaceVolume(digZoneId_, this->getTargetDigLayer(digZoneId_));
    }
    // safety checks
    if (digZoneId_ != -1) {
      if (dumpZoneId_ != -1) {
        ROS_INFO("[LocalPlanner]: dig zone %d, dump zone %d", digZoneId_, dumpZoneId_);
      } else {
        ROS_ERROR("[LocalPlanner]: dig zone %d, dump zone not selected", digZoneId_);
        // quit the node
        ros::shutdown();
      }
    } else {
      ROS_WARN("[LocalPlanner]: dig zone not selected, dump zone %d", dumpZoneId_);
    }
    // dump and dig zone cannot be the same
    if (digZoneId_ == -1) {
      if (digZoneId_ == dumpZoneId_) {
        ROS_WARN("[LocalPlanner]: dig and dump zone not selected. Finished working here.");
      }
    } else {
      if (digZoneId_ == dumpZoneId_) {
        ROS_ERROR("[LocalPlanner]: dig zone and dump zone cannot be the same");
        // quit the no
        ros::shutdown();
      }
    }
    // publish the updated map
    if (digZoneId_ != -1 && dumpZoneId_ != -1) {
      this->markDigDumpAreas();
    }
  }

  bool LocalPlanner::isZoneActive(int zoneId, bool isDigging) {
    // a zone is considered active if it does not overlap with an already dug area
    // a dug areas in marked in the layer "dug_area" of the planning map with a value of 1
    double numDugCells = 0;
    double numCellsToBeDug = 0;
    double numCellsExcavationArea = 0;
    double totalCells = 0;
    bool active = true;
    for (grid_map::PolygonIterator iterator(planningMap_,
                                            planningZones_.at(zoneId)); !iterator.isPastEnd(); ++iterator) {
      // get the position of the point
      grid_map::Index index(*iterator);
      // get the elevation of the point
      double dugValue = planningMap_.at("dug_area", index);
      double excavationZone = planningMap_.at("current_excavation_mask", index);
      totalCells++;
      if (planningMap_.at("working_area", index) == 1) {
        numCellsToBeDug++;
      }
      if (dugValue == 1) {
        numDugCells++;
      }
      if (excavationZone == -1) {
        numCellsExcavationArea++;
      }
    }
    if (isDigging) {
      // ratio of cells to be dug in the zone
      //    ROS_INFO_STREAM("[LocalPlanner]: Dig zone " << zoneId << " has excavation cell ratio " << (double) numCellsExcavationArea /
      //    totalCells); ROS_INFO_STREAM("[LocalPlanner]: It should be bigger than " << (1 - inactiveAreaRatio_));
      active = active && ((double) numCellsExcavationArea / totalCells) > (1 - inactiveAreaRatio_);
    } else {
      // number of already dug cells in the zone
      //    ROS_INFO_STREAM("[LocalPlanner]: Dump zone " << zoneId << " has dug cell ratio " << (double) numCellsToBeDug / totalCells);
      //    ROS_INFO_STREAM("[LocalPlanner]: It should be smaller than " <<  inactiveAreaRatio_);
      active = active && ((double) numCellsToBeDug / totalCells) < inactiveAreaRatio_;
    }

    //  if (isDigging){
    //    for (grid_map::PolygonIterator iterator(planningMap_, planningZones_.at(zoneId)); !iterator.isPastEnd(); ++iterator) {
    //      // get the position of the point
    //      grid_map::Index index(*iterator);
    //      // get the elevation of the point
    //      double dugValue = planningMap_.at("working_area", index);
    //      double excavationZone = planningMap_.at("current_excavation_mask", index);
    //      totalCells++;
    //      if (dugValue == 1) {
    //        numDugCells++;
    //      }
    //      if (excavationZone == -1) {
    //        numCellsExcavationArea++;
    //      }
    //    }
    //    active = (double) numCellsExcavationArea / totalCells > 0.1;
    //  }
    //  for (grid_map::PolygonIterator iterator(planningMap_, planningZones_.at(zoneId)); !iterator.isPastEnd(); ++iterator) {
    //    // get the position of the point
    //    grid_map::Index index(*iterator);
    //    // get the elevation of the point
    //    double dugValue = planningMap_.at("working_area", index);
    //    double excavationZone = planningMap_.at("current_excavation_mask", index);
    //    totalCells++;
    //    if (dugValue == 1) {
    //      numDugCells++;
    //    }
    //    if (excavationZone == -1) {
    //      numCellsExcavationArea++;
    //    }
    //  }
    //  active = (double) numDugCells / totalCells < inactiveAreaRatio_;
    //
    //  if (!isDigging){
    //    if (zoneId == 1 || zoneId == 2) {
    //      active = active && numCellsExcavationArea / totalCells < inactiveAreaRatio_;
    //    }
    //  }
    //  ROS_INFO_STREAM("[LocalPlanner]: Zone " << zoneId << " is active: " << active << " (" << numDugCells << "/" << totalCells << ")" <<
    //  " digging: " << isDigging);
    if (active) {
      for (grid_map::PolygonIterator iterator(planningMap_,
                                              planningZones_.at(zoneId)); !iterator.isPastEnd(); ++iterator) {
        // get the position of the point
        grid_map::Index index(*iterator);
        if (isDigging) {
          planningMap_.at("active_dig_zones", index) = 1;
          if (zoneId == 0 && planningMap_.at("current_excavation_mask", index) == -1) {
            planningMap_.at("working_area", index) = 1;
            planningMap_.at("dug_area", index) = 1;
          }
        } else {
          planningMap_.at("active_dump_zones", index) = 1;
        }
      }
    }
    return active;
  }

  void LocalPlanner::markDigDumpAreas() {
    // mark the dig and dump areas in the planning map
    for (grid_map::PolygonIterator iterator(planningMap_,
                                            planningZones_.at(digZoneId_)); !iterator.isPastEnd(); ++iterator) {
      // get the position of the point
      grid_map::Index index(*iterator);
      planningMap_.at("active_dig_zones", index) = 2;
    }
    for (grid_map::PolygonIterator iterator(planningMap_,
                                            planningZones_.at(dumpZoneId_)); !iterator.isPastEnd(); ++iterator) {
      // get the position of the point
      grid_map::Index index(*iterator);
      planningMap_.at("active_dump_zones", index) = 2;
    }
  }

  void LocalPlanner::resetDigDumpAreas() {
    // reset the dig and dump areas in the planning map
    for (int zoneId = 0; zoneId < planningZones_.size(); zoneId++) {
      for (grid_map::PolygonIterator iterator(planningMap_,
                                              planningZones_.at(zoneId)); !iterator.isPastEnd(); ++iterator) {
        // get the position of the point
        grid_map::Index index(*iterator);
        planningMap_.at("active_dig_zones", index) = 0;
      }
      for (grid_map::PolygonIterator iterator(planningMap_,
                                              planningZones_.at(zoneId)); !iterator.isPastEnd(); ++iterator) {
        // get the position of the point
        grid_map::Index index(*iterator);
        planningMap_.at("active_dump_zones", index) = 0;
      }
    }
    createNewZones_ = true;
  }

// this could be quite noisy in the real world but it's good for testing
  void LocalPlanner::updateDugZones() {
    // iterate over the map and check if difference between the desired elevation and the elevation is less than the heightThreshold_
    // if it is, mark the cell as dug
    for (grid_map::GridMapIterator iterator(planningMap_); !iterator.isPastEnd(); ++iterator) {
      // get the position of the point
      grid_map::Index index(*iterator);
      // get the elevation of the point
      double elevation = planningMap_.at("elevation", index);
      double desiredElevation = planningMap_.at("desired_elevation", index);
      double originalElevation = planningMap_.at("original_elevation", index);
      double heightDifference = std::max(0.0, elevation - desiredElevation);
      double heightOriginalDifference = std::max(0.0, originalElevation - desiredElevation);
      int toBeDugArea = planningMap_.at("current_excavation_mask", index);
      if (heightDifference < heightThreshold_ && heightOriginalDifference > targetHeightDiffThreshold_ && toBeDugArea == -1) {
        planningMap_.at("dug_area", index) = 1;
        planningMap_.at("working_area", index) = 1;
      }
    }
  }

  void LocalPlanner::sdfDumpingAreas() {
    //  ROS_INFO("[LocalPlanner]: creating the sdf!");
    geometry_msgs::TransformStamped T_bw;
    std::vector<Eigen::Vector2d> w_PosDigVertex_bd;
    try {
      T_bw = tfBuffer_->lookupTransform("map", "BASE", ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("[LocalPlanner]: %s", ex.what());
      ros::Duration(1.0).sleep();
    }
    Eigen::Quaterniond targetOrientation(T_bw.transform.rotation.w, T_bw.transform.rotation.x,
                                         T_bw.transform.rotation.y,
                                         T_bw.transform.rotation.z);
    Eigen::Vector3d targetPosition(T_bw.transform.translation.x, T_bw.transform.translation.y,
                                   T_bw.transform.translation.z);
    double yawAngle = targetOrientation.toRotationMatrix().eulerAngles(0, 1, 2).z();
    // copy the distance back to the planning map

    geometry_msgs::Pose nextWaypoint = globalPath_[waypointIndex_];
    double roll_b, pitch_b, yaw_b;
    Eigen::Quaterniond q_mw(nextWaypoint.orientation.w, nextWaypoint.orientation.x,
                            nextWaypoint.orientation.y,
                            nextWaypoint.orientation.z);
    // translation vector
    Eigen::Vector3d t_mw(nextWaypoint.position.x, nextWaypoint.position.y,
                         0);
    // transform from map frame to waypoint frame (this is the frame of the waypoint)
    // iterate over the all map
    for (grid_map::GridMapIterator iterator(planningMap_); !iterator.isPastEnd(); ++iterator) {
      // get the position of the point
      grid_map::Index index(*iterator);
      // position
      Eigen::Vector2d position;
      planningMap_.getPosition(*iterator, position);
      Eigen::Vector3d p_mp = Eigen::Vector3d(position.x(), position.y(), 0.0);
      // tranform position from map frame to waypoint frame
      Eigen::Vector3d w_position_wp = q_mw.inverse() * (p_mp - t_mw);
      if ((w_position_wp.x() > circularWorkspaceOuterRadius_ || (w_position_wp.x() > 0) && w_position_wp.norm() > 1.2 * dumpingZoneOuterRadius_) &&
          planningMap_.at("current_excavation_mask", index) == 1) {
        planningMap_.at("current_excavation_mask", index) = 0;
      }
    }
    // now recompute the sdf
    this->computeSdf("current_excavation_mask");

    for (grid_map::GridMapIterator iterator(planningMap_); !iterator.isPastEnd(); ++iterator) {
      // get the position of the point
      grid_map::Index index(*iterator);
      // position
      Eigen::Vector2d position;
      planningMap_.getPosition(*iterator, position);
      //   Eigen::Vector3d position3d(position.x(), position.y(), 0.5);
      //   // get the elevation of the point
      //   double distance = sdf.getDistanceAt(position3d);
      double distance;
      // this means you can dump there so it gets a low score/distance
      if (planningMap_.at("current_excavation_mask", index) == 1) {
        distance = -1;
      } else if (planningMap_.at("dug_area", index) == 1) {
        distance = 1;
      } else {
          // this commented out section is an analitical sdf for the case of a square
          // if yaw angle is negative then distance is linear function starting from 10 at y=10 and going to 0 at y=-10
          // if yaw angle is positive then distance is linear function starting from 10 at y=-10 and going to 0 at y=10
          //      if (yawAngle < 0) {
          //        distance = (5 - position.y() / 2);
          //      } else {
          //        distance = (-5 + position.y() / 2);
          //      }
          //    distance = 0.05 * (10 - abs(position.x())) * (10 - abs(position.y()));

          // compute the signed distance field for the current zone wrt the boundaries of the zone which have excavation_mask = 1
          // grid_map sdf
          // grid_map::SignedDistanceField sdf(planningMap_, "current_excavation_mask");

          distance = planningMap_.at("excavation_sdf", index);
      }
      planningMap_.at("dumping_distance", index) = distance;
      // ROS_INFO_STREAM("[LocalPlanner]: distance at " << index << ": " << distance);
    }
    // ROS_INFO_STREAM("[LocalPlanner]: SDF calculated");
  }

void LocalPlanner::computeSdf(std::string targetLayer) {
  // convert the excavation mask layer, which contains +1, 0, -1 to an occupancy layer by mapping the 0s and -1s to 0s
  grid_map::Matrix layer = planningMap_.get(targetLayer);
  Eigen::Matrix<bool, -1, -1> occupancyLayer(layer.rows(), layer.cols());
  for (int i = 0; i < layer.rows(); i++) {
    for (int j = 0; j < layer.cols(); j++) {
      if (layer(i, j) == 0 || layer(i, j) == -1) {
        occupancyLayer(i, j) = false;
      } else {
        occupancyLayer(i, j) = true;
      }
    }
  }
  // publish the map message every 5 seconds
  grid_map::Matrix sdfLayer = grid_map::signed_distance_field::signedDistanceFromOccupancy(occupancyLayer, planningMap_.getResolution());
  // add the layer to the map
  planningMap_["excavation_sdf"] = sdfLayer;
  ROS_INFO_STREAM("Signed distance field added to map");
}

  double LocalPlanner::getDumpingScore(int zoneId) {
    // the dumping score is proportional to the average distance to the dump zone
    // the distances are stored in the "dumping_distance" layer of the planning map
    //  ROS_INFO_STREAM("[LocalPlanner]: Calculating dumping score for zone " << zoneId);
    geometry_msgs::TransformStamped T_mba;
    // get transform from base to cabin frame
    try {
      T_mba = tfBuffer_->lookupTransform("map", "BASE", ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    // get the position of the BASE in the map frame
    Eigen::Vector3d w_P_wba = Eigen::Vector3d(T_mba.transform.translation.x, T_mba.transform.translation.y,
                                              T_mba.transform.translation.z);
    // from geometric message to tf2 transform
    tf2::Transform T_mba_tf2 = tf2::Transform(
        tf2::Quaternion(T_mba.transform.rotation.x, T_mba.transform.rotation.y, T_mba.transform.rotation.z,
                        T_mba.transform.rotation.w),
        tf2::Vector3(T_mba.transform.translation.x, T_mba.transform.translation.y,
                     T_mba.transform.translation.z));
    tf2::Transform T_bam_tf2 = T_mba_tf2.inverse();

    double score;
    double totalDistance = 0;
    this->sdfDumpingAreas();
    for (grid_map::PolygonIterator iterator(planningMap_,
                                            planningZones_.at(zoneId)); !iterator.isPastEnd(); ++iterator) {
      // get the position of the point
      grid_map::Index index(*iterator);
      // get the elevation of the point
      double distance = planningMap_.at("dumping_distance", index);
      // add the distance to the total distance
      totalDistance += distance;
    }
    double scoreGlocalDistance = dumpingZoneDistanceWeight_ * totalDistance / planningZones_.at(zoneId).getArea();
    // get the center of the zone
    Eigen::Vector2d w_zoneCenter_wc = zoneCenters_.at(zoneId);
    // get the position of the zoneCenter in base frame (w_ZoneCenter_wc -> b_ZoneCenter_bc)
    // does this change the orientation or also the position?
    tf2::Vector3 b_zoneCenter_bc = T_bam_tf2 * tf2::Vector3(w_zoneCenter_wc.x(), w_zoneCenter_wc.y(), 0.0);
    Eigen::Vector2d b_zoneCenter_bc_2d(b_zoneCenter_bc.x(), b_zoneCenter_bc.y());
    //  ROS_INFO_STREAM("[LocalPlanner]: zone " << zoneId << " center in base frame: " << b_zoneCenter_bc_2d << " and in map frame: " <<
    //  w_zoneCenter_wc);
    // this is only used when at the end of coverage line the robot has to decide weather to dump
    // on its back-left or back-right. It should do so in the opposite direction of where it's gonna drive next.
    // dot product between eigen vectors w_zoneCenter_bc and workingDirection_
    // the least the better
    // transform the working direction in base frame
    tf2::Vector3 b_workingDirection_bc = T_bam_tf2 * tf2::Vector3(workingDirection_.x() + T_mba.transform.translation.x, workingDirection_.y() + T_mba.transform.translation.y, 0.0);
    Eigen::Vector2d b_workingDirection_bc_2d(b_workingDirection_bc.x(), b_workingDirection_bc.y());
    double scoreWorkingDir = workingDirWeight_ * b_workingDirection_bc_2d.dot(b_zoneCenter_bc_2d);
    double scoreLocalDistance = digDumpDistanceWeight_ * this->shovelDistanceFromZone(zoneId);
    score += scoreWorkingDir + scoreLocalDistance + scoreGlocalDistance;
    ROS_INFO_STREAM("[LocalPlanner]: dumping score for zone " << zoneId << " is " << score);
    //  print detailed breakdown of the score contribution
    ROS_INFO_STREAM(
        "[LocalPlanner]: total score " << score << ", dumping score breakdown for zone " << zoneId << " is "
                                       << scoreWorkingDir
                                       << " working dir bias, " << scoreLocalDistance << " local distance, "
                                       << scoreGlocalDistance << " global distance");
    ROS_INFO_STREAM("[LocalPlanner]: --------------------------------------------------------------------------");
    return score;
  }

  void LocalPlanner::createPlanningZones() {
    ROS_INFO_STREAM("[LocalPlanner]: **************Creating planning zones**************");
    // update parameters to decide wether to recreate the zones
    createNewZones_ = false;
    planningZones_.clear();
    zoneCenters_.clear();
    // reset vector completedAreas to all zeros
    completedDumpAreas_.clear();
    for (int i = 0; i < 4; i++) {
      completedDumpAreas_.push_back(0);
    }
    completedDigAreas_.clear();
    for (int i = 0; i < 3; i++) {
      completedDigAreas_.push_back(0);
    }

    // get vertices for the zones
    std::vector<Eigen::Vector2d> b_PosDigVertex_bd = getDiggingPatchVertices();
    std::vector<Eigen::Vector2d> b_PosDumpFrontLeft_bdu = getLeftCircularFrontSegmentPatch();
    std::vector<Eigen::Vector2d> b_PosDumpFrontRight_bdu = getRightCircularFrontSegmentPatch();
    std::vector<Eigen::Vector2d> b_PosDumpBackLeft_bdu = getLeftCircularBackSegmentPatch();
    std::vector<Eigen::Vector2d> b_PosDumpBackRight_bdu = getRightCircularBackSegmentPatch();

    Eigen::Vector2d frontZoneCenter;
    Eigen::Vector2d frontLeftZoneCenter;
    Eigen::Vector2d frontRightZoneCenter;
    Eigen::Vector2d backLeftZoneCenter;
    Eigen::Vector2d backRightZoneCenter;

    // diggingFrame_ defines the frame in which the vertices are defined
    switch (diggingFrame_) {
      case BASE: {
        // base frame
        // compute the zones centers by averaging the coordinates of the vertices
        Eigen::Vector2d frontZoneCenter;
        for (int i = 0; i < b_PosDigVertex_bd.size(); i++) {
          frontZoneCenter += b_PosDigVertex_bd.at(i);
        }
        frontZoneCenter /= b_PosDigVertex_bd.size();

        for (int i = 0; i < b_PosDumpFrontLeft_bdu.size(); i++) {
          frontLeftZoneCenter += b_PosDumpFrontLeft_bdu.at(i);
        }
        frontLeftZoneCenter /= b_PosDumpFrontLeft_bdu.size();

        for (int i = 0; i < b_PosDumpFrontRight_bdu.size(); i++) {
          frontRightZoneCenter += b_PosDumpFrontRight_bdu.at(i);
        }
        frontRightZoneCenter /= b_PosDumpFrontRight_bdu.size();

        for (int i = 0; i < b_PosDumpBackLeft_bdu.size(); i++) {
          backLeftZoneCenter += b_PosDumpBackLeft_bdu.at(i);
        }
        backLeftZoneCenter /= b_PosDumpBackLeft_bdu.size();

        for (int i = 0; i < b_PosDumpBackRight_bdu.size(); i++) {
          backRightZoneCenter += b_PosDumpBackRight_bdu.at(i);
        }
        backRightZoneCenter /= b_PosDumpBackRight_bdu.size();

        // create the polygon for the digging zone
        this->publishWorkspacePts(b_PosDigVertex_bd, "BASE");
        digZone_ = planning_utils::toPolygon(b_PosDigVertex_bd);                      // zone 0
        dumpingLeftFrontZone_ = planning_utils::toPolygon(b_PosDumpFrontLeft_bdu);    // zone 1
        dumpingRightFrontZone_ = planning_utils::toPolygon(b_PosDumpFrontRight_bdu);  // zone 2
        dumpingLeftBackZone_ = planning_utils::toPolygon(b_PosDumpBackLeft_bdu);      // zone 3
        dumpingRightBackZone_ = planning_utils::toPolygon(b_PosDumpBackRight_bdu);    // zone 4
        break;
      }
      case MAP: {
        // map frame
        std::vector<Eigen::Vector2d> w_PosDigVertex_wd;
        std::vector<Eigen::Vector2d> w_PosDumpFrontLeft_wdu;
        std::vector<Eigen::Vector2d> w_PosDumpFrontRight_wdu;
        std::vector<Eigen::Vector2d> w_PosDumpBackLeft_wdu;
        std::vector<Eigen::Vector2d> w_PosDumpBackRight_wdu;

        // transform the vertices from base frame to world frame
        geometry_msgs::TransformStamped T_bw;
        std::vector<Eigen::Vector2d> w_PosDigVertex_bd;
        try {
          T_bw = tfBuffer_->lookupTransform("map", "BASE", ros::Time(0));
        } catch (tf2::TransformException &ex) {
          ROS_WARN("[LocalPlanner]: %s", ex.what());
          ros::Duration(1.0).sleep();
        }
        // get translation vector from T_bw
        Eigen::Vector3d t_bw;
        Eigen::Quaterniond targetOrientation;
        if (isWorkspacePoseSet_) {
          //    ROS_INFO_STREAM("[LocalPlanner]: using set workspace pose");
          targetOrientation = workspaceOrientation_;
          // todo: put proper translation vector
          // t_bw = Eigen::Vector3d(T_bw.transform.translation.x, T_bw.transform.translation.y, T_bw.transform.translation.z);
          t_bw = workspacePos_;
          ROS_INFO_STREAM(
              "*****************************************************************************************");
          ROS_INFO_STREAM("[LocalPlanner]: workspace pos is " << workspacePos_);
          // print the T_bw.translation
          ROS_INFO_STREAM("[LocalPlanner]: T_bw.translation is " << T_bw.transform.translation.x << " "
                                                                 << T_bw.transform.translation.y << " "
                                                                 << T_bw.transform.translation.z);
          // print workspace position
          isWorkspacePoseSet_ = false;
        } else {
          targetOrientation =
              Eigen::Quaterniond(T_bw.transform.rotation.w, T_bw.transform.rotation.x,
                                 T_bw.transform.rotation.y, T_bw.transform.rotation.z);
          t_bw = Eigen::Vector3d(T_bw.transform.translation.x, T_bw.transform.translation.y,
                                 T_bw.transform.translation.z);
        }
        // get the x body axis expressed in world frame
        Eigen::Vector3d x_bw = targetOrientation * Eigen::Vector3d::UnitX();
        // 2d projection
        currentOrientation_ = x_bw.head(2);
        // normalize it
        currentOrientation_.normalize();
        ROS_INFO_STREAM("[LocalPlanner]: current orientation: " << currentOrientation_.transpose());

        // labmda function that checks weather vertex coordiinates are within bounds [-100, 100] and return true if they are
        auto isWithinBounds = [&](Eigen::Vector2d v) {
          if (v.x() > -100 && v.x() < 100 && v.y() > -100 && v.y() < 100) {
            return true;
          } else {
            return false;
          }
        };

        // get yaw angle
        // transform the vertices
        for (Eigen::Vector2d vertex: b_PosDigVertex_bd) {
          if (!isWithinBounds(vertex)) {
            ROS_WARN("[LocalPlanner]: vertex of dig zone is not within bounds");
            continue;
          }
          Eigen::Vector3d vertex_eigen(vertex(0), vertex(1), 0);
          Eigen::Vector3d vertex_w_eigen;
          vertex_w_eigen = targetOrientation.toRotationMatrix() * vertex_eigen + t_bw;
          Eigen::Vector2d vertex_w(vertex_w_eigen(0), vertex_w_eigen(1));
          frontZoneCenter += vertex_w;
          w_PosDigVertex_bd.push_back(vertex_w);
        }
        frontZoneCenter /= b_PosDigVertex_bd.size();

        // get the center
        for (Eigen::Vector2d vertex: b_PosDumpFrontLeft_bdu) {
          if (!isWithinBounds(vertex)) {
            ROS_WARN("[LocalPlanner]: vertex of dumping front left zone is not within bounds");
            continue;
          }
          Eigen::Vector3d vertex_eigen(vertex(0), vertex(1), 0);
          Eigen::Vector3d vertex_w_eigen;
          vertex_w_eigen = targetOrientation.toRotationMatrix() * vertex_eigen + t_bw;
          Eigen::Vector2d vertex_w(vertex_w_eigen(0), vertex_w_eigen(1));
          frontLeftZoneCenter += vertex_w;
          w_PosDumpFrontLeft_wdu.push_back(vertex_w);
        }
        frontLeftZoneCenter /= b_PosDumpFrontLeft_bdu.size();

        // get the center
        for (Eigen::Vector2d vertex: b_PosDumpFrontRight_bdu) {
          if (!isWithinBounds(vertex)) {
            ROS_WARN("[LocalPlanner]: vertex of dumping front right zone is not within bounds");
            continue;
          }
          Eigen::Vector3d vertex_eigen(vertex(0), vertex(1), 0);
          Eigen::Vector3d vertex_w_eigen;
          vertex_w_eigen = targetOrientation.toRotationMatrix() * vertex_eigen + t_bw;
          Eigen::Vector2d vertex_w(vertex_w_eigen(0), vertex_w_eigen(1));
          frontRightZoneCenter += vertex_w;
          w_PosDumpFrontRight_wdu.push_back(vertex_w);
        }
        frontRightZoneCenter /= b_PosDumpFrontRight_bdu.size();

        // get the center
        for (Eigen::Vector2d vertex: b_PosDumpBackLeft_bdu) {
          if (!isWithinBounds(vertex)) {
            ROS_WARN("[LocalPlanner]: vertex of dumping back left zone is not within bounds");
            continue;
          }
          Eigen::Vector3d vertex_eigen(vertex(0), vertex(1), 0);
          Eigen::Vector3d vertex_w_eigen;
          vertex_w_eigen = targetOrientation.toRotationMatrix() * vertex_eigen + t_bw;
          Eigen::Vector2d vertex_w(vertex_w_eigen(0), vertex_w_eigen(1));
          backLeftZoneCenter += vertex_w;
          w_PosDumpBackLeft_wdu.push_back(vertex_w);
        }
        backLeftZoneCenter /= b_PosDumpBackLeft_bdu.size();

        // get the center
        for (Eigen::Vector2d vertex: b_PosDumpBackRight_bdu) {
          if (!isWithinBounds(vertex)) {
            ROS_WARN("[LocalPlanner]: vertex of dumping back right zone is not within bounds");
            continue;
          }
          Eigen::Vector3d vertex_eigen(vertex(0), vertex(1), 0);
          Eigen::Vector3d vertex_w_eigen;
          vertex_w_eigen = targetOrientation.toRotationMatrix() * vertex_eigen + t_bw;
          Eigen::Vector2d vertex_w(vertex_w_eigen(0), vertex_w_eigen(1));
          backRightZoneCenter += vertex_w;
          w_PosDumpBackRight_wdu.push_back(vertex_w);
        }
        backRightZoneCenter /= b_PosDumpBackRight_bdu.size();

        // create the polygon for the digging zone
        this->publishWorkspacePts(w_PosDigVertex_bd, "map");
        digZone_ = planning_utils::toPolygon(w_PosDigVertex_bd);                      // zone 0
        dumpingLeftFrontZone_ = planning_utils::toPolygon(w_PosDumpFrontLeft_wdu);    // zone 1
        dumpingRightFrontZone_ = planning_utils::toPolygon(w_PosDumpFrontRight_wdu);  // zone 2
        dumpingLeftBackZone_ = planning_utils::toPolygon(w_PosDumpBackLeft_wdu);      // zone 3
        dumpingRightBackZone_ = planning_utils::toPolygon(w_PosDumpBackRight_wdu);    // zone 4
        break;
      }
    }

    // print the polygon
    //  for (grid_map::PolygonIterator iterator(excavationMappingPtr_->gridMap_, digZone); !iterator.isPastEnd(); ++iterator) {
    //    try {
    ////      ROS_INFO_STREAM("[LocalPlanner]: Adding dig
    /// zone to map at " << *iterator);
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
    // append the centers
    zoneCenters_.push_back(frontZoneCenter);
    zoneCenters_.push_back(frontLeftZoneCenter);
    zoneCenters_.push_back(frontRightZoneCenter);
    zoneCenters_.push_back(backLeftZoneCenter);
    zoneCenters_.push_back(backRightZoneCenter);
    ROS_INFO_STREAM("[LocalPlanner]: Planning zones created");
    // print the zone centers

    std::vector<double> zoneValues = {0, 1, 2, 3, 4};
    // create a new thread to add the zones to the map
    std::thread zoneThread(&LocalPlanner::addPlanningZonesToMap, this, zoneValues);
    zoneThread.detach();
    // update the original elevation layer with the lastest elevation before starting the digging
    // to avoid mapping errors
    planningMap_["predig_elevation"] = planningMap_["elevation"];
    // compute the volumes of the dig zones
    std::string targetLayer;
    for (int i = 0; i < 3; i++) {
      if (i == 0){
        targetLayer = "desired_elevation";
      } else {
        targetLayer = "original_elevation";
      }
      digZonesVolume_.push_back(this->computeWorkspaceVolume(i, targetLayer));
    }
    //  ROS_INFO_STREAM("[LocalPlanner]: planning zones created");
    // update the excavation mask to prevent leaving dirt along the future path
    this->setExcavationMaskAtFutureStates();
  }

  void LocalPlanner::setWorkspacePose(Eigen::Vector3d &workspacePos, Eigen::Quaterniond &workspaceOrientation) {
    //  ROS_INFO_STREAM("[LocalController]: Setting workspace pose");
    workspacePos_ = workspacePos;
    workspaceOrientation_ = workspaceOrientation;
    isWorkspacePoseSet_ = true;
  }

  double LocalPlanner::distanceZones(int zoneId1, int zoneId2) {
    //  ROS_INFO_STREAM("[LocalPlanner]: distance between zone " << zoneId1 << " and zone " << zoneId2);
    // we define the distance between two zones as the distance between the centers of the zones
    // print zone centers
    return (zoneCenters_.at(zoneId1) - zoneCenters_.at(zoneId2)).norm();
  }

  double LocalPlanner::shovelDistanceFromZone(int zoneId) {
    // get shovel position in world coordinates from tf
    geometry_msgs::TransformStamped T_sm;
    // get transform from base to cabin frame
    try {
      T_sm = tfBuffer_->lookupTransform("map", "ENDEFFECTOR_CONTACT", ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    // get the position of the shovel in the map frame
    Eigen::Vector3d w_P_ws = Eigen::Vector3d(T_sm.transform.translation.x, T_sm.transform.translation.y,
                                             T_sm.transform.translation.z);
    // get center of the zone
    Eigen::Vector2d zoneCenter = zoneCenters_.at(zoneId);
    // get the distance between the shovel and the zone center
    double distance = (zoneCenter - w_P_ws.head(2)).norm();
    return distance;
  }

  Eigen::Vector3d LocalPlanner::projectVectorOntoSubspace(Eigen::Vector3d &vector, Eigen::Matrix3Xd &subspaceBasis) {
    // each column of subspaceBasis is a subspace basis vector, normalize it
    //  for (int i = 0; i < subspaceBasis.cols(); i++) {
    //    subspaceBasis.col(i) = subspaceBasis.col(i).normalized();
    //  }
    // projection matrix
    Eigen::Matrix3d projectionMatrix =
        subspaceBasis * (subspaceBasis.transpose() * subspaceBasis).inverse() * subspaceBasis.transpose();
    // project vector onto subspace
    Eigen::Vector3d projectedVector = projectionMatrix * vector;
    return projectedVector;
  };

  Eigen::Vector3d LocalPlanner::findDiggingPointTrack() {
    // find the 3d coordinates of the digging point in the base frame
    // get number of tracks by diving the arc length by the track width
    double arcLength = circularWorkspaceOuterRadius_ * circularWorkspaceAngle_;
    int numTracks = arcLength / trackWidth_;
    return Eigen::Vector3d(0, 0, 0);
  }

  Eigen::Vector3d
  LocalPlanner::findShovelDesiredOrientation(Eigen::Vector3d &world_diggingPoint, Eigen::Vector3d &diggingDirection) {
    // wanna find the orientation of the shovel that will be used to dig the digging point
    // the angle is selected such that the angle between the shovel and the normal of the digging point is minimized is fixed to a desired
    // local attitude

    // get normal vector at the digging point
    Eigen::Vector3d normalVector =
        this->excavationMappingPtr_->calculateSurfaceNormalAtPosition(world_diggingPoint.head(2), 0.3,
                                                                      planningMap_, "elevation");
    this->publishNormal(world_diggingPoint, normalVector, "map");
    //  ROS_INFO_STREAM("[LocalPlanner]: normal vector at digging point " << normalVector.transpose());

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
    //  ROS_INFO_STREAM("[LocalPlanner]: shovel plane normal " << shovelPlaneNormal.transpose());
    // get the angle between the shovel plane and the normal vector
    double shovelPlaneAngle = acos(shovelPlaneNormal.dot(normalVector));
    //  ROS_INFO_STREAM("[LocalPlanner]: shovel plane angle " << shovelPlaneAngle);

    // get the angle between the shovel and the normal vector
    double localAttitude = -M_PI / 4.;  // local attitude angle
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

// Trajectory LocalPlanner::getDigTrajectoryWorldFrame(Eigen::Vector3d& w_P_wd){
//   // I need to obtain the position vector form the end effector to the boom expressed in map frame
//   // express the end effector in world frame
//   Eigen::Vector3d w_P_wb;
//   // get the transform between the ENDEFFECTOR_CONTACT frame and the BOOM frame using tf2
//   geometry_msgs::TransformStamped T_mb;
//   // get transform from base to cabin frame
//   try {
//     T_mb = tfBuffer_->lookupTransform("map", "BOOM", ros::Time(0));
//   } catch (tf2::TransformException &ex) {
//     ROS_WARN("%s", ex.what());
//     ros::Duration(1.0).sleep();
//   }
//   // get the position of the boom in the map frame
//   w_P_wb = Eigen::Vector3d(T_mb.transform.translation.x, T_mb.transform.translation.y, T_mb.transform.translation.z);
//   ROS_INFO_STREAM("[LocalPlanner]: digging point in world frame " << w_P_wd.transpose());
//   ROS_INFO_STREAM("[LocalPlanner]: boom origin in world frame " << w_P_wb.transpose());
//
//
//   // positions 2d direction vector w_P_bd = w_P_bw - w_P_dw
//   Eigen::Vector3d w_P_bd = - w_P_wb + w_P_wd;
//   // z coordinate is set to 0
//   w_P_bd(2) = 0;
//   // make it unit vector
//   w_P_bd.normalize();
//
//   // initial orientation of the shovel
//   double heading = - atan2(w_P_bd(1), w_P_bd(0));
////  ROS_INFO_STREAM("[LocalPlanner]: heading " << heading);
//  // this must be wrt the worldf frame event though its weird if the y and x axis are not aligned with the shovel axis
////  loco_m545::RotationQuaternion shovelOrientation_loco = this->get_R_sw(0, -M_PI/2 , heading);
//  // get boom heading
//
//  // get the transform between the ENDEFFECTOR_CONTACT frame and the map frame using tf2
//  geometry_msgs::TransformStamped T_sw;
//  // get transform from base to cabin frame
//  try {
//    T_sw = tfBuffer_->lookupTransform("CABIN", "map", ros::Time(0));
//  } catch (tf2::TransformException &ex) {
//    ROS_WARN("%s", ex.what());
//    ros::Duration(1.0).sleep();
//  }
//  // get rpy
//  double roll, pitch, yaw;
//  tf2::Matrix3x3(tf2::Quaternion(T_sw.transform.rotation.x, T_sw.transform.rotation.y, T_sw.transform.rotation.z,
//  T_sw.transform.rotation.w)).getRPY(roll, pitch, yaw);
////  ROS_INFO_STREAM("[LocalPlanner]: roll " << roll);
////  ROS_INFO_STREAM("[LocalPlanner]: pitch " << pitch);
////  ROS_INFO_STREAM("[LocalPlanner]: yaw " << yaw);
//
//  // transform quaternion to euler angles
//  Eigen::Quaterniond R_sw(T_sw.transform.rotation.w, T_sw.transform.rotation.x, T_sw.transform.rotation.y,
//                          T_sw.transform.rotation.z);
//  Eigen::Vector3d R_sw_euler = R_sw.toRotationMatrix().eulerAngles(0, 1, 2);
////  ROS_INFO_STREAM("[LocalPlanner]: Euler angles " << R_sw_euler.transpose());
//  double shovelYaw = R_sw_euler(2);
////  ROS_INFO_STREAM("[LocalPlanner]: boom heading " << shovelYaw);
//
//
//  Eigen::Quaterniond R_ws_0 = get_R_sw(0, -M_PI / 4, yaw);
////  ROS_INFO_STREAM("[LocalPlanner]: Euler angles 0 " << R_ws_0.toRotationMatrix().eulerAngles(0, 1, 2).transpose());
////  ROS_INFO_STREAM("[LocalPlanner]: orientation of the shovel " << R_ws_0.x() << " " << R_ws_0.y() << " " << R_ws_0.z() << " " <<
/// R_ws_0.w());
//  // transform quaternion orientation into a 3d vector
//
//  // shovel orientation in world frame
//  Eigen::Vector3d C_ws = this->findShovelDesiredOrientation(w_P_wd, w_P_bd);
//
////  ROS_INFO_STREAM("[LocalPlanner]: digging vector in world frame " << w_P_bd.transpose());
//  // get heading from the vector, this is useful if you wanna go from distance to coordinates
//  // get desired height
//  grid_map::Position wg_P_wd(w_P_wd(0), w_P_wd(1));
//  double elevation = excavationMappingPtr_->getElevation(wg_P_wd);
//  double desiredElevation = excavationMappingPtr_->getDesiredElevation(wg_P_wd);
//
//  // penetration vector
//  Eigen::Vector3d w_P_dd1 = Eigen::Vector3d(0, 0, 0);
//  // vertical displacement is the difference between the desired elevation and the elevation of the digging point
//  w_P_dd1(2) = desiredElevation - elevation;
//  // these angles are all wrt the digging direction
//  // for flat ground the desired attitude angle corresponds does not
//  double slopeAngle = 0;
//  double desiredLocalAttitudeAngle = 45 * M_PI / 180;
//  double attitudeAngle = desiredLocalAttitudeAngle + slopeAngle;
//  // the projection in 2d of the penetration vector is parallel to the digging vector and has length
//  // angle of attack, if 0 shovel moves along the local z axis of the end effector
//  double alpha = 0;
//  double diggingPathAngle = desiredLocalAttitudeAngle - alpha;
//  double horizontalDisplacement = desiredElevation/tan(diggingPathAngle);
//  w_P_dd1.head(2) = - wg_P_wd.normalized() * horizontalDisplacement;
////  ROS_INFO_STREAM("[LocalPlanner]: digging vector in world frame " << w_P_dd1.transpose());
////  ROS_INFO_STREAM("[LocalPlanner]: horizontal displacement " << horizontalDisplacement);
//
//  Eigen::Vector3d w_P_wd1 = w_P_wd + w_P_dd1;
//  Eigen::Quaterniond R_ws_d1 = this->get_R_sw(0, -M_PI / 4, yaw);
////  ROS_INFO_STREAM("[LocalPlanner]: Euler angles 1 " << R_ws_d1.toRotationMatrix().eulerAngles(0, 1, 2).transpose());
//
////  this->publishVector(w_P_wd, w_P_dd1, "map");
//
//  double trajectoryLength = 2;
//  Eigen::Vector3d w_P_d1d2 = - w_P_bd * trajectoryLength;
//  // displace the w_P_wd1 along the digging vector for the desired length
//  Eigen::Vector3d w_P_wd2 = w_P_wd1 + w_P_d1d2;
//  Eigen::Quaterniond R_ws_d2 = this->get_R_sw(0, - M_PI / 2., yaw);
////  ROS_INFO_STREAM("[LocalPlanner]: Euler angles 2 " << R_ws_d2.toRotationMatrix().eulerAngles(0, 1, 2).transpose());
//
//
////  this->publishVector(w_P_wd1, w_P_bd, "map");
//  // closing offset in cabin frame
//  Eigen::Vector3d closingOffset(0.1, 0, 0.7);
//  Eigen::Vector3d w_P_d2d3 = - w_P_bd.normalized() * closingOffset(0);
//  // get desired height at the end of the trajectory
//  grid_map::Position wg_P_wd2(w_P_wd2(0), w_P_wd2(1));
//  double elevation2 = excavationMappingPtr_->getElevation(wg_P_wd2);
//  double desiredElevation2 = excavationMappingPtr_->getDesiredElevation(wg_P_wd2);
//  // vertical displacement is the difference between the desired elevation and the elevation of the digging point
//  w_P_d2d3(2) = elevation2 - desiredElevation + closingOffset(2);
//  // transfrom to world frame by rotating yaw angle around the z axis
//  Eigen::AngleAxisd R_wc(shovelYaw, Eigen::Vector3d::UnitZ());
//  Eigen::Vector3d w_P_wd3 = w_P_wd2 + w_P_d2d3;
//  double theta = M_PI/2 - M_PI / 2; // last quadrant of the circle
////  Eigen::Vector3d w_P_wd3 =
////      w_P_wd2 +
//  Eigen::Quaterniond R_ws_d3 = this->get_R_sw(0, - M_PI * 3 / 4, yaw);
////  ROS_INFO_STREAM("[LocalPlanner]: Euler angles 3 " << R_ws_d3.toRotationMatrix().eulerAngles(0, 1, 2).transpose());
//
//  std::vector<Eigen::Vector3d> positions;
//  std::vector<Eigen::Quaterniond> orientations;
//
//  positions.push_back(w_P_wd);
//  orientations.push_back(R_ws_0);
////  this->publishDesiredShovelPose(w_P_wd, R_ws_0);
//  positions.push_back(w_P_wd1);
//  orientations.push_back(R_ws_d1);
////  this->publishDesiredShovelPose(w_P_wd1, R_ws_d1);
//  positions.push_back(w_P_wd2);
//  orientations.push_back(R_ws_d2);
////  this->publishDesiredShovelPose(w_P_wd2, R_ws_d2);
//  positions.push_back(w_P_wd3);
//  orientations.push_back(R_ws_d3);
////  this->publishDesiredShovelPose(w_P_wd3, R_ws_d3);
////  this->publishTrajectoryPoses(positions, orientations);
//  Trajectory trajectory;
//  trajectory.positions = positions;
//  trajectory.orientations = orientations;
////  this->publishMarkersTrajectory(positions, "map");
//  return trajectory;
//}

// Eigen::Quaterniond LocalPlanner::C_sw(double shovelRollAngle, double shovelPitchAngle, double shovelYawAngle) {
//   // assume that only the cabin is flat (zero roll and pitch)
//   // TODO: make it more general C_{SW} = C_{SB} * C_{BW}
//   Eigen::Matrix3d C_ws;
//   C_ws = Eigen::AngleAxisd(shovelPitchAngle, Eigen::Vector3d::UnitY())  *   Eigen::AngleAxisd(shovelRollAngle,
//   Eigen::Vector3d::UnitX())
//   *           // shovel roll
//                                        Eigen::AngleAxisd(shovelYawAngle, Eigen::Vector3d::UnitZ()) ;  // world to cabin yaw
//   // transform the rotation matrix to quaternion
//   Eigen::Quaterniond C_sw_quat(C_ws);
//   return C_sw_quat;
// }

  void LocalPlanner::findDumpPoint() {
    this->updatePlanningMap();
    this->chooseDumpZone(digZoneId_);

    ROS_INFO("[LocalPlanner]: findDumpPoint in zone %d", dumpZoneId_);
    geometry_msgs::TransformStamped T_mba;
    // get transform from base to cabin frame
    try {
      T_mba = tfBuffer_->lookupTransform("map", "BASE", ros::Time(0));
      // get also invese transform
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    // from geometric message to tf2 transform
    tf2::Transform T_mba_tf2 = tf2::Transform(
        tf2::Quaternion(T_mba.transform.rotation.x, T_mba.transform.rotation.y, T_mba.transform.rotation.z,
                        T_mba.transform.rotation.w),
        tf2::Vector3(T_mba.transform.translation.x, T_mba.transform.translation.y,
                     T_mba.transform.translation.z));
    tf2::Transform T_bam_tf2 = T_mba_tf2.inverse();

    Eigen::Quaterniond R_mba_q(T_mba.transform.rotation.w, T_mba.transform.rotation.x, T_mba.transform.rotation.y,
                               T_mba.transform.rotation.z);
    // select a point to dump the soil contained in the shovel
    // the dump point belongs to the planning zone with id zoneId
    // the dump point should be one with least elevation and further in the back of the base frame
    double minSignedBaseDistance = 0;  // dump area start here
    grid_map::PolygonIterator iterator(planningMap_, planningZones_.at(dumpZoneId_));
    double digPointBestValue = -100;

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
      // get position in base frame, used for the optimization
      // convert T_bam_Tf2 into transform stamped;
      // get the y coordinates
      double x_base = b_P_d.x();
      double y_base = b_P_d.y();

      std::vector<grid_map::Position> w_shovelFilter_sv;
      // rotate filter to have same orientation as base frame
      // transform the vertices to the world frame using the current position as center of the shovel\
    std::vector<Eigen::Vector3d> s_shovelFilter_wv;
      for (int i = 0; i < shovelFilter_.size(); i++) {
        // transform to world frame
        Eigen::Vector3d s_shovelFilter_sv_3d = Eigen::Vector3d(shovelFilter_.at(i).x(), shovelFilter_.at(i).y(),
                                                               0);
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
        if (!planningZones_.at(dumpZoneId_).isInside(w_shovelFilter_sv.at(i))) {
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
      double dumpCells = 0;
      for (grid_map::PolygonIterator iterator(planningMap_, polygon_w); !iterator.isPastEnd(); ++iterator) {
        grid_map::Index index(*iterator);
        double elevation = planningMap_.at("elevation", index);
        double originalElevation = planningMap_.at("original_elevation", index);
        if (planningMap_.at("current_excavation_mask", index) == 1) {
          dumpCells++;
        }
        // do not dump if the cell is already excavated
        if (planningMap_.at("dug_area", index) == 1) {
          valid = false;
          break;
        }
        double heightDiff = elevation - originalElevation;
        if (heightDiff > maxHeightDiff) {
          maxHeightDiff = heightDiff;
        }
        filterVolume += heightDiff * planningMap_.getResolution() * planningMap_.getResolution();
      }
      if (!valid) {
        continue;
      }
      double diggingPointValue = 0;
      //    ROS_INFO_STREAM("filterVolume: " << filterVolume << " volumeThreshold: " << volumeDirtThreshold_);
      //    ROS_INFO_STREAM("[LocalPlanner] : maHeightDiff: " << maxHeightDiff << " heightDiffThreshold: " << heightDirtThreshold_);
      //    ROS_INFO("maxHeightDiff: " << maxHeightDiff << " heightDiffThreshold: " << heightDiffThreshold_);
      // check if volume is below the threshold
      // if volume is below the threshold, then the dump point is the current position
      // check if filterVolume or maxHeightDiff is nan
      bool nanValues = false;
      if (std::isnan(filterVolume) || std::isnan(maxHeightDiff)) {
        nanValues = true;
      }
      if ((filterVolume < volumeDirtThreshold_ && maxHeightDiff < heightDumpThreshold_) || nanValues) {
        //      ROS_INFO("[LocalPlanner]: findDumpPoint: found dump point at %f, %f", positionZone.x(), positionZone.y());
        //      this->publishShovelFilter(w_shovelFilter_sv, "map");
        // pause ros for 0.2 seconds
        // pause for 0.3 seconds
        //      ros::Duration(0.3).sleep();
        // x posiiton in the back is useful only when dumping in area that will have to be excavated
        double xDumpWeight = xDumpWeight_;
        double yDumpWeight = yDumpWeight_;
        if (dumpCells == 0) {
          // closer to the front excavation area
          xDumpWeight = -xDumpWeight_;
          // futher away better, less risk of things falling back in the excavation area
          yDumpWeight = -yDumpWeight_;
        }
        //      // if dumping in the back give priority to position further in the front
        //      if (dumpZoneId_ > 2) {
        //        xDumpWeight = xDumpWeight_ * -1;
        //      }
        // print x and y in base frame
        //      ROS_INFO_STREAM("[LocalPlanner] : findDumpPoint: found dump point at x: " << x_base << " y: " << y_base);
        double xBaseScore = x_base * xDumpWeight;
        // assuming enough lateral distance
        double yBaseScore = abs(y_base) * yDumpWeight_;
        //      ROS_INFO_STREAM("[LocalPlanner] : findDumpPoint: xBaseScore: " << xBaseScore << " yBaseScore: " << yBaseScore);

        double baseScore = xBaseScore + yBaseScore;

        double dumpCellScore = dumpCells * dumpCellsWeight_;
        double volumeDirtScore = filterVolume * volumeDirtWeight_;
        //      ROS_INFO_STREAM("[LocalPlanner] : baseScore: " << baseScore << " dumpCellScore: " << dumpCellScore << " volumeDirtScore: "
        //      << volumeDirtScore);
        // print scores
        //      ROS_INFO_STREAM("[LocalPlanner]: findDumpPoint: baseScore: " << baseScore << " dumpCellScore: " << dumpCellScore);
        double pointScore = baseScore + dumpCellScore + volumeDirtScore;
//        // print score
//              ROS_INFO_STREAM("[LocalPlanner]: findDumpPoint: pointScore: " << pointScore);
//              ROS_INFO_STREAM("[LocalPlanner]: findDumpPoint: found dump point at " << x_base  << " with weight " << xDumpWeight_ << " and " << yDumpWeight_);
        //      score " << pointScore); ROS_INFO_STREAM("[LocalPlanner]: findDumpPoint: dump cells: " << dumpCells);
        if (pointScore > digPointBestValue) {
          //        ROS_INFO_STREAM("[Localplanner]: setting best dump point to " << x_base);
          digPointBestValue = pointScore;
          minSignedBaseDistance = x_base;
          dumpPointIndex_ = index;
        }
      }
    }
    //  ROS_INFO_STREAM("[LocalPlanner]: findDumpPoint: best dump point score: " << digPointBestValue);
    // raise a warning  if the dump point is not found
    if (iterator.isPastEnd() && minSignedBaseDistance == 0) {
      foundDumpPoint_ = false;
      completedDumpAreas_.at(dumpZoneId_ - 1) = 1;
      // if not all dump areas are completed (1), then continue unselect dumping zone
      if (completedDumpAreas_.at(0) == 1 && completedDumpAreas_.at(1) == 1 && completedDumpAreas_.at(2) == 1 && completedDumpAreas_.at(3) == 1) {
        dumpZoneId_ = -1;
        ROS_WARN("[LocalPlanner]: findDumpPoint: no available dumping spots");
      } else {
        this->findDumpPoint();
        ROS_WARN("Dump point not found");
      }
    } else {
      foundDumpPoint_ = true;
    }
  }

  Eigen::Vector3d LocalPlanner::getDumpPoint() {
    // return the position using the dumpPointIterator_
    grid_map::Position dumpPoint;
    // throw an exception if the dumpPointIndex_ of type Eigen::Array<int, 2 , 1> is not set
    if (dumpPointIndex_.size() == 0) {
      throw std::runtime_error("[LocalPlanner]: getDumpPoint: dumpPointIndex_ is not set");
    }
    planningMap_.getPosition(dumpPointIndex_, dumpPoint);
    // vertical offset is introduced to avoid having to introduce a more complex planner for the arm
    Eigen::Vector3d dumpPoint_3d(dumpPoint.x(), dumpPoint.y(),
                                 dumpAtHeight_ + planningMap_.at("elevation", dumpPointIndex_));
    return dumpPoint_3d;
  }

  void LocalPlanner::syncLayers() {
    unique_lock lock(mapMutex_);
    planningMap_["elevation"] = excavationMappingPtr_->gridMap_["elevation"];
    planningMap_["planning_elevation"] = planningMap_["elevation"];
    lock.unlock();
  }

  Eigen::Quaterniond LocalPlanner::get_R_sw(double shovelRollAngle, double shovelPitchAngle, double shovelYawAngle) {
    //  ROS_INFO_STREAM("[LocalPlanner] yaw angle: " << shovelYawAngle);
    return loco_m545::RotationQuaternion(
        loco_m545::AngleAxis(shovelPitchAngle, 0, 1, 0) * loco_m545::AngleAxis(shovelRollAngle, 1, 0, 0) *
        loco_m545::AngleAxis(shovelYawAngle, 0, 0, 1))
        .toImplementation();
  }

  std::vector<Eigen::Vector2d> LocalPlanner::getDiggingPatchVertices() {
    // the digging patch is approximated by a semicircle with radius 0.5
    // that spans from the angle -pi/5 to 1/5 pi
    // the circle is sampled at 10 points
    std::vector<Eigen::Vector2d> vertices;
    int numPoints = 15;
    for (int i = 1; i < numPoints - 1; i++) {
      double angle = -circularWorkspaceAngle_ / 2 + circularWorkspaceAngle_ * i / numPoints;
      Eigen::Vector2d vertex(circularWorkspaceOuterRadius_ * cos(angle),
                             circularWorkspaceOuterRadius_ * sin(angle));
      vertices.push_back(vertex);
    }
    // add vertices belonging to another arc with radius circularWorkspaceInnerRadius_
    // add them in the opposite order
    for (int i = numPoints - 1; i >= 0; i--) {
      double angle = -circularWorkspaceAngle_ / 2 + circularWorkspaceAngle_ * i / numPoints;
      Eigen::Vector2d vertex(circularWorkspaceInnerRadius_ * cos(angle),
                             circularWorkspaceInnerRadius_ * sin(angle));
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
      Eigen::Vector2d vertex(dumpingZoneOuterRadius_ * cos(angle), dumpingZoneOuterRadius_
                                                                   * sin(angle));
      vertices.push_back(vertex);
    }
    // add vertices belonging to another arc with radius dumpingInnerRadius_
    // add them in the opposite order
    for (int i = 9; i >= 0; i--) {
      double angle = M_PI / 5 + i * M_PI / 25;
      Eigen::Vector2d vertex(dumpingZoneInnerRadius_ * cos(angle), dumpingZoneInnerRadius_
                                                                   * sin(angle));
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
    for (int i = 0; i < 9; i++) {
      double angle = startAngle + i * M_PI / 30;
      Eigen::Vector2d vertex(dumpingZoneOuterRadius_ * cos(angle), dumpingZoneOuterRadius_
                                                                   * sin(angle));
      vertices.push_back(vertex);
    }
    // define a vertical line segment
    Eigen::Vector2d vertex(dumpingZoneInnerRadius_ * cos(startAngle), dumpingZoneInnerRadius_
                                                                      * sin(startAngle));
    Eigen::Vector2d vertex2(0, dumpingZoneInnerRadius_ * sin(startAngle));

    vertices.push_back(vertex2);
    vertices.push_back(vertex);
    return vertices;
  }

  std::vector<Eigen::Vector2d> LocalPlanner::getRightFrontPatch() {
    // the dumping patch is approximated by a semicircle with radius dumpingOuterRadius_;
    // that spans from the angle -1/5 pi to -1/2 pi
    // the circle is sampled at 10 points
    std::vector<Eigen::Vector2d> vertices;
    for (int i = 0; i < 9; i++) {
      double angle = -M_PI / 5 - i * M_PI / 30;
      Eigen::Vector2d vertex(dumpingZoneOuterRadius_ * cos(angle), dumpingZoneOuterRadius_
                                                                   * sin(angle));
      vertices.push_back(vertex);
    }
    // add vertices belonging to another arc with radius dumpingInnerRadius_
    // add them in the opposite order
    for (int i = 9; i >= 0; i--) {
      double angle = -M_PI / 5 - i * M_PI / 30;
      Eigen::Vector2d vertex(dumpingZoneInnerRadius_ * cos(angle), dumpingZoneInnerRadius_
                                                                   * sin(angle));
      vertices.push_back(vertex);
    }
    return vertices;
  }

  std::vector<Eigen::Vector2d> LocalPlanner::getRightCircularFrontSegmentPatch() {
    // the dumping patch is approximated by a circular segment with  radius dumpingOuterRadius_;
    // that spans from the angle -1/5 pi to -1/2 pi
    // the circle is sampled at 10 points
    double startAngle = -M_PI / 5;
    std::vector<Eigen::Vector2d> vertices;
    for (int i = 0; i < 9; i++) {
      double angle = startAngle - i * M_PI / 30;
      Eigen::Vector2d vertex(dumpingZoneOuterRadius_ * cos(angle), dumpingZoneOuterRadius_
                                                                   * sin(angle));
      vertices.push_back(vertex);
    }
    // define a vertical line segment
    Eigen::Vector2d vertex(dumpingZoneInnerRadius_ * cos(startAngle), dumpingZoneInnerRadius_
                                                                      * sin(startAngle));
    Eigen::Vector2d vertex2(0, dumpingZoneInnerRadius_ * sin(startAngle));

    vertices.push_back(vertex2);
    vertices.push_back(vertex);
    return vertices;
  }

  std::vector<Eigen::Vector2d> LocalPlanner::getLeftBackPatch() {
    // the dumping patch is approximated by a semicircle with radius dumpingOuterRadius_;
    // that spans from the angle 1/2 pi to 4/5 pi
    // the circle is sampled at 10 points
    std::vector<Eigen::Vector2d> vertices;
    for (int i = 0; i < 10; i++) {
      double angle = M_PI / 2 + i * M_PI / 25;
      Eigen::Vector2d vertex(dumpingZoneOuterRadius_ * cos(angle), dumpingZoneOuterRadius_
                                                                   * sin(angle));
      vertices.push_back(vertex);
    }
    // add vertices belonging to another arc with radius dumpingInnerRadius_
    // add them in the opposite order
    for (int i = 9; i >= 0; i--) {
      double angle = M_PI / 2 + i * M_PI / 25;
      Eigen::Vector2d vertex(dumpingZoneInnerRadius_ * cos(angle), dumpingZoneInnerRadius_
                                                                   * sin(angle));
      vertices.push_back(vertex);
    }
    return vertices;
  }

  std::vector<Eigen::Vector2d> LocalPlanner::getLeftCircularBackSegmentPatch() {
    // the dumping patch is approximated by a circular segment with  radius dumpingOuterRadius_;
    // that spans from the angle 1/2 pi to 4/5 pi
    // the circle is sampled at 10 points
    double startAngle = M_PI / 2;
    std::vector<Eigen::Vector2d> vertices;
    for (int i = 0; i < 10; i++) {
      double angle = startAngle + i * M_PI / 25;
      Eigen::Vector2d vertex(dumpingZoneOuterRadius_ * cos(angle), dumpingZoneOuterRadius_
                                                                   * sin(angle));
      vertices.push_back(vertex);
    }
    // define a vertical line segment
    double endAngle = 4. / 5 * M_PI;
    Eigen::Vector2d vertex(dumpingZoneInnerRadius_ * cos(endAngle), dumpingZoneInnerRadius_
                                                                    * sin(endAngle));
    Eigen::Vector2d vertex2(0, dumpingZoneInnerRadius_ * sin(endAngle));

    vertices.push_back(vertex);
    vertices.push_back(vertex2);
    return vertices;
  }

  std::vector<Eigen::Vector2d> LocalPlanner::getRightBackPatch() {
    // the dumping patch is approximated by a semicircle with radius dumpingOuterRadius_;
    // that spans from the angle -1/2 pi to -4/5 pi
    // the circle is sampled at 10 points
    std::vector<Eigen::Vector2d> vertices;
    for (int i = 0; i < 10; i++) {
      double angle = M_PI * 4 / 5 - i * M_PI / 25;
      Eigen::Vector2d vertex(dumpingZoneOuterRadius_ * cos(angle), dumpingZoneOuterRadius_
                                                                   * sin(angle));
      vertices.push_back(vertex);
    }
    // add vertices belonging to another arc with radius dumpingInnerRadius_
    // add them in the opposite order
    for (int i = 9; i >= 0; i--) {
      double angle = M_PI * 4 / 5 - i * M_PI / 25;
      Eigen::Vector2d vertex(dumpingZoneInnerRadius_ * cos(angle), dumpingZoneInnerRadius_
                                                                   * sin(angle));
      vertices.push_back(vertex);
    }
    return vertices;
  }

  std::vector<Eigen::Vector2d> LocalPlanner::getRightCircularBackSegmentPatch() {
    // the dumping patch is approximated by a circular segment with  radius dumpingOuterRadius_;
    // that spans from the angle -1/2 pi to -4/5 pi
    // the circle is sampled at 10 points
    double startAngle = -1. / 2 * M_PI;
    std::vector<Eigen::Vector2d> vertices;
    for (int i = 0; i < 10; i++) {
      double angle = startAngle - i * M_PI / 25;
      Eigen::Vector2d vertex(dumpingZoneOuterRadius_ * cos(angle), dumpingZoneOuterRadius_
                                                                   * sin(angle));
      vertices.push_back(vertex);
    }
    // define a vertical line segment
    double endAngle = -4. / 5 * M_PI;
    Eigen::Vector2d vertex(dumpingZoneInnerRadius_ * cos(endAngle), dumpingZoneInnerRadius_
                                                                    * sin(endAngle));
    Eigen::Vector2d vertex2(0, dumpingZoneInnerRadius_ * sin(endAngle));

    vertices.push_back(vertex);
    vertices.push_back(vertex2);
    return vertices;
  }

  void LocalPlanner::addPlanningZonesToMap(std::vector<double> values) {
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

  void LocalPlanner::updateWorkingArea() {
    ROS_INFO_STREAM("[LocalPlanner] Updating working area...");
    // first we assign the dug layer to the untraversable areas
    //  planningMap_["working_area"] = planningMap_["dug_area"];
    // iterate over the whole map and mark untraversable areas that differ more than heightTraversableThreshold_ from the original layer
    // with this heuristic we would like to exclude dump areas
    // accounting for dump areas is hard since dirt is constantly move around and marking and marking it could be error prone due to heading
    // precision
    for (grid_map::GridMapIterator iterator(planningMap_); !iterator.isPastEnd(); ++iterator) {
      // get iterator index
      grid_map::Index index(*iterator);
      // check the difference between the current elevation and the original elevation
      try {
        double difference = planningMap_.at("elevation", index) - planningMap_.at("original_elevation", index);
        if (abs(difference) > heightTraversableThreshold_) {
          planningMap_.at("working_area", index) = 1;
        } else {
          planningMap_.at("working_area", index) = 0;
        }
        double dugDifference = planningMap_.at("elevation", index) - planningMap_.at("desired_elevation", index);
        if (abs(dugDifference) < heightDigAreaThreshold_) {
          if (planningMap_.at("current_excavation_mask", index) == -1) {
            planningMap_.at("dug_area", index) = 1;
          } else {
            planningMap_.at("dug_area", index) = 0;
          }
        }
        // if occupancy is 1 at the cell mark it as untraversable
        if (planningMap_.at("occupancy", index) == 1) {
          planningMap_.at("working_area", index) = 1;
        }
      } catch (...) {
        // catch if a layer that does exist is queried
        ROS_ERROR("[LocalPlanner] Error while updating working area");
      }
    }
  }

  void LocalPlanner::publishWorkingArea() {
    // transform the planning map into a msgs
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(planningMap_, message);
    workingAreaPublisher_.publish(message);
  }

  void LocalPlanner::saveCurrentPlanningMap() {
    // save the current planning map to a bag file
    grid_map::GridMapRosConverter::saveToBag(planningMap_, saveMapPath_, "grid_map");
  }

//
// std::vector<Eigen::Vector3d> LocalPlanner::getDigStraightTrajectory(Eigen::Vector3d& diggingPoint) {
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

// std::vector<Eigen::Vector3d> LocalPlanner::digTrajectory(Eigen::Vector3d& base_digPosition) {
//     // create straight line from origin to start in base frame
//     Eigen::Vector3d start_pos(0, 0, 0);
//     Eigen::Vector3d end_pos(base_digPosition(0), base_digPosition(1), base_digPosition(2));
//     Eigen::Vector3d dir = end_pos - start_pos;
//     dir.normalize();
//     // create a vector of points along the line
//     std::vector<Eigen::Vector3d> trajectory;
//     for (int i = 0; i < 10; i++) {
//         trajectory.push_back(start_pos + dir * i);
//     }
//     // create line iterator for grid mpa that follow the trajectory
//     grid_map::LineIterator it(excavationMappingPtr_->gridMap_, start_pos, end_pos);
// }

  void LocalPlanner::publishMarker(grid_map::Position3 position, std::string frameId) const {
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

  void LocalPlanner::publishShovelPoints(Eigen::Vector3d &shovelLeftContactPoint,
                                         Eigen::Vector3d &shovelRightContactPoint) {
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

  void LocalPlanner::publishWorkspacePts(std::vector<Eigen::Vector2d> workspacePts, std::string frameId) {
    // ros info stream the workspace points
    //  std::cout << "workspace points: " << std::endl;
    //  for (int i = 0; i < workspacePts.size(); i++) {
    //    std::cout << workspacePts[i].transpose() << std::endl;
    //  }

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
    //  ROS_INFO("[LocalPlanner]: Published workspace points");
  }

  void LocalPlanner::publishShovelFilter(std::vector<Eigen::Vector2d> workspacePts, std::string frameId) {
    // ros info stream the workspace points
    //  std::cout << "workspace points: " << std::endl;

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
    //  ROS_INFO("[LocalPlanner]: Published shovel filter points");
  }

  void LocalPlanner::publishMarkersTrajectory(std::vector<grid_map::Position3> positions, std::string frameId) const {
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
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      markerArray.markers.push_back(marker);
    }
    markersTrajectoryPublisher_.publish(markerArray);
  }

  void LocalPlanner::publishVector(Eigen::Vector3d position, Eigen::Vector3d direction, std::string frameId) const {
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

  void LocalPlanner::publishHeading(Eigen::Vector3d position, Eigen::Vector3d direction, std::string frameId) const {
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

  void LocalPlanner::publishNormal(Eigen::Vector3d position, Eigen::Vector3d direction, std::string frameId) const {
    // publish an arrow located at position and with the direction of direction
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

  void LocalPlanner::publishVectors(std::vector<Eigen::Vector3d> positions, std::vector<Eigen::Vector3d> directions,
                                    std::string frameId) const {
    // publish an arrow located at position and with the direction of direction
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

  void LocalPlanner::publishDesiredShovelOrientation(Eigen::Vector3d position, Eigen::Vector3d direction,
                                                     std::string frameId) const {
    // publish an arrow located at position and with the direction of direction
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

  void LocalPlanner::publishProjectedVector(Eigen::Vector3d position, Eigen::Vector3d direction,
                                            std::string frameId) const {
    // publish an arrow located at position and with the direction of direction
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

  void
  LocalPlanner::publishJointVector(Eigen::Vector3d position, Eigen::Vector3d direction, std::string frameId) const {
    // publish an arrow located at position and with the direction of direction
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

  void LocalPlanner::publishTrajectoryPoses(std::vector<Eigen::Vector3d> positions,
                                            std::vector<Eigen::Quaterniond> orientations) const {
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

  void LocalPlanner::publishDesiredShovelPose(Eigen::Vector3d position, Eigen::Quaterniond orientation) const {
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

  void LocalPlanner::logWorkspaceData(){
    // log workspace data
    std::ofstream workspace_data;
    // get ros package path
    std::string package_path = ros::package::getPath("local_excavation");
    std::string file_path = package_path + "/log/workspace_data.txt";
    workspace_data.open(file_path, std::ios::app);
    // save the maps remainingVolumeRatio_, reaminingVolume_, remainingCellsRatio_ to the file
    // with the format waypointIndex, remainingVolumeRatio, remainingVolume, remainingCellsRatio
    for (int i = 0; i < remainingVolumeRatios_.size(); i++) {
      workspace_data << i << "," << remainingVolumeRatios_[i] << "," << remainingVolume_[i] << "," << remainingCellsRatio_[i] << std::endl;
    }
    workspace_data.close();
  }
}  // namespace local_excavation