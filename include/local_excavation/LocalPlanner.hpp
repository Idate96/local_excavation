#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <tf2_ros/transform_listener.h>
#include <eigen3/Eigen/Dense>
#include "excavation_mapping/ExcavationMapping.hpp"
#include "loco_m545/common/typedefs.hpp"
#include <excavator_model/ExcavatorModel.hpp>
#include "local_excavation/DigDumpSrv.h"
#include "m545_planner_msgs/M545Footprint.hpp"
#include <geometry_msgs/PoseArray.h>
#include "se2_planning/State.hpp"

// collisions
#include <collisions/CollisionGroup.hpp>
#include <collisions_bullet/ColliderManagerBullet.hpp>
#include "romo/common/CollisionBodyRomo.hpp"
#include <collisions/CollisionBody.hpp>
#include "collisions/CollisionBodySimple.hpp"
#include <collisions_geometry/CollisionGeometryBox.hpp>

namespace local_excavation {

inline Eigen::Matrix<double, 5, 2> footprintAtState(const se2_planning::SE2state& baseStateMapFrame, const Eigen::Matrix<double, 5, 2>& footprintBaseFrame, double inflationFactor) {
  // first rotate the footprint according to the state yaw and then translate it according to the state position
  double yaw = baseStateMapFrame.yaw_;
  // construct rotation matrix
  Eigen::Matrix2d rot;
  rot(0, 0) = cos(yaw);
  rot(0, 1) = -sin(yaw);
  rot(1, 0) = sin(yaw);
  rot(1, 1) = cos(yaw);
  Eigen::Vector2d baseStateMapFrame_vec(baseStateMapFrame.x_, baseStateMapFrame.y_);
  // apply inflation factor
  Eigen::Matrix<double, 5, 2> inflatedFootprintBaseFrame;
  for (int i = 0; i < 5; i++){
    inflatedFootprintBaseFrame.row(i) = inflationFactor * footprintBaseFrame.row(i);
  }
  // apply rotation and translation
  Eigen::Matrix<double, 2, 5> footprintMapFrame = (rot * inflatedFootprintBaseFrame.transpose()).colwise() + baseStateMapFrame_vec;
  return footprintMapFrame.transpose();
}
class Trajectory {
 public:
  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Quaterniond> orientations;
  double startDistanceFromBase = 0;
  double endDistanceFromBase = 0;
  double relativeHeading = 0;
  double scoopedVolume = -10;
  double workspaceVolume = -10;
  double length = 0;
  std::vector<double> stepVolumes;
  std::vector<double> otherVolumes;
  std::vector<double> totalVolumes;
};

class LocalPlanner {
 public:
  LocalPlanner(std::unique_ptr<excavation_mapping::ExcavationMapping> excavationMapping);

  // rectangle in cabin frame centered along the shovel

  bool initialize(std::string designMapBag);
  bool loadParameters();
  void reset();
  /*!
   * Update the submap and the global map using the measured elevation map.
   * @return
   */
  bool updatePlanningMap();
  double computeWorkspaceVolume(int zoneId, std::string targetLayer);
  std::string getTargetDigLayer(int zoneId);
  grid_map::GridMap& getPlanningMap() { return planningMap_; }
  void setLocalMap(grid_map::GridMap& localMap);

  // getters
  Eigen::Vector3d getDiggingPoint() const { return diggingPoint_; };
  Eigen::Vector3d getDiggingPointBaseFrame() const;
  Eigen::Vector3d findShovelDesiredOrientation(Eigen::Vector3d& world_diggingPoint, Eigen::Vector3d& diggingDirection);
  double getRadialOffset() { return radialOffset_; }
  double shovelDistanceFromZone(int zoneId);

  // get digging direction
  // planning
  std::vector<Eigen::Vector3d> getDigStraightTrajectory(Eigen::Vector3d& diggingPoint);
  Eigen::Vector3d findDiggingPointLocalMap();
  Eigen::Vector3d findDiggingPointTrack();
  //  Eigen::Vector3d findDumpPoint();
  Eigen::Vector3d findDumpingPointTrack();

  double objectiveDistanceAndElevation(grid_map::Position& base_diggingPoint);
  void computeSdf(std::string targetLayer);

  std::vector<Eigen::Vector3d> digTrajectory(Eigen::Vector3d& base_digPosition);
  void optimizeTrajectory();
  Trajectory computeTrajectory(Eigen::Vector3d& w_P_wd, std::string targetLayer, int zoneId);
  Trajectory computeDigTrajectory(Eigen::Vector3d& w_P_wd, std::string targetLayer);
  Trajectory computeDirtTrajectory(Eigen::Vector3d& w_P_wd, std::string targetLayer);
  double volumeObjective(Trajectory trajectory);
  loco_m545::RotationQuaternion findOrientationWorldToShovel(double shovelRollAngle, double shovelPitchAngle, double shovelYawAngle);
  Trajectory getDigTrajectoryWorldFrame(Eigen::Vector3d& w_P_wd);
  void getLayerHeightAlongBoom(std::string layer, std::vector<double>& layerValues, std::vector<double>& distanceFromBase);
  double getShovelHeightFromLayer(std::string layer);
  // this function is important to check for collisions
  void updateRobotState(excavator_model::ExcavatorState& excavatorState);

  // orientation shovel wrt world frame
  Eigen::Quaterniond C_sw(double shovelRollAngle, double shovelPitchAngle, double shovelYawAngle);
  Eigen::Quaterniond get_R_sw(double shovelRollAngle, double shovelPitchAngle, double shovelYawAngle);
  double getDumpingScore(int zoneId);
  // dump point stuff
  Eigen::Vector3d getDumpPoint();
  Eigen::Vector3d digStraight(Eigen::Vector3d& base_digPosition);
  bool findDiggingPoint();
  bool findRandomDiggingPoint();
  void findDumpPoint();
  bool foundDumpPoint_ = false;
  double getVolume();
  // zones functions
  bool isDigZoneComplete(int zoneId);
  bool isZoneActive(int digZoneId, bool isDigging);
  bool isLocalWorkspaceComplete();
  void updateWorkingArea();
  void setWorkingAreaInPreviousStates();
  void updateDugZones();
  // select dumping zone based on this score
  void sdfDumpingAreas();
  bool isLateralFrontZoneComplete(int zoneId);
  std::vector<int> completedDumpAreas_ = {0, 0, 0, 0};
  std::vector<int> completedDigAreas_ = {0, 0, 0};
  void checkScoopedVolume(double volume);
  double minScoopVolume_;
  int lowVolumeScoopAttempts_;

  void setDigTrajectory(Trajectory& trajectory) { digTrajectory_ = trajectory; };
  Trajectory getDigTrajectory() { return digTrajectory_; };
  Trajectory getOptimalTrajectory();

  Eigen::Vector3d projectVectorOntoSubspace(Eigen::Vector3d& vector, Eigen::Matrix3Xd& subspaceBasis);
  void markDigDumpAreas();
  void resetDigDumpAreas();

  // ros publishers
  void publishPlanningMap();
  void publishMaps();
  void publishMarker(grid_map::Position3 position, std::string frameId) const;
  void publishMarkersTrajectory(std::vector<Eigen::Vector3d> trajectory, std::string frameId) const;
  void publishVector(Eigen::Vector3d position, Eigen::Vector3d direction, std::string frameId) const;
  void publishNormal(Eigen::Vector3d position, Eigen::Vector3d normal, std::string frameId) const;
  void publishDesiredShovelOrientation(Eigen::Vector3d position, Eigen::Vector3d direction, std::string frameId) const;
  void publishProjectedVector(Eigen::Vector3d position, Eigen::Vector3d vector, std::string frameId) const;
  void publishJointVector(Eigen::Vector3d position, Eigen::Vector3d vector, std::string frameId) const;
  void publishDesiredShovelPose(Eigen::Vector3d value, Eigen::Quaterniond C_ws) const;
  void publishVectors(std::vector<Eigen::Vector3d> vectors, std::vector<Eigen::Vector3d> directions, std::string frameId) const;
  void publishTrajectoryPoses(std::vector<Eigen::Vector3d> poses, std::vector<Eigen::Quaterniond> orientations) const;
  void publishShovelPoints(Eigen::Vector3d& shovelRightContactPoint, Eigen::Vector3d& shovelLeftContactPoint);
  void publishWorkspacePts(std::vector<Eigen::Vector2d> workspacePts, std::string frameId);
  void publishHeading(Eigen::Vector3d position, Eigen::Vector3d direction, std::string frameId) const;
  void publishShovelFilter(std::vector<Eigen::Vector2d> workspacePts, std::string frameId);
  void publishWorkingArea();
  void saveCurrentPlanningMap();

  std::unique_ptr<excavation_mapping::ExcavationMapping> excavationMappingPtr_;
  void addPlanningZonesToMap(std::vector<double> values);
  void createPlanningZones();
  void choosePlanningZones();
  int chooseDigZone();
  int chooseDumpZone(int digZone);
  void setWorkspacePose(Eigen::Vector3d& workspacePos, Eigen::Quaterniond& workspaceOrientation);
  double distanceZones(int zoneId1, int zoneId2);


  std::vector<Eigen::Vector2d> getDiggingPatchVertices();
  std::vector<Eigen::Vector2d> getDiggingSawPatchVertices();
  std::vector<Eigen::Vector2d> getLeftFrontPatch();
  std::vector<Eigen::Vector2d> getLeftCircularFrontSegmentPatch();
  std::vector<Eigen::Vector2d> getRightFrontPatch();
  std::vector<Eigen::Vector2d> getRightCircularFrontSegmentPatch();
  std::vector<Eigen::Vector2d> getLeftBackPatch();
  std::vector<Eigen::Vector2d> getLeftCircularBackSegmentPatch();
  std::vector<Eigen::Vector2d> getRightBackPatch();
  std::vector<Eigen::Vector2d> getRightCircularBackSegmentPatch();
  void syncLayers();

  enum DiggingFrame { BASE, MAP };
  DiggingFrame diggingFrame_ = MAP;

  // set dig and dump zone
  void setDigZone(int zoneId);
  void setDumpZone(int zoneId);
  void setWaypointIndex(int index) { waypointIndex_ = index; }

  // trajectory helpers
  std::vector<Eigen::Vector3d>  smoothZCoordinates(std::vector<Eigen::Vector3d>& trajectory);
  void setWorkingDirection(Eigen::Vector2d& workingDirection) { workingDirection_ = workingDirection; }

 private:
  // sub-map representing the reachable workspace of the robot
  grid_map::GridMap localMap_ = grid_map::GridMap();
  grid_map::GridMap planningMap_ = grid_map::GridMap();
  // var to check if trackMap_ or localMap_ is used
  bool trackMapUsed_ = false;
  double shovelWidth_ = 0.7;
  double shovelLength_ = 1.5;
  double trackWidth_ = 0.8 * shovelWidth_;

  // local workspace
  bool isWorkspacePoseSet_ = false;
  // enum for the digging frame, it can be either "BASE" or "map"
  // this is particularly useful for testing as the digging in base frame eleminates the need to use navigation and mapping to get the robot
  // this is particularly useful for testing as the digging in base frame eleminates the need to use navigation and mapping to get the robot
  // to the digging point

  Eigen::Vector3d workspacePos_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond workspaceOrientation_ = Eigen::Quaterniond::Identity();

  grid_map::Position3 diggingPoint_;
  grid_map::Index diggingPointLocalIndex_;
  grid_map::Position3 dumpPoint_;
  grid_map::Index dumpPointIndex_;

  // trajectory
  Trajectory digTrajectory_;
  Trajectory optimalDigTrajectory_;
  Trajectory dumpTrajectory_;

  // self collisions
  excavator_model::ExcavatorModel model_; // copy of the model used for planning
  collisions::CollisionGroup armCollisionGroup_;
  collisions::CollisionGroup boomCollisionGroup_;
  collisions::CollisionGroup legCollisionGroup_;
  collisions_bullet::ColliderManagerBullet colliderManager_;
  collisions::CollisionGroup armGroup_;
  bool updateShovelCollisionBody(Eigen::Vector3d w_P_ws, Eigen::Quaterniond C_ws);
  std::shared_ptr<collisions::CollisionBodySimple> shovelBodyPtr_;

  // if we need more than one "coverage lane" to covers the global workspace
  // then this vector is perpendicular to the two lanes and point in the direction of the second lane
  // this is used to bias the local planner to dump soil against the direction of motion
  Eigen::Vector2d workingDirection_ = Eigen::Vector2d::Zero();

  // ros
  ros::NodeHandle nh_;
  ros::Publisher planningMapPublisher_;
  ros::Publisher markerPublisher_;
  ros::Publisher markersTrajectoryPublisher_;
  ros::Publisher penetrationDirectionPublisher_;
  ros::Publisher diggingDirectionPublisher_;
  ros::Publisher normalPublisher_;
  ros::Publisher desiredShovelOrientationPublisher_;
  ros::Publisher projectedVectorPublisher_;
  ros::Publisher jointVectorPublisher_;
  ros::Publisher desiredPosePublisher_;
  ros::Publisher vectorsPublisher_;
  ros::Publisher trajectoryPosesPublisher_;
  ros::Publisher shovelPointsPublisher_;
  ros::Publisher workspacePtsPublisher_;
  ros::Publisher headingPublisher_;
  ros::Publisher polygonPublisher_;
  ros::Publisher shovelFilterPublisher_;
  ros::Publisher workingAreaPublisher_;

  ros::Subscriber footprintSubscriber_;
  void footprintCallback(const m545_planner_msgs::M545FootprintRos& msg);
  boost::shared_mutex footprintMutex_;
  Eigen::Matrix<double, 5, 2> footprint_;
  double footprintInflation_ = 1.0;

  ros::Subscriber globalPathSub_;
  void globalPathCallback(const geometry_msgs::PoseArray& msg);
  std::vector<geometry_msgs::Pose> globalPath_;
  int waypointIndex_ = 0;
  void setExcavationMaskAtFutureStates();

  std::string pathTopic_;
  std::string footprintTopic_;

  // make a ros service to select dig and dump zones
  ros::ServiceServer digAndDumpService_;
  bool digAndDumpServiceCallback(DigDumpSrv::Request& request, DigDumpSrv::Response& response);
  bool autoZoneSelection_ = true;

  // transform listener
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  // update local map
  using unique_lock = boost::unique_lock<boost::shared_mutex>;
  using shared_lock = boost::shared_lock<boost::shared_mutex>;
  boost::shared_mutex mapMutex_;
  bool addDataFrom(grid_map::GridMap& map, const grid_map::GridMap& other, std::vector<std::string> layers);
  std::vector<grid_map::Polygon> planningZones_;
  std::vector<Eigen::Vector2d> zoneCenters_;
  grid_map::Polygon digZone_;
  grid_map::Polygon dumpingLeftFrontZone_;
  grid_map::Polygon dumpingRightFrontZone_;
  grid_map::Polygon dumpingLeftBackZone_;
  grid_map::Polygon dumpingRightBackZone_;

  void createShovelFilter();
  std::vector<Eigen::Vector2d> shovelFilter_;
  // compute volume between shovel pts
  std::tuple<double, double> computeVolumeBetweenShovelPoints(Eigen::Vector3d& w_posLeftShovel_wl, Eigen::Vector3d& w_posRightShovel_wr,
                                                              double previousTerrainElevation);
  // current dig and dump zone
  int previousDigZoneId_ = -1;
  int previousDumpZoneId_ = -1;
  int dumpZoneId_ = -1;
  int digZoneId_ = -1;
  // workspace volume
  double workspaceVolume_;
  std::vector<double> digZonesVolume_;

  // boolean to indicate whether a reset is needed
  bool createNewZones_ = true;
  double remainingVolumeRatio_;

  // optimization weight
  double volumeWeight_;
  double distanceWeight_;
  double headingWeight_;

  // parameters
  // dig trajectory depth
  double maxDigDepth_;
  double maxDigDirtDepth_;
  double closingZTranslation_;
  double minDistanceCollision_;
  double targetDigAttitude_;
  double targetDigDirtAttitude_;
  // drag shovel angle
  double draggingAngle_;
  // areas with more than this ratio (dug area / total area) of the total area are considered not active
  double inactiveAreaRatio_;
  double excavationAreaRatio_;
  // selection fo dumping workspace
  double digDumpDistanceWeight_;
  double workingDirWeight_;
  double dumpingZoneDistanceWeight_;

  // current orientation
  Eigen::Vector2d currentOrientation_;

  // dumping spot selection
  double dumpAtHeight_;
  double radialOffset_;
  double radialDirtOffset_;
  double verticalOffset_;
  double heightPrecision_;
  double xDumpWeight_;
  double yDumpWeight_;
  double dumpCellsWeight_;
  // condition for termination
  double volumeThreshold_;
  double heightThreshold_;
  double missingCellsThreshold_;
  double volumeDumpThreshold_ = 0.5 * shovelWidth_ * shovelLength_;
  // we don't account the initial volume that goes into the shovel in the penetration phase
  // todo: account properly for it
  double shovelVolumeBonus_;
  // the accurary required for dirt handling is lower than the accuracy of digging
  double volumeDirtThreshold_;
  double heightDirtThreshold_;
  double heightDigAreaThreshold_;
  double volumeDirtWeight_;
  double heightDumpThreshold_;
  // max volume in the shovel
  double maxVolume_;
  double maxDirtVolume_;
  // distance before the shovel attitude is flat against the soil
  double draggingDistance_;
  double draggingDirtDistance_;

  // index to keep track
  double circularWorkspaceOuterRadius_;
  double circularWorkspaceInnerRadius_;
  double dumpingZoneOuterRadius_;
  double dumpingZoneInnerRadius_;
  double minDistanceShovelToBase_;
  double circularWorkspaceAngle_;
  int currentTrackId = 0;
  // param to mark working areas that you can't step over
  double heightTraversableThreshold_;
  double targetHeightDiffThreshold_;
  // workspace logging
  // dictionary with key the waypoint index and as value the remaining volume ratio
  std::map<int, double> remainingVolumeRatios_;
  std::map<int, double> remainingVolume_;
  std::map<int, double> remainingCellsRatio_;
  void logWorkspaceData();
  // saving params maps
  std::string saveMapPath_;
  // recovery behaviour
  int lowVolumeScoopCounter_ = 0;
};

}  // namespace local_excavation