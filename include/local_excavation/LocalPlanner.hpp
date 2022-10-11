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
#include "collisions_visualization/geometry_to_marker.hpp"
#include "collisions_visualization/CollisionGeometryMarkerPublisher.hpp"
#include "collisions_visualization/CollisionsMarkerPublisher.hpp"
#include "collisions_visualization/MarkerOptions.hpp"


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
  Eigen::Vector3d startPosition;
  double startDistanceFromBase = 0;
  double endDistanceFromBase = 0;
  double relativeHeading = 0;
  double scoopedVolume = -10;
  double workspaceVolume = -10;
  double length = 0;
  double sweptArea = 0; // m^2
  std::vector<double> stepVolumes;
  std::vector<double> otherVolumes;
  std::vector<double> totalVolumes;
};

// DigArea data structure for logging
// workspace id, average precision, std precision, dig time, missing cells, missing volume,
// scoop id start, scoop id end
struct DigZone {
  int workspaceId;
  int digZoneId;
  std::string targetLayer;
  double heightThreshold;
  double rmsPrecision;
  double digTime; // seconds
  double volume; // m^3
  double area; // m^2
  int numMissingCells;
  int numTotalCells;
  double missingVolume;
  double missingArea;
  int scoopIdStart;
  int scoopIdEnd;
};

struct Scoop {
  int workspaceId;
  int digAreaId;
  int scoopId;
  double volume; // m^3
  double area; // m^2
  double estimatedVolume; // m^3
  double duration; // seconds

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
  void computeWorkspaceProperties(DigZone& digZone, std::function<bool(const grid_map::Index&)> filter);
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
  void computeSdf(std::string targetLayer, std::string sdfLayerName);

  std::vector<Eigen::Vector3d> digTrajectory(Eigen::Vector3d& base_digPosition);
  void optimizeTrajectory();
  void optimizeDigTrajectory();
  void optimizeRefinementTrajectory();
  bool refiningWorkspace() { return digZoneId_ == 3; };
  Trajectory computeTrajectory(Eigen::Vector3d& w_P_wd, std::string targetLayer, bool debug=false);
  Trajectory computeDigTrajectory(Eigen::Vector3d& w_P_wd, std::string targetLayer, bool debug=false);
  Trajectory computeDirtTrajectory(Eigen::Vector3d& w_P_wd, std::string targetLayer, bool debug=false);
  Trajectory computeRefinementTrajectory(Eigen::Vector3d& w_P_wd, std::string targetLayer, bool debug=false, bool markRefinedArea=false);
  void markDugAreaAsRefined();
  double getDigAttitude(double distanceFromBase);
  double volumeObjective(Trajectory trajectory);
  double sweptAreaObjective(Trajectory trajectory);
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
  bool isRefinementZoneComplete();
  bool isZoneActive(int digZoneId, bool isDigging);
  bool isLocalWorkspaceComplete();
  bool initializeLocalWorkspace();
  void updateWorkingArea();
  void setWorkingAreaInPreviousStates();
  void updateDugZones();
  // select dumping zone based on this score
  void sdfDumpingAreas();
  bool isLateralFrontZoneComplete(int zoneId);
  std::vector<int> completedDumpAreas_ = {0, 0, 0, 0};
  std::vector<int> completedDigAreas_ = {1, 0, 0, 0}; // last area is refinement area
  void completeDigArea(int zoneId);
  bool isRefinementComplete() { return completedDigAreas_.at(3); };
  double missingCellsAreaRatio_ = 0;
  int numMissingCellsArea_ = 0;
  void checkScoopedVolume(double volume);
  double minScoopVolume_;
  double ignoreScoopVolume_;
  double ignoreScoopArea_;
  int lowVolumeScoopAttempts_;

  void setDigTrajectory(Trajectory& trajectory) { digTrajectory_ = trajectory; };
  Trajectory getDigTrajectory() { return digTrajectory_; };
  Trajectory getOptimalTrajectory();

  Eigen::Vector3d projectVectorOntoSubspace(Eigen::Vector3d& vector, Eigen::Matrix3Xd& subspaceBasis);
  void markDigDumpAreas();
  void resetDigDumpAreas();

  void clearElevationMapAtShovelTrajectory();

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
  void addRefinementZoneToMap(double value);
  void createPlanningZones();
  void choosePlanningZones();
  int chooseDigZone();
  int chooseDumpZone(int digZone);
  void setWorkspacePose(Eigen::Vector3d& workspacePos, Eigen::Quaterniond& workspaceOrientation);
  double distanceZones(int zoneId1, int zoneId2);

  std::vector<Eigen::Vector2d> getOuterDiggingPatchVertices();
  std::vector<Eigen::Vector2d> getDiggingPatchVertices();
  std::vector<Eigen::Vector2d> getDiggingSawPatchVertices();
  std::vector<Eigen::Vector2d> getDiggingSawtoothVertices();
  std::vector<Eigen::Vector2d> getLeftFrontPatch(double startAngle = M_PI/4);
  std::vector<Eigen::Vector2d> getLeftFrontPatchAdaptive(std::vector<Eigen::Vector2d>& w_digPatch);
  std::vector<Eigen::Vector2d> getLeftCircularFrontSegmentPatch();
  std::vector<Eigen::Vector2d> getRightFrontPatch(double startAngle = -M_PI/4);
  std::vector<Eigen::Vector2d> getRightFrontPatchAdaptive(std::vector<Eigen::Vector2d>& w_digPatch);
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
  void loadWorkspace(double workspaceId = -1, bool fromSamePosition = false);
  void setCurrentWorkspaceIndex(int index) { currentWorkspaceIndex_ = index; }
  int getCurrentWorkspaceIndex() { return currentWorkspaceIndex_; }
  int getDigZoneId() { return digZoneId_; }
  void createConvexHull(std::vector<Eigen::Vector2d>& points, std::vector<Eigen::Vector2d>& hull);

  void removeSaddles(std::vector<Eigen::Vector3d>& points);
  void getShovelOrientation(std::vector<Eigen::Vector3d>& digPoints, std::vector<Eigen::Quaterniond>& orientations,
                            double draggingDistance, double targetPitch, double initialPitch, double heading);
  double getHeading(Eigen::Vector3d& w_P_wd);
  double getRelativeHeading(Eigen::Vector3d& w_P_wd);
  Eigen::Quaterniond getOrientation();
  Eigen::Vector3d getBasePosition();
  double getMaxDigAngle();
  // trajectory helpers
  std::vector<Eigen::Vector3d>  smoothZCoordinates(std::vector<Eigen::Vector3d>& trajectory);
  double smoothZCoordinates_ = true;
  void disableSmoothZCoordinates() { smoothZCoordinates_ = false; }
  void enableSmoothZCoordinates() { smoothZCoordinates_ = true; }
  void setWorkingDirection(Eigen::Vector2d& workingDirection) { workingDirection_ = workingDirection; }
  void publishCollisionTrajectory(std::vector<Eigen::Vector3d> pos, std::vector<Eigen::Quaterniond> orientations);
  bool updateShovelCollisionBody(Eigen::Vector3d w_P_ws, Eigen::Quaterniond C_ws);
  void publishCollisionPts(Eigen::Vector3d w_P_wd, Eigen::Quaterniond R_ws_d);
  void logPrecision();
  void logScoop(Trajectory& traj, double duration, double volume);
  void logDigArea(int zoneId);
  void createLogFiles();
  void savePlanningMap();
  // digAreaDuration
  ros::Time digAreaStartTime_;
  ros::Time digAreaEndTime_;
  std::string logPath_;
  std::string logName_;
  std::string logDir_;
  std::string logScoopsPath_;
  std::string logDigAreaPath_;
  int scoopCounter_ = 0;
  int startScoopAreaCounter_ = 0;

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
  Eigen::Vector3d workspaceDigPos_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond workspaceOrientation_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond workspaceDigOrientation_ = Eigen::Quaterniond::Identity();

  grid_map::Position3 diggingPoint_;
  grid_map::Index diggingPointLocalIndex_;
  grid_map::Position3 dumpPoint_;
  grid_map::Index dumpPointIndex_;

  // trajectory
  Trajectory digTrajectory_ = Trajectory();
  Trajectory optimalDigTrajectory_ = Trajectory();
  Trajectory dumpTrajectory_ = Trajectory();

  // self collisions
  excavator_model::ExcavatorModel model_; // copy of the model used for planning
  collisions::CollisionGroup armCollisionGroup_;
  collisions::CollisionGroup boomCollisionGroup_;
  collisions::CollisionGroup legCollisionGroup_;
  collisions_bullet::ColliderManagerBullet colliderManager_;
  collisions::CollisionGroup armGroup_;
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
  std::shared_ptr<collisions_visualization::CollisionGeometryMarkerPublisher> collisionPubPtr_;


  ros::Subscriber footprintSubscriber_;
  void footprintCallback(const m545_planner_msgs::M545FootprintRos& msg);
  boost::shared_mutex footprintMutex_;
  Eigen::Matrix<double, 5, 2> footprint_;
  double footprintInflation_ = 1.0;

  ros::Subscriber globalWorkspaceSub_;
  void globalWorkspaceCallback(const geometry_msgs::PoseArray& msg);
  std::vector<geometry_msgs::Pose> globalWorkspace_;
  int currentWorkspaceIndex_ = -1;
  void setExcavationMaskAtFutureStates();
  void setExcavationMaskAtDigZone();
  void processCurrentExcavationMask();

  std::string workspaceTopic_;
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
  grid_map::Polygon digOuterZone_;
  grid_map::Polygon dumpingLeftFrontZone_;
  grid_map::Polygon dumpingRightFrontZone_;
  grid_map::Polygon dumpingLeftBackZone_;
  grid_map::Polygon dumpingRightBackZone_;
  grid_map::Polygon refinementZone_;
  bool doRefinement_;

  // dig area headings
  double minRelHeading_;
  double maxRelHeading_;

  void createShovelFilter();
  std::vector<Eigen::Vector2d> shovelFilter_;
  // compute volume between shovel pts
  std::tuple<double, double> computeVolumeBetweenShovelPoints(Eigen::Vector3d& w_posLeftShovel_wl, Eigen::Vector3d& w_posRightShovel_wr,
                                                              double previousTerrainElevation);
  double computeSweptAreaBetweenShovelPoints(Eigen::Vector3d& w_posLeftShovel_wl, Eigen::Vector3d& w_posRightShovel_wr );
  // to speed up completion
  void markAsDugShovelPointsCloseToDesiredElevation();
  bool markDugShovelPointsCloseToDesiredElevation_;
  // used to check if layer is finished. Sometimes the edges are cause problems
  grid_map::Polygon shrinkGridMapPolygon(double shrinkFactor, grid_map::Polygon& polygon);
  // elevation map params
  double minElevationVar_;
  double maxElevationVar_;

  void computeDesiredAverageHeight();
  double desiredAverageHeight_;

  // to facilitate restart
  bool markDugPreviousWaypoints_ = false;

  // current dig and dump zone
  int previousDigZoneId_ = -1;
  int previousDumpZoneId_ = -1;
  int dumpZoneId_ = -1;
  int digZoneId_ = -1;

  // workspace volume
  double workspaceVolume_;
  double estimatedWorkspaceVolume_ = 0;
  std::vector<double> digZonesVolume_;

  // boolean to indicate whether a reset is needed
  bool createNewZones_ = true;
  bool nextWorkspaceFromSamePose_ = false;
  double remainingVolumeRatio_;
  double missingCellsRatio_;
  double remainingVolume_;
  double rmsPrecision_;
  double volume_;
  double numMissingCells_;
  double numTotalCells_;
  double remainingSweptAreaRatio_;

  // optimization weight
  double volumeWeight_;
  double distanceWeight_;
  double headingWeight_;
  double headingWeightRefinement_;
  double sweptAreaWeight_;

  // parameters
  // dig trajectory depth
  double maxDigDepth_;
  double maxDigDirtDepth_;
  double closingZTranslation_;
  double minDistanceCollision_;
  double minDistanceCollisionRefinement_;
  double targetDigAttitude_;
  double targetDigAttitudeInner_;
  double targetDigDirtAttitude_;
  double targetDigDirtAttitudeInner_;
  double targetRefinementAttitude_;
  double targetRefinementAttitudeInner_;
  // refinement
  double previousRefinementHeading_;
  double refinementAngleIncrement_;
  double effectiveShovelWidth_;
  double effectiveShovelWidthRefinement_;
  // drag shovel angle
  double draggingAngle_;
  // areas with more than this ratio (dug area / total area) of the total area are considered not active
  double inactiveAreaRatio_;
  double excavationAreaRatio_;
  // selection fo dumping workspace
  double digDumpDistanceWeight_;
  double workingDirWeight_;
  double dumpingZoneDistanceWeight_;
  // digging
  double closingRadialTranslation_;
  double closingRadialTranslationRefinement_;
  double closingHeightCoefficient_;
  double closingHeightCoefficientRefinement_;
  double closingAngleCoefficient_;
  double closingAngleCoefficientRefinement_;

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
  double refinementCellsThreshold_;
  double refinementAreaThreshold_;
  double volumeDumpThreshold_ = 0.5 * shovelWidth_ * shovelLength_;
  // we don't account the initial volume that goes into the shovel in the penetration phase
  // todo: account properly for it
  double shovelVolumeBonus_;
  // the accurary required for dirt handling is lower than the accuracy of digging
  double volumeDirtThreshold_;
  double heightDirtThreshold_;
  double heightDigAreaThreshold_;
  double digZoneShrinkFactor_;
  int digZoneTeethNumber_;
  double depthBias_;
  double refinementDepthBias_;
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
  double circularOuterWorkspaceOuterRadius_;
  double circularOuterWorkspaceInnerRadiusFactor_;
  double circularOuterWorkspaceInnerRadius_;
  double circularOuterWorkspaceOuterRadiusFactor_;
  double circularOuterWorkspaceAngle_;
  double circularOuterWorkspaceAngleFactor_;
  double dumpingZoneOuterRadius_;
  double dumpingZoneInnerRadius_;
  double minDistanceShovelToBase_;
  double minDistanceShovelToBaseDig_;
  double minDistanceShovelToBaseRefined_;
  double circularWorkspaceAngle_;
  int currentTrackId = 0;
  // param to mark working areas that you can't step over
  double heightTraversableThreshold_;
  double targetHeightDiffThreshold_;
  // workspace logging
  // dictionary with key the waypoint index and as value the remaining volume ratio
  std::map<int, double> remainingVolumeRatiosMap_;
  std::map<int, double> remainingVolumeMap_;
  std::map<int, double> remainingCellsRatioMap_;
  std::map<int, double> workspaceVolumeMap_;
  std::map<int, double> estimatedWorkspaceVolumeMap_;
  std::map<int, double> diggingTimeMap_;
  std::map<int, double> diggingDirtTimeMap_;
  std::map<int, double> refiningTimeMap_;

  // saving params maps
  std::string saveMapPath_;
  // recovery behaviour
  int lowVolumeScoopCounter_ = 0;
  
  // current positions and orientations
  void updateCurrentPositionsAndOrientations();
  Eigen::Vector3d w_P_wba_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond R_mba_q_ = Eigen::Quaterniond::Identity();  
  Eigen::Vector3d rpy_ = Eigen::Vector3d::Zero();
  //
  double dugSdfAlpha_ = 1;
  double dugSdfBeta_ = 1;
};

}  // namespace local_excavation