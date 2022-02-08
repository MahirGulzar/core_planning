/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef OP_TRAJECTORY_EVALUATOR_CORE
#define OP_TRAJECTORY_EVALUATOR_CORE

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/LaneArrayStamped.h>
#include <autoware_can_msgs/CANInfo.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/VehicleStatus.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

#include "vector_map_msgs/PointArray.h"
#include "vector_map_msgs/LaneArray.h"
#include "vector_map_msgs/NodeArray.h"
#include "vector_map_msgs/StopLineArray.h"
#include "vector_map_msgs/DTLaneArray.h"
#include "vector_map_msgs/LineArray.h"
#include "vector_map_msgs/AreaArray.h"
#include "vector_map_msgs/SignalArray.h"
#include "vector_map_msgs/StopLine.h"
#include "vector_map_msgs/VectorArray.h"

#include <autoware_msgs/TrafficLight.h>
#include <autoware_msgs/Signals.h>
#include <autoware_msgs/ControlCommandStamped.h>
#include <autoware_msgs/ControlCommand.h>
#include <autoware_msgs/Waypoint.h>
#include <autoware_lanelet2_msgs/MapBin.h>
#include "op_planner/PlannerCommonDef.h"
#include "op_utility/DataRW.h"

#include "op_planner/PlannerCommonDef.h"
#include "op_planner/TrajectoryEvaluator.h"

namespace TrajectoryEvaluatorNS
{

#define LOG_LOCAL_PLANNING_DATA

class TrajectoryEvalCore
{
protected:

    PlannerHNS::TrajectoryEvaluator m_TrajectoryCostsCalculator;
	bool m_bUseMoveingObjectsPrediction;

	geometry_msgs::Pose m_OriginPos;

	PlannerHNS::WayPoint m_CurrentPos;
	bool bNewCurrentPos;

	PlannerHNS::VehicleState m_VehicleStatus;
	bool bVehicleStatus;
	bool m_bKeepCurrentIfPossible;

	std::vector<PlannerHNS::WayPoint> m_temp_path;
	std::vector<int> m_CurrGlobalPathsIds;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPaths;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPathsToUse;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPathSections;
    ros::Time path_stamp;
	std::vector<int> m_prev_index;
	std::vector<PlannerHNS::WayPoint> t_centerTrajectorySmoothed;
	std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > > m_LanesRollOutsToUse;

	std::vector<PlannerHNS::DetectedObject> m_PredictedObjects;
	autoware_msgs::DetectedObjectArray m_EvaluatedObjects;
	std_msgs::Header latest_header;
	bool bPredictedObjects;


	struct timespec m_PlanningTimer;
  	std::vector<std::string>    m_LogData;

	PlannerHNS::MAP_SOURCE_TYPE m_MapType;
	std::string m_MapPath;

	PlannerHNS::RoadNetwork m_Map;
	bool bMap;
	bool bRoadSigns;


  	PlannerHNS::EvaluationParams m_EvaluationParams;
  	PlannerHNS::PlanningParams m_PlanningParams;
  	PlannerHNS::PlanningParams m_ModPlanningParams;
  	PlannerHNS::CAR_BASIC_INFO m_CarInfo;

  	PlannerHNS::BehaviorState m_CurrentBehavior;
  	double m_AdditionalFollowDistance;


  	visualization_msgs::MarkerArray m_CollisionsDummy;
	visualization_msgs::MarkerArray m_CollisionsActual;

	std::string m_ExperimentFolderName;
	std::string m_EstimatedObjectsTopicName;

	//ROS messages (topics)
	ros::NodeHandle nh;

	//define publishers
	ros::Publisher pub_CollisionPointsRviz;
	ros::Publisher pub_LocalWeightedTrajectoriesRviz;
	ros::Publisher pub_LocalWeightedTrajectories;
	ros::Publisher pub_TrajectoryCost;
	ros::Publisher pub_SafetyBorderRviz;
	ros::Publisher pub_yieldEvalAttention;
	ros::Publisher pub_roiMarkers;
	

	// define subscribers.
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_current_velocity;
	ros::Subscriber sub_robot_odom;
	ros::Subscriber sub_can_info;
	ros::Subscriber sub_vehicle_status;
	ros::Subscriber sub_GlobalPlannerPaths;
	ros::Subscriber sub_LocalPlannerPaths;
	ros::Subscriber sub_predicted_objects;
	ros::Subscriber sub_CurrGlobalLocalPathsIds;


	// Callback function for subscriber.
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
	void callbackGetAutowareStatus(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg);
	void callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg);
	void callbackGetVehicleStatus(const autoware_msgs::VehicleStatusConstPtr & msg);
	void callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg);
	void callbackGetLocalPlannerPath(const autoware_msgs::LaneArrayStampedConstPtr& msg);
	void callbackGetPredictedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg);
	void callbackGetTrajectoryInforFromBehaviorSelector(const std_msgs::Int32MultiArrayConstPtr& msg);

	//Helper Functions
  void UpdatePlanningParams(ros::NodeHandle& _nh);
  //int GetGlobalPathIndex(const int& iCurrTrajectory);
  void CollectRollOutsByGlobalPath(std::vector< std::vector<PlannerHNS::WayPoint> >& local_rollouts);
  void BalanceFactorsToOne(double& priority, double& transition, double& longi, double& lateral, double& change);
  bool FindBestLane(std::vector<PlannerHNS::TrajectoryCost> tcs, PlannerHNS::TrajectoryCost& best_l);
  bool CompareTrajectoriesWithIds(std::vector<std::vector<PlannerHNS::WayPoint> >& paths, std::vector<int>& local_ids);

public:
  TrajectoryEvalCore();
  ~TrajectoryEvalCore();
  void MainLoop();

  //Mapping Section
  //----------------------------
	UtilityHNS::MapRaw m_MapRaw;
	ros::Subscriber sub_bin_map;
	ros::Subscriber sub_lanes;
	ros::Subscriber sub_points;
	ros::Subscriber sub_dt_lanes;
	ros::Subscriber sub_intersect;
	ros::Subscriber sup_area;
	ros::Subscriber sub_lines;
	ros::Subscriber sub_stop_line;
	ros::Subscriber sub_signals;
	ros::Subscriber sub_signs;
	ros::Subscriber sub_vectors;
	ros::Subscriber sub_curbs;
	ros::Subscriber sub_edges;
	ros::Subscriber sub_way_areas;
	ros::Subscriber sub_cross_walk;
	ros::Subscriber sub_nodes;


	void callbackGetLanelet2(const autoware_lanelet2_msgs::MapBin& msg);
	void callbackGetVMLanes(const vector_map_msgs::LaneArray& msg);
	void callbackGetVMPoints(const vector_map_msgs::PointArray& msg);
	void callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg);
	void callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg);
	void callbackGetVMAreas(const vector_map_msgs::AreaArray& msg);
	void callbackGetVMLines(const vector_map_msgs::LineArray& msg);
	void callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg);
	void callbackGetVMSignal(const vector_map_msgs::SignalArray& msg);
	void callbackGetVMSign(const vector_map_msgs::RoadSignArray& msg);
	void callbackGetVMVectors(const vector_map_msgs::VectorArray& msg);
	void callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg);
	void callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg);
	void callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg);
	void callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg);
	void callbackGetVMNodes(const vector_map_msgs::NodeArray& msg);
	//----------------------------
};

	

}

#endif  // OP_TRAJECTORY_EVALUATOR_CORE
