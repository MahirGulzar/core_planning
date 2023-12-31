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

#ifndef OP_BEHAVIOR_SELECTOR_CORE
#define OP_BEHAVIOR_SELECTOR_CORE

#include <ros/ros.h>

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

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/LaneArrayStamped.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <autoware_can_msgs/CANInfo.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/TrafficLight.h>
#include <autoware_msgs/Signals.h>
#include <autoware_msgs/ControlCommandStamped.h>
#include <autoware_msgs/ControlCommand.h>
#include <autoware_msgs/Waypoint.h>
#include <autoware_msgs/VehicleStatus.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_lanelet2_msgs/MapBin.h>
#include "op_planner/PlannerCommonDef.h"
#include "op_planner/DecisionMaker.h"
#include "op_utility/DataRW.h"

#define LOG_LOCAL_PLANNING_DATA

namespace BehaviorGeneratorNS
{

class BehaviorGen
{
protected: //Planning Related variables

	//Control Related
	int m_ControlFrequency;
	std::vector<double> dt_list;

	geometry_msgs::Pose m_OriginPos;
	PlannerHNS::WayPoint m_CurrentPos;
	bool bNewCurrentPos;

	PlannerHNS::VehicleState m_VehicleStatus;
	bool bVehicleStatus;

	std::vector<PlannerHNS::WayPoint> m_temp_path;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPaths;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPathsToUse;
	std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > > m_LanesRollOutsToUse;

	PlannerHNS::MAP_SOURCE_TYPE m_MapType;
	std::string m_MapPath;

	PlannerHNS::RoadNetwork m_Map;
	bool bMap;

	PlannerHNS::TrajectoryCost m_TrajectoryBestCost;
	bool bBestCost;

	PlannerHNS::DecisionMaker m_BehaviorGenerator;
	PlannerHNS::BehaviorState m_CurrentBehavior;
	bool m_bRequestNewPlanSent;
	bool m_bShowActualDrivingPath;

  	std::vector<std::string> m_LogData;
  	std::vector<std::pair<PlannerHNS::WayPoint, PlannerHNS::PolygonShape> > m_ActualDrivingPath;

  	PlannerHNS::PlanningParams m_PlanningParams;
  	PlannerHNS::CAR_BASIC_INFO m_CarInfo;
  	PlannerHNS::ControllerParams m_ControlParams;

  	autoware_msgs::Lane m_CurrentTrajectoryToSend;
  	bool bNewLightStatus;
	bool bNewLightSignal;
	PlannerHNS::TRAFFIC_LIGHT_TYPE  m_CurrLightStatus;
	std::vector<PlannerHNS::TrafficLight> m_CurrTrafficLight;
	std::vector<PlannerHNS::TrafficLight> m_PrevTrafficLight;

	autoware_msgs::ControlCommandStamped m_Ctrl_cmd;
	autoware_msgs::ControlCommandStamped m_Ctrl_raw;

	std::string m_ExperimentFolderName;

	std_msgs::String stopline_rviz_info_text;

	//ROS messages (topics)
	ros::NodeHandle nh;

	//define publishers
	ros::Publisher pub_TotalLocalPath;
	ros::Publisher pub_LocalPath;
	ros::Publisher pub_LocalBasePath;
	ros::Publisher pub_ClosestIndex;
	ros::Publisher pub_BehaviorState;
	ros::Publisher pub_SimuBoxPose;
	ros::Publisher pub_SelectedPathRviz;
	ros::Publisher pub_TargetSpeedRviz;
	ros::Publisher pub_ActualSpeedRviz;
	ros::Publisher pub_DetectedLight;
	ros::Publisher pub_CurrGlobalLocalPathsIds;
	ros::Publisher pub_RequestReplan;
	ros::Publisher pub_BehaviorStateRviz;
	ros::Publisher pub_CurrDrivingPathRviz;
	ros::Publisher pub_stopLineInfoRviz;
	ros::Publisher pub_stoppingWall;

	// define subscribers.
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_current_velocity;
	ros::Subscriber sub_robot_odom;
	ros::Subscriber sub_can_info;
	ros::Subscriber sub_vehicle_status;
	ros::Subscriber sub_GlobalPlannerPaths;
	ros::Subscriber sub_LocalPlannerPaths;
	ros::Subscriber sub_Trajectory_Cost;
	ros::Subscriber sub_TrafficLightStatus;
	ros::Subscriber sub_TrafficLightSignals;

	ros::Subscriber sub_twist_cmd;
	ros::Subscriber sub_twist_raw;
	ros::Subscriber sub_ctrl_cmd;
	ros::Subscriber sub_ctrl_raw;

	// Control Topics Sections
	//----------------------------
	void callbackGetTwistRaw(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetTwistCMD(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetControlCMD(const autoware_msgs::ControlCommandStampedConstPtr& msg);
	void callbackGetControlRaw(const autoware_msgs::ControlCommandStampedConstPtr& msg);
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
	void callbackGetAutowareStatus(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg);
	void callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg);
	void callbackGetVehicleStatus(const autoware_msgs::VehicleStatusConstPtr & msg);
	//----------------------------

	//Path Planning Section
	//----------------------------
	void callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg);
	void callbackGetLocalPlannerPath(const autoware_msgs::LaneArrayStampedConstPtr& msg);
	void callbackGetLocalTrajectoryCost(const autoware_msgs::LaneConstPtr& msg);
	void CollectRollOutsByGlobalPath(std::vector< std::vector<PlannerHNS::WayPoint> >& local_rollouts);
	bool CompareTrajectoriesWithIds(std::vector<std::vector<PlannerHNS::WayPoint> >& paths, std::vector<int>& local_ids);
	//----------------------------

	//Traffic Information Section
	//----------------------------
	void callbackGetTrafficLightStatus(const autoware_msgs::TrafficLight & msg);
	void callbackGetTrafficLightSignals(const autoware_msgs::Signals& msg);
	//----------------------------

	//Helper Functions
  void UpdatePlanningParams(ros::NodeHandle& _nh);
  void SendLocalPlanningTopics();
  void VisualizeLocalPlanner();
  void LogLocalPlanningInfo(double dt);
  void InsertNewActualPathPair(const double& min_record_distance = 2.0);

public:
  BehaviorGen();
  ~BehaviorGen();
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

#endif  // OP_BEHAVIOR_SELECTOR_CORE
