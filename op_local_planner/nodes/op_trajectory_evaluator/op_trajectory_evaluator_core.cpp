/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#include "op_trajectory_evaluator_core.h"
#include "op_ros_helpers/op_ROSHelpers.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/KmlMapLoader.h"
#include "op_planner/Lanelet2MapLoader.h"
#include "op_planner/VectorMapLoader.h"
#include <numeric>


namespace TrajectoryEvaluatorNS
{

TrajectoryEvalCore::TrajectoryEvalCore()
{
	bNewCurrentPos = false;
	bVehicleStatus = false;
	bMap = false;
	m_bUseMoveingObjectsPrediction = false;
	m_bKeepCurrentIfPossible = false;
	m_AdditionalFollowDistance = 10; // meters

	ros::NodeHandle _nh;
	UpdatePlanningParams(_nh);

	tf::StampedTransform transform;
	tf::TransformListener tf_listener;
	PlannerHNS::ROSHelpers::getTransformFromTF("world", "map", tf_listener, transform);
	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();

	pub_CollisionPointsRviz = nh.advertise<visualization_msgs::MarkerArray>("dynamic_collision_points_rviz", 1);
	pub_LocalWeightedTrajectoriesRviz = nh.advertise<visualization_msgs::MarkerArray>("local_trajectories_eval_rviz", 1);
	pub_LocalWeightedTrajectories = nh.advertise<autoware_msgs::LaneArrayStamped>("local_weighted_trajectories", 1);
	pub_TrajectoryCost = nh.advertise<autoware_msgs::Lane>("local_trajectory_cost", 1);
	pub_SafetyBorderRviz = nh.advertise<visualization_msgs::Marker>("safety_border", 1);
	pub_yieldEvalAttention = nh.advertise<autoware_msgs::DetectedObjectArray>("yield_eval_attention", 1);
	pub_roiMarkers = nh.advertise<visualization_msgs::MarkerArray>("rviz_roi_points", 1);

	sub_current_pose = nh.subscribe("/current_pose", 1, &TrajectoryEvalCore::callbackGetCurrentPose, this);

	int bVelSource = 1;
	_nh.getParam("/op_common_params/velocitySource", bVelSource);
	std::string velocity_topic;
	if(bVelSource == 0)
	{
		sub_robot_odom = nh.subscribe("/vehicle/odom", 1, &TrajectoryEvalCore::callbackGetRobotOdom, this);
	}
	else if(bVelSource == 1)
	{
		sub_current_velocity = nh.subscribe("/current_velocity", 1, &TrajectoryEvalCore::callbackGetAutowareStatus, this);
	}
	else if(bVelSource == 2)
	{
		sub_can_info = nh.subscribe("/can_info", 1, &TrajectoryEvalCore::callbackGetCANInfo, this);
	}
	else if(bVelSource == 3)
	{
		_nh.getParam("/op_common_params/vehicle_status_topic", velocity_topic);
		sub_vehicle_status = _nh.subscribe(velocity_topic, 1, &TrajectoryEvalCore::callbackGetVehicleStatus, this);
	}

	sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array", 1, &TrajectoryEvalCore::callbackGetGlobalPlannerPath, this);
	sub_LocalPlannerPaths = nh.subscribe("/local_trajectories", 1, &TrajectoryEvalCore::callbackGetLocalPlannerPath, this);
	sub_predicted_objects = nh.subscribe("/predicted_objects", 1, &TrajectoryEvalCore::callbackGetPredictedObjects, this);
	sub_CurrGlobalLocalPathsIds = nh.subscribe("/op_curr_global_local_ids", 1, &TrajectoryEvalCore::callbackGetTrajectoryInforFromBehaviorSelector, this);

	m_TrajectoryCostsCalculator.SetEvalParams(m_EvaluationParams);
	PlannerHNS::ROSHelpers::InitCollisionPointsMarkers(500, m_CollisionsDummy);

	//Mapping Section
	if(m_MapType == PlannerHNS::MAP_AUTOWARE)
	{
		sub_bin_map = nh.subscribe("/lanelet_map_bin", 1, &TrajectoryEvalCore::callbackGetLanelet2, this);
		sub_lanes = nh.subscribe("/vector_map_info/lane", 1, &TrajectoryEvalCore::callbackGetVMLanes,  this);
		sub_points = nh.subscribe("/vector_map_info/point", 1, &TrajectoryEvalCore::callbackGetVMPoints,  this);
		sub_dt_lanes = nh.subscribe("/vector_map_info/dtlane", 1, &TrajectoryEvalCore::callbackGetVMdtLanes,  this);
		sub_intersect = nh.subscribe("/vector_map_info/cross_road", 1, &TrajectoryEvalCore::callbackGetVMIntersections,  this);
		sup_area = nh.subscribe("/vector_map_info/area", 1, &TrajectoryEvalCore::callbackGetVMAreas,  this);
		sub_lines = nh.subscribe("/vector_map_info/line", 1, &TrajectoryEvalCore::callbackGetVMLines,  this);
		sub_stop_line = nh.subscribe("/vector_map_info/stop_line", 1, &TrajectoryEvalCore::callbackGetVMStopLines,  this);
		sub_signals = nh.subscribe("/vector_map_info/signal", 1, &TrajectoryEvalCore::callbackGetVMSignal,  this);
		sub_signs = nh.subscribe("/vector_map_info/road_sign", 1, &TrajectoryEvalCore::callbackGetVMSign,  this);
		sub_vectors = nh.subscribe("/vector_map_info/vector", 1, &TrajectoryEvalCore::callbackGetVMVectors,  this);
		sub_curbs = nh.subscribe("/vector_map_info/curb", 1, &TrajectoryEvalCore::callbackGetVMCurbs,  this);
		sub_edges = nh.subscribe("/vector_map_info/road_edge", 1, &TrajectoryEvalCore::callbackGetVMRoadEdges,  this);
		sub_way_areas = nh.subscribe("/vector_map_info/way_area", 1, &TrajectoryEvalCore::callbackGetVMWayAreas,  this);
		sub_cross_walk = nh.subscribe("/vector_map_info/cross_walk", 1, &TrajectoryEvalCore::callbackGetVMCrossWalks,  this);
		sub_nodes = nh.subscribe("/vector_map_info/node", 1, &TrajectoryEvalCore::callbackGetVMNodes,  this);
	}
}

TrajectoryEvalCore::~TrajectoryEvalCore()
{
}

void TrajectoryEvalCore::UpdatePlanningParams(ros::NodeHandle& _nh)
{
	_nh.getParam("/op_trajectory_evaluator/enablePrediction", m_bUseMoveingObjectsPrediction);
	_nh.getParam("/op_trajectory_evaluator/keepCurrentTrajectory", m_bKeepCurrentIfPossible);


	_nh.getParam("/op_trajectory_evaluator/weight_priority", m_EvaluationParams.priority_weight_);
	_nh.getParam("/op_trajectory_evaluator/weight_transition", m_EvaluationParams.transition_weight_);
	_nh.getParam("/op_trajectory_evaluator/weight_longitudinal", m_EvaluationParams.longitudinal_weight_);
	_nh.getParam("/op_trajectory_evaluator/weight_lateral", m_EvaluationParams.lateral_weight_);
	_nh.getParam("/op_trajectory_evaluator/weight_lane_change", m_EvaluationParams.lane_change_weight_);
	_nh.getParam("/op_trajectory_evaluator/collision_time", m_EvaluationParams.collision_time_);
	_nh.getParam("/op_motion_predictor/min_prediction_time", m_EvaluationParams.min_prediction_time_);
	_nh.getParam("/op_motion_predictor/min_prediction_distance", m_EvaluationParams.min_prediction_distance_);

	_nh.getParam("/op_common_params/horizontalSafetyDistance", m_PlanningParams.horizontalSafetyDistancel);
	_nh.getParam("/op_common_params/verticalSafetyDistance", m_PlanningParams.verticalSafetyDistance);
	_nh.getParam("/op_common_params/enableSwerving", m_PlanningParams.enableSwerving);
	if(m_PlanningParams.enableSwerving)
		m_PlanningParams.enableFollowing = true;
	else
		_nh.getParam("/op_common_params/enableFollowing", m_PlanningParams.enableFollowing);

	_nh.getParam("/op_common_params/enableTrafficLightBehavior", m_PlanningParams.enableTrafficLightBehavior);
	_nh.getParam("/op_common_params/enableStopSignBehavior", m_PlanningParams.enableStopSignBehavior);

	_nh.getParam("/op_common_params/maxVelocity", m_PlanningParams.maxSpeed);
	_nh.getParam("/op_common_params/minVelocity", m_PlanningParams.minSpeed);
	_nh.getParam("/op_common_params/maxLocalPlanDistance", m_PlanningParams.microPlanDistance);

	_nh.getParam("/op_common_params/pathDensity", m_PlanningParams.pathDensity);

	_nh.getParam("/op_common_params/rollOutDensity", m_PlanningParams.rollOutDensity);
	if(m_PlanningParams.enableSwerving)
		_nh.getParam("/op_common_params/rollOutsNumber", m_PlanningParams.rollOutNumber);
	else
		m_PlanningParams.rollOutNumber = 0;


	int iSource = 0;
	_nh.getParam("/op_common_params/mapSource" , iSource);
	if(iSource == 0)
		m_MapType = PlannerHNS::MAP_AUTOWARE;
	else if (iSource == 1)
		m_MapType = PlannerHNS::MAP_FOLDER;
	else if(iSource == 2)
		m_MapType = PlannerHNS::MAP_KML_FILE;
	else if(iSource == 3)
	{
		m_MapType = PlannerHNS::MAP_LANELET_2;
		std::string str_origin;
		nh.getParam("/op_common_params/lanelet2_origin" , str_origin);
		std::vector<std::string> lat_lon_alt = PlannerHNS::MappingHelpers::SplitString(str_origin, ",");
		if(lat_lon_alt.size() == 3)
		{
			m_Map.origin.pos.lat = atof(lat_lon_alt.at(0).c_str());
			m_Map.origin.pos.lon = atof(lat_lon_alt.at(1).c_str());
			m_Map.origin.pos.alt = atof(lat_lon_alt.at(2).c_str());
		}
	}

	_nh.getParam("/op_common_params/horizonDistance", m_PlanningParams.horizonDistance);
	_nh.getParam("/op_common_params/minFollowingDistance", m_PlanningParams.minFollowingDistance);
	_nh.getParam("/op_common_params/minDistanceToAvoid", m_PlanningParams.minDistanceToAvoid);
	_nh.getParam("/op_common_params/maxDistanceToAvoid", m_PlanningParams.maxDistanceToAvoid);
	_nh.getParam("/op_common_params/speedProfileFactor", m_PlanningParams.speedProfileFactor);

	_nh.getParam("/op_common_params/enableLaneChange", m_PlanningParams.enableLaneChange);
	if(!m_PlanningParams.enableLaneChange)
	{
		m_EvaluationParams.lane_change_weight_ = 0;
		BalanceFactorsToOne(m_EvaluationParams.priority_weight_, m_EvaluationParams.transition_weight_,
				m_EvaluationParams.longitudinal_weight_, m_EvaluationParams.lateral_weight_, m_EvaluationParams.lane_change_weight_);

	}

	_nh.getParam("/op_common_params/front_length", m_CarInfo.front_length);
	_nh.getParam("/op_common_params/back_length", m_CarInfo.back_length);
	_nh.getParam("/op_common_params/height", m_CarInfo.height);
	_nh.getParam("/op_common_params/width", m_CarInfo.width);
	_nh.getParam("/op_common_params/length", m_CarInfo.length);
	_nh.getParam("/op_common_params/wheelBaseLength", m_CarInfo.wheel_base);
	_nh.getParam("/op_common_params/turningRadius", m_CarInfo.turning_radius);
	_nh.getParam("/op_common_params/maxWheelAngle", m_CarInfo.max_wheel_angle);
	_nh.getParam("/op_common_params/maxAcceleration", m_CarInfo.max_acceleration);
	_nh.getParam("/op_common_params/maxDeceleration", m_CarInfo.max_deceleration);
	m_CarInfo.max_speed_forward = m_PlanningParams.maxSpeed;
	m_CarInfo.min_speed_forward = m_PlanningParams.minSpeed;
	m_TrajectoryCostsCalculator.SetPlanningParams(m_PlanningParams);
}

void TrajectoryEvalCore::BalanceFactorsToOne(double& priority, double& transition, double& longitudinal, double& lateral, double& change)
	{
	std::vector<double> factors_list = {priority, transition, longitudinal, lateral, change};
	int nNonZero = 0;
	for(unsigned int i=0;i < factors_list.size(); i++)
	{
		if(factors_list.at(i) > 0.0)
			nNonZero++;
	}
	double all_factors = std::accumulate(factors_list.begin(), factors_list.end(), 0.0);

	while(all_factors > 1.01 || all_factors < 0.99)
	{
		double every_one_share = (1.0 - all_factors)/(double)nNonZero;
		for(unsigned int i=0;i < factors_list.size(); i++)
		{
			if(factors_list.at(i) > 0.0)
			{
				factors_list.at(i) += every_one_share;
			}

			if(factors_list.at(i) < 0.0)
				factors_list.at(i) = 0.0;
		}

		nNonZero = 0;
		for(unsigned int i=0;i < factors_list.size(); i++)
		{
			if(factors_list.at(i) > 0.0)
				nNonZero++;
		}
		all_factors = std::accumulate(factors_list.begin(), factors_list.end(), 0.0);

		if(all_factors == 0)
			break;
	}
	priority = factors_list.at(0);
	transition = factors_list.at(1);
	longitudinal = factors_list.at(2);
	lateral = factors_list.at(3);
	change = factors_list.at(4);

}

void TrajectoryEvalCore::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_CurrentPos.pos = PlannerHNS::GPSPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
	m_CurrentPos.pLane = PlannerHNS::MappingHelpers::GetClosestLaneFromMap(m_CurrentPos, m_Map);
	bNewCurrentPos = true;
}

void TrajectoryEvalCore::callbackGetAutowareStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.linear.x;
	m_CurrentPos.v = m_VehicleStatus.speed;
	if(fabs(msg->twist.linear.x) > 0.25)
		m_VehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.angular.z/msg->twist.linear.x);
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void TrajectoryEvalCore::callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg)
{
	m_VehicleStatus.speed = msg->speed/3.6;
	m_CurrentPos.v = m_VehicleStatus.speed;
	m_VehicleStatus.steer = msg->angle * m_CarInfo.max_wheel_angle / m_CarInfo.max_steer_value;
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void TrajectoryEvalCore::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.twist.linear.x;
	m_CurrentPos.v = m_VehicleStatus.speed;
	if(fabs(msg->twist.twist.linear.x) > 0.1)
		m_VehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void TrajectoryEvalCore::callbackGetVehicleStatus(const autoware_msgs::VehicleStatusConstPtr & msg)
{
	m_VehicleStatus.speed = msg->speed/3.6;
	m_VehicleStatus.steer = -msg->angle*DEG2RAD;
	m_CurrentPos.v = m_VehicleStatus.speed;
	bVehicleStatus = true;
	//std::cout << "Vehicle Real Status, Speed: " << m_VehicleStatus.speed << ", Steer Angle: " << m_VehicleStatus.steer << ", Steermode: " << msg->steeringmode << ", Org angle: " << msg->angle <<  std::endl;
}

void TrajectoryEvalCore::callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg)
{
	if(msg->lanes.size() > 0 && bMap)
	{
		m_GlobalPaths.clear();
		for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
		{
			PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(msg->lanes.at(i), m_temp_path);

			if(bMap)
			{
				PlannerHNS::Lane* pPrevValid = 0;
				for(unsigned int j = 0 ; j < m_temp_path.size(); j++)
				{
					PlannerHNS::Lane* pLane = 0;
					pLane = PlannerHNS::MappingHelpers::GetLaneById(m_temp_path.at(j).laneId, m_Map);
					if(!pLane)
					{
						pLane = PlannerHNS::MappingHelpers::GetClosestLaneFromMap(m_temp_path.at(j), m_Map, 1, true);

						if(!pLane && !pPrevValid)
						{
							ROS_ERROR("Map inconsistency between Global Path and Local Planer Map, Can't identify current lane.");
							return;
						}

						if(!pLane)
							m_temp_path.at(j).pLane = pPrevValid;
						else
						{
							m_temp_path.at(j).pLane = pLane;
							pPrevValid = pLane ;
						}

						m_temp_path.at(j).laneId = m_temp_path.at(j).pLane->id;
					}
					else
						m_temp_path.at(j).pLane = pLane;
				}
			}

			m_GlobalPaths.push_back(m_temp_path);
		}

		bool bOldGlobalPath = true;
		if(m_GlobalPathsToUse.size() == m_GlobalPaths.size())
		{
			for(unsigned int i=0; i < m_GlobalPaths.size(); i++)
			{
				bOldGlobalPath = PlannerHNS::PlanningHelpers::CompareTrajectories(m_GlobalPaths.at(i), m_GlobalPathsToUse.at(i));
			}
		}
		else
		{
			bOldGlobalPath = false;
		}

		if(!bOldGlobalPath)
		{
			//bWayGlobalPathLogs = true;
			for(unsigned int i = 0; i < m_GlobalPaths.size(); i++)
			{
				PlannerHNS::PlanningHelpers::FixPathDensity(m_GlobalPaths.at(i), m_PlanningParams.pathDensity);
				PlannerHNS::PlanningHelpers::SmoothPath(m_GlobalPaths.at(i), 0.4, 0.4, 0.05);
				PlannerHNS::PlanningHelpers::CalcAngleAndCost(m_temp_path);
				PlannerHNS::PlanningHelpers::GenerateRecommendedSpeed(m_GlobalPaths.at(i), m_CarInfo.max_speed_forward, m_PlanningParams.speedProfileFactor, m_PlanningParams.enableCost);

#ifdef LOG_LOCAL_PLANNING_DATA
				std::ostringstream str_out;
				str_out << UtilityHNS::UtilityH::GetHomeDirectory();
				if(m_ExperimentFolderName.size() == 0)
					str_out << UtilityHNS::DataRW::LoggingMainfolderName;
				else
					str_out << UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::ExperimentsFolderName + m_ExperimentFolderName;

				str_out << UtilityHNS::DataRW::GlobalPathLogFolderName;
				str_out << "GlobalPath_";
				str_out << i;
				str_out << "_";
				PlannerHNS::PlanningHelpers::WritePathToFile(str_out.str(), m_GlobalPaths.at(i));
#endif
			}

			// std::cout << "Received New Global Path Selector! " << std::endl;
		}
	}

}

void TrajectoryEvalCore::callbackGetLocalPlannerPath(const autoware_msgs::LaneArrayStampedConstPtr& msg)
{

	std::vector<PlannerHNS::WayPoint> path;

	if(!msg->lanearray.lanes.empty())
	{
		//m_RollOuts.clear();
		std::vector< std::vector<PlannerHNS::WayPoint> > received_local_rollouts;
		std::vector<int> globalPathsId_roll_outs;

		for(const auto & lane : msg->lanearray.lanes)
		{
			path.clear();
			PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(lane, path);
			received_local_rollouts.push_back(path);
			//m_RollOuts.push_back(path);

			int roll_out_gid = -1;
			if(!path.empty())
			{
				roll_out_gid = path.at(0).gid;
			}

			if(std::find(globalPathsId_roll_outs.begin(), globalPathsId_roll_outs.end(), roll_out_gid) == globalPathsId_roll_outs.end())
			{
				globalPathsId_roll_outs.push_back(roll_out_gid);
			}
		}

		if(CompareTrajectoriesWithIds(m_GlobalPathsToUse, globalPathsId_roll_outs))
		{
			CollectRollOutsByGlobalPath(received_local_rollouts);
		}
		else if(CompareTrajectoriesWithIds(m_GlobalPaths, globalPathsId_roll_outs))
		{
			m_GlobalPathsToUse.clear();
			m_prev_index.clear();
			for(auto& global_path: m_GlobalPaths)
			{
				m_GlobalPathsToUse.push_back(global_path);
				m_prev_index.push_back(0);
			}
			CollectRollOutsByGlobalPath(received_local_rollouts);
		}
		else
		{
			m_LanesRollOutsToUse.clear();
			m_GlobalPathsToUse.clear();
		}
	}



}

void TrajectoryEvalCore::callbackGetPredictedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg)
{
	m_PredictedObjects.clear();
	bPredictedObjects = true;

	latest_header = msg->header;

	PlannerHNS::DetectedObject obj;
	for(unsigned int i = 0 ; i <msg->objects.size(); i++)
	{
		PlannerHNS::ROSHelpers::ConvertFromAutowareDetectedObjectToOpenPlannerDetectedObject(msg->objects.at(i), obj);
		m_PredictedObjects.push_back(obj);
//		else
//		{
//			std::cout << " Ego Car avoid detecting itself in trajectory evaluator node! ID: " << msg->objects.at(i).id << std::endl;
//		}
	}
}

void TrajectoryEvalCore::callbackGetTrajectoryInforFromBehaviorSelector(const std_msgs::Int32MultiArrayConstPtr& msg)
{
	//Compare between m_CurrGlobalPathsIds the received global ids,
	//msg data -> first item is the lane_index , second item trajectory index, the rest are the current used global paths ids in the behavior selector
	std::vector<int> beh_selector_global_ids;
	int iLaneIndex = -1;
	int iTrajIndex = -1;
	for(unsigned int i=0; i < msg->data.size(); i++)
	{
		if(i == 0)
		{
			iLaneIndex = msg->data.at(0);
		}
		else if(i == 1)
		{
			iTrajIndex = msg->data.at(1);
		}
		else
		{
			beh_selector_global_ids.push_back(msg->data.at(i));
		}
	}

	if(CompareTrajectoriesWithIds(m_GlobalPathsToUse, beh_selector_global_ids) == true)
	{
		m_CurrentBehavior.iLane = iLaneIndex;
		m_CurrentBehavior.iTrajectory = iTrajIndex;
	}
}

bool TrajectoryEvalCore::CompareTrajectoriesWithIds(std::vector<std::vector<PlannerHNS::WayPoint> >& paths, std::vector<int>& local_ids)
{
	if(local_ids.size() != paths.size())
	{
		std::cout << "Warning From Trajectory Evaluator, paths size mismatch, GlobalPaths: " << paths.size() << ", LocalPaths: " << local_ids.size() << std::endl;
		return false;
	}

	for(auto& id : local_ids)
	{
		bool bFound = false;
		for(auto& path: paths)
		{
			if(path.size() > 0 && path.at(0).gid == id)
			{
				bFound = true;
			}
		}

		if(bFound == false)
		{
			std::cout << "Synchronization At Trajectory Evaluator: " << std::endl;
			std::cout << "## Local IDs: ";
			for(auto& id : local_ids)
			{
				std::cout << id << ",";
			}
			std::cout << std::endl << "## Global IDs: ";
			for(auto& path: paths)
			{
				if(path.size() > 0)
				{
					std::cout << path.at(0).gid << ",";
				}
			}
			std::cout << std::endl;

			return false;
		}
	}

	return true;
}

void TrajectoryEvalCore::CollectRollOutsByGlobalPath(std::vector< std::vector<PlannerHNS::WayPoint> >& local_rollouts)
{
	m_LanesRollOutsToUse.clear();
	std::vector< std::vector<PlannerHNS::WayPoint> > local_category;
	for(auto& g_path: m_GlobalPathsToUse)
	{
		if(g_path.size() > 0)
		{
			local_category.clear();
			for(auto& l_traj: local_rollouts)
			{
				if(l_traj.size() > 0 && l_traj.at(0).gid == g_path.at(0).gid)
				{
					local_category.push_back(l_traj);
	//				std::cout << "Global Lane ID: " << g_path.at(0).gid << ", Cost Global: " << g_path.at(0).laneChangeCost << ", Cost Local: " << l_traj.at(0).laneChangeCost << std::endl;
				}
			}
			m_LanesRollOutsToUse.push_back(local_category);
		}
	}
}

bool TrajectoryEvalCore::FindBestLane(std::vector<PlannerHNS::TrajectoryCost> tcs, PlannerHNS::TrajectoryCost& best_l)
{
	if(tcs.size() == 0) return false;

	if(tcs.size() == 1)
	{
		best_l = tcs.at(0);
		return true;
	}

	std::sort(tcs.begin(), tcs.end(), PlannerHNS::TrajectoryEvaluator::sortCosts);

	for(unsigned int i=0 ; i < tcs.size(); i++)
	{
		if(!tcs.at(i).bBlocked)
		{
			best_l = tcs.at(i);
			return true;
		}
	}

	best_l = tcs.at(0);

	return true;
}

void TrajectoryEvalCore::MainLoop()
{
	ros::Rate loop_rate(50);

	PlannerHNS::WayPoint prevState, state_change;
    autoware_msgs::LaneArrayStamped local_lanes;
    std::vector<PlannerHNS::TrajectoryCost> tcs;
    std::vector<PlannerHNS::WayPoint> collision_points;
    visualization_msgs::MarkerArray all_rollOuts;
    visualization_msgs::Marker safety_box;

	while (ros::ok())
	{
		ros::spinOnce();

		if(m_MapType == PlannerHNS::MAP_KML_FILE && !bMap)
		{
			bMap = true;
			PlannerHNS::KmlMapLoader kml_loader;
			kml_loader.LoadKML(m_MapPath, m_Map);
			PlannerHNS::MappingHelpers::ConvertVelocityToMeterPerSecond(m_Map);
		}
		else if (m_MapType == PlannerHNS::MAP_FOLDER && !bMap)
		{
			bMap = true;
			PlannerHNS::VectorMapLoader vec_loader(1, m_PlanningParams.enableLaneChange);
			vec_loader.LoadFromFile(m_MapPath, m_Map);
			PlannerHNS::MappingHelpers::ConvertVelocityToMeterPerSecond(m_Map);	
		}
		else if (m_MapType == PlannerHNS::MAP_LANELET_2 && !bMap)
		{
			bMap = true;
			PlannerHNS::Lanelet2MapLoader map_loader;
			map_loader.LoadMap(m_MapPath, m_Map);
			PlannerHNS::MappingHelpers::ConvertVelocityToMeterPerSecond(m_Map);
		}
		else if (m_MapType == PlannerHNS::MAP_AUTOWARE && !bMap)
		{
			if(m_MapRaw.AreMessagesReceived())
			{
				bMap = true;
				PlannerHNS::VectorMapLoader vec_loader(1, m_PlanningParams.enableLaneChange);
				vec_loader.LoadFromData(m_MapRaw, m_Map);
				PlannerHNS::MappingHelpers::ConvertVelocityToMeterPerSecond(m_Map);
			}
		}

		if(bNewCurrentPos && m_GlobalPathsToUse.size() > 0)
		{
			m_GlobalPathSections.clear();

			if(m_prev_index.size() != m_GlobalPathsToUse.size())
			{
				m_prev_index.clear();
				for(unsigned int i=0; i < m_GlobalPathsToUse.size(); i++)
				{
					m_prev_index.push_back(0);
				}
			}

			for(unsigned int i = 0; i < m_GlobalPathsToUse.size(); i++)
			{
				t_centerTrajectorySmoothed.clear();
				m_prev_index.at(i) = PlannerHNS::PlanningHelpers::ExtractPartFromPointToDistanceDirectionFast(m_GlobalPathsToUse.at(i), m_CurrentPos, m_PlanningParams.horizonDistance ,
						m_PlanningParams.pathDensity ,t_centerTrajectorySmoothed, m_prev_index.at(i));

				if(m_prev_index.at(i) > 0 ) m_prev_index.at(i) = m_prev_index.at(i) -1;

				m_GlobalPathSections.push_back(t_centerTrajectorySmoothed);
			}

			if(!m_GlobalPathSections.empty() && !m_LanesRollOutsToUse.empty())
			{
			    local_lanes.lanearray.lanes.clear();
			    local_lanes.header.stamp = path_stamp;
			    tcs.clear();
                collision_points.clear();
                all_rollOuts.markers.clear();

//				std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > > collected_local_roll_outs;
//				std::vector<std::vector<PlannerHNS::TrajectoryCost> > collected_trajectory_costs;

				if(!m_PlanningParams.enableLaneChange)
				{
					PlannerHNS::PlanningParams planningParams = m_PlanningParams;
					if(m_CurrentBehavior.state == PlannerHNS::FOLLOW_STATE)
					{
						planningParams.minFollowingDistance += m_AdditionalFollowDistance;
					}

					PlannerHNS::TrajectoryCost tc = m_TrajectoryCostsCalculator.doOneStep(0, m_GlobalPaths, m_LanesRollOutsToUse.at(0), m_GlobalPathSections, m_CurrentPos,
							planningParams, m_CarInfo,m_VehicleStatus, m_PredictedObjects, m_Map, !m_bUseMoveingObjectsPrediction, m_CurrentBehavior.iTrajectory, m_bKeepCurrentIfPossible);
					tcs.push_back(tc);

					for(unsigned int i=0; i < m_TrajectoryCostsCalculator.local_roll_outs_.size(); i++)
					{

							autoware_msgs::Lane lane;
							PlannerHNS::ROSHelpers::ConvertFromLocalLaneToAutowareLane(m_TrajectoryCostsCalculator.local_roll_outs_.at(i), lane);
							lane.closest_object_distance = m_TrajectoryCostsCalculator.trajectory_costs_.at(i).closest_obj_distance;
							lane.closest_object_velocity = m_TrajectoryCostsCalculator.trajectory_costs_.at(i).closest_obj_velocity;
							lane.cost = m_TrajectoryCostsCalculator.trajectory_costs_.at(i).cost;
							lane.is_blocked = m_TrajectoryCostsCalculator.trajectory_costs_.at(i).bBlocked;
							lane.lane_index = local_lanes.lanearray.lanes.size();
							lane.lane_id = 0;
                            local_lanes.lanearray.lanes.push_back(lane);
					}

//					collected_local_roll_outs.push_back(m_TrajectoryCostsCalculator.local_roll_outs_);
//					collected_trajectory_costs.push_back(m_TrajectoryCostsCalculator.trajectory_costs_);
					PlannerHNS::ROSHelpers::TrajectoriesToColoredMarkers(m_TrajectoryCostsCalculator.local_roll_outs_, m_TrajectoryCostsCalculator.trajectory_costs_, tc.index, all_rollOuts);
					collision_points.insert(collision_points.end(), m_TrajectoryCostsCalculator.collision_points_.begin(), m_TrajectoryCostsCalculator.collision_points_.end());
					PlannerHNS::ROSHelpers::ConvertFromPlannerHRectangleToAutowareRviz(m_TrajectoryCostsCalculator.safety_border_.points, safety_box);

				}
				else if(m_GlobalPathSections.size() == m_LanesRollOutsToUse.size())
				{
					PlannerHNS::PlanningParams planningParams = m_PlanningParams;
					if(m_CurrentBehavior.state == PlannerHNS::FOLLOW_STATE)
					{
						planningParams.minFollowingDistance += m_AdditionalFollowDistance;
					}

					//std::cout << "Start New Evaluations --------------------------- vvvvvvvvvvvvvvvv " <<  std::endl;
					for(unsigned int ig = 0; ig < m_GlobalPathSections.size(); ig++)
					{
//						std::cout << "Best Lane From Behavior Selector: " << m_CurrentBehavior.iLane << ", Trajectory: " << m_CurrentBehavior.iTrajectory << ", Curr Lane: " << ig << std::endl;

						PlannerHNS::TrajectoryCost temp_tc = m_TrajectoryCostsCalculator.doOneStep(ig, m_GlobalPaths, m_LanesRollOutsToUse.at(ig), m_GlobalPathSections, m_CurrentPos,
								planningParams, m_CarInfo, m_VehicleStatus, m_PredictedObjects, m_Map, !m_bUseMoveingObjectsPrediction, m_CurrentBehavior.iTrajectory, m_bKeepCurrentIfPossible);


//						if((m_GlobalPathSections.size() == 3 && ig == 2) || (m_GlobalPathSections.size() == 2 && ig == 1))
//						{
//							temp_tc.bBlocked = true;
//						}

						if(m_GlobalPathSections.at(ig).size() > 0)
						{
							temp_tc.lane_change_cost = m_GlobalPathSections.at(ig).at(0).laneChangeCost;
						}

						temp_tc.lane_index = ig;
						tcs.push_back(temp_tc);

						for(unsigned int i=0; i < m_TrajectoryCostsCalculator.local_roll_outs_.size(); i++)
						{
								autoware_msgs::Lane lane;
								PlannerHNS::ROSHelpers::ConvertFromLocalLaneToAutowareLane(m_TrajectoryCostsCalculator.local_roll_outs_.at(i), lane);
								lane.closest_object_distance = m_TrajectoryCostsCalculator.trajectory_costs_.at(i).closest_obj_distance;
								lane.closest_object_velocity = m_TrajectoryCostsCalculator.trajectory_costs_.at(i).closest_obj_velocity;
								lane.cost = m_TrajectoryCostsCalculator.trajectory_costs_.at(i).cost;
								lane.is_blocked = m_TrajectoryCostsCalculator.trajectory_costs_.at(i).bBlocked;
								lane.lane_index = local_lanes.lanearray.lanes.size();
								lane.lane_id = ig;
                                local_lanes.lanearray.lanes.push_back(lane);
						}

//						collected_local_roll_outs.push_back(m_TrajectoryCostsCalculator.local_roll_outs_);
//						collected_trajectory_costs.push_back(m_TrajectoryCostsCalculator.trajectory_costs_);

						PlannerHNS::ROSHelpers::TrajectoriesToColoredMarkers(m_TrajectoryCostsCalculator.local_roll_outs_, m_TrajectoryCostsCalculator.trajectory_costs_, temp_tc.index, all_rollOuts);
						collision_points.insert(collision_points.end(), m_TrajectoryCostsCalculator.collision_points_.begin(), m_TrajectoryCostsCalculator.collision_points_.end());
						PlannerHNS::ROSHelpers::ConvertFromPlannerHRectangleToAutowareRviz(m_TrajectoryCostsCalculator.safety_border_.points, safety_box);
					}

					PlannerHNS::EvaluationParams eval_params_for_lane_change = m_EvaluationParams;
					eval_params_for_lane_change.lane_change_weight_ = 0.6;
					BalanceFactorsToOne(eval_params_for_lane_change.priority_weight_, eval_params_for_lane_change.transition_weight_,
								eval_params_for_lane_change.longitudinal_weight_, eval_params_for_lane_change.lateral_weight_, eval_params_for_lane_change.lane_change_weight_);
					m_TrajectoryCostsCalculator.normalizeCosts(eval_params_for_lane_change, tcs);
				}
				else
				{
					std::cout << "# Error from trajectory generator, Lane change is enabled but Global paths doesn't match Local trajectories! Global Sections:  " << m_GlobalPathSections.size() << ", Local Rollouts: " <<  m_LanesRollOutsToUse.size() << std::endl;
				}

				m_EvaluatedObjects.objects.clear();
				autoware_msgs::DetectedObject evaluated_obj;
				for(unsigned int i = 0 ; i <m_TrajectoryCostsCalculator.objects_attention.size(); i++)
				{
					PlannerHNS::ROSHelpers::ConvertFromOpenPlannerDetectedObjectToAutowareDetectedObject(m_TrajectoryCostsCalculator.objects_attention.at(i), false, evaluated_obj);
					m_EvaluatedObjects.objects.push_back(evaluated_obj);
				}
				
				//Visualize yield ROIs
				visualization_msgs::MarkerArray roi_points;
				if(m_TrajectoryCostsCalculator.attention_rois.size() > 0)
				{
					PlannerHNS::ROSHelpers::GetROIPointsForVisualization(m_TrajectoryCostsCalculator.attention_rois, roi_points);
					pub_roiMarkers.publish(roi_points);
				}

				m_EvaluatedObjects.header.stamp = ros::Time().now();
				pub_yieldEvalAttention.publish(m_EvaluatedObjects);


				if(tcs.size() > 0)
				{
					PlannerHNS::TrajectoryCost best_lane_costs;
					if(FindBestLane(tcs, best_lane_costs))
					{
						autoware_msgs::Lane l;
						l.closest_object_distance = best_lane_costs.closest_obj_distance;
						l.closest_object_velocity = best_lane_costs.closest_obj_velocity;
						l.cost = best_lane_costs.cost;
						l.is_blocked = best_lane_costs.bBlocked;
						l.is_predictive_blocked = best_lane_costs.bPredictiveBlocked;
						l.lane_index = best_lane_costs.index;
						l.lane_id = best_lane_costs.lane_index;

						pub_TrajectoryCost.publish(l);
						pub_LocalWeightedTrajectories.publish(local_lanes);

						//Visualize results
						PlannerHNS::ROSHelpers::ConvertCollisionPointsMarkers(collision_points, m_CollisionsActual, m_CollisionsDummy);
						pub_CollisionPointsRviz.publish(m_CollisionsActual);
						pub_SafetyBorderRviz.publish(safety_box);
						pub_LocalWeightedTrajectoriesRviz.publish(all_rollOuts);
					}
					else
					{
						std::cout << "Warning from Trajectory Evaluator, Can't find suitable lane for driving !! " << std::endl;
					}

//					std::cout << "Costs For Lanes:  ---------------------- " << std::endl;
//					std::cout << "Best Lane From Behavior Selector: " << m_CurrentBehavior.iLane << ", Trajectory: " << m_CurrentBehavior.iTrajectory << ", Globals: " << m_GlobalPathSections.size() << ", Locals: " << m_LanesRollOuts.size() <<   std::endl;
//					for(unsigned int i=0; i < tcs.size(); i++)
//					{
//						std::cout << "i: " << i << ", Index: " << tcs.at(i).index << ", GlobalIndex: " << tcs.at(i).lane_index << ", Cost: " << tcs.at(i).cost << ", Blocked: " << tcs.at(i).bBlocked << std::endl;
//					}
//					std::cout << "Best Lane From Cost Calculator : " <<std::endl;
//					std::cout << "i: " << -1 << ", Index: " << best_lane_costs.index << ", GlobalIndex: " << best_lane_costs.lane_index << ", Cost: " << best_lane_costs.cost << ", Blocked: " << best_lane_costs.bBlocked << std::endl;
//					std::cout << "---------------------- " << std::endl;
				}
			}
			else
			{
				// std::cout << "From Trajectory Evaluator ! Local or Global Paths are not published ! " << std::endl;
 			}
		}
		else
		{
			sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array", 	1,		&TrajectoryEvalCore::callbackGetGlobalPlannerPath, 	this);
		}

		loop_rate.sleep();
	}
}



//----------------------------

//Mapping Section
//----------------------------
void TrajectoryEvalCore::callbackGetLanelet2(const autoware_lanelet2_msgs::MapBin& msg)
{
	PlannerHNS::Lanelet2MapLoader map_loader;
	map_loader.LoadMap(msg, m_Map);
	PlannerHNS::MappingHelpers::ConvertVelocityToMeterPerSecond(m_Map);
	bMap = true;
}

void TrajectoryEvalCore::callbackGetVMLanes(const vector_map_msgs::LaneArray& msg)
{
	std::cout << "Received Lanes" << endl;
	if(m_MapRaw.pLanes == nullptr)
		m_MapRaw.pLanes = new UtilityHNS::AisanLanesFileReader(msg);
}

void TrajectoryEvalCore::callbackGetVMPoints(const vector_map_msgs::PointArray& msg)
{
	std::cout << "Received Points" << endl;
	if(m_MapRaw.pPoints  == nullptr)
		m_MapRaw.pPoints = new UtilityHNS::AisanPointsFileReader(msg);
}

void TrajectoryEvalCore::callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg)
{
	std::cout << "Received dtLanes" << endl;
	if(m_MapRaw.pCenterLines == nullptr)
		m_MapRaw.pCenterLines = new UtilityHNS::AisanCenterLinesFileReader(msg);
}

void TrajectoryEvalCore::callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg)
{
	std::cout << "Received CrossRoads" << endl;
	if(m_MapRaw.pIntersections == nullptr)
		m_MapRaw.pIntersections = new UtilityHNS::AisanIntersectionFileReader(msg);
}

void TrajectoryEvalCore::callbackGetVMAreas(const vector_map_msgs::AreaArray& msg)
{
	std::cout << "Received Areas" << endl;
	if(m_MapRaw.pAreas == nullptr)
		m_MapRaw.pAreas = new UtilityHNS::AisanAreasFileReader(msg);
}

void TrajectoryEvalCore::callbackGetVMLines(const vector_map_msgs::LineArray& msg)
{
	std::cout << "Received Lines" << endl;
	if(m_MapRaw.pLines == nullptr)
		m_MapRaw.pLines = new UtilityHNS::AisanLinesFileReader(msg);
}

void TrajectoryEvalCore::callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg)
{
	std::cout << "Received StopLines" << endl;
	if(m_MapRaw.pStopLines == nullptr)
		m_MapRaw.pStopLines = new UtilityHNS::AisanStopLineFileReader(msg);
}

void TrajectoryEvalCore::callbackGetVMSignal(const vector_map_msgs::SignalArray& msg)
{
	std::cout << "Received Signals" << endl;
	if(m_MapRaw.pSignals  == nullptr)
		m_MapRaw.pSignals = new UtilityHNS::AisanSignalFileReader(msg);
}

void TrajectoryEvalCore::callbackGetVMSign(const vector_map_msgs::RoadSignArray& msg)
{
	std::cout << "Received Road Signs" << endl;
	if(m_MapRaw.pSigns  == nullptr)
		m_MapRaw.pSigns = new UtilityHNS::AisanRoadSignFileReader(msg);
}

void TrajectoryEvalCore::callbackGetVMVectors(const vector_map_msgs::VectorArray& msg)
{
	std::cout << "Received Vectors" << endl;
	if(m_MapRaw.pVectors  == nullptr)
		m_MapRaw.pVectors = new UtilityHNS::AisanVectorFileReader(msg);
}

void TrajectoryEvalCore::callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg)
{
	std::cout << "Received Curbs" << endl;
	if(m_MapRaw.pCurbs == nullptr)
		m_MapRaw.pCurbs = new UtilityHNS::AisanCurbFileReader(msg);
}

void TrajectoryEvalCore::callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg)
{
	std::cout << "Received Edges" << endl;
	if(m_MapRaw.pRoadedges  == nullptr)
		m_MapRaw.pRoadedges = new UtilityHNS::AisanRoadEdgeFileReader(msg);
}

void TrajectoryEvalCore::callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg)
{
	std::cout << "Received Wayareas" << endl;
	if(m_MapRaw.pWayAreas  == nullptr)
		m_MapRaw.pWayAreas = new UtilityHNS::AisanWayareaFileReader(msg);
}

void TrajectoryEvalCore::callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg)
{
	std::cout << "Received CrossWalks" << endl;
	if(m_MapRaw.pCrossWalks == nullptr)
		m_MapRaw.pCrossWalks = new UtilityHNS::AisanCrossWalkFileReader(msg);
}

void TrajectoryEvalCore::callbackGetVMNodes(const vector_map_msgs::NodeArray& msg)
{
	std::cout << "Received Nodes" << endl;
	if(m_MapRaw.pNodes == nullptr)
		m_MapRaw.pNodes = new UtilityHNS::AisanNodesFileReader(msg);
}
//----------------------------

}
