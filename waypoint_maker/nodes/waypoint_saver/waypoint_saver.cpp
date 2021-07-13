/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/NavSatFix.h>

#include <fstream>

#include "libwaypoint_follower/libwaypoint_follower.h"

static const int SYNC_FRAMES = 50;

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped, geometry_msgs::PoseStamped>
    TwistPoseSync;
typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped, geometry_msgs::PoseStamped, sensor_msgs::NavSatFix>
    TwistPoseGpsSync;

enum class Reference_location
{
  REAR_AXLE_CENTER = 0,
  CENTER_OF_GRAVITY = 1,
  FRONT_AXLE_CENTER = 2,
};

class WaypointSaver
{
public:
  WaypointSaver();
  ~WaypointSaver();

private:
  // functions

  void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg) const;
  void TwistPoseCallback(const geometry_msgs::TwistStampedConstPtr &twist_msg,
                         const geometry_msgs::PoseStampedConstPtr &pose_msg) const;
  void TwistPoseGpsCallback(const geometry_msgs::TwistStampedConstPtr &twist_msg,
                            const geometry_msgs::PoseStampedConstPtr &pose_msg,
                            const sensor_msgs::NavSatFixConstPtr &gps_msg) const;

  void displayMarker(const geometry_msgs::Pose& pose, double velocity) const;
  void outputProcessing(const geometry_msgs::Pose& updated_pose, double velocity, double lat, double lon) const;

  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher waypoint_saver_pub_;

  // subscriber
  message_filters::Subscriber<geometry_msgs::TwistStamped> *twist_sub_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> *pose_sub_;
  message_filters::Subscriber<sensor_msgs::NavSatFix> *gps_sub_;
  message_filters::Synchronizer<TwistPoseSync> *sync_tp_;
  message_filters::Synchronizer<TwistPoseGpsSync> *sync_tpg_;

  // variables
  bool save_velocity_;
  bool save_gps_;
  double interval_;
  std::string filename_, pose_topic_, velocity_topic_, gps_topic_;
  Reference_location reference_location_;
  // distance from the center of rear axle to the center of front axle.
  double wheelbase_;
  // distance from the center of rear axle to the center of gravity
  double rear_to_cog_;
};

WaypointSaver::WaypointSaver() : nh_(), private_nh_("~")
{
  // parameter settings
  private_nh_.param<std::string>("save_filename", filename_, std::string("data.txt"));
  private_nh_.param<std::string>("pose_topic", pose_topic_, std::string("current_pose"));
  private_nh_.param<std::string>("velocity_topic", velocity_topic_, std::string("current_velocity"));
  private_nh_.param<std::string>("gps_topic", gps_topic_, std::string("gps/fix"));
  private_nh_.param<double>("interval", interval_, 1.0);
  private_nh_.param<bool>("save_velocity", save_velocity_, false);
  private_nh_.param<bool>("save_gps", save_gps_, false);
  int32_t temp_location;
  private_nh_.param<int>("reference_location", temp_location, 0);
  reference_location_ = static_cast<Reference_location>(temp_location);
  nh_.param("vehicle_info/wheel_base", wheelbase_, 2.789);
  rear_to_cog_ = wheelbase_ / 2.0;

  // subscriber
  pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, pose_topic_, 50);


  if (save_gps_)
  {
    twist_sub_ = new message_filters::Subscriber<geometry_msgs::TwistStamped>(nh_, velocity_topic_, 50);
    gps_sub_ = new message_filters::Subscriber<sensor_msgs::NavSatFix>(nh_, gps_topic_, 50);
    sync_tpg_ = new message_filters::Synchronizer<TwistPoseGpsSync>(TwistPoseGpsSync(SYNC_FRAMES), *twist_sub_, *pose_sub_, *gps_sub_);
    sync_tpg_->registerCallback(boost::bind(&WaypointSaver::TwistPoseGpsCallback, this, _1, _2, _3));
  }
  else if (save_velocity_)
  {
    twist_sub_ = new message_filters::Subscriber<geometry_msgs::TwistStamped>(nh_, velocity_topic_, 50);
    sync_tp_ = new message_filters::Synchronizer<TwistPoseSync>(TwistPoseSync(SYNC_FRAMES), *twist_sub_, *pose_sub_);
    sync_tp_->registerCallback(boost::bind(&WaypointSaver::TwistPoseCallback, this, _1, _2));
  }
  else
  {
    pose_sub_->registerCallback(boost::bind(&WaypointSaver::poseCallback, this, _1));
  }

  // publisher
  waypoint_saver_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("waypoint_saver_marker", 10, true);
}

WaypointSaver::~WaypointSaver()
{
  delete twist_sub_;
  delete pose_sub_;
  delete sync_tp_;
}

void WaypointSaver::poseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg) const
{
  outputProcessing(pose_msg->pose, 0.0, 0.0, 0.0);
}

void WaypointSaver::TwistPoseCallback(const geometry_msgs::TwistStampedConstPtr &twist_msg,
                                      const geometry_msgs::PoseStampedConstPtr &pose_msg) const
{
  outputProcessing(pose_msg->pose, mps2kmph(twist_msg->twist.linear.x), 0.0, 0.0);
}

void WaypointSaver::TwistPoseGpsCallback(const geometry_msgs::TwistStampedConstPtr &twist_msg,
                                         const geometry_msgs::PoseStampedConstPtr &pose_msg,
                                         const sensor_msgs::NavSatFixConstPtr &gps_msg) const
{
  outputProcessing(pose_msg->pose, mps2kmph(twist_msg->twist.linear.x), gps_msg->latitude, gps_msg->longitude);
}

void WaypointSaver::outputProcessing(const geometry_msgs::Pose &updated_pose, double velocity, double lat, double lon) const
{
  std::ofstream ofs(filename_.c_str(), std::ios::app);
  static geometry_msgs::Pose previous_pose;
  static bool receive_once = false;
  static uint32_t waypoint_id_ = 0;
  geometry_msgs::Pose current_pose = updated_pose;
  double current_heading = tf::getYaw(updated_pose.orientation);

  if (reference_location_ == Reference_location::CENTER_OF_GRAVITY)
  {
    current_pose.position.x = updated_pose.position.x + rear_to_cog_ * std::cos(current_heading);
    current_pose.position.y = updated_pose.position.y + rear_to_cog_ * std::sin(current_heading);
  }
  else if (reference_location_ == Reference_location::FRONT_AXLE_CENTER)
  {
    current_pose.position.x = updated_pose.position.x + wheelbase_ * std::cos(current_heading);
    current_pose.position.y = updated_pose.position.y + wheelbase_ * std::sin(current_heading);
  }

  // first subscribe
  if (!receive_once)
  {
    if (save_gps_)
    {
      ofs << "wp_id,x,y,z,lat,lon,yaw,velocity,change_flag" << std::endl;
    }
    else
    {
      ofs << "wp_id,x,y,z,yaw,velocity,change_flag" << std::endl;
    }
    ofs << std::fixed << std::setprecision(4)
        << waypoint_id_++ << ","
        << current_pose.position.x << ","
        << current_pose.position.y << ","
        << current_pose.position.z << ",";

    if (save_gps_)
    {
      ofs << std::fixed << std::setprecision(8)
          << lat << ","
          << lon << ",";
    }

    ofs << std::fixed << std::setprecision(4)
        << current_heading << ",0,0"
        << std::endl;
    receive_once = true;
    displayMarker(current_pose, 0);
    previous_pose = current_pose;
  }
  else
  {
    double distance = sqrt(pow((current_pose.position.x - previous_pose.position.x), 2) +
                           pow((current_pose.position.y - previous_pose.position.y), 2));

    // if car moves [interval] meter
    if (distance > interval_)
    {
      ofs << std::fixed << std::setprecision(4)
          << waypoint_id_++ << ","
          << current_pose.position.x << ","
          << current_pose.position.y << ","
          << current_pose.position.z << ",";

      if (save_gps_)
      {
        ofs << std::fixed << std::setprecision(8)
            << lat << ","
            << lon << ",";
      }

      ofs << std::fixed << std::setprecision(4)
          << current_heading << ","
          << velocity << ",0"
          << std::endl;

      displayMarker(current_pose, velocity);
      previous_pose = current_pose;
    }
  }
}

void WaypointSaver::displayMarker(const geometry_msgs::Pose &pose, double velocity) const
{
  static visualization_msgs::MarkerArray marray;
  static int id = 0;

  // initialize marker
  visualization_msgs::Marker marker;
  marker.id = id;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.frame_locked = true;

  // create saved waypoint marker
  marker.scale.x = 0.5;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.ns = "saved_waypoint_arrow";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose;
  marray.markers.push_back(marker);

  // create saved waypoint velocity text
  marker.scale.z = 0.4;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.ns = "saved_waypoint_velocity";
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << velocity << " km/h";
  marker.text = oss.str();
  marray.markers.push_back(marker);

  waypoint_saver_pub_.publish(marray);
  id++;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_saver");
  WaypointSaver ws;
  ros::spin();
  return 0;
}
