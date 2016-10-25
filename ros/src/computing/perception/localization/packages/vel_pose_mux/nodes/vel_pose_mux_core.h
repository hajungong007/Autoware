/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _VEL_POSE_MUX_CORE_H_
#define _VEL_POSE_MUX_CORE_H_

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>

#include "vehicle_socket/CanInfo.h"

namespace vel_pose_mux
{

struct VehicleInfo {
  double wheel_base;
  double minimum_turning_radius;
  double maximum_steering_angle;
};

struct Odometry {
  double x;
  double y;
  double th;
  ros::Time stamp;
};

enum class VelMux : int32_t
{
  ndt_matching,
  can_info,

  Unknown = -1,
};

enum class PoseMux : int32_t
{
  ndt_matching,
  GNSS,

  Unknown = -1,
};

typedef std::underlying_type<VelMux>::type VelMuxInteger;
typedef std::underlying_type<PoseMux>::type PoseMuxInteger;

inline double kmph2mps(double velocity_kmph)
{
  return (velocity_kmph * 1000) / (60 * 60);
}

inline double mps2kmph(double velocity_mps)
{
  return (velocity_mps * 60 * 60) / 1000;
}

// convert degree to radian
inline double deg2rad(double deg)
{
  return deg * M_PI / 180;
}

// convert degree to radian
inline double rad2deg(double rad)
{
  return rad * 180 / M_PI;
}

class VelPoseMuxNode
{
public:
  VelPoseMuxNode();
  ~VelPoseMuxNode();

private:

  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher current_pose_pub_, current_vel_pub_, odom_pub_, linear_vel_viz_pub_;

  // subscriber
  ros::Subscriber pose_sub_, twist_sub_, can_info_sub_;

  // variables
  int32_t pose_mux_select_, vel_mux_select_;
  bool sim_mode_, publish_odom_, vehicle_info_flag_;
  VehicleInfo vehicle_info_;
  Odometry odom_;

  // callbacks
  void callbackFromPoseStamped(const geometry_msgs::PoseStampedConstPtr &msg);
  void callbackFromTwistStamped(const geometry_msgs::TwistStampedConstPtr &msg);
  void callbackFromCanInfo(const vehicle_socket::CanInfoConstPtr &msg);

  //initializer
  void initSubscriber();
  void initPublisher();
  void initParameter();

  // functions
  void publishOdometry(const vehicle_socket::CanInfoConstPtr& msg);
  void publishCurrentPose(const geometry_msgs::PoseStampedConstPtr & msg);
  void publishCurrentVelocity(const geometry_msgs::TwistStampedConstPtr & msg);
  void publishLinearVelViz(const geometry_msgs::TwistStampedConstPtr & msg);
  void publishCurrentVelocity(const vehicle_socket::CanInfoConstPtr& msg);
  void publishLinearVelViz(const vehicle_socket::CanInfoConstPtr& msg);
  double convertSteerAngle2AngularVel(double steer_angle_deg, double linear_vel_mps);

};
}
#endif  // vel_pose_mux
