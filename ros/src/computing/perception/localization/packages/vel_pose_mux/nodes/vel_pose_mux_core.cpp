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

#include "vel_pose_mux_core.h"

namespace vel_pose_mux
{
// Constructor
VelPoseMuxNode::VelPoseMuxNode()
  : private_nh_("~")
{
  initParameter();
  initSubscriber();
  initPublisher();
}

// Destructor
VelPoseMuxNode::~VelPoseMuxNode()
{
}


void VelPoseMuxNode::initParameter()
{
  private_nh_.param<int32_t>("pose_mux_select", pose_mux_select_, int32_t(-1));
  private_nh_.param<int32_t>("vel_mux_select", vel_mux_select_, int32_t(-1));
  private_nh_.param<bool>("sim_mode", sim_mode_, false);
  private_nh_.param<bool>("publish_odom", publish_odom_, false);
  private_nh_.param<double>("/vehicle_info/wheel_base", vehicle_info_.wheel_base, double(2.7));
  private_nh_.param<double>("/vehicle_info/minimum_turning_radius", vehicle_info_.minimum_turning_radius, double(5.1));
  private_nh_.param<double>("/vehicle_info/maximum_steering_angle", vehicle_info_.maximum_steering_angle, double(666.0));

  if(publish_odom_)
  {
    odom_.stamp = ros::Time::now();
  }
}

void VelPoseMuxNode::initSubscriber()
{
  //pose
  std::string pose_topic = sim_mode_ ? "sim_pose"
                          : pose_mux_select_ == static_cast<PoseMuxInteger>(PoseMux::ndt_matching) ? "ndt_pose"
                          : pose_mux_select_ == static_cast<PoseMuxInteger>(PoseMux::GNSS) ? "gnss_pose"
                          : "unknown_pose";

  pose_sub_ = nh_.subscribe(pose_topic, 100, &VelPoseMuxNode::callbackFromPoseStamped, this);

  //velocity
  if (vel_mux_select_ == static_cast<VelMuxInteger>(VelMux::can_info))
  {
    ROS_INFO("subscribe can_info");
    can_info_sub_ = nh_.subscribe("can_info", 100, &VelPoseMuxNode::callbackFromCanInfo, this);
  }
  else
  {
    std::string vel_topic = sim_mode_ ? "sim_velocity"
                           : vel_mux_select_ == static_cast<VelMuxInteger>(VelMux::ndt_matching) ? "estimate_twist"
                           : "unknown_velocity";

    twist_sub_ = nh_.subscribe(vel_topic, 100, &VelPoseMuxNode::callbackFromTwistStamped, this);
  }
}

void VelPoseMuxNode::initPublisher()
{
  current_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("current_pose",10);
  current_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("current_velocity",10);
  linear_vel_viz_pub_ = nh_.advertise<std_msgs::Float32>("linear_velocity_viz", 10);

  if(publish_odom_)
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom_pose",10);
}

void VelPoseMuxNode::publishOdometry(const vehicle_socket::CanInfoConstPtr& msg)
{
  //tf::TransformBroadcaster odom_broadcaster;
  double vx = kmph2mps(msg->speed); //meter per sec
  double vth = convertSteerAngle2AngularVel(msg->angle, kmph2mps(msg->speed)); //radian per sec
  ros::Time current_time = ros::Time::now();

  // compute odometry in a typical way given the velocities of the robot
  double dt = (current_time - odom_.stamp).toSec();
  double delta_x = (vx * cos(odom_.th)) * dt;
  double delta_y = (vx * sin(odom_.th)) * dt;
  double delta_th = vth * dt;

  odom_.x += delta_x;
  odom_.y += delta_y;
  odom_.th += delta_th;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_.th);

  // first, we'll publish the transform over tf
  /*geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = odom_.x;
  odom_trans.transform.translation.y = odom_.y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  // send the transform
  odom_broadcaster.sendTransform(odom_trans);
  */

  // next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = odom_.x;
  odom.pose.pose.position.y = odom_.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.angular.z = vth;

  //publish the message
  odom_pub_.publish(odom);
  odom_.stamp = current_time;
}

void VelPoseMuxNode::publishCurrentPose(const geometry_msgs::PoseStampedConstPtr & msg)
{
  current_pose_pub_.publish(msg);
}

void VelPoseMuxNode::publishCurrentVelocity(const geometry_msgs::TwistStampedConstPtr & msg)
{
  current_vel_pub_.publish(msg);
}
void VelPoseMuxNode::publishLinearVelViz(const geometry_msgs::TwistStampedConstPtr & msg)
{
  std_msgs::Float32 fl;
  fl.data = mps2kmph(msg->twist.linear.x);
  linear_vel_viz_pub_.publish(fl);
}

void VelPoseMuxNode::publishCurrentVelocity(const vehicle_socket::CanInfoConstPtr& msg)
{
  geometry_msgs::TwistStamped tw;
  tw.header = msg->header;

  // linear velocity
  tw.twist.linear.x = kmph2mps(msg->speed);  // km/h -> m/s

  if (vehicle_info_flag_)
    tw.twist.angular.z = convertSteerAngle2AngularVel(msg->angle, kmph2mps(msg->speed));

  current_vel_pub_.publish(tw);
}

void VelPoseMuxNode::publishLinearVelViz(const vehicle_socket::CanInfoConstPtr& msg)
{
  std_msgs::Float32 fl;
  fl.data = msg->speed;
  linear_vel_viz_pub_.publish(fl);
}

void VelPoseMuxNode::callbackFromPoseStamped(const geometry_msgs::PoseStampedConstPtr& msg)
{
  publishCurrentPose(msg);
}

void VelPoseMuxNode::callbackFromTwistStamped(const geometry_msgs::TwistStampedConstPtr& msg)
{
  publishCurrentVelocity(msg);
  publishLinearVelViz(msg);
}

void VelPoseMuxNode::callbackFromCanInfo(const vehicle_socket::CanInfoConstPtr& msg)
{
  if (publish_odom_)
    publishOdometry(msg);

  publishCurrentVelocity(msg);
  publishLinearVelViz(msg);
}

double VelPoseMuxNode::convertSteerAngle2AngularVel(double steer_angle_deg, double linear_vel_mps)
{
  double maximum_tire_angle =
      rad2deg(atan(vehicle_info_.wheel_base / vehicle_info_.minimum_turning_radius));  //[degree]
  double current_tire_angle =
     steer_angle_deg * maximum_tire_angle / vehicle_info_.maximum_steering_angle;  // steering [degree] -> tire [degree]
  return tan(deg2rad(current_tire_angle)) * linear_vel_mps / vehicle_info_.wheel_base;
}
}
