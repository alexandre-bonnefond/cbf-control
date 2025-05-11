#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/Octomap.h>
#include <mrs_msgs/VelocityReferenceStamped.h>

#include <Eigen/Dense>

namespace octomap {
  class OcTree;  // forward declaration to avoid full include in header
}

namespace cbf_filter
{

class CBFNode
{
public:
  explicit CBFNode(ros::NodeHandle& nh);

  void pilotCallback(const mrs_msgs::VelocityReferenceStamped::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);

private:
  // ROS interfaces
  ros::Subscriber sub_pilot_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_octomap_;
  ros::Publisher pub_filtered_;
  ros::Publisher pub_gradient_marker_;
  ros::Publisher pub_obstacle_marker_;

  // State
  Eigen::Vector3d current_position_;
  Eigen::Vector3d current_velocity_;
  mrs_msgs::VelocityReferenceStamped latest_pilot_cmd_;

  bool received_odom_ = false;
  bool received_cmd_ = false;
  bool octree_ready_ = false;

  // Octomap
  std::shared_ptr<octomap::OcTree> octree_;
  bool queryObstacleDistance(const Eigen::Vector3d& pos, double& dist, Eigen::Vector3d& grad);

  // Main logic
  void processCBF();
};

}
