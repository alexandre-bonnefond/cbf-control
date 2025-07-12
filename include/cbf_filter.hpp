// cbf_filter.hpp
#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <mrs_msgs/VelocityReferenceStamped.h>

#include <voxblox_ros/esdf_server.h>
#include <Eigen/Core>

namespace cbf_filter
{

class CBFNode
{
public:
  /// @param nh        Public NodeHandle (for topics in the UAV namespace)
  /// @param nh_private Private NodeHandle (“~”) for loading per-node params
  CBFNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

private:
  // ─────── Callbacks ─────────────────────────────────────────────────────
  void pilotCallback(const mrs_msgs::VelocityReferenceStamped::ConstPtr &msg);
  void odomCallback (const nav_msgs::Odometry::ConstPtr &msg);

  // ─────── CBF Processing ─────────────────────────────────────────────────
  void processCBF();

  /// Queries the ESDF for distance & gradient at @p pos.
  /// @return false if the ESDF isn’t yet ready or the query failed.
  bool queryObstacleDistance(const Eigen::Vector3d &pos,
                             double &dist,
                             Eigen::Vector3d &grad);

  // ─────── ROS Handles ────────────────────────────────────────────────────
  ros::NodeHandle   nh_, nhp_;
  ros::Subscriber   sub_pilot_, sub_odom_;
  ros::Publisher    pub_filtered_, pub_grad_marker_, pub_obs_marker_;

  // ─────── State ──────────────────────────────────────────────────────────
  Eigen::Vector3d                     current_position_{0, 0, 0};
  mrs_msgs::VelocityReferenceStamped latest_pilot_cmd_;
  bool                                received_odom_{false};
  bool                                received_cmd_{false};

  // ─────── Parameters ─────────────────────────────────────────────────────
  double d_min_{2.0};   ///< minimum safety distance [m]
  double lambda_{1.0};  ///< CBF gain [1/s]

  // ─────── Voxblox ESDF ───────────────────────────────────────────────────
  voxblox::EsdfServer esdf_server_;
};

}  // namespace cbf_filter
