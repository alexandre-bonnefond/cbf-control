// cbf_filter.cpp

#include "cbf_filter.hpp"

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox_ros/esdf_server.h>
#include <ros/ros.h>

namespace cbf_filter
{

CBFNode::CBFNode(const ros::NodeHandle& nh, const ros::NodeHandle& nhp)
  : nh_(nh)
  , nhp_(nhp)
  , esdf_server_(nh_, nhp_)
{
  // load private parameters
  nhp_.param("safe_distance", d_min_,  d_min_);
  nhp_.param("lambda",        lambda_, lambda_);

  // subscribe to private topics (no "~" in the name)
  sub_pilot_ = nhp_.subscribe("input_reference", 1,
                              &CBFNode::pilotCallback, this);
  sub_odom_  = nhp_.subscribe("odometry", 1,
                              &CBFNode::odomCallback,  this);

  // publish global outputs
  pub_filtered_    = nh_.advertise<mrs_msgs::VelocityReferenceStamped>(
                         "filtered_reference", 1);
  pub_grad_marker_ = nh_.advertise<visualization_msgs::Marker>(
                         "cbf_gradient_marker", 1);
  pub_obs_marker_  = nh_.advertise<visualization_msgs::Marker>(
                         "cbf_nearest_obstacle_marker", 1);

  ROS_INFO("[CBFNode] Initialised with Voxblox ESDF.");
}

void CBFNode::pilotCallback(const mrs_msgs::VelocityReferenceStamped::ConstPtr& msg)
{
  ROS_INFO_THROTTLE(5.0, "[CBFNode] pilotCallback()");
  latest_pilot_cmd_ = *msg;
  received_cmd_     = true;
}

void CBFNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO_THROTTLE(5.0, "[CBFNode] odomCallback()");
  current_position_.x() = msg->pose.pose.position.x;
  current_position_.y() = msg->pose.pose.position.y;
  current_position_.z() = msg->pose.pose.position.z;

  received_odom_ = true;
  processCBF();
}

bool CBFNode::queryObstacleDistance(const Eigen::Vector3d& pos,
                                    double& dist,
                                    Eigen::Vector3d& grad)
{
  auto esdf_map = esdf_server_.getEsdfMapPtr();
  if (!esdf_map) {
    ROS_WARN_THROTTLE(1.0, "[CBF] ESDF map ptr null.");
    return false;
  }

  // Use the interpolating overload
  bool ok = esdf_map->getDistanceAndGradientAtPosition(
                pos,
                /*interpolate=*/true,
                &dist, &grad);

  if (!ok) {
    ROS_WARN_THROTTLE(1.0,
      "[CBF] ESDF query failed at (%.2f, %.2f, %.2f)",
      pos.x(), pos.y(), pos.z());
  } else {
    ROS_INFO_THROTTLE(1.0,
      "[CBF] ESDF OK: dist=%.3f m, grad=(%.3f, %.3f, %.3f)",
      dist, grad.x(), grad.y(), grad.z());
  }
  return ok;
}

void CBFNode::processCBF()
{
  if (!received_odom_) return;

  ROS_INFO_THROTTLE(1.0, "[CBFNode] processCBF()");

  // 1 — desired velocity (pilot or zero)
  Eigen::Vector3d v_desired = Eigen::Vector3d::Zero();
  if (received_cmd_) {
    v_desired.x() = latest_pilot_cmd_.reference.velocity.x;
    v_desired.y() = latest_pilot_cmd_.reference.velocity.y;
    v_desired.z() = latest_pilot_cmd_.reference.velocity.z;
  }

  // 2 — ESDF query
  double dist;
  Eigen::Vector3d grad;
  if (!queryObstacleDistance(current_position_, dist, grad)) return;
  if (grad.norm() < 1e-6) return;  // ignore degenerate gradient

  Eigen::Vector3d n = grad.normalized();

  // log CBF values
  double h   = dist - d_min_;
  double phi = n.dot(v_desired) + lambda_ * h;
  ROS_INFO_THROTTLE(1.0,
    "[CBFNode] dist=%.2f  h=%.2f  phi=%.2f",
    dist, h, phi);

  // 3 — correct velocity if commanded
  if (received_cmd_ && phi < 0.0) {
    v_desired -= n * phi;
    ROS_WARN_THROTTLE(1.0,
      "[CBFNode] Velocity corrected by CBF (phi=%.2f)", phi);
  }

  // 4 — publish filtered velocity if any
  if (received_cmd_) {
    auto safe_cmd = latest_pilot_cmd_;
    safe_cmd.reference.velocity.x = v_desired.x();
    safe_cmd.reference.velocity.y = v_desired.y();
    safe_cmd.reference.velocity.z = v_desired.z();
    pub_filtered_.publish(safe_cmd);
    ROS_INFO_THROTTLE(1.0,
      "[CBFNode] Published filtered vel (%.2f, %.2f, %.2f)",
      v_desired.x(), v_desired.y(), v_desired.z());
  }

  // 5 — gradient arrow marker
  visualization_msgs::Marker grad_m;
  grad_m.header.frame_id = "uav1/fcu";
  grad_m.header.stamp    = ros::Time::now();
  grad_m.ns   = "cbf";
  grad_m.id   = 0;
  grad_m.type = visualization_msgs::Marker::ARROW;
  grad_m.action = visualization_msgs::Marker::ADD;
  grad_m.lifetime = ros::Duration(0.2);

  geometry_msgs::Point tail, tip_msg;
  tail.x = current_position_.x();
  tail.y = current_position_.y();
  tail.z = current_position_.z();

  double arrow_len = std::min(dist, 1.0);
  Eigen::Vector3d tip = current_position_ + n * arrow_len;
  tip_msg.x = tip.x();
  tip_msg.y = tip.y();
  tip_msg.z = tip.z();

  grad_m.points = {tail, tip_msg};
  grad_m.scale.x = 0.03;  // shaft diameter
  grad_m.scale.y = 0.06;  // head diameter
  grad_m.scale.z = 0.06;  // head length
  grad_m.color.r = 1.0;
  grad_m.color.a = 1.0;
  pub_grad_marker_.publish(grad_m);
  ROS_INFO_THROTTLE(1.0, "[CBFNode] Gradient marker published.");

  // 6 — nearest-obstacle sphere marker
  visualization_msgs::Marker obs_m = grad_m;
  obs_m.id   = 1;
  obs_m.type = visualization_msgs::Marker::SPHERE;
  Eigen::Vector3d boundary_pt = current_position_ - n * (dist - d_min_);
  obs_m.pose.position.x = boundary_pt.x();
  obs_m.pose.position.y = boundary_pt.y();
  obs_m.pose.position.z = boundary_pt.z();
  obs_m.scale.x = obs_m.scale.y = obs_m.scale.z = 0.25;
  obs_m.color.g = 1.0;
  obs_m.color.r = 0.0;
  pub_obs_marker_.publish(obs_m);
  ROS_INFO_THROTTLE(1.0, "[CBFNode] Obstacle sphere published.");
}

}  // namespace cbf_filter

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cbf_filter_node");
  ros::NodeHandle nh;      // global namespace
  ros::NodeHandle nhp("~"); // private namespace
  cbf_filter::CBFNode node(nh, nhp);
  ros::spin();
  return 0;
}
