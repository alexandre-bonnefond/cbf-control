#include "cbf_filter.hpp"
#include <voxblox_ros/esdf_server.h>
#include <voxblox/core/esdf_map.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>

namespace cbf_filter {

CBFNode::CBFNode(const ros::NodeHandle& nh, const ros::NodeHandle& nhp)
  : nh_(nh)
  , nhp_(nhp)
  , esdf_server_(nh_, nhp_)
{
  
  nhp_.param("safe_distance", d_min_,  d_min_);
  nhp_.param("cbf_lambda",    lambda_, lambda_);

  // Subscribers
  sub_pilot_ = nh_.subscribe("input_reference", 1,
                             &CBFNode::pilotCallback, this);
  sub_odom_  = nh_.subscribe("odometry",        1,
                             &CBFNode::odomCallback,  this);

  // Publishers: filtered velocity reference and visualization markers
  pub_filtered_   = nh_.advertise<mrs_msgs::VelocityReferenceStamped>(
                        "filtered_reference", 1);
  pub_grad_marker_= nh_.advertise<visualization_msgs::Marker>(
                        "cbf_gradient_marker", 1);
  pub_obs_marker_ = nh_.advertise<visualization_msgs::Marker>(
                        "cbf_nearest_obstacle_marker", 1);

  ROS_INFO("[CBFNode] Initialized with Voxblox ESDF.");
}

// Callback: store pilot's desired velocity and run CBF if odometry available
void CBFNode::pilotCallback(const mrs_msgs::VelocityReferenceStamped::ConstPtr& msg) {
  latest_pilot_cmd_ = *msg;
  received_cmd_     = true;
  if (received_odom_) {
    processCBF();
  }
}

// Callback: update current position, set received flag, and run CBF if command available
void CBFNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  current_position_.x() = msg->pose.pose.position.x;
  current_position_.y() = msg->pose.pose.position.y;
  current_position_.z() = msg->pose.pose.position.z;

  received_odom_ = true;
  if (received_cmd_) {
    processCBF();
  }
}

// Query the Voxblox ESDF: get true distance and gradient vector at 'pos'
bool CBFNode::queryObstacleDistance(const Eigen::Vector3d& pos,
                                    double& dist,
                                    Eigen::Vector3d& grad) {
  auto esdf_map = esdf_server_.getEsdfMapPtr();
  if (!esdf_map) {
    ROS_WARN_THROTTLE(1.0, "[CBFNode] ESDF map not yet available.");
    return false;
  }

  // Use the Eigen-based API for distance + gradient
  if (!esdf_map->getDistanceAndGradientAtPosition(pos, &dist, &grad)) {
    ROS_WARN_THROTTLE(1.0, "[CBFNode] ESDF query failed.");
    return false;
  }
  return true;
}

// Main barrier filter: correct pilot command minimally to satisfy safety
void CBFNode::processCBF() {
  // Desired velocity from pilot
  Eigen::Vector3d v_desired(
    latest_pilot_cmd_.reference.velocity.x,
    latest_pilot_cmd_.reference.velocity.y,
    latest_pilot_cmd_.reference.velocity.z);

  // Get distance to nearest obstacle and gradient direction
  double dist;
  Eigen::Vector3d grad;
  if (!queryObstacleDistance(current_position_, dist, grad)) {
    return;  // ESDF not yet ready
  }

  // Control Barrier Function: h = dist - d_min
  double h   = dist - d_min_;
  double phi = grad.dot(v_desired) + lambda_ * h;

  // If barrier is violated, apply minimal correction
  if (phi < 0.0) {
    v_desired -= grad * phi;
    ROS_WARN_THROTTLE(1.0, "[CBFNode] Velocity corrected by CBF.");
  }

  // Publish the corrected (safe) command
  auto safe_cmd = latest_pilot_cmd_;
  safe_cmd.reference.velocity.x = v_desired.x();
  safe_cmd.reference.velocity.y = v_desired.y();
  safe_cmd.reference.velocity.z = v_desired.z();
  pub_filtered_.publish(safe_cmd);

  // Visualization: gradient arrow
  visualization_msgs::Marker grad_m;
  grad_m.header.frame_id = "uav1/fcu";
  grad_m.header.stamp    = ros::Time::now();
  grad_m.ns              = "cbf";
  grad_m.id              = 0;
  grad_m.type            = visualization_msgs::Marker::ARROW;
  grad_m.action          = visualization_msgs::Marker::ADD;
  geometry_msgs::Point a, b;
  a.x = current_position_.x();
  a.y = current_position_.y();
  a.z = current_position_.z();
  Eigen::Vector3d tip = current_position_ + grad;
  b.x = tip.x(); b.y = tip.y(); b.z = tip.z();
  grad_m.points = {a, b};
  grad_m.scale.x = 0.05; grad_m.scale.y = 0.1; grad_m.scale.z = 0.1;
  grad_m.color.r = 1.0; grad_m.color.a = 1.0;
  pub_grad_marker_.publish(grad_m);

  // Visualization: nearest obstacle sphere
  visualization_msgs::Marker obs_m = grad_m;
  obs_m.id   = 1;
  obs_m.type = visualization_msgs::Marker::SPHERE;
  Eigen::Vector3d obs_pt = current_position_ - grad * dist;
  obs_m.pose.position.x = obs_pt.x();
  obs_m.pose.position.y = obs_pt.y();
  obs_m.pose.position.z = obs_pt.z();
  obs_m.scale.x = obs_m.scale.y = obs_m.scale.z = 0.25;
  obs_m.color.g = 1.0;
  pub_obs_marker_.publish(obs_m);
}

}  // namespace cbf_filter

// ──────────────────────────────────────────────── main ─────────────────────────────────────────────
int main(int argc, char** argv) {
  ros::init(argc, argv, "cbf_filter_node");
  ros::NodeHandle nh;              // public namespace
  ros::NodeHandle nhp("~");      // private namespace
  cbf_filter::CBFNode node(nh, nhp);
  ros::spin();
  return 0;
}
