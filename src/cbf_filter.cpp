#include <cbf_filter.hpp>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/Marker.h>


namespace cbf_filter
{

  CBFNode::CBFNode(ros::NodeHandle &nh)
  {
    sub_pilot_ = nh.subscribe("input_reference", 1, &CBFNode::pilotCallback, this);
    sub_odom_ = nh.subscribe("odometry", 1, &CBFNode::odomCallback, this);
    sub_octomap_ = nh.subscribe("octomap_binary", 1, &CBFNode::octomapCallback, this);

    pub_filtered_ = nh.advertise<mrs_msgs::VelocityReferenceStamped>("filtered_reference", 1);
    pub_gradient_marker_ = nh.advertise<visualization_msgs::Marker>("cbf_gradient_marker", 1);
    pub_obstacle_marker_ = nh.advertise<visualization_msgs::Marker>("cbf_nearest_obstacle_marker", 1);

    ROS_INFO("[CBFNode]: Initialized.");
  }

  void CBFNode::pilotCallback(const mrs_msgs::VelocityReferenceStamped::ConstPtr &msg)
  {
    latest_pilot_cmd_ = *msg;
    received_cmd_ = true;

    if (received_odom_ && octree_ready_)
      processCBF();
  }

  void CBFNode::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    ROS_INFO_THROTTLE(5.0, "odom received");

    current_position_ = Eigen::Vector3d(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);

    current_velocity_ = Eigen::Vector3d(
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z);

    received_odom_ = true;

    if (received_cmd_ && octree_ready_)
      processCBF();
  }

  void CBFNode::octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
  {
    auto tree_ptr = dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg));
    if (tree_ptr)
    {
      octree_ = std::shared_ptr<octomap::OcTree>(tree_ptr);
      octree_ready_ = true;
      ROS_INFO_THROTTLE(5.0, "[CBFNode]: Octomap received and converted.");
    }
    else
    {
      ROS_ERROR("[CBFNode]: Failed to convert Octomap message.");
    }
  }

  bool CBFNode::queryObstacleDistance(const Eigen::Vector3d &pos, double &dist, Eigen::Vector3d &grad)
  {
    if (!octree_ || !octree_ready_)
      return false;

    double min_dist = std::numeric_limits<double>::max();
    Eigen::Vector3d nearest;

    for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it)
    {
      if (octree_->isNodeOccupied(*it))
      {

        // ROS_INFO_STREAM("Found occupied voxel at: " << it.getX() << ", " << it.getY() << ", " << it.getZ());

        Eigen::Vector3d voxel(it.getX(), it.getY(), it.getZ());
        double d = (voxel - pos).norm();
        if (d < min_dist)
        {
          min_dist = d;
          nearest = voxel;
        }
      }      
    }

    ROS_INFO_STREAM("obstacle at: " << min_dist);

    if (min_dist == std::numeric_limits<double>::max()) {
      ROS_INFO_STREAM("Closest obstacle at: " << nearest.transpose() << ", distance: " << min_dist);
      return false;
    }

    dist = min_dist;
    grad = (pos - nearest).normalized(); // direction away from obstacle
    return true;
  }

  void CBFNode::processCBF()
  {
    Eigen::Vector3d v_desired(
        latest_pilot_cmd_.reference.velocity.x,
        latest_pilot_cmd_.reference.velocity.y,
        latest_pilot_cmd_.reference.velocity.z);

    double dist, h;
    Eigen::Vector3d grad;
    double d_min = 2;  // safe distance
    double lambda = 1.0; // CBF gain
    ROS_INFO("Entered the process");

    if (!queryObstacleDistance(current_position_, dist, grad))
    {
      ROS_WARN_THROTTLE(1.0, "[CBFNode]: No obstacle detected in octomap.");
      return;
    }

    h = dist - d_min;
    double cbf_value = grad.dot(v_desired) + lambda * h;

    if (cbf_value < 0)
    {
      Eigen::Vector3d correction = grad * (-cbf_value);
      v_desired += correction;
      ROS_WARN_THROTTLE(1.0, "[CBFNode]: Velocity corrected to avoid obstacle.");
    }

    mrs_msgs::VelocityReferenceStamped safe_cmd = latest_pilot_cmd_;
    safe_cmd.reference.velocity.x = v_desired.x();
    safe_cmd.reference.velocity.y = v_desired.y();
    safe_cmd.reference.velocity.z = v_desired.z();

    pub_filtered_.publish(safe_cmd);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "uav1/fcu";
    // marker.header.stamp = ros::Time::now();
    marker.ns = "cbf";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    // Origin: drone position
    geometry_msgs::Point start, end;
    start.x = current_position_.x();
    start.y = current_position_.y();
    start.z = current_position_.z();

    // Arrow direction: grad (normalized and scaled)
    Eigen::Vector3d end_pos = current_position_ + 1.0 * grad.normalized(); // scale for display

    end.x = end_pos.x();
    end.y = end_pos.y();
    end.z = end_pos.z();

    marker.points.push_back(start);
    marker.points.push_back(end);

    // Appearance
    marker.scale.x = 0.05; // shaft diameter
    marker.scale.y = 0.1;  // head diameter
    marker.scale.z = 0.1;  // head length
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    pub_gradient_marker_.publish(marker);

    // SPHERE marker to show nearest obstacle
    visualization_msgs::Marker obstacle_marker;
    obstacle_marker.header.frame_id = "uav1/fcu_untilted";
    // obstacle_marker.header.stamp = ros::Time::now();
    obstacle_marker.ns = "cbf";
    obstacle_marker.id = 1;
    obstacle_marker.type = visualization_msgs::Marker::SPHERE;
    obstacle_marker.action = visualization_msgs::Marker::ADD;

    obstacle_marker.pose.position.x = current_position_.x() - grad.x() * dist;
    obstacle_marker.pose.position.y = current_position_.y() - grad.y() * dist;
    obstacle_marker.pose.position.z = current_position_.z() - grad.z() * dist;

    obstacle_marker.scale.x = 2.2; // sphere diameter
    obstacle_marker.scale.y = 2.2;
    obstacle_marker.scale.z = 2.2;

    obstacle_marker.color.r = 1.0;
    obstacle_marker.color.g = 1.0;
    obstacle_marker.color.b = 0.0;
    obstacle_marker.color.a = 1.0;

    pub_obstacle_marker_.publish(obstacle_marker);
  }

} // namespace cbf_filter

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cbf_filter_node");
  ros::NodeHandle nh("~");

  cbf_filter::CBFNode node(nh);

  ros::spin();
  return 0;
}
