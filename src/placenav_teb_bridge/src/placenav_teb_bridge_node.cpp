#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class PlaceNavTebBridge
{
public:
  PlaceNavTebBridge()
    : nh_(),
      private_nh_("~"),
      tf_buffer_(ros::Duration(10.0)),
      tf_listener_(tf_buffer_),
      costmap_ros_(new costmap_2d::Costmap2DROS("local_costmap", tf_buffer_))
  {
    private_nh_.param<std::string>("path_topic", path_topic_, "/placenav/gnm_path");
    private_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    private_nh_.param("controller_frequency", controller_frequency_, 5.0);
    private_nh_.param("plan_position_epsilon", plan_position_epsilon_, 0.02);
    private_nh_.param("plan_orientation_epsilon", plan_orientation_epsilon_, 0.02);

    if (controller_frequency_ <= 0.0)
    {
      throw std::runtime_error("~controller_frequency must be positive");
    }

    costmap_ros_->start();
    teb_planner_.initialize("TebLocalPlannerROS", &tf_buffer_, costmap_ros_.get());

    cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    path_subscriber_ = nh_.subscribe(path_topic_, 1, &PlaceNavTebBridge::pathCallback, this);
    control_timer_ = nh_.createTimer(
      ros::Duration(1.0 / controller_frequency_),
      &PlaceNavTebBridge::controlCallback,
      this);

    ROS_INFO_STREAM("PlaceNav TEB bridge: " << path_topic_ << " -> setPlan() -> " << cmd_vel_topic_);
  }

  ~PlaceNavTebBridge()
  {
    publishStop();
  }

private:
  void pathCallback(const nav_msgs::Path::ConstPtr& path)
  {
    if (path->poses.size() < 2)
    {
      ROS_WARN_THROTTLE(1.0, "GNM Path must contain at least two poses");
      return;
    }
    if (path->header.frame_id.empty())
    {
      ROS_WARN_THROTTLE(1.0, "GNM Path has no frame_id");
      return;
    }

    std::lock_guard<std::mutex> lock(planner_mutex_);
    if (has_last_plan_ && plansEquivalent(*path, last_plan_))
    {
      return;
    }
    if (!teb_planner_.setPlan(path->poses))
    {
      ROS_ERROR_THROTTLE(1.0, "TEB rejected the GNM Path");
      has_plan_ = false;
      return;
    }
    last_plan_ = *path;
    has_last_plan_ = true;
    has_plan_ = true;
  }

  bool plansEquivalent(const nav_msgs::Path& first, const nav_msgs::Path& second) const
  {
    if (first.header.frame_id != second.header.frame_id ||
        first.poses.size() != second.poses.size())
    {
      return false;
    }

    const double position_limit_squared =
      plan_position_epsilon_ * plan_position_epsilon_;
    const double orientation_dot_limit = std::cos(plan_orientation_epsilon_ * 0.5);
    for (std::size_t index = 0; index < first.poses.size(); ++index)
    {
      const auto& first_pose = first.poses[index].pose;
      const auto& second_pose = second.poses[index].pose;
      const double dx = first_pose.position.x - second_pose.position.x;
      const double dy = first_pose.position.y - second_pose.position.y;
      if (dx * dx + dy * dy > position_limit_squared)
      {
        return false;
      }

      const auto& first_q = first_pose.orientation;
      const auto& second_q = second_pose.orientation;
      const double dot = std::abs(
        first_q.x * second_q.x + first_q.y * second_q.y +
        first_q.z * second_q.z + first_q.w * second_q.w);
      if (dot < orientation_dot_limit)
      {
        return false;
      }
    }
    return true;
  }

  void controlCallback(const ros::TimerEvent&)
  {
    geometry_msgs::Twist command;
    bool command_valid = false;

    {
      std::lock_guard<std::mutex> lock(planner_mutex_);
      if (has_plan_)
      {
        command_valid = teb_planner_.computeVelocityCommands(command);
        if (teb_planner_.isGoalReached())
        {
          has_plan_ = false;
          command_valid = false;
          ROS_INFO_THROTTLE(1.0, "TEB reached the end of the current GNM Path");
        }
      }
    }

    if (!command_valid)
    {
      command = geometry_msgs::Twist();
      if (has_plan_)
      {
        ROS_WARN_THROTTLE(1.0, "TEB could not compute a valid command; publishing zero velocity");
      }
    }
    cmd_vel_publisher_.publish(command);
  }

  void publishStop()
  {
    if (cmd_vel_publisher_)
    {
      cmd_vel_publisher_.publish(geometry_msgs::Twist());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<costmap_2d::Costmap2DROS> costmap_ros_;
  teb_local_planner::TebLocalPlannerROS teb_planner_;
  ros::Subscriber path_subscriber_;
  ros::Publisher cmd_vel_publisher_;
  ros::Timer control_timer_;
  std::mutex planner_mutex_;
  std::string path_topic_;
  std::string cmd_vel_topic_;
  double controller_frequency_{5.0};
  double plan_position_epsilon_{0.02};
  double plan_orientation_epsilon_{0.02};
  nav_msgs::Path last_plan_;
  bool has_last_plan_{false};
  bool has_plan_{false};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "placenav_teb_bridge");
  try
  {
    PlaceNavTebBridge bridge;
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL_STREAM("Failed to start PlaceNav TEB bridge: " << error.what());
    return 1;
  }
  return 0;
}
