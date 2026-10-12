#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <tf2/utils.h>
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
    private_nh_.param<std::string>(
      "teb_local_plan_topic", teb_local_plan_topic_,
      "/placenav_teb_bridge/TebLocalPlannerROS/local_plan");
    private_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    private_nh_.param("controller_frequency", controller_frequency_, 5.0);
    private_nh_.param("plan_position_epsilon", plan_position_epsilon_, 0.02);
    private_nh_.param("plan_orientation_epsilon", plan_orientation_epsilon_, 0.02);
    private_nh_.param("obstacle_trigger_distance", obstacle_trigger_distance_, 2.5);
    private_nh_.param("obstacle_corridor_width", obstacle_corridor_width_, 0.8);
    private_nh_.param("obstacle_min_forward_distance", obstacle_min_forward_distance_, 0.1);
    private_nh_.param("obstacle_cost_threshold", obstacle_cost_threshold_, 253);
    private_nh_.param("avoidance_clear_time", avoidance_clear_time_, 1.0);
    private_nh_.param("recovery_stable_time", recovery_stable_time_, 1.0);
    private_nh_.param("minimum_avoidance_plan_poses", minimum_avoidance_plan_poses_, 3);

    if (controller_frequency_ <= 0.0)
    {
      throw std::runtime_error("~controller_frequency must be positive");
    }
    if (obstacle_trigger_distance_ <= 0.0 || obstacle_corridor_width_ <= 0.0)
    {
      throw std::runtime_error("Obstacle trigger distance and corridor width must be positive");
    }

    costmap_ros_->start();
    teb_planner_.initialize("TebLocalPlannerROS", &tf_buffer_, costmap_ros_.get());

    cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    active_path_publisher_ = private_nh_.advertise<nav_msgs::Path>("active_global_path", 1, true);
    state_publisher_ = private_nh_.advertise<std_msgs::String>("navigation_state", 1, true);
    path_subscriber_ = nh_.subscribe(path_topic_, 1, &PlaceNavTebBridge::pathCallback, this);
    teb_local_plan_subscriber_ = nh_.subscribe(
      teb_local_plan_topic_, 1, &PlaceNavTebBridge::tebLocalPlanCallback, this);
    control_timer_ = nh_.createTimer(
      ros::Duration(1.0 / controller_frequency_),
      &PlaceNavTebBridge::controlCallback,
      this);

    publishState();
    ROS_INFO_STREAM(
      "PlaceNav TEB bridge: latest " << path_topic_
      << ", avoidance seed " << teb_local_plan_topic_
      << " -> " << cmd_vel_topic_);
  }

  ~PlaceNavTebBridge()
  {
    publishStop();
  }

private:
  enum class NavigationState
  {
    NORMAL,
    AVOIDING,
    RECOVERING
  };

  static const char* stateName(const NavigationState state)
  {
    switch (state)
    {
      case NavigationState::NORMAL:
        return "NORMAL";
      case NavigationState::AVOIDING:
        return "AVOIDING";
      case NavigationState::RECOVERING:
        return "RECOVERING";
    }
    return "UNKNOWN";
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& path)
  {
    if (!validPath(*path, "GNM Path"))
    {
      return;
    }

    std::lock_guard<std::mutex> lock(planner_mutex_);
    latest_placenav_path_ = *path;
    has_latest_placenav_path_ = true;

    // Keep receiving the newest PlaceNav path during avoidance, but do not let
    // it overwrite the promoted TEB avoidance path.
    if (state_ == NavigationState::AVOIDING)
    {
      return;
    }

    setActivePlan(
      *path,
      state_ == NavigationState::NORMAL ? "latest PlaceNav" : "recovery PlaceNav");
  }

  void tebLocalPlanCallback(const nav_msgs::Path::ConstPtr& path)
  {
    if (path->poses.size() < static_cast<std::size_t>(minimum_avoidance_plan_poses_) ||
        path->header.frame_id.empty())
    {
      return;
    }

    std::lock_guard<std::mutex> lock(planner_mutex_);
    latest_teb_local_path_ = *path;
    has_latest_teb_local_path_ = true;
  }

  bool validPath(const nav_msgs::Path& path, const char* label) const
  {
    if (path.poses.size() < 2)
    {
      ROS_WARN_THROTTLE(1.0, "%s must contain at least two poses", label);
      return false;
    }
    if (path.header.frame_id.empty())
    {
      ROS_WARN_THROTTLE(1.0, "%s has no frame_id", label);
      return false;
    }
    return true;
  }

  bool setActivePlan(const nav_msgs::Path& path, const char* source)
  {
    // After reaching an endpoint, an equivalent new path must reactivate TEB.
    // Deduplicate only while the previous plan is still usable.
    if (has_plan_ && has_active_plan_ && plansEquivalent(path, active_plan_))
    {
      return true;
    }
    if (!teb_planner_.setPlan(path.poses))
    {
      ROS_ERROR_THROTTLE(1.0, "TEB rejected the %s path", source);
      has_plan_ = false;
      return false;
    }

    active_plan_ = path;
    active_plan_.header.stamp = ros::Time::now();
    has_active_plan_ = true;
    has_plan_ = true;
    active_path_publisher_.publish(active_plan_);
    ROS_INFO_STREAM("TEB active global path <- " << source << " (" << path.poses.size() << " poses)");
    return true;
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

  static double squaredDistanceToPath(
    const double x, const double y, const nav_msgs::Path& path)
  {
    double best = std::numeric_limits<double>::infinity();
    for (const auto& pose : path.poses)
    {
      const double dx = x - pose.pose.position.x;
      const double dy = y - pose.pose.position.y;
      best = std::min(best, dx * dx + dy * dy);
    }
    return best;
  }

  bool obstacleBlocksLatestPlaceNavPath()
  {
    if (!has_latest_placenav_path_)
    {
      return false;
    }

    geometry_msgs::PoseStamped robot_pose;
    if (!costmap_ros_->getRobotPose(robot_pose))
    {
      ROS_WARN_THROTTLE(1.0, "Cannot obtain robot pose for obstacle-state detection");
      return false;
    }

    const double yaw = tf2::getYaw(robot_pose.pose.orientation);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double trigger_squared = obstacle_trigger_distance_ * obstacle_trigger_distance_;
    const double corridor_squared = obstacle_corridor_width_ * obstacle_corridor_width_;
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));

    for (unsigned int mx = 0; mx < costmap->getSizeInCellsX(); ++mx)
    {
      for (unsigned int my = 0; my < costmap->getSizeInCellsY(); ++my)
      {
        const unsigned char cost = costmap->getCost(mx, my);
        if (cost == costmap_2d::NO_INFORMATION || cost < obstacle_cost_threshold_)
        {
          continue;
        }

        double wx = 0.0;
        double wy = 0.0;
        costmap->mapToWorld(mx, my, wx, wy);
        const double dx = wx - robot_pose.pose.position.x;
        const double dy = wy - robot_pose.pose.position.y;
        const double forward = cos_yaw * dx + sin_yaw * dy;
        const double distance_squared = dx * dx + dy * dy;
        if (forward < obstacle_min_forward_distance_ || distance_squared > trigger_squared)
        {
          continue;
        }
        if (squaredDistanceToPath(wx, wy, latest_placenav_path_) <= corridor_squared)
        {
          return true;
        }
      }
    }
    return false;
  }

  void transitionToAvoiding()
  {
    if (!has_latest_teb_local_path_)
    {
      ROS_WARN_THROTTLE(1.0, "Obstacle detected, waiting for a valid TEB local avoidance path");
      return;
    }
    if (!setActivePlan(latest_teb_local_path_, "promoted TEB local avoidance"))
    {
      return;
    }
    state_ = NavigationState::AVOIDING;
    state_entered_at_ = ros::Time::now();
    last_obstacle_at_ = state_entered_at_;
    publishState();
    ROS_WARN("Navigation state: NORMAL/RECOVERING -> AVOIDING");
  }

  void transitionToRecovering()
  {
    if (!has_latest_placenav_path_ ||
        !setActivePlan(latest_placenav_path_, "latest PlaceNav recovery"))
    {
      ROS_WARN_THROTTLE(1.0, "Cannot recover: no valid latest PlaceNav path");
      return;
    }
    state_ = NavigationState::RECOVERING;
    state_entered_at_ = ros::Time::now();
    publishState();
    ROS_INFO("Navigation state: AVOIDING -> RECOVERING");
  }

  void transitionToNormal()
  {
    state_ = NavigationState::NORMAL;
    state_entered_at_ = ros::Time::now();
    publishState();
    ROS_INFO("Navigation state: RECOVERING -> NORMAL");
  }

  void updateNavigationState(const bool obstacle_detected)
  {
    const ros::Time now = ros::Time::now();
    if (obstacle_detected)
    {
      last_obstacle_at_ = now;
      if (state_ != NavigationState::AVOIDING)
      {
        transitionToAvoiding();
      }
      return;
    }

    if (state_ == NavigationState::AVOIDING &&
        !last_obstacle_at_.isZero() &&
        (now - last_obstacle_at_).toSec() >= avoidance_clear_time_)
    {
      transitionToRecovering();
    }
    else if (state_ == NavigationState::RECOVERING &&
             (now - state_entered_at_).toSec() >= recovery_stable_time_)
    {
      transitionToNormal();
    }
  }

  void controlCallback(const ros::TimerEvent&)
  {
    geometry_msgs::Twist command;
    bool command_valid = false;
    bool had_plan = false;

    {
      std::lock_guard<std::mutex> lock(planner_mutex_);
      updateNavigationState(obstacleBlocksLatestPlaceNavPath());
      had_plan = has_plan_;
      if (has_plan_)
      {
        command_valid = teb_planner_.computeVelocityCommands(command);
        if (teb_planner_.isGoalReached())
        {
          if (state_ == NavigationState::AVOIDING && has_latest_placenav_path_)
          {
            transitionToRecovering();
          }
          else
          {
            has_plan_ = false;
          }
          command_valid = false;
          ROS_INFO_THROTTLE(1.0, "TEB reached the end of the active path");
        }
      }
    }

    if (!command_valid)
    {
      command = geometry_msgs::Twist();
      if (had_plan)
      {
        ROS_WARN_THROTTLE(1.0, "TEB could not compute a valid command; publishing zero velocity");
      }
    }
    cmd_vel_publisher_.publish(command);
  }

  void publishState()
  {
    std_msgs::String message;
    message.data = stateName(state_);
    state_publisher_.publish(message);
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
  ros::Subscriber teb_local_plan_subscriber_;
  ros::Publisher cmd_vel_publisher_;
  ros::Publisher active_path_publisher_;
  ros::Publisher state_publisher_;
  ros::Timer control_timer_;
  std::mutex planner_mutex_;
  std::string path_topic_;
  std::string teb_local_plan_topic_;
  std::string cmd_vel_topic_;
  double controller_frequency_{5.0};
  double plan_position_epsilon_{0.02};
  double plan_orientation_epsilon_{0.02};
  double obstacle_trigger_distance_{2.5};
  double obstacle_corridor_width_{0.8};
  double obstacle_min_forward_distance_{0.1};
  int obstacle_cost_threshold_{253};
  double avoidance_clear_time_{1.0};
  double recovery_stable_time_{1.0};
  int minimum_avoidance_plan_poses_{3};
  NavigationState state_{NavigationState::NORMAL};
  ros::Time state_entered_at_;
  ros::Time last_obstacle_at_;
  nav_msgs::Path latest_placenav_path_;
  nav_msgs::Path latest_teb_local_path_;
  nav_msgs::Path active_plan_;
  bool has_latest_placenav_path_{false};
  bool has_latest_teb_local_path_{false};
  bool has_active_plan_{false};
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
