#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/footprint.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <tf2/utils.h>
#include <placenav_teb_bridge/retained_path.h>
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

    private_nh_.param<std::string>("reference_mode", reference_mode_, "state_machine");
    private_nh_.param("path_timeout", path_timeout_, 0.0);
    if (reference_mode_ != "state_machine" && reference_mode_ != "latest" && reference_mode_ != "avoidance_commit")
      throw std::runtime_error("reference_mode must be state_machine, latest or avoidance_commit");
    if (!std::isfinite(path_timeout_) || path_timeout_ < 0.)
      throw std::runtime_error("path_timeout must be nonnegative");

    if (controller_frequency_ <= 0.0)
    {
      throw std::runtime_error("~controller_frequency must be positive");
    }
    if (obstacle_trigger_distance_ <= 0.0 || obstacle_corridor_width_ <= 0.0)
    {
      throw std::runtime_error("Obstacle trigger distance and corridor width must be positive");
    }

    private_nh_.param("retain_safe_path", retain_safe_path_, false);
    private_nh_.param("retention_max_duration", retention_max_duration_, 3.0);
    private_nh_.param("retention_max_deviation", retention_max_deviation_, 0.20);
    private_nh_.param("retention_lookahead", retention_lookahead_, 0.30);
    private_nh_.param("retention_goal_shift", retention_goal_shift_, 0.75);
    if (reference_mode_ == "avoidance_commit") retain_safe_path_ = true;
    if (retain_safe_path_ && reference_mode_ != "latest" && reference_mode_ != "avoidance_commit")
      throw std::runtime_error("retain_safe_path requires reference_mode=latest");
    for (double v : {retention_max_duration_, retention_max_deviation_, retention_lookahead_, retention_goal_shift_})
      if (!std::isfinite(v) || v <= 0.) throw std::runtime_error("retention parameters must be finite and positive");
    costmap_ros_->start();
    teb_planner_.initialize("TebLocalPlannerROS", &tf_buffer_, costmap_ros_.get());

    cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    active_path_publisher_ = private_nh_.advertise<nav_msgs::Path>("active_global_path", 1, true);
    state_publisher_ = private_nh_.advertise<std_msgs::String>("navigation_state", 1, true);
    retained_path_publisher_ = private_nh_.advertise<nav_msgs::Path>("retained_safe_path", 1, true);
    diagnostics_publisher_ = private_nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);
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
    std::lock_guard<std::mutex> lock(planner_mutex_);
    if (!validPath(*path, "GNM Path"))
    {
      invalidatePlan("INVALID_INPUT_PATH");
      return;
    }
    latest_placenav_path_ = *path;
    latest_path_wall_ = ros::WallTime::now();
    has_latest_placenav_path_ = true;

    if (retain_safe_path_ && (reference_mode_ != "avoidance_commit" || state_ != NavigationState::NORMAL)) return;

    // Keep receiving the newest PlaceNav path during avoidance, but do not let
    // it overwrite the promoted TEB avoidance path.
    if (reference_mode_ == "state_machine" && state_ == NavigationState::AVOIDING)
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
    if (retain_safe_path_) {
      for (const auto& stamped : path.poses) {
        const auto& p=stamped.pose;
        const auto& q=p.orientation;
        const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
        if (!std::isfinite(p.position.x)||!std::isfinite(p.position.y)||!std::isfinite(norm)||norm<1e-12)
          return false;
      }
    }
    return true;
  }

  bool setActivePlan(const nav_msgs::Path& path, const char* source)
  {
    if (has_plan_ && has_active_plan_ && plansEquivalent(path, active_plan_))
    {
      return true;
    }
    if (!teb_planner_.setPlan(path.poses))
    {
      ROS_ERROR_THROTTLE(1.0, "TEB rejected the %s path", source);
      invalidatePlan("SET_PLAN_REJECTED");
      return false;
    }

    active_source_ = source;
    stop_reason_ = "NONE";
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
    cost_cells_ = range_cells_ = corridor_cells_ = 0;
    detection_status_ = "NO_PATH";
    if (!has_latest_placenav_path_)
    {
      return false;
    }

    geometry_msgs::PoseStamped robot_pose;
    if (!costmap_ros_->getRobotPose(robot_pose))
    {
      detection_status_ = "ROBOT_TF_UNAVAILABLE";
      ROS_WARN_THROTTLE(1.0, "Cannot obtain robot pose for obstacle-state detection");
      return false;
    }

    if (latest_placenav_path_.header.frame_id != costmap_ros_->getGlobalFrameID())
    {
      detection_status_ = "PATH_FRAME_MISMATCH";
      return false;
    }
    detection_status_ = "CLEAR";
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

        ++cost_cells_;
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
        ++range_cells_;
        if (squaredDistanceToPath(wx, wy, latest_placenav_path_) <= corridor_squared)
        {
          ++corridor_cells_;
        }
      }
    }
    if (corridor_cells_) detection_status_ = "BLOCKED";
    return corridor_cells_ > 0;
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

  bool pathExpired() const
  {
    return path_timeout_ > 0. && has_latest_placenav_path_ &&
      (ros::WallTime::now()-latest_path_wall_).toSec() > path_timeout_;
  }

  void invalidatePlan(const std::string& reason)
  {
    clearRetained();
    has_plan_ = has_active_plan_ = has_latest_placenav_path_ = false;
    has_latest_teb_local_path_ = false;
    active_plan_ = nav_msgs::Path();
    active_plan_.header.frame_id = costmap_ros_->getGlobalFrameID();
    active_plan_.header.stamp = ros::Time::now();
    active_source_ = "none";
    stop_reason_ = reason;
    state_ = NavigationState::NORMAL;
    active_path_publisher_.publish(active_plan_);
    publishState();
    publishStop();
  }

  static double pathLength(const nav_msgs::Path& path)
  {
    double length = 0.;
    for (std::size_t i=1; i<path.poses.size(); ++i)
      length += std::hypot(path.poses[i].pose.position.x-path.poses[i-1].pose.position.x,
                           path.poses[i].pose.position.y-path.poses[i-1].pose.position.y);
    return length;
  }

  void publishDiagnostics(const std::string& reason, int outcome,
                          const std::string& detail, const geometry_msgs::Twist& command)
  {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "placenav_teb_bridge/control";
    status.hardware_id = "navigation";
    status.level = reason == "COMMAND_VALID" ? 0 : 1;
    status.message = reason;
    auto add = [&status](const std::string& key, const std::string& value) {
      diagnostic_msgs::KeyValue item; item.key=key; item.value=value;
      status.values.push_back(item);
    };
    add("reference_mode", reference_mode_);
    add("retain_safe_path", retain_safe_path_ ? "true" : "false");
    add("retention_status", retention_status_);
    add("latest_reference_stamp", has_latest_placenav_path_?std::to_string(latest_placenav_path_.header.stamp.toSec()):"none");
    add("avoidance_reference_stamp", std::to_string(avoidance_reference_.header.stamp.toSec()));
    add("retention_age_wall_s", retained_.empty() ? "0" : std::to_string((ros::WallTime::now()-retained_at_).toSec()));
    add("state", reference_mode_ == "latest" ? "TRACKING" : stateName(state_));
    add("active_source", active_source_);
    add("teb_outcome", std::to_string(outcome));
    add("teb_message", detail);
    add("detection_status", detection_status_);
    add("cells_above_cost_threshold", std::to_string(cost_cells_));
    add("cells_in_forward_radius", std::to_string(range_cells_));
    add("cells_near_path_points", std::to_string(corridor_cells_));
    add("waiting_for_teb_seed", reference_mode_ == "state_machine" && corridor_cells_ &&
        state_ != NavigationState::AVOIDING && !has_latest_teb_local_path_ ? "true" : "false");
    add("input_poses", std::to_string(has_latest_placenav_path_ ? latest_placenav_path_.poses.size() : 0));
    add("input_length_m", std::to_string(has_latest_placenav_path_ ? pathLength(latest_placenav_path_) : 0.));
    add("active_poses", std::to_string(active_plan_.poses.size()));
    add("active_length_m", std::to_string(pathLength(active_plan_)));
    add("input_age_wall_s", has_latest_placenav_path_ ? std::to_string((ros::WallTime::now()-latest_path_wall_).toSec()) : "unavailable");
    add("input_stamp", has_latest_placenav_path_ ? std::to_string(latest_placenav_path_.header.stamp.toSec()) : "unavailable");
    add("cmd_v", std::to_string(command.linear.x));
    add("cmd_w", std::to_string(command.angular.z));
    bool shrink = false;
    private_nh_.getParam("TebLocalPlannerROS/shrink_horizon_backup", shrink);
    add("shrink_horizon_enabled", shrink ? "true" : "false");
    // Cell cost is context, not a footprint collision test or TEB's pruned local goal.
    std::string goal_cost = "unavailable";
    if (!active_plan_.poses.empty() && active_plan_.header.frame_id == costmap_ros_->getGlobalFrameID())
    {
      auto* map = costmap_ros_->getCostmap();
      std::unique_lock<costmap_2d::Costmap2D::mutex_t> map_lock(*(map->getMutex()));
      unsigned int x, y;
      const auto& goal = active_plan_.poses.back().pose.position;
      goal_cost = map->worldToMap(goal.x, goal.y, x, y) ? std::to_string(map->getCost(x,y)) : "outside_map";
    }
    add("reference_goal_cell_cost", goal_cost);
    array.status.push_back(status);
    diagnostics_publisher_.publish(array);
  }

  // Geometric trigger: includes the robot body and configured clearance, not
  // just a distance from the sampled centerline points.
  bool committedReferenceBlocked()
  {
    detection_status_="NO_PATH";
    if (!has_latest_placenav_path_) return false;
    geometry_msgs::PoseStamped pose;
    if (!costmap_ros_->getRobotPose(pose)) {
      detection_status_="ROBOT_TF_UNAVAILABLE";return true;
    }
    if (latest_placenav_path_.header.frame_id != costmap_ros_->getGlobalFrameID()) {
      detection_status_="PATH_FRAME_MISMATCH";return true;
    }
    placenav_teb_bridge::RetainedPath reference;
    if (!reference.assign(latest_placenav_path_)) {detection_status_="INVALID_REFERENCE";return true;}
    // The trigger checks the whole input, even across reversing cusps.
    reference.poses.clear();
    for (const auto& p:latest_placenav_path_.poses)
      reference.poses.push_back({p.pose.position.x,p.pose.position.y,tf2::getYaw(p.pose.orientation)});
    const placenav_teb_bridge::PathPose robot{pose.pose.position.x,pose.pose.position.y,tf2::getYaw(pose.pose.orientation)};
    // Prune already passed segments; a lateral mismatch is checked by the
    // swept connection from the current pose, not mistaken for an obstacle.
    reference.advance(robot,std::numeric_limits<double>::infinity());
    auto footprint=costmap_ros_->getRobotFootprint();double clearance=.2;
    private_nh_.param("TebLocalPlannerROS/min_obstacle_dist",clearance,.2);
    if (!std::isfinite(clearance)||clearance<0.) {detection_status_="INVALID_CLEARANCE";return true;}
    costmap_2d::padFootprint(footprint,clearance);
    auto* map=costmap_ros_->getCostmap();
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(map->getMutex()));
    const bool blocked=!reference.safe(*map,footprint,robot);
    detection_status_=blocked?"REFERENCE_FOOTPRINT_BLOCKED":"CLEAR";
    return blocked;
  }

  void clearRetained()
  {
    retained_.clear(); retained_tail_=nav_msgs::Path(); awaiting_retained_candidate_ = false;
    has_latest_teb_local_path_ = false; last_retained_command_ = geometry_msgs::Twist();
    nav_msgs::Path empty; empty.header.frame_id=costmap_ros_->getGlobalFrameID();
    empty.header.stamp=ros::Time::now(); retained_path_publisher_.publish(empty);
  }

  bool adoptRetained(const nav_msgs::Path& path, const placenav_teb_bridge::PathPose& robot,
                     costmap_2d::Costmap2D& map, const std::vector<geometry_msgs::Point>& footprint)
  {
    if (!retained_.assign(path) || !retained_.advance(robot,retention_max_deviation_) ||
        !retained_.safe(map,footprint,robot)) {retained_.clear();return false;}
    retained_tail_=nav_msgs::Path();
    if (reference_mode_ == "avoidance_commit" && state_ == NavigationState::AVOIDING &&
        retained_.poses.size()<path.poses.size()) {
      retained_tail_=path;
      retained_tail_.poses.erase(retained_tail_.poses.begin(),retained_tail_.poses.begin()+retained_.poses.size()-1);
      // Check all remaining gear segments before accepting the complete route.
      auto full=retained_;
      for (std::size_t i=retained_.poses.size();i<path.poses.size();++i) {
        const auto& p=path.poses[i].pose;
        full.poses.push_back({p.position.x,p.position.y,tf2::getYaw(p.orientation)});
      }
      if (!full.safe(map,footprint,robot)) {retained_.clear();retained_tail_=nav_msgs::Path();return false;}
    }
    retained_at_=ros::WallTime::now();retained_goal_=requested_goal_;
    nav_msgs::Path published=path;
    // In commit mode RViz shows the entire committed route, including later gears.
    if (reference_mode_ != "avoidance_commit") published.poses.resize(retained_.poses.size());
    retained_path_publisher_.publish(published);return true;
  }

  bool retainedRouteSafe(costmap_2d::Costmap2D& map,const std::vector<geometry_msgs::Point>& footprint,
                         const placenav_teb_bridge::PathPose& robot) const
  {
    auto full=retained_;
    for (std::size_t i=1;i<retained_tail_.poses.size();++i) {
      const auto& p=retained_tail_.poses[i].pose;
      full.poses.push_back({p.position.x,p.position.y,tf2::getYaw(p.orientation)});
    }
    return full.safe(map,footprint,robot);
  }

  void retainedControl()
  {
    using placenav_teb_bridge::PathPose;
    geometry_msgs::Twist command;
    const auto now=ros::Time::now(); const auto wall=ros::WallTime::now();
    const double dt=last_retention_tick_.isZero()?0.:std::min(.25,std::max(0.,(wall-last_retention_tick_).toSec()));
    last_retention_tick_=wall;
    geometry_msgs::PoseStamped pose;
    std::string reason="COMMAND_VALID";
    int teb_outcome=-1;
    if (!last_retention_ros_.isZero() && now<last_retention_ros_) {
      invalidatePlan("CLOCK_RESET"); retention_status_="CLOCK_RESET";last_retention_ros_=now;return;
    }
    last_retention_ros_=now;
    if (!has_latest_placenav_path_) reason=stop_reason_;
    else if (latest_placenav_path_.header.frame_id!=costmap_ros_->getGlobalFrameID()) reason="RETENTION_INPUT_FRAME_MISMATCH";
    else if (!costmap_ros_->getRobotPose(pose)) reason="ROBOT_TF_UNAVAILABLE";
    if (reason!="COMMAND_VALID") {
      clearRetained(); retention_status_=reason;cmd_vel_publisher_.publish(command);
      publishDiagnostics(reason,-1,"",command); return;
    }
    const PathPose robot{pose.pose.position.x,pose.pose.position.y,tf2::getYaw(pose.pose.orientation)};
    auto* map=costmap_ros_->getCostmap();
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> map_lock(*(map->getMutex()));
    auto footprint=costmap_ros_->getRobotFootprint();
    double clearance=.2;
    private_nh_.param("TebLocalPlannerROS/min_obstacle_dist",clearance,.2);
    if (!std::isfinite(clearance)||clearance<0.) {
      map_lock.unlock();clearRetained();retention_status_="INVALID_CLEARANCE";
      cmd_vel_publisher_.publish(command);publishDiagnostics("INVALID_CLEARANCE",-1,"",command);return;
    }
    costmap_2d::padFootprint(footprint,clearance);
    if (!retained_.empty()) {
      const auto& incoming=latest_placenav_path_.poses.back().pose;
      const bool goal_changed=std::hypot(incoming.position.x-retained_goal_.x,incoming.position.y-retained_goal_.y)>retention_goal_shift_ ||
        std::abs(placenav_teb_bridge::angle(tf2::getYaw(incoming.orientation)-retained_goal_.yaw))>.78;
      const bool policy_release=reference_mode_ != "avoidance_commit" &&
        (goal_changed || (wall-retained_at_).toSec()>retention_max_duration_);
      if (policy_release ||
          !retained_.advance(robot,retention_max_deviation_) || !retainedRouteSafe(*map,footprint,robot)) {
        retention_status_=policy_release?"GOAL_CHANGED_OR_EXPIRED":"DEVIATED_OR_BLOCKED";clearRetained();
      }
    }
    if (retained_.empty() && awaiting_retained_candidate_ && has_latest_teb_local_path_ &&
        latest_teb_local_path_.header.stamp>=retention_request_ &&
        latest_teb_local_path_.header.stamp<=now &&
        (now-latest_teb_local_path_.header.stamp).toSec()<.5 &&
        latest_teb_local_path_.header.frame_id==costmap_ros_->getGlobalFrameID()) {
      const auto& incoming=latest_placenav_path_.poses.back().pose;
      const bool request_current=(reference_mode_ == "avoidance_commit" && state_ == NavigationState::AVOIDING) || (std::hypot(incoming.position.x-requested_goal_.x,incoming.position.y-requested_goal_.y)<=retention_goal_shift_ &&
        std::abs(placenav_teb_bridge::angle(tf2::getYaw(incoming.orientation)-requested_goal_.yaw))<=.78);
      if (request_current && adoptRetained(latest_teb_local_path_,robot,*map,footprint))
        retention_status_="ADOPTED";
      else {retained_.clear();retention_status_="CANDIDATE_UNSAFE";}
      awaiting_retained_candidate_=false;
    }
    if (!retained_.empty()) {
      double vx=.2,back=.1,w=.4,ax=.3,aw=.4;
      private_nh_.getParam("TebLocalPlannerROS/max_vel_x",vx);
      private_nh_.getParam("TebLocalPlannerROS/max_vel_x_backwards",back);
      private_nh_.getParam("TebLocalPlannerROS/max_vel_theta",w);
      private_nh_.getParam("TebLocalPlannerROS/acc_lim_x",ax);
      private_nh_.getParam("TebLocalPlannerROS/acc_lim_theta",aw);
      bool valid=true;for(double v:{vx,back,w,ax,aw})valid=valid&&std::isfinite(v)&&v>0.;
      command=valid?retained_.command(robot,retention_lookahead_,retained_.gear>0?vx:back,w,.12):geometry_msgs::Twist();
      if (!valid || std::abs(command.linear.x)+std::abs(command.angular.z)<1e-6) {
        const bool reached=std::hypot(robot.x-retained_.poses.back().x,robot.y-retained_.poses.back().y)<=.12;
        if (valid && reference_mode_ == "avoidance_commit" && state_ == NavigationState::AVOIDING && reached) {
          if (!retained_tail_.poses.empty()) {
            const auto next=retained_tail_;clearRetained();
            if (adoptRetained(next,robot,*map,footprint)) {
              retention_status_="GEAR_CHANGE";
              map_lock.unlock();cmd_vel_publisher_.publish(command);
              publishDiagnostics("RETENTION_GEAR_CHANGE",-1,retention_status_,command);return;
            }
          } else {state_=NavigationState::RECOVERING;publishState();}
        }
        if (valid && reference_mode_ == "avoidance_commit" && state_ == NavigationState::RECOVERING && reached) {
          clearRetained();state_=NavigationState::NORMAL;publishState();
          setActivePlan(latest_placenav_path_,"latest PlaceNav at recovery goal");
          retention_status_="RECOVERY_GOAL_REACHED";
          map_lock.unlock();cmd_vel_publisher_.publish(command);
          publishDiagnostics("GOAL_REACHED",0,retention_status_,command);return;
        }
        clearRetained();retention_status_=reached?"ENDPOINT_REACHED":"UNTRACKABLE";
      } else {
        command.linear.x=std::max(last_retained_command_.linear.x-ax*dt,std::min(last_retained_command_.linear.x+ax*dt,command.linear.x));
        command.angular.z=std::max(last_retained_command_.angular.z-aw*dt,std::min(last_retained_command_.angular.z+aw*dt,command.angular.z));
        command.linear.x=std::max(-back,std::min(vx,command.linear.x));
        command.angular.z=std::max(-w,std::min(w,command.angular.z));
        // The tracking arc can cut a corner: check the actual command as well as the retained path.
        bool arc_safe=true;
        const double horizon=std::max(.5,std::abs(command.linear.x)/ax);
        double radius=0.;for(const auto& p:footprint)radius=std::max(radius,std::hypot(p.x,p.y));
        const int steps=std::max(1,int(std::ceil(horizon*(std::abs(command.linear.x)+radius*std::abs(command.angular.z))/(map->getResolution()*.5))));
        for(int i=0;i<=steps && arc_safe;++i) {
          const double t=horizon*i/steps,turn=command.angular.z*t;
          const double dx=std::abs(command.angular.z)>1e-6?command.linear.x*std::sin(turn)/command.angular.z:command.linear.x*t;
          const double dy=std::abs(command.angular.z)>1e-6?command.linear.x*(1-std::cos(turn))/command.angular.z:0.;
          arc_safe=placenav_teb_bridge::footprintSafe(*map,footprint,{robot.x+std::cos(robot.yaw)*dx-std::sin(robot.yaw)*dy,robot.y+std::sin(robot.yaw)*dx+std::cos(robot.yaw)*dy,robot.yaw+turn});
        }
        if (!arc_safe) {clearRetained();command=geometry_msgs::Twist();retention_status_="TRACKING_ARC_BLOCKED";}
        else retention_status_="FOLLOWING";
      }
    }
    map_lock.unlock();
    if (retained_.empty()) {
      command=geometry_msgs::Twist();
      const auto& reference=reference_mode_ == "avoidance_commit" && state_ == NavigationState::AVOIDING
        ? avoidance_reference_ : latest_placenav_path_;
      setActivePlan(reference,state_ == NavigationState::RECOVERING?"recovery latest PlaceNav":"retention replan");
      geometry_msgs::TwistStamped velocity,result;std::string detail;
      const auto& goal=reference.poses.back().pose;
      requested_goal_={goal.position.x,goal.position.y,tf2::getYaw(goal.orientation)};
      retention_request_=now;awaiting_retained_candidate_=true;has_latest_teb_local_path_=false;
      teb_outcome=teb_planner_.computeVelocityCommands(pose,velocity,result,detail);
      if(teb_outcome!=0) {awaiting_retained_candidate_=false;retention_status_="TEB_REPLAN_FAILED";}
      reason="RETENTION_WAITING_FOR_SAFE_PLAN";
    }
    // A collision-checked connection to the newest reference is now available.
    // Resume the normal latest-reference planner after this first tracking tick.
    if (reference_mode_ == "avoidance_commit" && state_ == NavigationState::RECOVERING &&
        !retained_.empty() && retention_status_ == "FOLLOWING") {
      clearRetained();state_=NavigationState::NORMAL;publishState();
      setActivePlan(latest_placenav_path_,"latest PlaceNav after recovery");
      retention_status_="RECOVERY_CONNECTION_VALID";
    }
    last_retained_command_=command;
    if (pathExpired()) {invalidatePlan("INPUT_TIMEOUT");command=geometry_msgs::Twist();reason="INPUT_TIMEOUT";}
    cmd_vel_publisher_.publish(command);publishDiagnostics(reason,teb_outcome,retention_status_,command);
  }

  void controlCallback(const ros::TimerEvent&)
  {
    std::lock_guard<std::mutex> lock(planner_mutex_);
    geometry_msgs::Twist command;
    if (pathExpired()) invalidatePlan("INPUT_TIMEOUT");
    bool blocked=false;
    if (reference_mode_ == "avoidance_commit") {
      if (state_ == NavigationState::NORMAL) {
        const bool requires_avoidance=committedReferenceBlocked();
        if (detection_status_ != "CLEAR" && detection_status_ != "REFERENCE_FOOTPRINT_BLOCKED" &&
            detection_status_ != "NO_PATH") {
          // Missing sensing/TF is a stop condition, not evidence of an obstacle.
          publishStop();publishDiagnostics(detection_status_,-1,"",command);return;
        }
        if (requires_avoidance) {
          avoidance_reference_=latest_placenav_path_;
          clearRetained();state_=NavigationState::AVOIDING;
          retention_status_="AVOIDANCE_START";publishState();
        }
      }
      if (state_ != NavigationState::NORMAL) {retainedControl();return;}
    } else {
      if (retain_safe_path_) {retainedControl();return;}
      blocked=obstacleBlocksLatestPlaceNavPath();
    }
    if (reference_mode_ == "state_machine" &&
        (detection_status_ == "CLEAR" || detection_status_ == "BLOCKED"))
      updateNavigationState(blocked);
    std::string reason = stop_reason_, detail;
    int outcome = -1;
    if (has_plan_)
    {
      geometry_msgs::PoseStamped robot;
      if (!costmap_ros_->getRobotPose(robot)) reason = "ROBOT_TF_UNAVAILABLE";
      else
      {
        geometry_msgs::TwistStamped velocity, result;
        // This is the same public overload called by TEB's bool wrapper.
        outcome = teb_planner_.computeVelocityCommands(robot, velocity, result, detail);
        command = result.twist;
        if (teb_planner_.isGoalReached())
        {
          reason = "GOAL_REACHED";
          has_plan_ = false;
          if (reference_mode_ == "state_machine" && state_ == NavigationState::AVOIDING && has_latest_placenav_path_)
            transitionToRecovering();
          stop_reason_ = reason;
        }
        else if (outcome != 0) reason = "TEB_FAILED";
        else if (!std::isfinite(command.linear.x) || !std::isfinite(command.linear.y) || !std::isfinite(command.angular.z))
          reason = "NONFINITE_COMMAND";
        else reason = std::abs(command.linear.x)+std::abs(command.linear.y)+std::abs(command.angular.z) < 1e-6 ? "TEB_ZERO_COMMAND" : "COMMAND_VALID";
      }
    }
    if (pathExpired()) { invalidatePlan("INPUT_TIMEOUT"); reason = "INPUT_TIMEOUT"; }
    if (reason != "COMMAND_VALID")
    {
      command = geometry_msgs::Twist();
      ROS_WARN_THROTTLE(1., "Bridge stop: %s; TEB: %s", reason.c_str(), detail.c_str());
    }
    cmd_vel_publisher_.publish(command);
    publishDiagnostics(reason, outcome, detail, command);
  }

  void publishState()
  {
    std_msgs::String message;
    message.data = reference_mode_ == "latest" ? "TRACKING" : stateName(state_);
    state_publisher_.publish(message);
  }

  void publishStop()
  {
    if (cmd_vel_publisher_)
    {
      cmd_vel_publisher_.publish(geometry_msgs::Twist());
    }
  }

  bool retain_safe_path_{false},awaiting_retained_candidate_{false};
  double retention_max_duration_{3.},retention_max_deviation_{.2},retention_lookahead_{.3},retention_goal_shift_{.75};
  placenav_teb_bridge::RetainedPath retained_;
  placenav_teb_bridge::PathPose retained_goal_{},requested_goal_{};
  ros::WallTime retained_at_,last_retention_tick_;
  ros::Time retention_request_,last_retention_ros_;
  geometry_msgs::Twist last_retained_command_;
  std::string retention_status_{"DISABLED"};
  ros::Publisher retained_path_publisher_;
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
  ros::Publisher diagnostics_publisher_;
  std::string reference_mode_{"state_machine"};
  std::string active_source_{"none"}, stop_reason_{"NO_INPUT_PATH"};
  std::string detection_status_{"NO_PATH"};
  std::size_t cost_cells_{0}, range_cells_{0}, corridor_cells_{0};
  ros::WallTime latest_path_wall_;
  double path_timeout_{0.};
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
  nav_msgs::Path retained_tail_;
  nav_msgs::Path avoidance_reference_;
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
