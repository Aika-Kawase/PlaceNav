#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include <atomic>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "avoidance_guidance.h"
#include "rolling_planner.h"

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
    private_nh_.param("avoidance_use_placenav_goal", avoidance_use_placenav_goal_, true);
    private_nh_.param("rolling_replan_enabled", rolling_enabled_, true);
    private_nh_.param("input_timeout", input_timeout_, 1.0);
    private_nh_.param("control_watchdog_timeout", watchdog_timeout_, 0.6);
    private_nh_.param("replan_interval", replan_interval_, 0.5);
    private_nh_.param("renewal_distance", renewal_distance_, 0.8);
    private_nh_.param("renewal_latency", renewal_latency_, 1.0);
    private_nh_.param("search_clearance", search_clearance_, 0.25);
    private_nh_.param("recovery_max_connection", recovery_max_connection_, 0.6);
    private_nh_.param("direction_rate_limit", direction_rate_limit_, 0.8);
    private_nh_.param("TebLocalPlannerROS/acc_lim_x", deceleration_, 0.3);
    private_nh_.param("search_horizon", rolling_planner_.horizon, 2.0);
    private_nh_.param("search_budget", rolling_planner_.budget_seconds, 0.12);
    private_nh_.param("search_max_expansions", rolling_planner_.max_expansions, 12000);
    private_nh_.param("direction_weight", rolling_planner_.direction_weight, 0.3);
    for(double v : {input_timeout_,watchdog_timeout_,replan_interval_,renewal_distance_,renewal_latency_,
                    recovery_max_connection_,direction_rate_limit_,deceleration_,rolling_planner_.horizon,rolling_planner_.budget_seconds})
      if(!std::isfinite(v)||v<=0) throw std::runtime_error("Rolling planner time/distance parameters must be finite and positive");
    if(!std::isfinite(search_clearance_)||search_clearance_<0 ||
       !std::isfinite(rolling_planner_.direction_weight)||rolling_planner_.direction_weight<0 ||
       rolling_planner_.max_expansions<1) throw std::runtime_error("Invalid rolling planner clearance/cost/budget");
    if(rolling_enabled_) {
      // Verify the complete optimized trajectory, not just its first five poses.
      private_nh_.setParam("TebLocalPlannerROS/feasibility_check_no_poses", -1);
      private_nh_.setParam("TebLocalPlannerROS/feasibility_check_lookahead_distance", -1.0);
    }

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
    reason_publisher_ = private_nh_.advertise<std_msgs::String>("control_status", 1, true);
    std::string cloud_topic,odom_topic,goal_topic,stop_topic;
    private_nh_.param<std::string>("local_costmap/stvl/depth_cloud/topic",cloud_topic,"/c2/depth/points");
    private_nh_.param<std::string>("TebLocalPlannerROS/odom_topic",odom_topic,"/odom");
    private_nh_.param<std::string>("final_goal_topic",goal_topic,"/toponav/reached_goal");
    private_nh_.param<std::string>("stop_topic",stop_topic,"/toponav/stop");
    cloud_subscriber_=nh_.subscribe<sensor_msgs::PointCloud2>(cloud_topic,1,[this](const sensor_msgs::PointCloud2::ConstPtr& m){
      if(m->header.frame_id.empty() || m->data.empty()) return;
      std::lock_guard<std::mutex> sensor_lock(sensor_mutex_);
      cloud_frame_=m->header.frame_id;
      cloud_stamp_=m->header.stamp.toSec(); cloud_wall_=ros::WallTime::now().toSec();
    });
    odom_subscriber_=nh_.subscribe<nav_msgs::Odometry>(odom_topic,1,[this](const nav_msgs::Odometry::ConstPtr& m){
      if(!std::isfinite(m->twist.twist.linear.x)||!std::isfinite(m->twist.twist.angular.z))return;
      speed_=m->twist.twist.linear.x; odom_stamp_=m->header.stamp.toSec(); odom_wall_=ros::WallTime::now().toSec();
    });
    goal_subscriber_=nh_.subscribe<std_msgs::Bool>(goal_topic,1,[this](const std_msgs::Bool::ConstPtr& m){
      if(m->data) { final_goal_=true; emitStop("FINAL_GOAL_REACHED"); }
    });
    stop_subscriber_=nh_.subscribe<std_msgs::Bool>(stop_topic,1,[this](const std_msgs::Bool::ConstPtr& m){
      stop_requested_=m->data; if(m->data)emitStop("STOP_REQUESTED");
    });
    reset_service_=private_nh_.advertiseService("reset_navigation",&PlaceNavTebBridge::resetNavigation,this);
    last_control_wall_=ros::WallTime::now().toSec();
    watchdog_=nh_.createWallTimer(ros::WallDuration(.1),[this](const ros::WallTimerEvent&){
      if(rolling_enabled_ && ros::WallTime::now().toSec()-last_control_wall_.load()>watchdog_timeout_)
        emitStop(final_goal_ ? "FINAL_GOAL_REACHED" : "CLOCK_OR_CONTROL_STALLED");
    });
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
    path_wall_=ros::WallTime::now().toSec();
    if(rolling_enabled_ || final_goal_) return;

    // Keep receiving the newest PlaceNav path during avoidance, but do not let
    // it overwrite the promoted TEB avoidance path.
    if (state_ == NavigationState::AVOIDING)
    {
      refreshAvoidanceGuidance();
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
    for(const auto& p:path.poses) {
      const auto& q=p.pose.orientation;
      double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
      if(!std::isfinite(p.pose.position.x)||!std::isfinite(p.pose.position.y)||
         !std::isfinite(norm)||norm<.9||norm>1.1) return false;
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
    for (std::size_t i = 0; i < path.poses.size(); ++i)
    {
      const auto& a = path.poses[i].pose.position;
      const auto& b = path.poses[std::min(i + 1, path.poses.size() - 1)].pose.position;
      const double sx = b.x - a.x;
      const double sy = b.y - a.y;
      const double len2 = sx * sx + sy * sy;
      const double t = len2 > 1e-12 ?
        std::max(0.0, std::min(1.0, ((x - a.x) * sx + (y - a.y) * sy) / len2)) : 0.0;
      const double dx = x - (a.x + t * sx);
      const double dy = y - (a.y + t * sy);
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
    avoidance_seed_ = latest_teb_local_path_;
    refreshAvoidanceGuidance();
    state_ = NavigationState::AVOIDING;
    state_entered_at_ = ros::Time::now();
    last_obstacle_at_ = state_entered_at_;
    publishState();
    ROS_WARN("Navigation state: NORMAL/RECOVERING -> AVOIDING");
  }

  void refreshAvoidanceGuidance()
  {
    if (!avoidance_use_placenav_goal_ || !has_latest_placenav_path_ ||
        avoidance_seed_.header.frame_id != latest_placenav_path_.header.frame_id ||
        avoidance_seed_.header.frame_id != costmap_ros_->getGlobalFrameID()) return;
    geometry_msgs::PoseStamped robot;
    if (!costmap_ros_->getRobotPose(robot)) return;
    // Refuse stale observations; this does not extend the lifetime of old inputs.
    const double age = (ros::Time::now()-latest_placenav_path_.header.stamp).toSec();
    if (age < -0.1 || age > 1.0) return;
    std::vector<avoidance_guidance::Point> seed, route;
    for (const auto& p : avoidance_seed_.poses)
      seed.push_back({p.pose.position.x, p.pose.position.y});
    for (const auto& p : latest_placenav_path_.poses)
      route.push_back({p.pose.position.x, p.pose.position.y});
    if (route.empty()) return;
    const double yaw=tf2::getYaw(robot.pose.orientation);
    const double gx=route.back().x-robot.pose.position.x;
    const double gy=route.back().y-robot.pose.position.y;
    if (gx*std::cos(yaw)+gy*std::sin(yaw) < 0.3) return;
    const auto points=avoidance_guidance::blend(seed, route,
      {robot.pose.position.x, robot.pose.position.y}, 1.2, 2.0);
    if (points.empty()) return;
    nav_msgs::Path guidance;
    guidance.header=latest_placenav_path_.header;
    for (std::size_t i=0; i<points.size(); ++i) {
      geometry_msgs::PoseStamped pose;
      pose.header=guidance.header;
      pose.pose.position.x=points[i].x;
      pose.pose.position.y=points[i].y;
      const auto a=points[i ? i-1 : 0];
      const auto b=points[i+1<points.size() ? i+1 : i];
      const double heading=std::atan2(b.y-a.y,b.x-a.x);
      pose.pose.orientation.z=std::sin(heading/2);
      pose.pose.orientation.w=std::cos(heading/2);
      guidance.poses.push_back(pose);
    }
    setActivePlan(guidance, "TEB avoidance prefix + latest PlaceNav forward goal");
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
    if(rolling_enabled_) { rollingControl(); return; }
    if(final_goal_ || stop_requested_) { emitStop(final_goal_ ? "FINAL_GOAL_REACHED" : "STOP_REQUESTED"); return; }
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
          if (state_ == NavigationState::AVOIDING)
          {
            // Recovery is decided only by updateNavigationState after the
            // obstacle-clear interval, not by reaching a short local endpoint.
            // Keep replanning the held avoidance seed; never reuse old velocity.
            ROS_WARN_THROTTLE(1.0, "Avoidance endpoint reached; holding until obstacle-clear recovery");
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

  void report(const std::string& reason)
  {
    // Caller holds output_mutex_; never takes planner_mutex_ from the watchdog.
    if(reason!=last_reason_) {
      std_msgs::String m; m.data=reason; reason_publisher_.publish(m);
      if(reason=="RUNNING" || reason=="RUNNING_VALID_PREVIOUS_PATH" || reason=="FINAL_GOAL_REACHED") ROS_INFO_STREAM("Control: "<<reason);
      else ROS_WARN_STREAM("Control: "<<reason);
      last_reason_=reason;
    }
  }

  void emitStop(const std::string& reason)
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    cmd_vel_publisher_.publish(geometry_msgs::Twist()); report(reason);
  }

  bool resetNavigation(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
  {
    std::lock_guard<std::mutex> lock(planner_mutex_);
    final_goal_=false; has_plan_=false; has_active_plan_=false;
    has_latest_placenav_path_=false; direction_valid_=false;
    recovery_clear_since_=ros::Time(); last_replan_=ros::Time();
    active_plan_=nav_msgs::Path(); state_=NavigationState::NORMAL;
    active_path_publisher_.publish(active_plan_); publishState();
    emitStop("WAITING_FOR_NEW_PATH");
    response.success=true;response.message="Reset; a new path is required. Stop request is not cleared.";
    return true;
  }

  bool fresh(double stamp,double wall,double now) const
  {
    return stamp>0 && now-stamp>=-.1 && now-stamp<=input_timeout_ &&
           ros::WallTime::now().toSec()-wall<=input_timeout_;
  }

  rolling::Grid snapshot()
  {
    auto* map=costmap_ros_->getCostmap();
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(map->getMutex()));
    rolling::Grid g;
    g.width=map->getSizeInCellsX();g.height=map->getSizeInCellsY();g.resolution=map->getResolution();
    g.ox=map->getOriginX();g.oy=map->getOriginY();
    g.data.assign(map->getCharMap(),map->getCharMap()+g.width*g.height);
    return g;
  }

  static std::vector<rolling::Pose> points(const nav_msgs::Path& path)
  {
    std::vector<rolling::Pose> p;
    for(const auto& v:path.poses)p.push_back({v.pose.position.x,v.pose.position.y,tf2::getYaw(v.pose.orientation)});
    return p;
  }

  nav_msgs::Path message(const std::vector<rolling::Pose>& points) const
  {
    nav_msgs::Path path;path.header.frame_id=costmap_ros_->getGlobalFrameID();path.header.stamp=ros::Time::now();
    for(auto p:points) {
      geometry_msgs::PoseStamped v;v.header=path.header;v.pose.position.x=p.x;v.pose.position.y=p.y;
      v.pose.orientation.z=std::sin(p.yaw/2);v.pose.orientation.w=std::cos(p.yaw/2);path.poses.push_back(v);
    }
    return path;
  }

  void changeState(NavigationState next)
  {
    if(state_==next)return;
    state_=next;state_entered_at_=ros::Time::now();publishState();
    ROS_INFO_STREAM("Navigation state: "<<stateName(state_));
  }

  bool commandSafe(const geometry_msgs::Twist& cmd,rolling::Pose robot,const rolling::Collision& check) const
  {
    if(!std::isfinite(cmd.linear.x)||!std::isfinite(cmd.angular.z)||!std::isfinite(cmd.linear.y))return false;
    // Independent short-horizon swept-footprint check including watchdog latency.
    if(std::abs(cmd.linear.y)>1e-6)return false;
    double duration=watchdog_timeout_+std::abs(cmd.linear.x)/deceleration_;
    const double dt=.025;
    if(!check.free(robot))return false;
    for(double t=0;t<duration;t+=dt) {
      rolling::Pose next=robot;
      next.x+=cmd.linear.x*std::cos(robot.yaw+cmd.angular.z*dt/2)*dt;
      next.y+=cmd.linear.x*std::sin(robot.yaw+cmd.angular.z*dt/2)*dt;
      next.yaw+=cmd.angular.z*dt;
      if(!check.edge(robot,next))return false;robot=next;
    }
    return true;
  }

  void rollingControl()
  {
    // Skip overlapping timer callbacks; wall watchdog remains independent.
    std::unique_lock<std::mutex> lock(planner_mutex_,std::try_to_lock);
    if(!lock.owns_lock())return;
    const double started=ros::WallTime::now().toSec();
    const ros::Time now=ros::Time::now();
    auto stop=[&](const std::string& reason){last_control_wall_=ros::WallTime::now().toSec();emitStop(reason);};
    if(final_goal_) {stop("FINAL_GOAL_REACHED");return;}
    if(stop_requested_) {stop("STOP_REQUESTED");return;}
    if(!last_cycle_.isZero() && now<last_cycle_) {
      has_plan_=false;direction_valid_=false;has_latest_placenav_path_=false;
      recovery_clear_since_=ros::Time();last_replan_=ros::Time();
      last_cycle_=now;stop("CLOCK_RESET");return;
    }
    double dt=last_cycle_.isZero() ? 1./controller_frequency_ : std::min(1.,(now-last_cycle_).toSec());last_cycle_=now;
    if(!has_latest_placenav_path_ || !fresh(latest_placenav_path_.header.stamp.toSec(),path_wall_,now.toSec())) {
      recovery_clear_since_=ros::Time();stop("INPUT_STALE_PATH");return;
    }
    if(!fresh(cloud_stamp_,cloud_wall_,now.toSec())) {recovery_clear_since_=ros::Time();stop("INPUT_STALE_CLOUD");return;}
    if(!fresh(odom_stamp_,odom_wall_,now.toSec())) {recovery_clear_since_=ros::Time();stop("INPUT_STALE_ODOM");return;}
    std::string cloud_frame;double cloud_time;
    {std::lock_guard<std::mutex> sensor_lock(sensor_mutex_);cloud_frame=cloud_frame_;cloud_time=cloud_stamp_;}
    if(!tf_buffer_.canTransform(costmap_ros_->getGlobalFrameID(),cloud_frame,ros::Time(cloud_time),ros::Duration(.02))) {
      recovery_clear_since_=ros::Time();stop("TF_CLOUD_UNAVAILABLE");return;
    }
    if(latest_placenav_path_.header.frame_id!=costmap_ros_->getGlobalFrameID()) {stop("PATH_FRAME_MISMATCH");return;}
    geometry_msgs::PoseStamped pose;
    if(!costmap_ros_->getRobotPose(pose)) {recovery_clear_since_=ros::Time();stop("TF_UNAVAILABLE");return;}
    if(!costmap_ros_->isCurrent()) {recovery_clear_since_=ros::Time();stop("COSTMAP_STALE");return;}
    rolling::Pose robot{pose.pose.position.x,pose.pose.position.y,tf2::getYaw(pose.pose.orientation)};
    auto grid=snapshot();std::vector<rolling::Pose> footprint;
    for(auto p:costmap_ros_->getRobotFootprint())footprint.push_back({p.x,p.y,0});
    rolling::Collision check(grid,footprint,search_clearance_);
    if(!check.free(robot)) {recovery_clear_since_=ros::Time();stop("FOOTPRINT_BLOCKED");return;}
    auto raw=points(latest_placenav_path_);
    auto route=rolling::remaining(robot,raw);
    double nearest=std::numeric_limits<double>::infinity();
    for(auto p:raw)nearest=std::min(nearest,rolling::distance(robot,p));
    const double renew=rolling::renewalDistance(speed_,renewal_distance_,renewal_latency_,deceleration_);
    bool route_safe=nearest<=recovery_max_connection_ && check.path(route) && rolling::length(route)>renew;
    double gx=raw.back().x-robot.x,gy=raw.back().y-robot.y;
    bool direction_available=std::hypot(gx,gy)>.3;
    if(direction_available) {
      double desired=std::atan2(gy,gx);
      // Do not extend an input that now points behind the vehicle.
      direction_available=std::abs(rolling::angle(desired-robot.yaw))<rolling::pi*.55;
      if(direction_available) {
        if(!direction_valid_){direction_=desired;direction_valid_=true;}
        else direction_+=std::max(-direction_rate_limit_*dt,std::min(direction_rate_limit_*dt,rolling::angle(desired-direction_)));
      }
    }
    auto old=has_active_plan_ ? rolling::remaining(robot,points(active_plan_)) : std::vector<rolling::Pose>{};
    bool old_safe=has_plan_ && rolling::length(old)>.25 && check.path(old);
    bool near_end=!old_safe || rolling::length(old)<=renew;
    if(!route_safe) {recovery_clear_since_=ros::Time();changeState(NavigationState::AVOIDING);}
    bool recover=false;
    if(state_==NavigationState::AVOIDING && route_safe) {
      if(recovery_clear_since_.isZero() || rolling::distance(recovery_last_goal_,raw.back())>.4 ||
         std::abs(rolling::angle(recovery_last_goal_.yaw-raw.back().yaw))>.35) recovery_clear_since_=now;
      recovery_last_goal_=raw.back();
      recover=(now-recovery_clear_since_).toSec()>=avoidance_clear_time_;
    }
    if(state_==NavigationState::RECOVERING && route_safe &&
       (now-state_entered_at_).toSec()>=recovery_stable_time_)changeState(NavigationState::NORMAL);
    std::vector<rolling::Pose> candidate;
    if(route_safe && (state_!=NavigationState::AVOIDING || recover)) candidate=route;
    else if(direction_available && direction_valid_ &&
            (last_replan_.isZero() || (now-last_replan_).toSec()>=(near_end ? .2 : replan_interval_))) {
      last_replan_=now;
      double search_start=ros::WallTime::now().toSec();
      candidate=rolling_planner_.plan(grid,check,robot,direction_,old,renew);
      ROS_INFO_THROTTLE(2.0,"Rolling search: %zu poses, %.1f ms, remaining %.2f m, renew %.2f m",
        candidate.size(),1000*(ros::WallTime::now().toSec()-search_start),rolling::length(old),renew);
    }
    geometry_msgs::Twist command;
    bool valid=false,endpoint=false,accepted=false;
    auto compute=[&](const std::vector<rolling::Pose>& p)->bool {
      auto plan=message(p);
      if(!teb_planner_.setPlan(plan.poses))return false;
      bool ok=teb_planner_.computeVelocityCommands(command);
      endpoint=teb_planner_.isGoalReached();
      return ok && !endpoint && commandSafe(command,robot,check);
    };
    if(!candidate.empty() && check.path(candidate)) {
      valid=compute(candidate);accepted=valid;
    }
    if(!valid && old_safe) valid=compute(old);
    if(!valid) {
      has_plan_=false;
      if(endpoint || (has_active_plan_ && rolling::length(old)<=.25))stop("ENDPOINT_WAIT");
      else if(!direction_available && candidate.empty())stop("DIRECTION_UNAVAILABLE");
      else stop("PLANNING_FAILED");
      return;
    }
    // Recheck sensor ages, robot pose and current map after possibly expensive search/TEB.
    if(final_goal_ || stop_requested_) {stop(final_goal_ ? "FINAL_GOAL_REACHED" : "STOP_REQUESTED");return;}
    if(ros::WallTime::now().toSec()-started>watchdog_timeout_) {stop("CONTROL_DEADLINE_EXCEEDED");return;}
    double current=ros::Time::now().toSec();
    if(!fresh(cloud_stamp_,cloud_wall_,current)||!fresh(odom_stamp_,odom_wall_,current)||
       !fresh(latest_placenav_path_.header.stamp.toSec(),path_wall_,current)) {stop("INPUT_STALE_DURING_PLAN");return;}
    if(!costmap_ros_->getRobotPose(pose)||!costmap_ros_->isCurrent()){stop("TF_OR_COSTMAP_UNAVAILABLE");return;}
    auto final_grid=snapshot();rolling::Collision final_check(final_grid,footprint,search_clearance_);
    rolling::Pose final_robot{pose.pose.position.x,pose.pose.position.y,tf2::getYaw(pose.pose.orientation)};
    if(!final_check.path(rolling::remaining(final_robot,accepted ? candidate : old)) ||
       !commandSafe(command,final_robot,final_check)){stop("PATH_INVALIDATED");return;}
    if(accepted) {
      active_plan_=message(candidate);has_active_plan_=true;active_path_publisher_.publish(active_plan_);
      if(recover)changeState(NavigationState::RECOVERING);
    }
    has_plan_=true;
    std::lock_guard<std::mutex> output_lock(output_mutex_);
    last_control_wall_=ros::WallTime::now().toSec();
    if(final_goal_ || stop_requested_) {
      cmd_vel_publisher_.publish(geometry_msgs::Twist());report(final_goal_ ? "FINAL_GOAL_REACHED" : "STOP_REQUESTED");
    } else {cmd_vel_publisher_.publish(command);report(accepted ? "RUNNING" : "RUNNING_VALID_PREVIOUS_PATH");}
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
  nav_msgs::Path avoidance_seed_;
  bool avoidance_use_placenav_goal_{true};
  bool has_latest_placenav_path_{false};
  bool has_latest_teb_local_path_{false};
  bool has_active_plan_{false};
  bool has_plan_{false};
  bool rolling_enabled_{true},direction_valid_{false};
  double input_timeout_{1},watchdog_timeout_{.6},replan_interval_{.5},renewal_distance_{.8},renewal_latency_{1};
  double search_clearance_{.25},recovery_max_connection_{.6},direction_rate_limit_{.8},direction_{0},path_wall_{0};
  rolling::Planner rolling_planner_;
  rolling::Pose recovery_last_goal_;
  double deceleration_{.3};
  ros::Time last_replan_,last_cycle_,recovery_clear_since_;
  std::atomic<double> cloud_stamp_{0},cloud_wall_{0},odom_stamp_{0},odom_wall_{0},speed_{0},last_control_wall_{0};
  std::atomic<bool> final_goal_{false},stop_requested_{false};
  ros::Subscriber cloud_subscriber_,odom_subscriber_,goal_subscriber_,stop_subscriber_;
  ros::Publisher reason_publisher_;
  ros::ServiceServer reset_service_;
  ros::WallTimer watchdog_;
  std::mutex output_mutex_;
  std::mutex sensor_mutex_;
  std::string cloud_frame_;
  std::string last_reason_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "placenav_teb_bridge");
  try
  {
    PlaceNavTebBridge bridge;
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL_STREAM("Failed to start PlaceNav TEB bridge: " << error.what());
    return 1;
  }
  return 0;
}
