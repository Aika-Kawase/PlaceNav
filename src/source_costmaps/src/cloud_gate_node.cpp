#include <source_costmaps/cloud.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <array>
#include <cmath>
#include <map>
#include <stdexcept>

namespace sc = source_costmaps;

// One process per source: no synchronization, concatenation or shared cache.
class CloudGate {
  using Key = std::array<int64_t, 3>;
  struct Cell { std::array<double, 3> sum{{0, 0, 0}}; unsigned count = 0; };
  ros::NodeHandle nh_, pnh_{"~"};
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_{tf_};
  ros::Subscriber sub_;
  ros::Publisher pub_, diagnostics_;
  ros::WallTimer timer_, diagnostic_timer_;
  sensor_msgs::PointCloud2ConstPtr pending_;
  ros::WallTime pending_since_;
  ros::Time last_now_, newest_, previous_stamp_, last_output_;
  std::map<Key, unsigned> previous_;
  std::string input_, output_, fixed_, base_, expected_, override_, state_{"missing"}, error_;
  double max_age_, future_, tf_wait_, min_height_, max_height_, min_range_, max_range_, voxel_, confirmation_gap_;
  int confirmations_, min_points_;
  uint64_t received_ = 0, published_ = 0, stale_ = 0, invalid_ = 0, order_ = 0, tf_drops_ = 0, resets_ = 0;
  size_t retained_ = 0;

  double number(const std::string& name, double fallback, bool zero = false) {
    double value; pnh_.param(name, value, fallback);
    if (!std::isfinite(value) || value < 0 || (!zero && value == 0))
      throw std::invalid_argument("invalid parameter: " + name);
    return value;
  }
  void checkClock(const ros::Time& now) {
    if (!last_now_.isZero() && now < last_now_) {
      pending_.reset(); previous_.clear(); newest_ = previous_stamp_ = last_output_ = ros::Time();
      tf_.clear(); ++resets_; state_ = "clock_reset";
    }
    last_now_ = now;
  }
  bool fresh(const ros::Time& stamp, const ros::Time& now) const {
    const auto age = (now - stamp).toSec();
    return !stamp.isZero() && !now.isZero() && age >= -future_ && age <= max_age_;
  }
  void receive(const sensor_msgs::PointCloud2ConstPtr& msg) {
    const auto now = ros::Time::now(); checkClock(now); ++received_;
    if (msg->header.stamp.isZero() || msg->header.frame_id.empty() ||
        (!expected_.empty() && msg->header.frame_id != expected_)) {
      ++invalid_; state_ = "invalid_header_or_frame"; return;
    }
    if (!now.isZero() && !fresh(msg->header.stamp, now)) { ++stale_; state_ = "stale_or_future"; return; }
    if (!newest_.isZero() && msg->header.stamp <= newest_) { ++order_; state_ = "out_of_order"; return; }
    newest_ = msg->header.stamp; pending_ = msg; pending_since_ = ros::WallTime::now(); state_ = "pending_tf";
  }
  void tick(const ros::WallTimerEvent&) {
    const auto now = ros::Time::now(); checkClock(now);
    if (!pending_ || now.isZero()) return;
    if (!fresh(pending_->header.stamp, now)) { pending_.reset(); ++stale_; state_ = "stale_or_future"; return; }
    geometry_msgs::TransformStamped sensor_to_base, base_to_fixed;
    try {
      // Reinterpret only the explicitly expected C2 optical coordinates under
      // the separate corrected mounting frame. Never overwrite the original TF.
      const auto frame = override_.empty() ? pending_->header.frame_id : override_;
      sensor_to_base = tf_.lookupTransform(base_, frame, pending_->header.stamp, ros::Duration(0));
      base_to_fixed = tf_.lookupTransform(fixed_, base_, pending_->header.stamp, ros::Duration(0));
    } catch (const tf2::TransformException& e) {
      error_ = e.what();
      if ((ros::WallTime::now() - pending_since_).toSec() >= tf_wait_) {
        pending_.reset(); ++tf_drops_; state_ = "tf_unavailable";
      }
      return;
    }
    try {
      const auto all = sc::transformXYZ(*pending_, sensor_to_base);
      const auto& origin = sensor_to_base.transform.translation;
      sc::Points selected;
      for (const auto& p : all) {
        const double dx = p[0]-origin.x, dy = p[1]-origin.y, dz = p[2]-origin.z;
        const double r2 = dx*dx+dy*dy+dz*dz;
        if (p[2] >= min_height_ && p[2] <= max_height_ && r2 >= min_range_*min_range_ && r2 <= max_range_*max_range_)
          selected.push_back(p);
      }
      const auto global = sc::transformXYZ(sc::makeCloud(selected, pending_->header), base_to_fixed);
      // Current-frame centroids only. Historical frames confirm a voxel; their
      // XYZ and timestamps are never republished as new observations.
      std::map<Key, Cell> cells;
      for (const auto& p : global) {
        Key key{{int64_t(std::floor(p[0]/voxel_)), int64_t(std::floor(p[1]/voxel_)), int64_t(std::floor(p[2]/voxel_))}};
        auto& c = cells[key]; ++c.count;
        for (size_t j = 0; j < 3; ++j) c.sum[j] += p[j];
      }
      if (previous_stamp_.isZero() || (pending_->header.stamp-previous_stamp_).toSec() > confirmation_gap_)
        previous_.clear();
      std::map<Key, unsigned> next;
      sc::Points result;
      for (const auto& item : cells) {
        const auto& c = item.second;
        if (c.count < unsigned(min_points_)) continue;
        const auto old = previous_.find(item.first);
        const unsigned count = old == previous_.end() ? 1 : std::min(unsigned(confirmations_), old->second+1);
        next[item.first] = count;
        if (count >= unsigned(confirmations_))
          result.push_back({{float(c.sum[0]/c.count), float(c.sum[1]/c.count), float(c.sum[2]/c.count)}});
      }
      previous_ = std::move(next); previous_stamp_ = pending_->header.stamp;
      auto header = pending_->header; header.frame_id = fixed_;
      pub_.publish(sc::makeCloud(result, header));
      retained_ = result.size(); last_output_ = header.stamp; ++published_; state_ = "ok"; error_.clear();
    } catch (const std::exception& e) { ++invalid_; state_ = "invalid_cloud"; error_ = e.what(); }
    pending_.reset();
  }
  void diagnose(const ros::WallTimerEvent&) {
    diagnostic_msgs::DiagnosticArray msg; msg.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus s; s.name = ros::this_node::getName(); s.hardware_id = input_;
    const bool current = fresh(last_output_, msg.header.stamp);
    s.level = current && error_.empty() ? 0 : 1; s.message = current ? state_ : "missing_or_expired";
    const auto add = [&s](const std::string& k, const std::string& v) { diagnostic_msgs::KeyValue x; x.key=k; x.value=v; s.values.push_back(x); };
    add("received", std::to_string(received_)); add("published", std::to_string(published_));
    add("stale_dropped", std::to_string(stale_)); add("invalid_dropped", std::to_string(invalid_));
    add("out_of_order_dropped", std::to_string(order_)); add("tf_dropped", std::to_string(tf_drops_));
    add("clock_resets", std::to_string(resets_)); add("retained_points", std::to_string(retained_));
    add("last_stamp", std::to_string(last_output_.toSec())); add("error", error_);
    msg.status.push_back(s); diagnostics_.publish(msg);
  }
public:
  CloudGate() {
    pnh_.param<std::string>("input_topic", input_, ""); pnh_.param<std::string>("output_topic", output_, "");
    pnh_.param<std::string>("fixed_frame", fixed_, "odom"); pnh_.param<std::string>("base_frame", base_, "base_link");
    pnh_.param<std::string>("expected_input_frame", expected_, ""); pnh_.param<std::string>("frame_override", override_, "");
    if (input_.empty() || output_.empty() || nh_.resolveName(input_) == nh_.resolveName(output_) ||
        (!override_.empty() && expected_.empty()) || fixed_ == base_)
      throw std::invalid_argument("invalid topics/frames; frame_override requires expected_input_frame");
    max_age_=number("max_age", .3); future_=number("future_tolerance", .05, true); tf_wait_=number("tf_wait", .3);
    min_height_=number("min_height", .10, true); max_height_=number("max_height", 1.5);
    min_range_=number("min_range", .15, true); max_range_=number("max_range", 3.0);
    voxel_=number("voxel_size", .05); confirmation_gap_=number("confirmation_max_gap", .4);
    pnh_.param("confirmation_frames", confirmations_, 1); pnh_.param("voxel_min_points", min_points_, 1);
    if (min_height_ >= max_height_ || min_range_ >= max_range_ || confirmations_ < 1 || min_points_ < 1)
      throw std::invalid_argument("invalid filter limits");
    pub_=nh_.advertise<sensor_msgs::PointCloud2>(output_, 1);
    diagnostics_=pnh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);
    sub_=nh_.subscribe(input_, 1, &CloudGate::receive, this);
    timer_=nh_.createWallTimer(ros::WallDuration(.02), &CloudGate::tick, this);
    diagnostic_timer_=nh_.createWallTimer(ros::WallDuration(.5), &CloudGate::diagnose, this);
  }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_gate");
  try { CloudGate gate; ros::spin(); }
  catch (const std::exception& e) { ROS_FATAL("Cloud gate: %s", e.what()); return 1; }
  return 0;
}
