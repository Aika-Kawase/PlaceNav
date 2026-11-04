#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_layer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <memory>
#include <map>
#include <mutex>
#include <stdexcept>

class SourceCostmaps {
  ros::NodeHandle pnh_{"~"};
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_{buffer_};
  std::unique_ptr<costmap_2d::Costmap2DROS> map_;
  ros::Publisher combined_;
  std::map<std::string, ros::Publisher> layers_;
  ros::Timer timer_;
  nav_msgs::OccupancyGrid grid(costmap_2d::Costmap2D& map, const ros::Time& stamp) {
    nav_msgs::OccupancyGrid msg; msg.header.frame_id = map_->getGlobalFrameID(); msg.header.stamp = stamp;
    msg.info.resolution = map.getResolution(); msg.info.width = map.getSizeInCellsX(); msg.info.height = map.getSizeInCellsY();
    msg.info.origin.position.x = map.getOriginX(); msg.info.origin.position.y = map.getOriginY(); msg.info.origin.orientation.w = 1;
    msg.data.reserve(msg.info.width*msg.info.height);
    for (size_t i = 0; i < size_t(msg.info.width)*msg.info.height; ++i) {
      const auto c = map.getCharMap()[i];
      msg.data.push_back(c == 255 ? -1 : c == 254 ? 100 : c == 253 ? 99 : c == 0 ? 0 : 1+97*(c-1)/251);
    }
    return msg;
  }
  void publish(const ros::TimerEvent&) {
    // Same update boundary and origin for all three inspection maps.
    auto* master = map_->getCostmap();
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*master->getMutex());
    const auto stamp = ros::Time::now();
    for (const auto& plugin : *map_->getLayeredCostmap()->getPlugins()) {
      auto* layer = dynamic_cast<costmap_2d::CostmapLayer*>(plugin.get());
      if (!layer) continue;
      const auto name = plugin->getName().substr(plugin->getName().find_last_of('/')+1);
      const auto publisher = layers_.find(name);
      if (publisher != layers_.end()) publisher->second.publish(grid(*layer, stamp));
    }
    combined_.publish(grid(*master, stamp));
  }
public:
  SourceCostmaps() {
    combined_=pnh_.advertise<nav_msgs::OccupancyGrid>("combined/costmap", 1, true);
    map_.reset(new costmap_2d::Costmap2DROS("local_costmap", buffer_));
    for (const auto& plugin : *map_->getLayeredCostmap()->getPlugins()) {
      const auto name = plugin->getName().substr(plugin->getName().find_last_of('/')+1);
      if (name == "d435" || name == "unik3d" || name == "fused")
        layers_.emplace(name, pnh_.advertise<nav_msgs::OccupancyGrid>(name+"/costmap", 1, true));
    }
    double rate; pnh_.param("inspection_frequency", rate, 5.0);
    if (!std::isfinite(rate) || rate <= 0) throw std::invalid_argument("invalid inspection_frequency");
    timer_=pnh_.createTimer(ros::Duration(1.0/rate), &SourceCostmaps::publish, this);
  }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "source_costmaps");
  try { SourceCostmaps maps; ros::spin(); }
  catch (const std::exception& e) { ROS_FATAL("Source costmaps: %s", e.what()); return 1; }
  return 0;
}
