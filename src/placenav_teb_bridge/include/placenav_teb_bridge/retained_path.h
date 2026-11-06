#pragma once
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>

namespace placenav_teb_bridge {
struct PathPose { double x, y, yaw; };
inline double angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline double segmentDistance(double x, double y, const PathPose& a, const PathPose& b) {
  const double dx=b.x-a.x, dy=b.y-a.y, n=dx*dx+dy*dy;
  const double u=n>0 ? std::max(0.,std::min(1.,((x-a.x)*dx+(y-a.y)*dy)/n)) : 0.;
  return std::hypot(x-a.x-u*dx,y-a.y-u*dy);
}
// Check cell interiors as well as the polygon boundary. Unknown/outside are unsafe.
inline bool footprintSafe(costmap_2d::Costmap2D& map, const std::vector<geometry_msgs::Point>& footprint,
                          const PathPose& pose) {
  if (footprint.size()<3) return false;
  std::vector<PathPose> poly;
  double minx=INFINITY,miny=INFINITY,maxx=-INFINITY,maxy=-INFINITY;
  for (const auto& p:footprint) {
    PathPose q{pose.x+std::cos(pose.yaw)*p.x-std::sin(pose.yaw)*p.y,
               pose.y+std::sin(pose.yaw)*p.x+std::cos(pose.yaw)*p.y,0};
    poly.push_back(q); minx=std::min(minx,q.x); maxx=std::max(maxx,q.x);
    miny=std::min(miny,q.y); maxy=std::max(maxy,q.y);
  }
  unsigned int lx,ly,hx,hy;
  if (!map.worldToMap(minx,miny,lx,ly)||!map.worldToMap(maxx,maxy,hx,hy)) return false;
  const double margin=map.getResolution()/std::sqrt(2.);
  for (unsigned int y=ly;y<=hy;++y) for (unsigned int x=lx;x<=hx;++x) {
    if (map.getCost(x,y)<costmap_2d::LETHAL_OBSTACLE) continue;
    double wx,wy; map.mapToWorld(x,y,wx,wy); bool inside=false; double edge=INFINITY;
    for (std::size_t i=0,j=poly.size()-1;i<poly.size();j=i++) {
      const auto& a=poly[j]; const auto& b=poly[i];
      if ((a.y>wy)!=(b.y>wy) && wx<(b.x-a.x)*(wy-a.y)/(b.y-a.y)+a.x) inside=!inside;
      edge=std::min(edge,segmentDistance(wx,wy,a,b));
    }
    if (inside||edge<=margin) return false;
  }
  return true;
}
class RetainedPath {
public:
  std::vector<PathPose> poses;
  std::size_t cursor=0;
  int gear=1;
  void clear() { poses.clear(); cursor=0; }
  bool empty() const { return poses.size()<2; }
  bool assign(const nav_msgs::Path& path) {
    clear();
    for (const auto& p:path.poses) {
      const auto& rot=p.pose.orientation;
      const double norm=rot.x*rot.x+rot.y*rot.y+rot.z*rot.z+rot.w*rot.w;
      if(!std::isfinite(norm)||norm<1e-12) {clear();return false;}
      PathPose q{p.pose.position.x,p.pose.position.y,tf2::getYaw(p.pose.orientation)};
      if (!std::isfinite(q.x)||!std::isfinite(q.y)||!std::isfinite(q.yaw)) { clear(); return false; }
      poses.push_back(q);
    }
    if (empty()) { clear(); return false; }
    bool found=false;
    for (std::size_t i=1;i<poses.size();++i) {
      const auto& a=poses[i-1]; const auto& b=poses[i];
      if (std::hypot(b.x-a.x,b.y-a.y)<1e-5) continue;
      const int g=((b.x-a.x)*std::cos(a.yaw)+(b.y-a.y)*std::sin(a.yaw)>=0)?1:-1;
      if (!found) {gear=g;found=true;}
      // Stop retaining at a gear change; never jump across a reversing cusp.
      else if (g!=gear) {poses.resize(i);break;}
    }
    if (!found||empty()) {clear();return false;} return true;
  }
  bool advance(const PathPose& robot, double max_deviation) {
    if (empty()) return false;
    // Local, monotone progress prevents jumps to a crossing later in the path.
    while (cursor+2<poses.size()) {
      const auto& a=poses[cursor]; const auto& b=poses[cursor+1];
      const double dx=b.x-a.x,dy=b.y-a.y;
      if ((robot.x-b.x)*dx+(robot.y-b.y)*dy<0) break;
      ++cursor;
    }
    return segmentDistance(robot.x,robot.y,poses[cursor],poses[cursor+1])<=max_deviation;
  }
  bool safe(costmap_2d::Costmap2D& map, const std::vector<geometry_msgs::Point>& footprint,
            const PathPose& robot) const {
    if (empty()) return false;
    double radius=0.;for(const auto& p:footprint)radius=std::max(radius,std::hypot(p.x,p.y));
    PathPose previous=robot;
    for (std::size_t i=cursor+1;i<poses.size();++i) {
      const auto& next=poses[i]; const double dyaw=angle(next.yaw-previous.yaw);
      const double sweep=std::hypot(next.x-previous.x,next.y-previous.y)+radius*std::abs(dyaw);
      const int steps=std::max(1,int(std::ceil(sweep/(map.getResolution()*.5))));
      for(int j=0;j<=steps;++j) {
        const double u=double(j)/steps;
        if(!footprintSafe(map,footprint,{previous.x+u*(next.x-previous.x),previous.y+u*(next.y-previous.y),previous.yaw+u*dyaw}))return false;
      }
      previous=next;
    }
    return true;
  }
  geometry_msgs::Twist command(const PathPose& robot,double lookahead,double speed,
                              double max_w,double goal_tolerance) const {
    geometry_msgs::Twist cmd;if(empty())return cmd;
    const auto& end=poses.back();
    if(std::hypot(end.x-robot.x,end.y-robot.y)<=goal_tolerance)return cmd;
    PathPose target=poses.back(),prev=robot;double remaining=lookahead;
    for(std::size_t i=cursor+1;i<poses.size();++i){
      const double d=std::hypot(poses[i].x-prev.x,poses[i].y-prev.y);
      if(d>=remaining && d>1e-6){const double u=remaining/d;target={prev.x+u*(poses[i].x-prev.x),prev.y+u*(poses[i].y-prev.y),0};break;}
      remaining-=d;prev=poses[i];
    }
    const double dx=target.x-robot.x,dy=target.y-robot.y;
    const double x=std::cos(robot.yaw)*dx+std::sin(robot.yaw)*dy;
    const double y=-std::sin(robot.yaw)*dx+std::cos(robot.yaw)*dy;
    if(gear*x<=0)return cmd;
    double v=std::min(speed,std::hypot(end.x-robot.x,end.y-robot.y));
    const double curvature=2*y/std::max(1e-6,dx*dx+dy*dy);
    if(std::abs(curvature)>1e-6)v=std::min(v,max_w/std::abs(curvature));
    cmd.linear.x=gear*v;cmd.angular.z=cmd.linear.x*curvature;return cmd;
  }
};
} // namespace
