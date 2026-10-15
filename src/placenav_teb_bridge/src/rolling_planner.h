#pragma once
// Local, bounded SE(2) lattice search. No ROS dependency: exercised by unit tests.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

namespace rolling {
constexpr double pi = 3.14159265358979323846;
inline double angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
struct Pose { double x=0, y=0, yaw=0; };
struct Grid {
  int width=0, height=0;
  double resolution=.05, ox=0, oy=0;
  std::vector<unsigned char> data;
  bool cell(double x, double y, int& ix, int& iy) const {
    ix=static_cast<int>(std::floor((x-ox)/resolution));
    iy=static_cast<int>(std::floor((y-oy)/resolution));
    return ix>=0 && iy>=0 && ix<width && iy<height;
  }
};
inline double distance(Pose a, Pose b) { return std::hypot(a.x-b.x,a.y-b.y); }
inline double segmentDistance(double x, double y, Pose a, Pose b) {
  double dx=b.x-a.x, dy=b.y-a.y, d=dx*dx+dy*dy;
  double t=d>1e-12 ? std::max(0.,std::min(1.,((x-a.x)*dx+(y-a.y)*dy)/d)) : 0;
  return std::hypot(x-a.x-t*dx,y-a.y-t*dy);
}

class Collision {
public:
  Collision(const Grid& g, const std::vector<Pose>& footprint, double clearance)
      : g_(g), footprint_(footprint), clearance_(clearance) {
    for(int i=0;i<static_cast<int>(g.data.size());++i)
      if(g.data[i]>=254) occupied_.push_back(i);
  }
  bool free(Pose p) const {
    if (footprint_.size()<3 || !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw)) return false;
    double c=std::cos(p.yaw),s=std::sin(p.yaw);
    std::vector<Pose> poly;
    double xmin=1e30,ymin=1e30,xmax=-1e30,ymax=-1e30;
    for (auto v:footprint_) {
      Pose q{p.x+c*v.x-s*v.y,p.y+s*v.x+c*v.y,0}; poly.push_back(q);
      xmin=std::min(xmin,q.x); xmax=std::max(xmax,q.x);
      ymin=std::min(ymin,q.y); ymax=std::max(ymax,q.y);
    }
    // Occupied cell squares are conservatively covered by their half diagonal.
    double margin=clearance_+g_.resolution/std::sqrt(2.);
    int x0,y0,x1,y1;
    if (!g_.cell(xmin-margin,ymin-margin,x0,y0) || !g_.cell(xmax+margin,ymax+margin,x1,y1)) return false;
    auto intersects=[&](int x,int y) {
      double wx=g_.ox+(x+.5)*g_.resolution, wy=g_.oy+(y+.5)*g_.resolution;
      bool inside=false;
      for(std::size_t i=0,j=poly.size()-1;i<poly.size();j=i++) {
        auto a=poly[j],b=poly[i];
        if(segmentDistance(wx,wy,a,b)<=margin) return true;
        if((a.y>wy)!=(b.y>wy) && wx<(b.x-a.x)*(wy-a.y)/(b.y-a.y)+a.x) inside=!inside;
      }
      return inside;
    };
    // Sparse depth maps: avoid scanning hundreds of free cells for every pose.
    if(occupied_.size()<static_cast<std::size_t>((x1-x0+1)*(y1-y0+1))) {
      for(int cell:occupied_) {
        int x=cell%g_.width,y=cell/g_.width;
        if(x>=x0&&x<=x1&&y>=y0&&y<=y1&&intersects(x,y))return false;
      }
    } else {
      for(int y=y0;y<=y1;++y)for(int x=x0;x<=x1;++x)
        if(g_.data[y*g_.width+x]>=254&&intersects(x,y))return false;
    }
    return true;
  }
  bool edge(Pose a, Pose b) const {
    // Interpolate translation AND rotation, including footprint sweep.
    double radius=0;for(auto p:footprint_)radius=std::max(radius,std::hypot(p.x,p.y));
    int n=std::max(1,static_cast<int>(std::ceil((distance(a,b)+radius*std::abs(angle(b.yaw-a.yaw)))/(g_.resolution*.5))));
    for(int i=0;i<=n;++i) {
      double t=double(i)/n;
      if(!free({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.yaw+t*angle(b.yaw-a.yaw)})) return false;
    }
    return true;
  }
  bool path(const std::vector<Pose>& p) const {
    if(p.empty() || !free(p[0])) return false;
    for(std::size_t i=1;i<p.size();++i) if(!edge(p[i-1],p[i])) return false;
    return true;
  }
private:
  const Grid& g_;
  const std::vector<Pose>& footprint_;
  double clearance_;
  std::vector<int> occupied_;
};

inline std::vector<Pose> remaining(Pose robot,const std::vector<Pose>& path) {
  if(path.empty()) return {};
  std::size_t nearest=0;
  for(std::size_t i=1;i<path.size();++i)
    if(distance(robot,path[i])<distance(robot,path[nearest])) nearest=i;
  std::vector<Pose> result{robot};
  result.insert(result.end(),path.begin()+nearest,path.end());
  return result;
}
inline double length(const std::vector<Pose>& p) {
  double n=0; for(std::size_t i=1;i<p.size();++i)n+=distance(p[i-1],p[i]);return n;
}
inline double renewalDistance(double speed,double base,double planning_latency,double deceleration) {
  return std::max(base,std::abs(speed)*planning_latency+speed*speed/(2*deceleration)+.2);
}

// Recompute the WHOLE seed. An old endpoint is preferred only while forward,
// reachable, and outside the renewal distance. Direction is a weak cost term.
class Planner {
public:
  double horizon=2.0, min_progress=.6, direction_weight=.3;
  double budget_seconds=.12;
  int max_expansions=12000;
  std::vector<Pose> plan(const Grid& g,const Collision& collision,Pose start,
                         double direction,const std::vector<Pose>& previous,double renew) const {
    auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(budget_seconds);
    if(!collision.free(start)) return {};
    std::vector<Pose> candidates;
    if(!previous.empty()) {
      auto old=previous.back();
      double bearing=std::atan2(old.y-start.y,old.x-start.x);
      if(distance(start,old)>renew && distance(start,old)<horizon*1.5 &&
         std::abs(angle(bearing-direction))<pi/3) candidates.push_back(old);
    }
    for(double radius:{horizon,horizon*.75})
      for(double offset:{0.,pi/6,-pi/6,pi/3,-pi/3}) {
        double a=direction+offset;
        candidates.push_back({start.x+radius*std::cos(a),start.y+radius*std::sin(a),a});
      }
    int expanded=0;
    for(auto goal:candidates) {
      if(std::chrono::steady_clock::now()>=deadline || expanded>=max_expansions) break;
      if((goal.x-start.x)*std::cos(direction)+(goal.y-start.y)*std::sin(direction)<min_progress) continue;
      auto p=search(g,collision,start,goal,direction,deadline,expanded);
      if(!p.empty()) return p;
    }
    return {};
  }
private:
  using Deadline=std::chrono::time_point<std::chrono::steady_clock,std::chrono::duration<double>>;
  std::vector<Pose> search(const Grid& g,const Collision& collision,Pose start,Pose goal,
                           double direction,Deadline deadline,int& expanded) const {
    int sx,sy,gx,gy;
    if(!g.cell(start.x,start.y,sx,sy)||!g.cell(goal.x,goal.y,gx,gy)||!collision.free(goal)) return {};
    constexpr int dirs=8;
    const int dx[dirs]={1,1,0,-1,-1,-1,0,1},dy[dirs]={0,1,1,1,0,-1,-1,-1};
    auto id=[&](int x,int y,int h){return (y*g.width+x)*dirs+h;};
    auto pose=[&](int node){int h=node%dirs,cell=node/dirs;return Pose{g.ox+(cell%g.width+.5)*g.resolution,g.oy+(cell/g.width+.5)*g.resolution,h*pi/4};};
    int sh=(static_cast<int>(std::lround(start.yaw/(pi/4)))%dirs+dirs)%dirs;
    int first=id(sx,sy,sh),size=g.width*g.height*dirs;
    if(!collision.edge(start,pose(first))) return {};
    std::vector<double> cost(size,std::numeric_limits<double>::infinity());
    std::vector<int> parent(size,-1);
    std::vector<unsigned char> closed(size,0);
    using Entry=std::pair<double,int>;
    std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> queue;
    cost[first]=0;queue.push({distance(start,goal),first});
    while(!queue.empty() && expanded<max_expansions) {
      if(std::chrono::steady_clock::now()>=deadline) return {};
      int cur=queue.top().second;queue.pop();if(closed[cur])continue;
      closed[cur]=1;++expanded;Pose a=pose(cur);
      int cell=cur/dirs,x=cell%g.width,y=cell/g.width;
      if(x==gx && y==gy) {
        std::vector<Pose> path;
        for(int n=cur;n!=-1;n=parent[n])path.push_back(pose(n));
        std::reverse(path.begin(),path.end());path.insert(path.begin(),start);
        std::vector<Pose> swept{path.front()};
        for(std::size_t i=1;i<path.size();++i) {
          Pose turn=swept.back();turn.yaw=path[i].yaw;
          swept.push_back(turn);swept.push_back(path[i]);
        }
        return swept;
      }
      for(int h=0;h<dirs;++h) {
        int nx=x+dx[h],ny=y+dy[h];
        if(nx<0||ny<0||nx>=g.width||ny>=g.height)continue;
        int next=id(nx,ny,h);if(closed[next]||g.data[ny*g.width+nx]>=254)continue;
        Pose b=pose(next),turn=a;turn.yaw=b.yaw;
        // In-place turn then forward move: no sideways translation shortcut.
        if(!collision.edge(a,turn)||!collision.edge(turn,b))continue;
        double step=distance(a,b);
        double score=cost[cur]+step*(1.+2.*g.data[ny*g.width+nx]/253.+
                    direction_weight*(1.-std::cos(b.yaw-direction)))+.1*std::abs(angle(b.yaw-a.yaw));
        if(score>=cost[next])continue;
        cost[next]=score;parent[next]=cur;
        queue.push({score+distance(b,goal),next});
      }
    }
    return {};
  }
};
}
