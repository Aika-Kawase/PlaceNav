#include "../src/placenav_teb_bridge/src/rolling_planner.h"
#include <iostream>
#include <stdexcept>
using namespace rolling;
void require(bool v,const char* s){if(!v)throw std::runtime_error(s);}
Grid map(){Grid g;g.width=g.height=100;g.resolution=.1;g.ox=g.oy=-5;g.data.resize(10000);return g;}
void block(Grid& g,double x,double y,unsigned char cost=254){int ix,iy;if(g.cell(x,y,ix,iy))g.data[iy*g.width+ix]=cost;}
int main(){
  auto g=map();std::vector<Pose> fp{{-.15,-.3},{.85,-.3},{.85,.3},{-.15,.3}};
  Collision c(g,fp,.25);Planner planner;planner.budget_seconds=2;
  auto straight=planner.plan(g,c,{0,0,0},0,{},.8);
  require(!straight.empty() && c.path(straight),"free forward path");
  require(straight.back().x>1.5,"forward progress");
  auto renewed=planner.plan(g,c,{1.5,0,0},0,straight,.8);
  require(!renewed.empty() && renewed.back().x>3,"renew before old endpoint");
  require(renewalDistance(.4,.2,1,.3)>.8,"speed dependent renewal");
  require(renewalDistance(0,.8,1,.3)==.8,"minimum renewal");
  auto unsafe=g;block(unsafe,.4,0);Collision inside(unsafe,fp,0);
  require(!inside.free({0,0,0}),"obstacle inside polygon, not only boundary");
  block(unsafe,.4,0,255);Collision unknown(unsafe,fp,0);
  require(!unknown.free({0,0,0}),"unknown blocked");
  require(!c.free({4.5,0,0}),"map boundary footprint");
  auto hall=map();
  for(double x=-4;x<4;x+=.05){block(hall,x,.9);block(hall,x,-.9);}
  Collision hc(hall,fp,.25);
  require(hc.free({0,0,0}),"straight fits corridor");
  require(!hc.edge({0,0,0},{0,0,pi/2}),"rotation sweep hits corridor");
  auto hp=planner.plan(hall,hc,{0,0,0},0,{},.8);
  require(!hp.empty() && hc.path(hp),"walls do not imply path blocked");
  auto obs=map();block(obs,1.4,0);
  Collision oc(obs,fp,.1);
  auto detour=planner.plan(obs,oc,{0,0,0},0,{},.8);
  require(!detour.empty() && oc.path(detour),"find collision checked detour");
  bool lateral=false;for(auto p:detour)lateral|=std::abs(p.y)>.35;
  require(lateral,"detour uses lateral progress");
  auto wall=map();for(double y=-5;y<5;y+=.05)block(wall,1.2,y);
  Collision wc(wall,fp,.25);planner.budget_seconds=.05;
  auto begin=std::chrono::steady_clock::now();
  require(planner.plan(wall,wc,{0,0,0},0,{},.8).empty(),"unreachable goals stop");
  require(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()<.2,"bounded computation");
  auto near=remaining({1.9,0,0},straight);
  require(length(near)<.4,"prune passed prefix");
  require(!oc.path({{0,0,0},{2,0,0}}),"recovery connector cannot cross obstacle");
  Grid fine;fine.width=fine.height=160;fine.resolution=.05;fine.ox=fine.oy=-4;fine.data.resize(25600);
  for(double x=1.75;x<1.9;x+=.05)for(double y=-.05;y<.1;y+=.05)block(fine,x,y);
  std::vector<Pose> padded{{-.17,-.32},{.87,-.32},{.87,.32},{-.17,.32}};
  Collision fc(fine,padded,.25);planner.budget_seconds=.12;
  begin=std::chrono::steady_clock::now();
  auto finepath=planner.plan(fine,fc,{0,0,0},0,{},.8);
  std::cout<<"fine-grid path poses="<<finepath.size()<<" seconds="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()<<std::endl;
  require(!finepath.empty(),"default budget detour on 5cm map");
  std::cout<<"rolling planner tests passed\n";
}
