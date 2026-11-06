#include <gtest/gtest.h>
#include <placenav_teb_bridge/retained_path.h>
using namespace placenav_teb_bridge;
std::vector<geometry_msgs::Point> foot() {
  std::vector<geometry_msgs::Point> f(4);double xy[4][2]={{-.15,-.3},{.85,-.3},{.85,.3},{-.15,.3}};
  for(int i=0;i<4;++i){f[i].x=xy[i][0];f[i].y=xy[i][1];}return f;
}
nav_msgs::Path path(std::initializer_list<std::pair<double,double>> pts) {
  nav_msgs::Path p;for(auto q:pts){geometry_msgs::PoseStamped r;r.pose.position.x=q.first;r.pose.position.y=q.second;r.pose.orientation.w=1;p.poses.push_back(r);}return p;
}
void obstacle(costmap_2d::Costmap2D& m,double x,double y,unsigned char cost=254){unsigned a,b;ASSERT_TRUE(m.worldToMap(x,y,a,b));m.setCost(a,b,cost);}
TEST(Retained, InteriorUnknownOutside) {
 costmap_2d::Costmap2D m(120,120,.05,-3,-3,0);ASSERT_TRUE(footprintSafe(m,foot(),{0,0,0}));
 obstacle(m,.4,0);EXPECT_FALSE(footprintSafe(m,foot(),{0,0,0}));
 obstacle(m,.4,0,255);EXPECT_FALSE(footprintSafe(m,foot(),{0,0,0}));EXPECT_FALSE(footprintSafe(m,foot(),{2.5,0,0}));
}
TEST(Retained, WholePathAndConnectionChecked) {
 costmap_2d::Costmap2D m(120,120,.05,-3,-3,0);RetainedPath p;ASSERT_TRUE(p.assign(path({{0,0},{.1,0},{.2,0},{.3,0},{.4,0},{.5,0},{1.5,0}})));
 ASSERT_TRUE(p.safe(m,foot(),{0,0,0}));obstacle(m,2.,0);EXPECT_FALSE(p.safe(m,foot(),{0,0,0}));
 m.resetMap(0,0,120,120);obstacle(m,-.3,0);EXPECT_FALSE(p.safe(m,foot(),{-.8,0,0}));
}
TEST(Retained, RotationSweep) {
 costmap_2d::Costmap2D m(120,120,.05,-3,-3,0);RetainedPath p;ASSERT_TRUE(p.assign(path({{0,0},{0,.1}})));
 p.poses={{0,0,0},{0,0,1.57079632679}};obstacle(m,.55,.55);
 ASSERT_TRUE(footprintSafe(m,foot(),p.poses[0]));ASSERT_TRUE(footprintSafe(m,foot(),p.poses[1]));EXPECT_FALSE(p.safe(m,foot(),{0,0,0}));
}
TEST(Retained, TrackingBoundsAndDeviation) {
 RetainedPath p;ASSERT_TRUE(p.assign(path({{0,0},{.5,.2},{1,.4}})));EXPECT_TRUE(p.advance({.1,.04,0},.2));
 auto c=p.command({.1,.04,0},.3,.2,.1,.1);EXPECT_GT(c.linear.x,0);EXPECT_LE(c.linear.x,.2);EXPECT_LE(std::abs(c.angular.z),.100001);
 EXPECT_FALSE(p.advance({0,1,0},.2));
}
TEST(Retained, ReverseCuspAndFinish) {
 RetainedPath p;ASSERT_TRUE(p.assign(path({{0,0},{-.5,0},{-1,0},{-.5,0}})));EXPECT_EQ(p.poses.size(),3u);EXPECT_EQ(p.gear,-1);
 EXPECT_LT(p.command({0,0,0},.3,.1,.4,.1).linear.x,0);EXPECT_EQ(p.command({-1,0,0},.3,.1,.4,.1).linear.x,0);
}
TEST(Retained, InvalidInput) {
 RetainedPath p;auto a=path({{0,0},{1,0}});a.poses[1].pose.orientation.w=0;EXPECT_FALSE(p.assign(a));EXPECT_TRUE(p.empty());
 a=path({{0,0},{1,0}});a.poses[1].pose.position.x=NAN;EXPECT_FALSE(p.assign(a));EXPECT_TRUE(p.empty());
}
TEST(Retained, CurvedClosedLoopTracking) {
 nav_msgs::Path curve;
 for(int i=0;i<=24;++i){double a=1.2*i/24;geometry_msgs::PoseStamped q;
   q.pose.position.x=std::sin(a);q.pose.position.y=1-std::cos(a);
   q.pose.orientation.z=std::sin(a/2);q.pose.orientation.w=std::cos(a/2);curve.poses.push_back(q);}
 RetainedPath p;ASSERT_TRUE(p.assign(curve));PathPose robot{0,0,0};
 for(int i=0;i<200;++i){ASSERT_TRUE(p.advance(robot,.15));auto c=p.command(robot,.3,.2,.4,.12);
   robot.x+=c.linear.x*std::cos(robot.yaw)*.1;robot.y+=c.linear.x*std::sin(robot.yaw)*.1;robot.yaw+=c.angular.z*.1;}
 EXPECT_LT(std::hypot(robot.x-p.poses.back().x,robot.y-p.poses.back().y),.13);
}
TEST(Retained, ReverseClosedLoopTracking) {
 RetainedPath p;ASSERT_TRUE(p.assign(path({{0,0},{-.5,0},{-1,0}})));PathPose robot{0,0,0};
 for(int i=0;i<200;++i){ASSERT_TRUE(p.advance(robot,.15));auto c=p.command(robot,.3,.1,.4,.12);robot.x+=c.linear.x*.1;}
 EXPECT_LT(std::abs(robot.x+1),.13);
}
int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
