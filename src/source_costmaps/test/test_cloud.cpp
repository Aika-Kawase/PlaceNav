#include <source_costmaps/cloud.h>
#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace sc = source_costmaps;
geometry_msgs::TransformStamped identity() {
  geometry_msgs::TransformStamped t;
  t.transform.rotation.w = 1;
  return t;
}
TEST(Cloud, MetricRigidTransformAndFiniteFilter) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const auto c = sc::makeCloud({{{1, 0, 0}}, {{nan, 0, 1}}}, std_msgs::Header());
  auto t = identity();
  t.transform.rotation.z = std::sqrt(0.5); t.transform.rotation.w = std::sqrt(0.5);
  t.transform.translation.x = 2; t.transform.translation.z = 0.5;
  const auto p = sc::transformXYZ(c, t);
  ASSERT_EQ(1u, p.size());
  EXPECT_NEAR(2, p[0][0], 1e-6); EXPECT_NEAR(1, p[0][1], 1e-6); EXPECT_NEAR(.5, p[0][2], 1e-6);
}
TEST(Cloud, OrganizedPaddedRowsAndBigEndianFloat64) {
  sensor_msgs::PointCloud2 c;
  c.width = 1; c.height = 2; c.point_step = 32; c.row_step = 40; c.is_bigendian = true;
  c.data.resize(80, 0xff);
  for (uint32_t j = 0; j < 3; ++j) {
    sensor_msgs::PointField f;
    f.name = std::string(1, "xyz"[j]); f.offset = j * 8; f.datatype = f.FLOAT64; f.count = 1;
    c.fields.push_back(f);
    for (size_t row = 0; row < 2; ++row) {
      double value = row * 3 + j + 1;
      uint8_t bytes[8]; std::memcpy(bytes, &value, 8);
      const uint16_t one = 1;
      if (*reinterpret_cast<const uint8_t*>(&one)) std::reverse(bytes, bytes + 8);
      std::copy(bytes, bytes + 8, c.data.begin() + row * 40 + j * 8);
    }
  }
  const auto p = sc::transformXYZ(c, identity());
  ASSERT_EQ(2u, p.size()); EXPECT_EQ(1, p[0][0]); EXPECT_EQ(6, p[1][2]);
}
TEST(Cloud, RejectsMalformedLayoutAndTransform) {
  auto c = sc::makeCloud({{{1, 2, 3}}}, std_msgs::Header());
  c.data.pop_back(); EXPECT_THROW(sc::transformXYZ(c, identity()), std::invalid_argument);
  c.data.push_back(0); c.fields[0].offset = 20;
  EXPECT_THROW(sc::transformXYZ(c, identity()), std::invalid_argument);
  c.fields[0].offset = 0; c.fields[1].datatype = sensor_msgs::PointField::UINT32;
  EXPECT_THROW(sc::transformXYZ(c, identity()), std::invalid_argument);
  c.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  auto t = identity(); t.transform.rotation.w = 0;
  EXPECT_THROW(sc::transformXYZ(c, t), std::invalid_argument);
}
TEST(Cloud, OutputHasOnlyXYZAndPreservesHeader) {
  std_msgs::Header h; h.frame_id = "odom"; h.stamp = ros::Time(123, 45);
  const auto c = sc::makeCloud({{{1, 2, 3}}, {{4, 5, 6}}}, h);
  EXPECT_EQ(h, c.header); EXPECT_EQ(3u, c.fields.size()); EXPECT_EQ(12u, c.point_step);
  EXPECT_EQ(24u, c.row_step); EXPECT_TRUE(c.is_dense);
  const auto p = sc::transformXYZ(c, identity()); ASSERT_EQ(2u, p.size()); EXPECT_EQ(6, p[1][2]);
}
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
