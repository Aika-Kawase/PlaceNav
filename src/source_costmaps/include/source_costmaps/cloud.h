#pragma once
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <array>
#include <vector>

namespace source_costmaps {
using Points = std::vector<std::array<float, 3>>;
// Validates the wire layout, supports padded rows, FLOAT32/FLOAT64, either endian.
// Nonfinite input/output points are omitted. Throws std::invalid_argument on bad layout/TF.
Points transformXYZ(const sensor_msgs::PointCloud2& cloud,
                    const geometry_msgs::TransformStamped& transform);
sensor_msgs::PointCloud2 makeCloud(const Points& points, const std_msgs::Header& header);
}  // namespace source_costmaps
