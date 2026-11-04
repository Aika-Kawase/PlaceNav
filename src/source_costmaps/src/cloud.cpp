#include <source_costmaps/cloud.h>
#include <sensor_msgs/PointField.h>
#include <tf2/LinearMath/Transform.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace source_costmaps {
namespace {
bool hostBigEndian() {
  const uint16_t value = 1;
  return *reinterpret_cast<const uint8_t*>(&value) == 0;
}
template <typename T> T read(const uint8_t* data, bool swap) {
  uint8_t bytes[sizeof(T)];
  std::copy(data, data + sizeof(T), bytes);
  if (swap) std::reverse(bytes, bytes + sizeof(T));
  T value;
  std::memcpy(&value, bytes, sizeof(T));
  return value;
}
}  // namespace

Points transformXYZ(const sensor_msgs::PointCloud2& c,
                    const geometry_msgs::TransformStamped& t) {
  using F = sensor_msgs::PointField;
  const double values[] = {t.transform.translation.x, t.transform.translation.y,
                          t.transform.translation.z, t.transform.rotation.x,
                          t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w};
  for (double v : values)
    if (!std::isfinite(v)) throw std::invalid_argument("nonfinite TF");
  tf2::Quaternion q(values[3], values[4], values[5], values[6]);
  if (q.length2() < 1e-12) throw std::invalid_argument("zero TF quaternion");
  q.normalize();
  const tf2::Transform tf(q, tf2::Vector3(values[0], values[1], values[2]));
  std::array<const F*, 3> fields{{nullptr, nullptr, nullptr}};
  const char* names[] = {"x", "y", "z"};
  for (const auto& f : c.fields) {
    for (size_t i = 0; i < 3; ++i) {
      if (f.name != names[i]) continue;
      if (fields[i]) throw std::invalid_argument("duplicate XYZ field");
      fields[i] = &f;
    }
  }
  for (const F* f : fields) {
    if (!f || f->count != 1 || (f->datatype != F::FLOAT32 && f->datatype != F::FLOAT64))
      throw std::invalid_argument("XYZ fields must be scalar FLOAT32/FLOAT64");
    const uint64_t bytes = f->datatype == F::FLOAT32 ? 4 : 8;
    if (uint64_t(f->offset) + bytes > c.point_step)
      throw std::invalid_argument("XYZ outside point_step");
  }
  const uint64_t row_bytes = uint64_t(c.width) * c.point_step;
  if (row_bytes > c.row_step || uint64_t(c.row_step) * c.height > c.data.size())
    throw std::invalid_argument("truncated point cloud or invalid row_step");
  const uint64_t count = uint64_t(c.width) * c.height;
  if (count > std::numeric_limits<uint32_t>::max() / 12)
    throw std::invalid_argument("point cloud exceeds output wire size");
  Points out;
  out.reserve(count);
  const bool swap = c.is_bigendian != hostBigEndian();
  for (uint32_t row = 0; row < c.height; ++row) {
    for (uint32_t col = 0; col < c.width; ++col) {
      const uint8_t* p = c.data.data() + size_t(row) * c.row_step + size_t(col) * c.point_step;
      double xyz[3];
      for (size_t i = 0; i < 3; ++i)
        xyz[i] = fields[i]->datatype == F::FLOAT32
            ? read<float>(p + fields[i]->offset, swap)
            : read<double>(p + fields[i]->offset, swap);
      if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2])) continue;
      const auto v = tf * tf2::Vector3(xyz[0], xyz[1], xyz[2]);
      std::array<float, 3> point{{float(v.x()), float(v.y()), float(v.z())}};
      if (std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2]))
        out.push_back(point);
    }
  }
  return out;
}

sensor_msgs::PointCloud2 makeCloud(const Points& points, const std_msgs::Header& header) {
  if (points.size() > std::numeric_limits<uint32_t>::max() / 12)
    throw std::invalid_argument("combined cloud exceeds output wire size");
  sensor_msgs::PointCloud2 out;
  out.header = header;
  out.height = 1;
  out.width = points.size();
  out.is_bigendian = hostBigEndian();
  out.is_dense = true;
  out.point_step = 12;
  out.row_step = out.width * out.point_step;
  out.data.resize(out.row_step);
  const char* names[] = {"x", "y", "z"};
  for (uint32_t i = 0; i < 3; ++i) {
    sensor_msgs::PointField f;
    f.name = names[i]; f.offset = i * 4; f.datatype = sensor_msgs::PointField::FLOAT32; f.count = 1;
    out.fields.push_back(f);
  }
  for (size_t i = 0; i < points.size(); ++i)
    for (size_t j = 0; j < 3; ++j)
      std::memcpy(out.data.data() + i * 12 + j * 4, &points[i][j], 4);
  return out;
}
}  // namespace source_costmaps
