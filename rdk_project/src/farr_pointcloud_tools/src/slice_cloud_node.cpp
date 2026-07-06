#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

class SliceCloudNode : public rclcpp::Node {
 public:
  SliceCloudNode() : Node("farr_slice_cloud_cpp") {
    input_topic_ = declare_parameter<std::string>("input_cloud_topic", "/cloud_registered");
    output_topic_ = declare_parameter<std::string>("output_cloud_topic", "/farr_slice_cloud");
    z_min_ = declare_parameter<double>("z_min", -0.35);
    z_max_ = declare_parameter<double>("z_max", 0.85);
    min_range_ = declare_parameter<double>("min_range", 0.35);
    max_range_ = declare_parameter<double>("max_range", 12.0);
    mount_roll_ = declare_parameter<double>("mount_roll", 0.0);
    mount_pitch_ = declare_parameter<double>("mount_pitch", 0.0);
    mount_yaw_ = declare_parameter<double>("mount_yaw", 0.0);
    process_every_ = std::max<int>(1, static_cast<int>(declare_parameter<int>("process_every_n_clouds", 2)));
    max_points_ = std::max<int>(100, static_cast<int>(declare_parameter<int>("max_points", 8000)));
    updateMountCorrection();

    auto sub_qos = rclcpp::SensorDataQoS().keep_last(1);
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    pub_ = create_publisher<PointCloud2>(output_topic_, pub_qos);
    sub_ = create_subscription<PointCloud2>(
        input_topic_, sub_qos, std::bind(&SliceCloudNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "slice cloud C++: %s -> %s, z=[%.2f, %.2f], mount_rpy=[%.3f, %.3f, %.3f] rad",
                input_topic_.c_str(), output_topic_.c_str(), z_min_, z_max_,
                mount_roll_, mount_pitch_, mount_yaw_);
  }

 private:
  static int offsetOf(const PointCloud2 &msg, const std::string &name) {
    for (const auto &field : msg.fields) {
      if (field.name == name && field.datatype == PointField::FLOAT32) {
        return static_cast<int>(field.offset);
      }
    }
    return -1;
  }

  static float readFloat(const std::vector<uint8_t> &data, size_t index) {
    float value;
    std::memcpy(&value, data.data() + index, sizeof(float));
    return value;
  }

  void updateMountCorrection() {
    const double cr = std::cos(mount_roll_);
    const double sr = std::sin(mount_roll_);
    const double cp = std::cos(mount_pitch_);
    const double sp = std::sin(mount_pitch_);
    const double cy = std::cos(mount_yaw_);
    const double sy = std::sin(mount_yaw_);

    rot_[0][0] = cy * cp;
    rot_[0][1] = cy * sp * sr - sy * cr;
    rot_[0][2] = cy * sp * cr + sy * sr;
    rot_[1][0] = sy * cp;
    rot_[1][1] = sy * sp * sr + cy * cr;
    rot_[1][2] = sy * sp * cr - cy * sr;
    rot_[2][0] = -sp;
    rot_[2][1] = cp * sr;
    rot_[2][2] = cp * cr;
  }

  void correctPoint(float x, float y, float z, float &cx, float &cy, float &cz) const {
    cx = static_cast<float>(rot_[0][0] * x + rot_[0][1] * y + rot_[0][2] * z);
    cy = static_cast<float>(rot_[1][0] * x + rot_[1][1] * y + rot_[1][2] * z);
    cz = static_cast<float>(rot_[2][0] * x + rot_[2][1] * y + rot_[2][2] * z);
  }

  void cloudCallback(const PointCloud2::SharedPtr msg) {
    ++received_count_;
    if (received_count_ % process_every_ != 0) {
      return;
    }

    const int ox = offsetOf(*msg, "x");
    const int oy = offsetOf(*msg, "y");
    const int oz = offsetOf(*msg, "z");
    if (ox < 0 || oy < 0 || oz < 0 || msg->point_step == 0 || msg->is_bigendian) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Unsupported PointCloud2 layout");
      return;
    }

    const size_t count = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    const double min_r2 = min_range_ * min_range_;
    const double max_r2 = max_range_ * max_range_;

    std::vector<float> xyz;
    xyz.reserve(static_cast<size_t>(max_points_) * 3U);

    int accepted_seen = 0;
    for (size_t i = 0; i < count; ++i) {
      const size_t base = i * msg->point_step;
      if (base + msg->point_step > msg->data.size()) {
        break;
      }
      const float x = readFloat(msg->data, base + static_cast<size_t>(ox));
      const float y = readFloat(msg->data, base + static_cast<size_t>(oy));
      const float z = readFloat(msg->data, base + static_cast<size_t>(oz));
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      float cx, cy, cz;
      correctPoint(x, y, z, cx, cy, cz);
      const double r2 = static_cast<double>(cx) * cx + static_cast<double>(cy) * cy;
      if (cz < z_min_ || cz > z_max_ || r2 < min_r2 || r2 > max_r2) {
        continue;
      }
      ++accepted_seen;
      if (accepted_seen <= max_points_) {
        xyz.push_back(cx); xyz.push_back(cy); xyz.push_back(cz);
      } else {
        const size_t replace_index = static_cast<size_t>(accepted_seen % max_points_);
        if (replace_index == 0) {
          continue;
        }
        const size_t base_idx = (replace_index - 1) * 3U;
        xyz[base_idx] = cx;
        xyz[base_idx + 1] = cy;
        xyz[base_idx + 2] = cz;
      }
    }

    PointCloud2 out;
    out.header = msg->header;
    if (out.header.frame_id.empty()) out.header.frame_id = "camera_init";
    out.header.stamp = msg->header.stamp;
    out.height = 1;
    out.width = static_cast<uint32_t>(xyz.size() / 3U);
    out.fields.resize(3);
    out.fields[0].name = "x"; out.fields[0].offset = 0; out.fields[0].datatype = PointField::FLOAT32; out.fields[0].count = 1;
    out.fields[1].name = "y"; out.fields[1].offset = 4; out.fields[1].datatype = PointField::FLOAT32; out.fields[1].count = 1;
    out.fields[2].name = "z"; out.fields[2].offset = 8; out.fields[2].datatype = PointField::FLOAT32; out.fields[2].count = 1;
    out.is_bigendian = false;
    out.point_step = 12;
    out.row_step = out.point_step * out.width;
    out.is_dense = false;
    out.data.resize(xyz.size() * sizeof(float));
    if (!xyz.empty()) {
      std::memcpy(out.data.data(), xyz.data(), out.data.size());
    }
    pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  double z_min_;
  double z_max_;
  double min_range_;
  double max_range_;
  double mount_roll_;
  double mount_pitch_;
  double mount_yaw_;
  double rot_[3][3]{};
  int process_every_;
  int max_points_;
  uint64_t received_count_{0};
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SliceCloudNode>());
  rclcpp::shutdown();
  return 0;
}
