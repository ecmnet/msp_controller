#include "msp_controller/ros2/tf_transformer.h"

#include <string>

#include "msp_controller/ros2/geometry_msg_conversions.h"

namespace wavemap {
bool TfTransformer::isTransformAvailable(
    const std::string& to_frame_id, const std::string& from_frame_id,
    const rclcpp::Time&  frame_timestamp) const {
  return tf_buffer_.canTransform(sanitizeFrameId(to_frame_id),
                                 sanitizeFrameId(from_frame_id),
                                 frame_timestamp);
}

bool TfTransformer::waitForTransform(const std::string& to_frame_id,
                                     const std::string& from_frame_id,
                                     const rclcpp::Time& frame_timestamp) {
  return waitForTransformImpl(sanitizeFrameId(to_frame_id),
                              sanitizeFrameId(from_frame_id), frame_timestamp);
}

std::optional<Transformation3D> TfTransformer::lookupLatestTransform(
    const std::string& to_frame_id, const std::string& from_frame_id) {
  return lookupTransformImpl(sanitizeFrameId(to_frame_id),
                             sanitizeFrameId(from_frame_id), {});
}

std::optional<Transformation3D> TfTransformer::lookupTransform(
    const std::string& to_frame_id, const std::string& from_frame_id,
    const rclcpp::Time& frame_timestamp) {
  return lookupTransformImpl(sanitizeFrameId(to_frame_id),
                             sanitizeFrameId(from_frame_id), frame_timestamp);
}

std::string TfTransformer::sanitizeFrameId(const std::string& string) {
  if (string[0] == '/') {
    return string.substr(1, string.length());
  } else {
    return string;
  }
}

bool TfTransformer::waitForTransformImpl(
    const std::string& to_frame_id, const std::string& from_frame_id,
    const rclcpp::Time& frame_timestamp) const {
  // Total time spent waiting for the updated pose
  rclcpp::Duration t_waited = rclcpp::Duration::from_seconds(0.0);
  while (t_waited < transform_lookup_max_time_) {
    if (tf_buffer_.canTransform(to_frame_id, from_frame_id, frame_timestamp)) {
      return true;
    }
    auto duration_chrono = transform_lookup_retry_period_.to_chrono<std::chrono::nanoseconds>();
    rclcpp::sleep_for(duration_chrono);
    // transform_lookup_retry_period_.sleep_for();
    t_waited = t_waited + transform_lookup_retry_period_;
  }
  // ROS_WARN(
  //     "Waited %.3fs, but still could not get the TF from %s to %s at timestamp "
  //     "%u seconds",
  //     t_waited.toSec(), from_frame_id.c_str(), to_frame_id.c_str(),
  //     frame_timestamp.sec);
  return false;
}

std::optional<Transformation3D> TfTransformer::lookupTransformImpl(
    const std::string& to_frame_id, const std::string& from_frame_id,
    const rclcpp::Time&  frame_timestamp) {
  if (!isTransformAvailable(to_frame_id, from_frame_id, frame_timestamp)) {
    return std::nullopt;
  }
  geometry_msgs::msg::TransformStamped transform_msg =
      tf_buffer_.lookupTransform(to_frame_id, from_frame_id, frame_timestamp);
  return convert::transformMsgToTransformation3D(transform_msg.transform);
}
}  // namespace wavemap
