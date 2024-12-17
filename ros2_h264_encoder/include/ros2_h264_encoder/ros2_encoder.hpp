#ifndef ROS2_ENCODER_H
#define ROS2_ENCODER_H
// System Includes
#include <chrono>
#include <string>
#include <unordered_map>

// External ROS Includes
#include <rclcpp/rclcpp.hpp>

#include "h264_msgs/msg/packet.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

// External Includes
extern "C" {
#include <libswscale/swscale.h>
#include <x264.h>
}

#define MAX_FRAME_RATE_SAMPLES 30

class ROS2Encoder {
 public:
  ROS2Encoder(rclcpp::Logger logger);

  ~ROS2Encoder();
  bool encode_image(const sensor_msgs::msg::Image& msg,
                    h264_msgs::msg::Packet& packet);
  bool convert_image_to_h264(const sensor_msgs::msg::Image& msg,
                             x264_picture_t* out);

  constexpr int get_seq() const { return frame_sequence_; }

  using ClockBase = std::chrono::high_resolution_clock;
  template <typename T, typename _Per = std::ratio<1>>
  using Duration = std::chrono::duration<T,_Per>;

  template <typename T>
  using DurationUs = Duration<T, std::micro>;
  template <typename T>
  using DurationMs = Duration<T, std::milli>;

 private:
  // Encoder variables
  x264_t* encoder_handle_;
  x264_param_t encoder_params_;

  x264_picture_t input;
  x264_picture_t output;
  x264_nal_t* nals_;
  int i_nals_;
  int64_t pts_;

  int frame_sequence_;

  // flag to indicate sampling of video stream
  bool is_sampling_;
  ClockBase::time_point last_frame_time_;
  DurationUs<double> frame_time_delta_;

  // Conversion
  struct SwsContext* p_conversion_context_;

  static std::unordered_map<std::string, AVPixelFormat>
  create_ROS_encoding_to_AV_Pixel_format() {
    std::unordered_map<std::string, AVPixelFormat> formats;
    formats[sensor_msgs::image_encodings::BGR16] = AV_PIX_FMT_BGR48;
    formats[sensor_msgs::image_encodings::BGR8] = AV_PIX_FMT_BGR24;
    formats[sensor_msgs::image_encodings::BGRA16] = AV_PIX_FMT_BGRA64;
    formats[sensor_msgs::image_encodings::BGRA8] = AV_PIX_FMT_BGRA;
    formats[sensor_msgs::image_encodings::MONO16] = AV_PIX_FMT_GRAY16;
    formats[sensor_msgs::image_encodings::MONO8] = AV_PIX_FMT_GRAY8;
    formats[sensor_msgs::image_encodings::RGB16] = AV_PIX_FMT_RGB48;
    formats[sensor_msgs::image_encodings::RGB8] = AV_PIX_FMT_RGB24;
    formats[sensor_msgs::image_encodings::RGBA16] = AV_PIX_FMT_RGBA64;
    formats[sensor_msgs::image_encodings::RGBA8] = AV_PIX_FMT_RGBA;
    return formats;
  }

  static const std::unordered_map<std::string, AVPixelFormat>
      ROS_encoding_to_AV_Pixel_format;

  // ROS
  rclcpp::Logger logger_;
};

#endif
