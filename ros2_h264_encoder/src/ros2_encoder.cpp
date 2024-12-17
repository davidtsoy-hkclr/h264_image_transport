// System Includes
#include <iostream>

// Project Includes
#include "ros2_h264_encoder/ros2_encoder.hpp"

const std::unordered_map<std::string, AVPixelFormat>
    ROS2Encoder::ROS_encoding_to_AV_Pixel_format =
        ROS2Encoder::create_ROS_encoding_to_AV_Pixel_format();

ROS2Encoder::ROS2Encoder(rclcpp::Logger logger)
    : pts_(0), frame_sequence_(0), is_sampling_(true)
    , last_frame_time_(), frame_time_delta_(0.0),logger_(logger) {
  x264_param_default_preset(&encoder_params_, "veryfast", "zerolatency");
  encoder_params_.b_repeat_headers = 1;
  encoder_params_.i_threads = 1;
  encoder_params_.i_fps_den = 1;
}

ROS2Encoder::~ROS2Encoder() {
  x264_picture_clean(&input);
  x264_encoder_close(encoder_handle_);
  sws_freeContext(p_conversion_context_);
}

bool ROS2Encoder::encode_image(const sensor_msgs::msg::Image &msg,
                               h264_msgs::msg::Packet &packet) {
  if (is_sampling_) {
    if (frame_sequence_ == 0) {
      last_frame_time_ = ClockBase::now();
    }
    frame_time_delta_ = (frame_time_delta_ + (ClockBase::now() - last_frame_time_)) / 2.0;
    last_frame_time_ = ClockBase::now();
    frame_sequence_++;

    if (frame_sequence_ >= MAX_FRAME_RATE_SAMPLES) {
      auto fps = 1.0 / Duration<double>(frame_time_delta_).count();
      RCLCPP_INFO_STREAM(logger_, "Avg Frame Time:" << frame_time_delta_.count() << ", " << "FPS: " << fps);
      // change encoder FPS to sampled interval
      this->encoder_params_.i_fps_num = static_cast<uint32_t>(std::round(fps));
      this->encoder_params_.i_keyint_max = 2 * static_cast<uint32_t>(std::round(fps));
      is_sampling_ = false;
    } else {
      return false;
    }
  }

  if (!encoder_handle_) {
    RCLCPP_INFO_STREAM(logger_, "Creating x264 Encoder with Image: "
                                    << msg.width << "x" << msg.height << ", "
                                    << "Encoding: " << msg.encoding << ", "
                                    << encoder_params_.i_fps_num << " FPS");

    encoder_params_.i_width = msg.width;
    encoder_params_.i_height = msg.height;

    encoder_handle_ = x264_encoder_open(&encoder_params_);
    if (!encoder_handle_) {
      RCLCPP_WARN_STREAM(logger_, "Could not open encoder!");
    }

    if (x264_picture_alloc(&input, X264_CSP_I420, encoder_params_.i_width,
                           encoder_params_.i_height)) {
      RCLCPP_WARN_STREAM(logger_, "Cannot allocate x264 picture");
    }

    p_conversion_context_ = sws_getContext(
        msg.width, msg.height, ROS_encoding_to_AV_Pixel_format.at(msg.encoding),
        msg.width, msg.height, AV_PIX_FMT_YUV420P, SWS_FAST_BILINEAR, NULL,
        NULL, NULL);
    if (!p_conversion_context_) {
      RCLCPP_WARN_STREAM(logger_, "Failed to get conversion context!");
    }
    RCLCPP_INFO_STREAM(logger_, "Successfully created X264 Encoder with Image: "
                                    << msg.width << "x" << msg.height << ", "
                                    << "Encoding:" << msg.encoding << ", "
                                    << encoder_params_.i_fps_num << " FPS");
  }
  frame_sequence_++;
  if (convert_image_to_h264(msg, &input)) {
    int frame_size =
        x264_encoder_encode(encoder_handle_, &nals_, &i_nals_, &input, &output);
    packet.seq = pts_;
    pts_++;
    if (frame_size >= 0) {
      for (int i = 0; i < i_nals_; i++) {
        std::copy(nals_[i].p_payload, nals_[i].p_payload + nals_[i].i_payload,
                  std::back_inserter(packet.data));
      }
      return true;
    } else {
      RCLCPP_WARN_STREAM(logger_, "Could not encode image!");
    }
  }
  return false;
}

bool ROS2Encoder::convert_image_to_h264(const sensor_msgs::msg::Image &msg,
                                        x264_picture_t *out) {
  int stride[3] = {static_cast<int>(msg.width) * 3, 0, 0};
  uint8_t *src[3] = {const_cast<uint8_t *>(&msg.data[0]), NULL, NULL};
  int returnedHeight =
      sws_scale(p_conversion_context_, src, stride, 0, encoder_params_.i_height,
                out->img.plane, out->img.i_stride);
  out->i_pts = pts_;
  return returnedHeight == encoder_params_.i_height;
}