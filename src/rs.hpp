#pragma once
#include <iostream>
#include <string.h>

#include <librealsense2/rs.hpp>

#include "common.hpp"

static void print_rsframe_timestamps(const rs2::frame &frame) {
  const auto ts_ms = frame.get_timestamp();
  const auto frame_ts_meta_key = RS2_FRAME_METADATA_FRAME_TIMESTAMP;
  const auto frame_ts_us = frame.get_frame_metadata(frame_ts_meta_key);
  const auto sensor_ts_meta_key = RS2_FRAME_METADATA_SENSOR_TIMESTAMP;
  const auto sensor_ts_us = frame.get_frame_metadata(sensor_ts_meta_key);

  printf("metadata frame timestamp [s]: %.6f\n", frame_ts_us * 1e-6);
  printf("metadata frame timestamp [us]: %lld\n", frame_ts_us);
  printf("metadata sensor timestamp [s]: %.6f\n", sensor_ts_us * 1e-6);
  printf("metadata sensor timestamp [us]: %lld\n", sensor_ts_us);

  switch (frame.get_frame_timestamp_domain()) {
  case RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK:
    printf("timestamp domain: hardware clock!\n");
    break;
  case RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME:
    printf("timestamp domain: system time!\n");
    break;
  case RS2_TIMESTAMP_DOMAIN_COUNT: printf("Not a valid input!\n"); break;
  default: printf("Not a valid time domain!\n"); break;
  }
}

rs2::device rs2_connect() {
  rs2::context ctx;
  rs2::device_list devices = ctx.query_devices();
  rs2::device device;
  if (devices.size() == 0) {
    FATAL("No device connected, please connect a RealSense device");
  }

  return devices[0];
}

void rs2_list_sensors(const int device_idx = 0) {
  // Connect to the device
  rs2::context ctx;
  rs2::device_list devices = ctx.query_devices();
  rs2::device device;
  if (devices.size() == 0) {
    FATAL("No device connected, please connect a RealSense device");
  } else {
    device = devices[device_idx]; // Default to first device
  }

  printf("Sensors:\n");
  for (const auto &query_sensor : device.query_sensors()) {
    const auto sensor_name = query_sensor.get_info(RS2_CAMERA_INFO_NAME);
    printf("  - %s\n", sensor_name);
  }
}

int rs2_get_sensor(const rs2::device &device,
                   const std::string &target,
                   rs2::sensor &sensor) {
  for (const auto &query_sensor : device.query_sensors()) {
    const auto sensor_name = query_sensor.get_info(RS2_CAMERA_INFO_NAME);
    if (strcmp(sensor_name, target.c_str()) == 0) {
      sensor = query_sensor;
      return 0;
    }
  }

  return -1;
}

rs2_format rs2_format_convert(const std::string &format) {
  if (format == "ANY") return RS2_FORMAT_ANY;
  if (format == "Z16") return RS2_FORMAT_Z16;
  if (format == "DISPARITY16") return RS2_FORMAT_DISPARITY16;
  if (format == "XYZ32F") return RS2_FORMAT_XYZ32F;
  if (format == "YUYV") return RS2_FORMAT_YUYV;
  if (format == "RGB8") return RS2_FORMAT_RGB8;
  if (format == "BGR8") return RS2_FORMAT_BGR8;
  if (format == "RGBA8") return RS2_FORMAT_RGBA8;
  if (format == "BGRA8") return RS2_FORMAT_BGRA8;
  if (format == "Y8") return RS2_FORMAT_Y8;
  if (format == "Y16") return RS2_FORMAT_Y16;
  if (format == "RAW10") return RS2_FORMAT_RAW10;
  if (format == "RAW16") return RS2_FORMAT_RAW16;
  if (format == "RAW8") return RS2_FORMAT_RAW8;
  if (format == "UYVY") return RS2_FORMAT_UYVY;
  if (format == "MOTION_RAW") return RS2_FORMAT_MOTION_RAW;
  if (format == "MOTION_XYZ32F") return RS2_FORMAT_MOTION_XYZ32F;
  if (format == "GPIO_RAW") return RS2_FORMAT_GPIO_RAW;
  if (format == "6DOF") return RS2_FORMAT_6DOF;
  if (format == "DISPARITY32") return RS2_FORMAT_DISPARITY32;
  if (format == "Y10BPACK") return RS2_FORMAT_Y10BPACK;
  if (format == "DISTANCE") return RS2_FORMAT_DISTANCE;
  if (format == "MJPEG") return RS2_FORMAT_MJPEG;
  if (format == "COUNT") return RS2_FORMAT_COUNT;

  FATAL("Opps! Unsupported format [%s]!", format.c_str());
}

struct rs_motion_module_config_t {
  bool global_time = true;
  bool enable_motion = true;
  int accel_hz = 250;
  int gyro_hz = 400;
};

struct rs_motion_module_t {
  bool is_running_ = false;

  const rs2::device &device_;
  rs2::sensor sensor_;
  rs2::pipeline pipe_;
  rs_motion_module_config_t config_;

  rs_motion_module_t(const rs2::device &device,
                     const rs_motion_module_config_t &config,
                     const std::function<void(const rs2::frame &frame)> cb)
      : device_{device}, config_{config} {
    // Connect to sensor
    if (rs2_get_sensor(device_, "Motion Module", sensor_) != 0) {
      FATAL("This RealSense device does not have a [Motion Module]");
    }

    // Enable global time
    sensor_.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, config_.global_time);

    // Configure and start pipeline
    if (config_.enable_motion) {
      rs2::config cfg;
      cfg.enable_stream(RS2_STREAM_ACCEL,
                        RS2_FORMAT_MOTION_XYZ32F,
                        config_.accel_hz);
      cfg.enable_stream(RS2_STREAM_GYRO,
                        RS2_FORMAT_MOTION_XYZ32F,
                        config_.gyro_hz);
      pipe_.start(cfg, cb);
      is_running_ = true;
    }
  }

  ~rs_motion_module_t() {
    if (is_running_) {
      pipe_.stop();
      is_running_ = false;
    }
  }
};

struct rs_rgbd_module_config_t {
  bool global_time = true;
  int sync_size = 300;
  bool enable_rgb = true;
  bool enable_ir = true;
  bool enable_depth = true;
  bool enable_emitter = true;

  int rgb_width = 640;
  int rgb_height = 480;
  std::string rgb_format = "BGR8";
  int rgb_frame_rate = 30;
  double rgb_exposure = 100.0;

  int ir_width = 640;
  int ir_height = 480;
  std::string ir_format = "Y8";
  int ir_frame_rate = 30;
  double ir_exposure = 10000.0;

  int depth_width = 640;
  int depth_height = 480;
  std::string depth_format = "Z16";
  int depth_frame_rate = 30;
};

struct rs_rgbd_module_t {
  bool is_running_ = false;
  bool configured_ = false;

  const rs2::device &device_;
  rs2::sensor rgb_sensor_;
  rs2::sensor stereo_sensor_;
  rs2::pipeline pipe_;
  rs_rgbd_module_config_t config_;

  rs_rgbd_module_t(const rs2::device &device,
                   const rs_rgbd_module_config_t &config,
                   const std::function<void(const rs2::frame &frame)> cb)
      : device_{device}, config_{config} {
    // Connect and configure stereo module
    // clang-format off
    {
      if (rs2_get_sensor(device_, "Stereo Module", stereo_sensor_) != 0) {
        FATAL("This RealSense device does not have a [Stereo Module]");
      }

      if (config_.enable_depth == true && config_.enable_emitter == false) {
        LOG_WARN("IR emitter is not enabled!");
      }
      stereo_sensor_.set_option(RS2_OPTION_EMITTER_ENABLED, config_.enable_emitter);
      stereo_sensor_.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, config_.global_time);
      stereo_sensor_.set_option(RS2_OPTION_EXPOSURE, config_.ir_exposure);
    }
    // clang-format on

    // Connect and configure RGB module
    // clang-format off
    {
      if (rs2_get_sensor(device_, "RGB Camera", rgb_sensor_) != 0) {
        FATAL("This RealSense device does not have a [Stereo Module]");
      }
      rgb_sensor_.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, config_.global_time);
      rgb_sensor_.set_option(RS2_OPTION_EXPOSURE, config_.rgb_exposure);
    }
    // clang-format on

    // Configure and start pipeline
    rs2::config cfg;
    // -- RGB
    if (config_.enable_rgb) {
      cfg.enable_stream(RS2_STREAM_COLOR,
                        config_.rgb_width,
                        config_.rgb_height,
                        rs2_format_convert(config_.rgb_format),
                        config_.rgb_frame_rate);
      configured_ = true;
    }
    // -- Stereo infrared cameras
    // IMPORTANT: Indexing for stereo camera starts from 1 not 0.
    if (config_.enable_ir) {
      cfg.enable_stream(RS2_STREAM_INFRARED,
                        1,
                        config_.ir_width,
                        config_.ir_height,
                        rs2_format_convert(config_.ir_format),
                        config_.ir_frame_rate);
      cfg.enable_stream(RS2_STREAM_INFRARED,
                        2,
                        config_.ir_width,
                        config_.ir_height,
                        rs2_format_convert(config_.ir_format),
                        config_.ir_frame_rate);
      configured_ = true;
    }
    // -- Depth sensor
    if (config_.enable_depth) {
      cfg.enable_stream(RS2_STREAM_DEPTH,
                        config_.depth_width,
                        config_.depth_height,
                        rs2_format_convert(config_.depth_format),
                        config_.depth_frame_rate);
      configured_ = true;
    }

    if (configured_) {
      pipe_.start(cfg, cb);
      is_running_ = true;
    }
  }

  ~rs_rgbd_module_t() {
    if (is_running_) {
      pipe_.stop();
      is_running_ = false;
    }
  }
};
