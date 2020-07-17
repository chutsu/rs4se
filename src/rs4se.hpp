#pragma once
#include <iostream>
#include <string.h>
#include <unistd.h>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

#define __FILENAME__                                                           \
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define FATAL(M, ...)                                                          \
  fprintf(stdout,                                                              \
          "\033[31m[FATAL] [%s:%d] " M "\033[0m\n",                            \
          __FILENAME__,                                                        \
          __LINE__,                                                            \
          ##__VA_ARGS__);                                                      \
  exit(-1)

#define LOG_WARN(M, ...)                                                       \
  fprintf(stdout, "\033[33m[WARN] " M "\033[0m\n", ##__VA_ARGS__)

#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

#define ROS_PARAM(NH, X, Y)                                                    \
  if (NH.getParam(X, Y) == false) {                                            \
    std::cerr << "Failed to get ROS param [" << X << "]!" << std::endl;        \
    exit(-1);                                                                  \
  }

#define ROS_OPTIONAL_PARAM(NH, X, Y, DEFAULT_VALUE)                            \
  if (NH.getParam(X, Y) == false) { Y = DEFAULT_VALUE; }

void print_rsframe_timestamps(const rs2::frame &frame) {
  const auto ts_ms = frame.get_timestamp();
  const auto frame_ts_meta_key = RS2_FRAME_METADATA_FRAME_TIMESTAMP;
  const auto frame_ts_us = frame.get_frame_metadata(frame_ts_meta_key);
  const auto sensor_ts_meta_key = RS2_FRAME_METADATA_SENSOR_TIMESTAMP;
  const auto sensor_ts_us = frame.get_frame_metadata(sensor_ts_meta_key);

  printf("RS2_FRAME_METADATA_FRAME_TIMESTAMP [us]:  %lld\n", frame_ts_us);
  printf("RS2_FRAME_METADATA_SENSOR_TIMESTAMP [us]: %lld\n", sensor_ts_us);
  printf("RS2_FRAME_METADATA_FRAME_TIMESTAMP [s]:   %.6f\n",
         frame_ts_us * 1e-6);
  printf("RS2_FRAME_METADATA_SENSOR_TIMESTAMP [s]:  %.6f\n",
         sensor_ts_us * 1e-6);

  switch (frame.get_frame_timestamp_domain()) {
  case RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK:
    printf("Timestamp domain: hardware clock!\n");
    printf("Measured in relation to the camera clock\n");
    break;
  case RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME:
    printf("Timestamp domain: system time!\n");
    printf("Measured in relation to the OS system clock\n");
    break;
  case RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME:
    printf("Timestamp domain: global time!\n");
    printf("Measured in relation to the camera clock and converted to\n");
    printf("OS system clock by constantly measure the difference\n");
    break;
  case RS2_TIMESTAMP_DOMAIN_COUNT:
    printf("Timestamp domain: count! Not a valid domain\n");
    printf("Not a valid input: intended to be used in for-loops!\n");
    break;
  default: printf("Not a valid time domain!\n"); break;
  }
}

static inline uint64_t str2ts(const std::string &s) {
  uint64_t ts = 0;
  size_t end = s.length() - 1;

  int idx = 0;
  for (int i = 0; i <= end; i++) {
    const char c = s.at(end - i);

    if (c != '.') {
      const uint64_t base = static_cast<uint64_t>(pow(10, idx));
      ts += std::atoi(&c) * base;
      idx++;
    }
  }

  return ts;
}

static cv::Mat frame2cvmat(const rs2::frame &frame,
                           const int width,
                           const int height,
                           const int format) {
  const cv::Size size(width, height);
  const auto stride = cv::Mat::AUTO_STEP;
  const cv::Mat cv_frame(size, format, (void *) frame.get_data(), stride);
  return cv_frame;
}

static uint64_t vframe2ts(const rs2::video_frame &vf) {
  // Calculate half of the exposure time
  // -- Frame metadata timestamp
  const auto frame_meta_key = RS2_FRAME_METADATA_FRAME_TIMESTAMP;
  const auto frame_ts_us = vf.get_frame_metadata(frame_meta_key);
  const auto frame_ts_ns = static_cast<uint64_t>(frame_ts_us) * 1000;
  // -- Sensor metadata timestamp
  const auto sensor_meta_key = RS2_FRAME_METADATA_SENSOR_TIMESTAMP;
  const auto sensor_ts_us = vf.get_frame_metadata(sensor_meta_key);
  const auto sensor_ts_ns = static_cast<uint64_t>(sensor_ts_us) * 1000;
  // -- Half exposure time
  const auto half_exposure_time_ns = frame_ts_ns - sensor_ts_ns;

  // Calculate corrected timestamp
  const auto ts_ms = vf.get_timestamp();
  const auto ts_ns = str2ts(std::to_string(ts_ms));
  const auto ts_corrected_ns = ts_ns - half_exposure_time_ns;

  return static_cast<uint64_t>(ts_corrected_ns);
}

static void debug_imshow(const cv::Mat &frame_left,
                         const cv::Mat &frame_right) {
  // Display in a GUI
  cv::Mat frame;
  cv::hconcat(frame_left, frame_right, frame);
  cv::namedWindow("Stereo Module", cv::WINDOW_AUTOSIZE);
  cv::imshow("Stereo Module", frame);

  if (cv::waitKey(1) == 'q') { exit(-1); }
}

template <typename T>
T lerp(const T &a, const T &b, const double t) {
  return a * (1.0 - t) + b * t;
}

struct lerp_buf_t {
  std::deque<std::string> buf_type_;
  std::deque<double> buf_ts_;
  std::deque<Eigen::Vector3d> buf_data_;

  std::deque<double> lerped_gyro_ts_;
  std::deque<Eigen::Vector3d> lerped_gyro_data_;
  std::deque<double> lerped_accel_ts_;
  std::deque<Eigen::Vector3d> lerped_accel_data_;

  lerp_buf_t() {}

  bool ready() {
    if (buf_ts_.size() >= 3 && buf_type_.back() == "A") { return true; }
    return false;
  }

  void addAccel(const double ts_s,
                const double ax,
                const double ay,
                const double az) {
    buf_type_.push_back("A");
    buf_ts_.push_back(ts_s);
    buf_data_.emplace_back(ax, ay, az);
  }

  void addGyro(const double ts_s,
               const double wx,
               const double wy,
               const double wz) {
    if (buf_type_.size() && buf_type_.front() == "A") {
      buf_type_.push_back("G");
      buf_ts_.push_back(ts_s);
      buf_data_.emplace_back(wx, wy, wz);
    }
  }

  void print() {
    for (size_t i = 0; i < buf_ts_.size(); i++) {
      const double ts = buf_ts_.at(i);
      const std::string dtype = buf_type_.at(i);
      const Eigen::Vector3d data = buf_data_.at(i);
      const double x = data(0);
      const double y = data(1);
      const double z = data(1);
      printf("[%.6f] - [%s] - (%.2f, %.2f, %.2f)\n",
             ts,
             dtype.c_str(),
             x,
             y,
             z);
    }
  }

  void interpolate() {
    // Lerp data
    double t0 = 0;
    Eigen::Vector3d d0;
    double t1 = 0;
    Eigen::Vector3d d1;
    bool t0_set = false;

    std::deque<double> lerp_ts;
    std::deque<Eigen::Vector3d> lerp_data;

    double ts = 0.0;
    std::string dtype;
    Eigen::Vector3d data;

    while (buf_ts_.size()) {
      // Timestamp
      ts = buf_ts_.front();
      buf_ts_.pop_front();

      // Datatype
      dtype = buf_type_.front();
      buf_type_.pop_front();

      // Data
      data = buf_data_.front();
      buf_data_.pop_front();

      // Lerp
      if (t0_set == false && dtype == "A") {
        t0 = ts;
        d0 = data;
        t0_set = true;

      } else if (t0_set && dtype == "A") {
        t1 = ts;
        d1 = data;

        while (lerp_ts.size()) {
          const double lts = lerp_ts.front();
          const Eigen::Vector3d ldata = lerp_data.front();
          const double dt = t1 - t0;
          const double alpha = (lts - t0) / dt;

          lerped_accel_ts_.push_back(lts);
          lerped_accel_data_.push_back(lerp(d0, d1, alpha));

          lerped_gyro_ts_.push_back(lts);
          lerped_gyro_data_.push_back(ldata);

          lerp_ts.pop_front();
          lerp_data.pop_front();
        }

        t0 = t1;
        d0 = d1;

      } else if (t0_set && ts >= t0 && dtype == "G") {
        lerp_ts.push_back(ts);
        lerp_data.push_back(data);
      }
    }

    buf_ts_.push_back(ts);
    buf_type_.push_back(dtype);
    buf_data_.push_back(data);
  }

  void clear() {
    lerped_gyro_ts_.clear();
    lerped_gyro_data_.clear();
    lerped_accel_ts_.clear();
    lerped_accel_data_.clear();
  }
};

rs2::device rs2_connect() {
  rs2::context ctx;
  rs2::device_list devices = ctx.query_devices();
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
    // Connect to stereo module
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

    // Connect to RGB module
    // clang-format off
    {
      if (rs2_get_sensor(device_, "RGB Camera", rgb_sensor_) != 0) {
        FATAL("This RealSense device does not have a [Stereo Module]");
      }
      rgb_sensor_.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, config_.global_time);
      rgb_sensor_.set_option(RS2_OPTION_EXPOSURE, config_.rgb_exposure);
    }
    // clang-format on

    // Configure
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

struct intel_d435i_t {
  bool is_running_ = false;
  bool configured_ = false;

  const rs2::device &device_;
  rs2::sensor motion_;
  rs2::sensor rgb_;
  rs2::sensor stereo_;
  rs2::pipeline pipe_;
  rs_rgbd_module_config_t rgbd_config_;
  rs_motion_module_config_t motion_config_;

  intel_d435i_t(const rs2::device &device,
                const rs_rgbd_module_config_t &rgbd_config,
                const rs_motion_module_config_t &motion_config,
                const std::function<void(const rs2::frame &frame)> cb)
      : device_{device}, rgbd_config_{rgbd_config}, motion_config_{
                                                        motion_config} {
    // Configure
    rs2::config cfg;

    // ------------------------------ Motion -----------------------------------
    // clang-format off
    if (motion_config_.enable_motion) {
      if (rs2_get_sensor(device_, "Motion Module", motion_) != 0) {
        FATAL("This RealSense device does not have a [Motion Module]");
      }
      motion_.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, motion_config_.global_time);
      cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, motion_config_.accel_hz);
      cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, motion_config_.gyro_hz);
      configured_ = true;
    }
    // clang-format on

    // -------------------------------- RGB ------------------------------------
    // clang-format off
    if (rgbd_config_.enable_rgb) {
      if (rs2_get_sensor(device_, "RGB Camera", rgb_) != 0) {
        FATAL("This RealSense device does not have a [Stereo Module]");
      }
      rgb_.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, rgbd_config_.global_time);
			rgb_.set_option(RS2_OPTION_EXPOSURE, rgbd_config_.rgb_exposure);
			cfg.enable_stream(RS2_STREAM_COLOR,
                        rgbd_config.rgb_width,
                        rgbd_config.rgb_height,
                        rs2_format_convert(rgbd_config.rgb_format),
                        rgbd_config.rgb_frame_rate);
      configured_ = true;
    }
    // clang-format on

    // ------------------------------- STEREO ----------------------------------
    // clang-format off
    if (rgbd_config.enable_ir) {
      if (rs2_get_sensor(device_, "Stereo Module", stereo_) != 0) {
        FATAL("This RealSense device does not have a [Stereo Module]");
      }

      if (rgbd_config_.enable_depth == true && rgbd_config_.enable_emitter == false) {
        LOG_WARN("IR emitter is not enabled!");
      }
      stereo_.set_option(RS2_OPTION_EMITTER_ENABLED, rgbd_config_.enable_emitter);
      stereo_.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, rgbd_config_.global_time);
			stereo_.set_option(RS2_OPTION_EXPOSURE, rgbd_config_.ir_exposure);

      // IMPORTANT: Indexing for stereo camera starts from 1 not 0.
      cfg.enable_stream(RS2_STREAM_INFRARED,
                        1,
                        rgbd_config.ir_width,
                        rgbd_config.ir_height,
                        rs2_format_convert(rgbd_config.ir_format),
                        rgbd_config.ir_frame_rate);
      cfg.enable_stream(RS2_STREAM_INFRARED,
                        2,
                        rgbd_config.ir_width,
                        rgbd_config.ir_height,
                        rs2_format_convert(rgbd_config.ir_format),
                        rgbd_config.ir_frame_rate);
      configured_ = true;
    }
    // clang-format on

    // ------------------------------- DEPTH -----------------------------------
    // clang-format off
    if (rgbd_config.enable_depth) {
      cfg.enable_stream(RS2_STREAM_DEPTH,
                        rgbd_config.depth_width,
                        rgbd_config.depth_height,
                        rs2_format_convert(rgbd_config.depth_format),
                        rgbd_config.depth_frame_rate);
      configured_ = true;
    }
    // clang-format on

    if (configured_) {
      pipe_.start(cfg, cb);
      is_running_ = true;
    }
  }

  ~intel_d435i_t() {
    if (is_running_) {
      pipe_.stop();
      is_running_ = false;
    }
  }
};
