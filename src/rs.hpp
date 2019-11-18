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

  // switch (frame.get_frame_timestamp_domain()) {
  // case RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK:
  //   printf("timestamp domain: hardware clock!\n");
  //   break;
  // case RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME:
  //   printf("timestamp domain: system time!\n");
  //   break;
  // case RS2_TIMESTAMP_DOMAIN_COUNT:
  //   printf("Not a valid input!\n");
  //   break;
  // default:
  //   printf("Not a valid time domain!\n");
  //   break;
  // }
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

int rs2_get_sensors(const rs2::device &device, const std::string &target,
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

struct rs_motion_module_config_t {
  bool global_time = true;
  int accel_hz = 250;
  int gyro_hz = 400;
  unsigned int fq_size = 10;
};

class rs_motion_module_t {
  const rs2::device &device_;
  rs2::frame_queue fq_;
  rs2::sensor sensor_;
  rs2::stream_profile accel_profile_;
  rs2::stream_profile gyro_profile_;
  rs_motion_module_config_t config_;

public:
  rs_motion_module_t(const rs2::device &device)
      : device_{device}, fq_{config_.fq_size} {
    setup(config_.accel_hz, config_.gyro_hz);
  }

  rs_motion_module_t(const rs2::device &device,
                     const rs_motion_module_config_t &config)
      : device_{device}, fq_{config.fq_size}, config_{config} {
    setup(config_.accel_hz, config_.gyro_hz);
  }

  ~rs_motion_module_t() {
    sensor_.stop();
    sensor_.close();
  }

  void listStreamProfiles() {
    // Go through Stream profiles
    std::cout << "Motion module stream profiles:" << std::endl;
    const auto stream_profiles = sensor_.get_stream_profiles();
    for (const auto &stream_profile : stream_profiles) {
      const auto stream_name = stream_profile.stream_name();
      const auto stream_rate = stream_profile.fps();
      std::cout << " - " << stream_name << " " << stream_rate << " hz ";
      std::cout << std::endl;
    }
  }

  void setStreamProfiles(const int accel_hz, const int gyro_hz) {
    bool accel_ok = false;
    bool gyro_ok = false;
    rs2_list_sensors();
    // Go through Stream profiles
    const auto stream_profiles = sensor_.get_stream_profiles();
    for (const auto &stream_profile : stream_profiles) {
      const auto stream_name = stream_profile.stream_name();
      const auto stream_rate = stream_profile.fps();

      if (stream_name == "Accel" && stream_rate == accel_hz) {
        accel_profile_ = stream_profile;
        accel_ok = true;
      }

      if (stream_name == "Gyro" && stream_rate == gyro_hz) {
        gyro_profile_ = stream_profile;
        gyro_ok = true;
      }
    }

    // Check results
    if (accel_ok == false) {
      FATAL("Failed to get accel stream profile at %d Hz", accel_hz);
    }
    if (gyro_ok == false) {
      FATAL("Failed to get gyr stream profile at %d Hz", gyro_hz);
    }
  }

  void setup(const int accel_hz = 250, const int gyro_hz = 400) {
    if (rs2_get_sensors(device_, "Motion Module", sensor_) != 0) {
      FATAL("This RealSense device does not have a [Motion Module]");
    }
    setStreamProfiles(accel_hz, gyro_hz);

    // Enable global time
    sensor_.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, config_.global_time);

    // Start sensor
    sensor_.open({accel_profile_, gyro_profile_});
    sensor_.start(fq_);
  }

  rs2::frame waitForFrame() { return fq_.wait_for_frame(); }
};

struct rs_stereo_module_config_t {
  bool global_time = true;
  int sync_size = 30;
  bool enable_emitter = false;

  int frame_rate = 30;
  std::string format = "Y8";
  int width = 640;
  int height = 480;
  double exposure = 0.0f;
};

class rs_stereo_module_t {
  const rs2::device &device_;
  rs2::syncer sync_;
  rs2::sensor sensor_;
  rs2::stream_profile profile1_;
  rs2::stream_profile profile2_;
  bool profile1_set_ = false;
  bool profile2_set_ = false;
  rs_stereo_module_config_t config_;

public:
  rs_stereo_module_t(const rs2::device &device)
      : device_{device}, sync_{config_.sync_size} {
    setup();
  }

  rs_stereo_module_t(const rs2::device &device,
                     const rs_stereo_module_config_t &config)
      : device_{device}, sync_{config.sync_size}, config_{config} {
    setup();
  }

  ~rs_stereo_module_t() {
    sensor_.stop();
    sensor_.close();
  }

  void listStreamProfiles() {
    // Go through Stream profiles
    std::cout << "Stereo module stream profiles:" << std::endl;
    const auto stream_profiles = sensor_.get_stream_profiles();
    for (const auto &stream_profile : stream_profiles) {
      const auto stream_name = stream_profile.stream_name();
      const auto stream_rate = stream_profile.fps();
      const auto format = rs2_format_to_string(stream_profile.format());
      const auto vp = stream_profile.as<rs2::video_stream_profile>();
      printf("- %s [%d hz] [%s] [%dx%d] \n", stream_name.c_str(), stream_rate,
             format, vp.width(), vp.height());
    }
  }

  void setStreamProfile() {
    const auto stream_profiles = sensor_.get_stream_profiles();
    //listStreamProfiles();
    //printf("************************************************************* \n");
    //printf("- [%d hz] [%s] [%dx%d] \n", config_.frame_rate,
    //         config_.format.c_str(), config_.width, config_.height);
    //printf("************************************************************* \n");
    for (const auto &stream_profile : stream_profiles) {
      const auto name = stream_profile.stream_name();
      //printf("stream profile: %s \n", name.c_str());
      const auto rate = stream_profile.fps();
      const auto format = rs2_format_to_string(stream_profile.format());
      const auto vp = stream_profile.as<rs2::video_stream_profile>();
      const int width = vp.width();
      const int height = vp.height();
      //printf("- %s [%d hz] [%s] [%dx%d] \n", name.c_str(), rate,
      //       format, width, height);
      const bool rate_ok = (config_.frame_rate == rate);
      const bool format_ok = (config_.format == format);
      const bool width_ok = (config_.width == width);
      const bool height_ok = (config_.height == height);
      const bool res_ok = (width_ok && height_ok);

      if (rate_ok && format_ok && res_ok) {
        if ("Infrared 1" == name) {
          profile1_ = stream_profile;
          profile1_set_ = true;
        } else if ("Infrared 2" == name) {
          profile2_ = stream_profile;
          profile2_set_ = true;
        }
      }
    }

    if (profile1_set_ == false && profile2_set_ == false) {
      FATAL("Failed to get stereo module stream profile!");
    }
  }

  void setup() {
    if (rs2_get_sensors(device_, "Stereo Module", sensor_) != 0) {
      FATAL("This RealSense device does not have a [Stereo Module]");
    }
    setStreamProfile();

    // Switch off laser emitter
    sensor_.set_option(RS2_OPTION_EMITTER_ENABLED, config_.enable_emitter);

    // Enable global time
    sensor_.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, config_.global_time);

    // Enable global time
    sensor_.set_option(RS2_OPTION_EXPOSURE, config_.exposure);

    // Start sensor
    sensor_.open({profile1_, profile2_});
    sensor_.start(sync_);
  }

  rs2::frameset waitForFrame() { return sync_.wait_for_frames(10000); }
};

// added by Saeed Bastani
struct rs_camera_module_config_t {
  int sync_size = 30;
  bool global_time = true;
  bool enable_color = true;
  int color_fps = 30;
  //std::string color_format = "CV_8UC3";
  std::string color_format = "RGB8";
  int color_width = 640;
  int color_height = 480;
  unsigned int fq_size = 1;

};

class rs_camera_module_t {
  //bool global_time = true;
  rs2::syncer sync_;
  const rs2::device &device_;
  rs2::frame_queue fq_;
  rs2::sensor sensor_;
  rs2::stream_profile profile_;
  bool profile_set_ = false;

  rs_camera_module_config_t config_;

public:
  rs_camera_module_t(const rs2::device &device)
      : device_{device}, sync_{config_.sync_size} {
    setup();
  }

  rs_camera_module_t(const rs2::device &device,
                     const rs_camera_module_config_t &config)
      : device_{device}, sync_{config_.sync_size}, config_{config} {
    setup();
  }

  ~rs_camera_module_t() {
    sensor_.stop();
    sensor_.close();
  }

  void listStreamProfiles() {
    // Go through Stream profiles
    std::cout << "Camera module stream profiles:" << std::endl;
    const auto stream_profiles = sensor_.get_stream_profiles();
    for (const auto &stream_profile : stream_profiles) {
      const auto stream_name = stream_profile.stream_name();
      const auto stream_rate = stream_profile.fps();
      const auto format = rs2_format_to_string(stream_profile.format());
      const auto vp = stream_profile.as<rs2::video_stream_profile>();
      printf("- %s [%d hz] [%s] [%dx%d] \n", stream_name.c_str(), stream_rate,
             format, vp.width(), vp.height());
    }
  }

  void setStreamProfile() {
    const auto stream_profiles = sensor_.get_stream_profiles();
    listStreamProfiles();
    printf("************************************************************* \n");
    printf("- [%d hz] [%s] [%dx%d] \n", config_.color_fps,
             config_.color_format.c_str(), config_.color_width, config_.color_height);
    printf("************************************************************* \n");
    for (const auto &stream_profile : stream_profiles) {
      const auto name = stream_profile.stream_name();
      printf("stream profile: %s \n", name.c_str());
      const auto rate = stream_profile.fps();
      const auto format = rs2_format_to_string(stream_profile.format());
      const auto vp = stream_profile.as<rs2::video_stream_profile>();
      const int width = vp.width();
      const int height = vp.height();
      printf("- %s [%d hz] [%s] [%dx%d] \n", name.c_str(), rate,
             format, width, height);
      const bool rate_ok = (config_.color_fps == rate);
      const bool format_ok = (config_.color_format == format);
      const bool width_ok = (config_.color_width == width);
      const bool height_ok = (config_.color_height == height);
      const bool res_ok = (width_ok && height_ok);

      if (rate_ok && format_ok && res_ok) {
        if ("Color" == name) {
          profile_ = stream_profile;
          profile_set_ = true;
      }
    }
    }

    if (profile_set_ == false) {
      FATAL("Failed to get camera module stream profile!");
    }
  }

  void setup() {
    if (rs2_get_sensors(device_, "RGB Camera", sensor_) != 0) {
      FATAL("This RealSense device does not have a [Camera Module]");
    }
    setStreamProfile();

     
    //sensor_.set_option(RS2_OPTION_COLOR_ENABLED, config_.enable_color);
    // Enable global time
    sensor_.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, config_.global_time);


    // Start sensor
    sensor_.open({profile_});
    sensor_.start(sync_);
  }

  rs2::frameset waitForFrame() { return sync_.wait_for_frames(10000);  }
};
