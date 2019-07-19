#pragma once
#include <iostream>
#include <string.h>

#include <librealsense2/rs.hpp>

#include "common.hpp"


static void print_rsframe_timestamps(const rs2::frame &frame) {
  printf("frame timestamp: %.8f\n", frame.get_timestamp());
  const auto frame_ts = frame.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);
  printf("metadata frame timestamp: %lld\n", frame_ts);
  const auto sensor_ts = frame.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP);
  printf("metadata sensor timestamp: %lld\n", sensor_ts);

  switch (frame.get_frame_timestamp_domain()) {
    case RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK:
      printf("timestamp domain: hardware clock!\n");
      break;
    case RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME:
      printf("timestamp domain: system time!\n");
      break;
    case RS2_TIMESTAMP_DOMAIN_COUNT:
      printf("Not a valid input!\n");
      break;
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

void rs2_list_sensors() {
  // Connect to the device
  rs2::context ctx;
  rs2::device_list devices = ctx.query_devices();
  rs2::device device;
  if (devices.size() == 0) {
    FATAL("No device connected, please connect a RealSense device");
  } else {
    device = devices[0];
  }

  printf("Sensors:\n");
  for (const auto &query_sensor : device.query_sensors()) {
    const auto sensor_name = query_sensor.get_info(RS2_CAMERA_INFO_NAME);
    printf("  - %s\n", sensor_name);
  }
}

int rs2_get_sensors(const rs2::device &device,
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

struct rs_motion_module_t {
  const rs2::device &device_;
  rs2::frame_queue fq_;
  rs2::sensor sensor_;
  rs2::stream_profile accel_profile_;
  rs2::stream_profile gyro_profile_;

  rs_motion_module_t(const rs2::device &device,
                     const int accel_hz=250,
                     const int gyro_hz=400,
                     const unsigned int fq_size=10)
      : device_{device}, fq_{fq_size} {
    setup(accel_hz, gyro_hz);
    sensor_.open({accel_profile_, gyro_profile_});
    sensor_.start(fq_);
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
    }
  }

  void setStreamProfiles(const int accel_hz, const int gyro_hz) {
    bool accel_ok = false;
    bool gyro_ok = false;

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

  void setup(const int accel_hz=250, const int gyro_hz=400) {
    if (rs2_get_sensors(device_, "Motion Module", sensor_) != 0) {
      FATAL("This RealSense device does not have a [Motion Module]");
    }
    setStreamProfiles(accel_hz, gyro_hz);
  }

  rs2::frame waitForFrame() {
    return fq_.wait_for_frame();
  }
};

struct rs_stereo_module_t {
  const rs2::device &device_;
  rs2::syncer sync_;
  rs2::sensor sensor_;
  rs2::stream_profile profile1_;
  rs2::stream_profile profile2_;
  bool profile1_set_ = false;
  bool profile2_set_ = false;

  rs_stereo_module_t(const rs2::device &device,
                     const std::string &target_profile="Infrared",
                     const int target_rate=90,
                     const std::string &target_format="Y8",
                     const int target_width=640,
                     const int target_height=480,
                     const int sync_size=20)
      : device_{device}, sync_{sync_size} {
    setup(target_profile,
          target_rate,
          target_format,
          target_width,
          target_height);
    sensor_.open({profile1_, profile2_});
    sensor_.start(sync_);
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
      printf("- %s [%d hz] [%s] [%dx%d] \n",
             stream_name.c_str(),
             stream_rate,
             format,
             vp.width(),
             vp.height());
    }
  }

  void setStreamProfile(const std::string &target_profile,
                        const int target_rate,
                        const std::string &target_format,
                        const int target_width,
                        const int target_height) {
    const auto stream_profiles = sensor_.get_stream_profiles();
    for (const auto &stream_profile : stream_profiles) {
      const auto name = stream_profile.stream_name();
      const auto rate = stream_profile.fps();
      const auto format = rs2_format_to_string(stream_profile.format());
      const auto vp = stream_profile.as<rs2::video_stream_profile>();
      const int width = vp.width();
      const int height = vp.height();

      const bool rate_ok = (target_rate == rate);
      const bool format_ok = (target_format == format);
      const bool width_ok = (target_width == width);
      const bool height_ok = (target_height == height);
      const bool res_ok = (width_ok && height_ok);

      if (rate_ok && format_ok && res_ok) {
        if (target_profile + " 1" == name) {
          profile1_ = stream_profile;
          profile1_set_ = true;
        } else if (target_profile + " 2" == name) {
          profile2_ = stream_profile;
          profile2_set_ = true;
        }
      }
    }

    if (profile1_ == false && profile2_ == false) {
      FATAL("Failed to get stereo module stream profile!");
    }
  }

  void setup(const std::string &target_profile,
             const int target_rate,
             const std::string &target_format,
             const int target_width,
             const int target_height) {
    if (rs2_get_sensors(device_, "Stereo Module", sensor_) != 0) {
      FATAL("This RealSense device does not have a [Stereo Module]");
    }
    setStreamProfile(target_profile,
                     target_rate,
                     target_format,
                     target_width,
                     target_height);

    // Sensor options
    sensor_.set_option(RS2_OPTION_EMITTER_ENABLED, 0.0f);

    // const auto exposure_range = sensor_.get_option_range(RS2_OPTION_EXPOSURE);
    // printf("Exposure:\n");
    // printf("- min: %f\n", exposure_range.min);
    // printf("- max: %f\n", exposure_range.max);
    // printf("- def: %f\n", exposure_range.def);
    // printf("- step: %f\n", exposure_range.step);

    sensor_.set_option(RS2_OPTION_EXPOSURE, 10.0f);
    // sensor_.set_option(RS2_OPTION_GAIN, 0.0f);
  }

  rs2::frameset waitForFrame() {
    return sync_.wait_for_frames();
  }
};
