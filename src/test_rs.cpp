#include "munit.hpp"
#include "rs.hpp"
#include "lerp.hpp"

#include <chrono>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

int test_rs2_connect() {
  rs2::device device = rs2_connect();
  const auto fm_ver = device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
  printf("Firmware Version: %s\n", fm_ver);

  return 0;
}

int test_rs2_list_sensors() {
  rs2_list_sensors();
  return 0;
}

int test_rs2_get_sensor() {
  const auto device = rs2_connect();

  rs2::sensor sensor;
  MU_CHECK(rs2_get_sensor(device, "Stereo Module", sensor) == 0);
}

int test_rs2_motion_module() {
  const rs2::device device = rs2_connect();
  const rs_motion_module_config_t config;

  bool keep_running = true;
  int frame_counter = 0;
  rs_motion_module_t motion(device, config, [&](const rs2::frame &frame) {
    auto motion = frame.as<rs2::motion_frame>();
    if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO &&
        motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
      double ts = motion.get_timestamp();
      rs2_vector gyro_data = motion.get_motion_data();
      std::cout << "G\t" << gyro_data << std::endl;
    }

    if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL &&
        motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
      double ts = motion.get_timestamp();
      rs2_vector accel_data = motion.get_motion_data();
      std::cout << "A\t" << accel_data << std::endl;
    }

    frame_counter++;
    if (frame_counter >= 10) { keep_running = false; }
  });

  // Block until frame counter threadhold is reached
  while (keep_running) {
    sleep(0.1);
  }

  return 0;
}

int test_rs2_rgbd_module() {
  const rs2::device device = rs2_connect();
  rs_rgbd_module_config_t config;

  bool keep_running = true;
  int frame_counter = 0;
  rs_rgbd_module_t rgbd(device, config, [&](const rs2::frame &frame) {
    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      // Show rgbd ir image
      const auto ir_left = fs.get_infrared_frame(1);
      const auto ir_right = fs.get_infrared_frame(2);
      const int width = ir_left.get_width();
      const int height = ir_left.get_height();
      cv::Mat frame_left = frame2cvmat(ir_left, width, height, CV_8UC1);
      cv::Mat frame_right = frame2cvmat(ir_right, width, height, CV_8UC1);
      debug_imshow(frame_left, frame_right);

      // Show depth image
      if (config.enable_depth) {
        const auto depth_frame = fs.get_depth_frame();
        cv::Mat depth = frame2cvmat(depth_frame, width, height, CV_16UC1);
        cv::imshow("Depth", depth);
        cv::waitKey(1);
      }

      frame_counter++;
      if (frame_counter >= 100) { keep_running = false; }
    }
  });

  // Block until frame counter threadhold is reached
  while (keep_running) {
    sleep(0.1);
  }

  return 0;
}

int test_ts_correction() {
  // Calculate half of the exposure time
  const long long int frame_ts_us = 927545119;
  const uint64_t frame_ts_ns = static_cast<uint64_t>(frame_ts_us) * 1000;
  const long long int sensor_ts_us = 927462708;
  const uint64_t sensor_ts_ns = static_cast<uint64_t>(sensor_ts_us) * 1000;
  const uint64_t half_exposure_time_ns = frame_ts_ns - sensor_ts_ns;

  // Calculate corrected timestamp
  const double ts_ms = 1564436052531.301514;
  const uint64_t ts_ns = str2ts(std::to_string(ts_ms));
  const uint64_t ts_corrected_ns = ts_ns - half_exposure_time_ns;

  printf("frame timestamp [ms]:           %f\n", ts_ms);
  printf("frame timestamp [us]:           %f\n", ts_ms * 1e3);
  printf("frame timestamp [ns]:           %f\n", ts_ms * 1e6);
  printf("converted frame timestamp [ns]: %zu\n\n", ts_ns);

  printf("frame timestamp [us]:     %lld\n", frame_ts_us);
  printf("sensor timestamp [us]:    %lld\n", sensor_ts_us);
  printf("frame timestamp [ns]:  %zu\n", frame_ts_ns);
  printf("sensor timestamp [ns]: %zu\n", sensor_ts_ns);
  printf("half exposure time [ns]:   %zu\n\n", half_exposure_time_ns);

  printf("converted frame timestamp [ns]: %zu\n", ts_ns);
  printf("half exposure time [ns]:                   %zu\n",
         half_exposure_time_ns);
  printf("corrected timestamp [ns]:       %zu\n\n", ts_corrected_ns);

  return 0;
}

// int test_vframe2ts() {
//   rs2::device device = rs2_connect();
//   rs_stereo_module_t stereo{device};
//
//   const auto fs = stereo.waitForFrame();
//   const auto frame = fs[0];
//   const auto vf = frame.as<rs2::video_frame>();
//   printf("corrected timestamp: %zu\n\n", vframe2ts(vf));
//
//   return 0;
// }

void test_suite() {
  MU_ADD_TEST(test_rs2_connect);
  MU_ADD_TEST(test_rs2_list_sensors);
  MU_ADD_TEST(test_rs2_get_sensor);
  MU_ADD_TEST(test_rs2_motion_module);
  MU_ADD_TEST(test_rs2_rgbd_module);
  MU_ADD_TEST(test_ts_correction);
  // MU_ADD_TEST(test_vframe2ts);
}

MU_RUN_TESTS(test_suite);
