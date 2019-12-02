#include "munit.hpp"
#include "rs.hpp"

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

int test_rs2_get_sensors() {
  const auto device = rs2_connect();

  rs2::sensor sensor;
  MU_CHECK(rs2_get_sensors(device, "Stereo Module", sensor) == 0);
}

int test_rs2_motion_module() {
  rs2::device device = rs2_connect();
  rs_motion_module_t motion{device};

  motion.listStreamProfiles();
  motion.setStreamProfiles(250, 400);

  return 0;
}

int test_rs2_stereo_module() {
  rs2::device device = rs2_connect();
  rs_stereo_module_t stereo{device};

  stereo.listStreamProfiles();

  return 0;
}

int test_rs2_rgb_module() {
  rs2::device device = rs2_connect();
  rs_rgb_module_t rgb{device};

  rgb.listStreamProfiles();

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

int test_vframe2ts() {
  rs2::device device = rs2_connect();
  rs_stereo_module_t stereo{device};

  const auto fs = stereo.waitForFrame();
  const auto frame = fs[0];
  const auto vf = frame.as<rs2::video_frame>();
  printf("corrected timestamp: %zu\n\n", vframe2ts(vf));

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_rs2_connect);
  MU_ADD_TEST(test_rs2_list_sensors);
  MU_ADD_TEST(test_rs2_get_sensors);
  MU_ADD_TEST(test_rs2_motion_module);
  MU_ADD_TEST(test_rs2_stereo_module);
  // MU_ADD_TEST(test_rs2_rgb_module);
  // MU_ADD_TEST(test_ts_correction);
  // MU_ADD_TEST(test_vframe2ts);
}

MU_RUN_TESTS(test_suite);
