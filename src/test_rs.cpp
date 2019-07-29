#include "rs.hpp"
#include "munit.hpp"

int test_rs2_connect() {
  rs2::device device = rs2_connect();
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

  // for (int i = 0; i < 10; i++) {
  // }
  const auto fs = stereo.waitForFrame();
  for (const auto &frame : fs) {
   print_rsframe_timestamps(frame);
  }

  return 0;
}

void test_suite() {
  // MU_ADD_TEST(test_rs2_connect);
  // MU_ADD_TEST(test_rs2_list_sensors);
  // MU_ADD_TEST(test_rs2_get_sensors);
  // MU_ADD_TEST(test_rs2_motion_module);
  MU_ADD_TEST(test_rs2_stereo_module);
}

MU_RUN_TESTS(test_suite);
