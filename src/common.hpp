#pragma once
#include <deque>

#include <Eigen/Dense>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>

#define __FILENAME__                                                           \
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define FATAL(M, ...)                                                          \
  fprintf(stdout,                                                              \
          "\033[31m[FATAL] [%s:%d] " M "\033[0m\n",                            \
          __FILENAME__,                                                        \
          __LINE__,                                                            \
          ##__VA_ARGS__);                                                      \
  exit(-1)

static cv::Mat frame2cvmat(const rs2::frame &frame,
                           const int width,
                           const int height) {
  const cv::Size size(width, height);
  const auto format = CV_8UC1;
  const auto stride = cv::Mat::AUTO_STEP;
  const cv::Mat cv_frame(size, format, (void*)frame.get_data(), stride);
  return cv_frame;
}

static double vframe2ts(const rs2::video_frame &vf) {
  // Calculate half of the exposure time
  const auto frame_ts_us = vf.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);
  const auto sensor_ts_us = vf.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP);
  const auto half_exposure_time_ms = (frame_ts_us - sensor_ts_us) * 1e-3;

  // Calculate corrected timestamp
  const auto ts_s = (vf.get_timestamp() - half_exposure_time_ms) * 1e-3;

  return ts_s;
}

static sensor_msgs::ImagePtr create_image_msg(const rs2::video_frame &vf,
                                              const std::string &frame_id) {
  // Form msg stamp
  const double ts_s = vframe2ts(vf);
  ros::Time msg_stamp;
  msg_stamp.fromSec(ts_s);

  // Form msg header
  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = msg_stamp;

  // Image message
  const int width = vf.get_width();
  const int height = vf.get_height();
  cv::Mat cv_frame = frame2cvmat(vf, width, height);
  const auto msg = cv_bridge::CvImage(header, "mono8", cv_frame).toImageMsg();

  return msg;
}

static geometry_msgs::Vector3Stamped create_vec3_msg(
      const rs2::motion_frame &f,
      const std::string &frame_id) {
  // Form msg stamp
  double ts_s = f.get_timestamp() * 1e-3;
  ros::Time stamp;
  stamp.fromSec(ts_s);
  // printf("[%s]: %.9f\n", frame_id.c_str(), ts_s);

  // Form msg header
  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = stamp;

  // Form msg
  const rs2_vector data = f.get_motion_data();
  geometry_msgs::Vector3Stamped msg;
  msg.header = header;
  msg.vector.x = data.x;
  msg.vector.y = data.y;
  msg.vector.z = data.z;

  return msg;
}

static sensor_msgs::Imu create_imu_msg(const double ts,
                                       const Eigen::Vector3d &gyro,
                                       const Eigen::Vector3d &accel) {
  sensor_msgs::Imu msg;

  msg.header.frame_id = "imu0";
  msg.header.stamp = ros::Time{ts};
  msg.angular_velocity.x = gyro(0);
  msg.angular_velocity.y = gyro(1);
  msg.angular_velocity.z = gyro(2);
  msg.linear_acceleration.x = accel(0);
  msg.linear_acceleration.y = accel(1);
  msg.linear_acceleration.z = accel(2);

  return msg;
}

static void debug_imshow(const cv::Mat &frame_left, const cv::Mat &frame_right) {
  // Display in a GUI
  cv::Mat frame;
  cv::hconcat(frame_left, frame_right, frame);
  cv::namedWindow("Stereo Module", cv::WINDOW_AUTOSIZE);
  cv::imshow("Stereo Module", frame);

  if (cv::waitKey(1) == 'q') {
    exit(-1);
  }
}
