#pragma once
#include "common.hpp"

template <typename T>
T lerp(const T &a, const T &b, const double t) {
  return a * (1.0 - t) + b * t;
}

struct lerp_buf_t {
  std::deque<std::string> buf_dtype;
  std::deque<double> buf_ts;
  std::deque<Eigen::Vector3d> buf_data;

  std::deque<double> lerped_gyro_ts;
  std::deque<Eigen::Vector3d> lerped_gyro_data;
  std::deque<double> lerped_accel_ts;
  std::deque<Eigen::Vector3d> lerped_accel_data;

  lerp_buf_t() {}

  bool ready() {
    if (buf_ts.size() >= 3 && buf_dtype.back() == "A") {
      return true;
    }
    return false;
  }

  void addAccel(const geometry_msgs::Vector3Stamped &msg) {
    buf_dtype.push_back("A");
    buf_ts.push_back(msg.header.stamp.toSec());
    buf_data.emplace_back(msg.vector.x, msg.vector.y, msg.vector.z);
  }

  void addGyro(const geometry_msgs::Vector3Stamped &msg) {
    if (buf_dtype.size() && buf_dtype.front() == "A") {
      buf_dtype.push_back("G");
      buf_ts.push_back(msg.header.stamp.toSec());
      buf_data.emplace_back(msg.vector.x, msg.vector.y, msg.vector.z);
    }
  }

  void print() {
    for (size_t i = 0; i < buf_ts.size(); i ++) {
      const double ts = buf_ts.at(i);
      const std::string dtype = buf_dtype.at(i);
      const Eigen::Vector3d data = buf_data.at(i);
      const double x = data(0);
      const double y = data(1);
      const double z = data(1);
      printf("[%.6f] - [%s] - (%.2f, %.2f, %.2f)\n", ts, dtype.c_str(), x, y, z);
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

    while (buf_ts.size()) {
      // Timestamp
      ts = buf_ts.front();
      buf_ts.pop_front();

      // Datatype
      dtype = buf_dtype.front();
      buf_dtype.pop_front();

      // Data
      data = buf_data.front();
      buf_data.pop_front();

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

          lerped_accel_ts.push_back(lts);
          lerped_accel_data.push_back(lerp(d0, d1, alpha));

          lerped_gyro_ts.push_back(lts);
          lerped_gyro_data.push_back(ldata);

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

    buf_ts.push_back(ts);
    buf_dtype.push_back(dtype);
    buf_data.push_back(data);
  }

  void publishIMUMessages(const ros::Publisher &imu_pub) {
    while (lerped_gyro_ts.size()) {
      // Timestamp
      const auto ts = lerped_gyro_ts.front();
      lerped_gyro_ts.pop_front();
      lerped_accel_ts.pop_front();

      // Accel
      const auto accel = lerped_accel_data.front();
      lerped_accel_data.pop_front();

      // Gyro
      const auto gyro = lerped_gyro_data.front();
      lerped_gyro_data.pop_front();

      // Publish imu messages
      const auto msg = create_imu_msg(ts, gyro, accel);
      imu_pub.publish(msg);
    }

    // Clear
    lerped_gyro_ts.clear();
    lerped_gyro_data.clear();
    lerped_accel_ts.clear();
    lerped_accel_data.clear();
  }
};
