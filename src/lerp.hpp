#pragma once
#include "common.hpp"

template <typename T> T lerp(const T &a, const T &b, const double t) {
  return a * (1.0 - t) + b * t;
}

class lerp_buf_t {
  std::deque<std::string> buf_type_;
  std::deque<double> buf_ts_;
  std::deque<Eigen::Vector3d> buf_data_;

  std::deque<double> lerped_gyro_ts_;
  std::deque<Eigen::Vector3d> lerped_gyro_data_;
  std::deque<double> lerped_accel_ts_;
  std::deque<Eigen::Vector3d> lerped_accel_data_;

public:
  lerp_buf_t() {}

  bool ready() {
    if (buf_ts_.size() >= 3 && buf_type_.back() == "A") {
      return true;
    }
    return false;
  }

  void addAccel(const geometry_msgs::Vector3Stamped &msg) {
    buf_type_.push_back("A");
    buf_ts_.push_back(msg.header.stamp.toSec());
    buf_data_.emplace_back(msg.vector.x, msg.vector.y, msg.vector.z);
  }

  void addGyro(const geometry_msgs::Vector3Stamped &msg) {
    if (buf_type_.size() && buf_type_.front() == "A") {
      buf_type_.push_back("G");
      buf_ts_.push_back(msg.header.stamp.toSec());
      buf_data_.emplace_back(msg.vector.x, msg.vector.y, msg.vector.z);
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
      printf("[%.6f] - [%s] - (%.2f, %.2f, %.2f)\n", ts, dtype.c_str(), x, y,
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

  void publishIMUMessages(const ros::Publisher &imu_pub) {
    while (lerped_gyro_ts_.size()) {
      // Timestamp
      const auto ts = lerped_gyro_ts_.front();
      lerped_gyro_ts_.pop_front();
      lerped_accel_ts_.pop_front();

      // Accel
      const auto accel = lerped_accel_data_.front();
      lerped_accel_data_.pop_front();

      // Gyro
      const auto gyro = lerped_gyro_data_.front();
      lerped_gyro_data_.pop_front();

      // Publish imu messages
      const auto msg = create_imu_msg(ts, gyro, accel);
      imu_pub.publish(msg);
    }

    // Clear
    lerped_gyro_ts_.clear();
    lerped_gyro_data_.clear();
    lerped_accel_ts_.clear();
    lerped_accel_data_.clear();
  }
};
