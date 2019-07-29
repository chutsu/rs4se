#include <chrono>
#include <mutex>
#include <thread>
#include <deque>
#include <string>

#include <signal.h>
#include <unistd.h>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <librealsense2/rs.hpp>

#include "rs.hpp"
#include "lerp.hpp"
#include "common.hpp"

struct intel_d435i_node_t {
  image_transport::Publisher cam0_pub;
  image_transport::Publisher cam1_pub;
  ros::Publisher gyro0_pub;
  ros::Publisher accel0_pub;
  ros::Publisher imu0_pub;

  intel_d435i_node_t(int argc, char **argv) {
    // Setup ros node
    ros::init(argc, argv, argv[0]);
    ros::NodeHandle nh;

    // Publishers
    // -- Stereo module
    image_transport::ImageTransport it(nh);
    cam0_pub = it.advertise("stereo/camera0/image", 1);
    cam1_pub = it.advertise("stereo/camera1/image", 1);
    // -- Motion module
    gyro0_pub = nh.advertise<geometry_msgs::Vector3Stamped>("motion/gyro0", 1);
    accel0_pub = nh.advertise<geometry_msgs::Vector3Stamped>("motion/accel0", 1);
    imu0_pub = nh.advertise<sensor_msgs::Imu>("motion/imu0", 1);
  }
};

static void stereo_handler(const rs2::frameset &fs,
                           const intel_d435i_node_t &node,
                           const bool debug=false) {
  if (fs.size() != 2) {
    return;
  }

  // Create cv::Mat image
  const auto ir_left = fs.get_infrared_frame(1);
  const auto ir_right = fs.get_infrared_frame(2);
  const int width = ir_left.get_width();
  const int height = ir_left.get_height();
  cv::Mat frame_left = frame2cvmat(ir_left, width, height);
  cv::Mat frame_right = frame2cvmat(ir_right, width, height);

  // Build image messages
  const auto cam0_msg = create_image_msg(ir_left, "stereo/camera0");
  const auto cam1_msg = create_image_msg(ir_right, "stereo/camera1");

  // Publish image messages
  node.cam0_pub.publish(cam0_msg);
  node.cam1_pub.publish(cam1_msg);

  // Debug
  if (debug) {
    debug_imshow(frame_left, frame_right);
  }
}

static void motion_handler(const rs2::frame &f,
                           const intel_d435i_node_t &node,
                           lerp_buf_t &lerp_buf) {
  const auto mf = f.as<rs2::motion_frame>();

  if (mf && mf.get_profile().stream_type() == RS2_STREAM_ACCEL) {
    const auto msg = create_vec3_msg(mf, "accel0");
    node.accel0_pub.publish(msg);
    lerp_buf.addAccel(msg);

  } else if (mf && mf.get_profile().stream_type() == RS2_STREAM_GYRO) {
    const auto msg = create_vec3_msg(mf, "gyro0");
    node.gyro0_pub.publish(msg);
    lerp_buf.addGyro(msg);
  }

  if (lerp_buf.ready()) {
    lerp_buf.interpolate();
    lerp_buf.publishIMUMessages(node.imu0_pub);
  }
}

int main(int argc, char **argv) {
  try {
    // ROS
    intel_d435i_node_t node(argc, argv);

    // Setup RealSense sensor
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    rs2::device device = rs2_connect();
    rs_motion_module_t motion{device};
    rs_stereo_module_t stereo{device};
    // rs_stereo_module_t stereo{device, "Infrared", 6};

    // Process IMU stream
    lerp_buf_t lerp_buf;
    std::thread motion_thread([&]() {
      while (true) {
        const auto f = motion.waitForFrame();
        motion_handler(f, node, lerp_buf);
      }
    });

    // Process stereo stream
    std::thread stereo_thread([&]() {
      while (true) {
        const auto fs = stereo.waitForFrame();
        stereo_handler(fs, node);
      }
    });

    // Join threads
    motion_thread.join();
    stereo_thread.join();

  } catch (const rs2::error& e) {
    FATAL("[RealSense Exception]: %s", e.what());
  }

  return 0;
}
