#include <chrono>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include <signal.h>
#include <unistd.h>

#include <Eigen/Dense>

#include <geometry_msgs/Vector3Stamped.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <librealsense2/rs.hpp>

#include "common.hpp"
#include "lerp.hpp"
#include "rs.hpp"

#define Vector3StampedMsg geometry_msgs::Vector3Stamped
#define ImuMsg sensor_msgs::Imu

struct intel_d435i_node_t {
  image_transport::Publisher cam0_pub;
  image_transport::Publisher cam1_pub;
  image_transport::Publisher depth_pub;
  ros::Publisher gyro0_pub;
  ros::Publisher accel0_pub;
  ros::Publisher imu0_pub;

  bool global_time;

  rs_stereo_module_config_t stereo_config;
  rs_motion_module_config_t motion_config;

  intel_d435i_node_t(int argc, char **argv) {
    // Parse args
    std::string node_name;
    for (int i = 1; i < argc; i++) {
      std::string arg(argv[i]);
      if (arg.find("__name:=") != std::string::npos) {
        node_name = arg.substr(8);
        break;
      }
    }

    // Setup ros node
    ros::init(argc, argv, argv[0]);
    ros::NodeHandle nh;

    // ROS params
    const std::string ns = "stereo";
    ROS_PARAM(nh, ns + "/global_time", global_time);
    ROS_PARAM(nh, ns + "/sync_size", stereo_config.sync_size);
    ROS_PARAM(nh, ns + "/enable_depth", stereo_config.enable_depth);
    ROS_PARAM(nh, ns + "/format_stereo", stereo_config.format_stereo);
    if (stereo_config.enable_depth) {
      ROS_PARAM(nh, ns + "/format_depth", stereo_config.format_depth);
    }
    ROS_PARAM(nh, ns + "/frame_rate", stereo_config.frame_rate);
    ROS_PARAM(nh, ns + "/width", stereo_config.width);
    ROS_PARAM(nh, ns + "/height", stereo_config.height);
    ROS_PARAM(nh, ns + "/exposure", stereo_config.exposure);

    // Publishers
    // -- Stereo module
    image_transport::ImageTransport img_it(nh);
    cam0_pub = img_it.advertise("stereo/camera0/image", 1);
    cam1_pub = img_it.advertise("stereo/camera1/image", 1);
    // -- RGB module
    // rgb_pub = it.advertise("rgb/image");
    // -- Motion module
    gyro0_pub = nh.advertise<Vector3StampedMsg>("motion/gyro0", 1);
    accel0_pub = nh.advertise<Vector3StampedMsg>("motion/accel0", 1);
    imu0_pub = nh.advertise<ImuMsg>("motion/imu0", 1);
    // -- Depth module
    if (stereo_config.enable_depth) {
      image_transport::ImageTransport depth_it(nh);
      depth_pub = depth_it.advertise("stereo/depth0/image", 1);
    }
  }
};

static void stereo_handler(const rs2::frameset &fs,
                           const intel_d435i_node_t &node,
                           const bool debug = false) {
  if ((fs.size() < 2) || (fs.size() > 3)) {
    return;
  }

  // Get stereo infrared frames and publish
  const auto ir_left = fs.get_infrared_frame(1);
  const auto ir_right = fs.get_infrared_frame(2);
  const auto cam0_msg = create_image_msg(ir_left, "stereo/camera0");
  const auto cam1_msg = create_image_msg(ir_right, "stereo/camera1");
  node.cam0_pub.publish(cam0_msg);
  node.cam1_pub.publish(cam1_msg);

  // Get depth frame and publish
  if (node.stereo_config.enable_depth) {
    const auto depth_frame = fs.get_depth_frame();
    const auto depth_msg = create_depth_msg(depth_frame, "stereo/depth0");
    node.depth_pub.publish(depth_msg);
  }

  // Debug
  if (debug) {
    const int width = ir_left.get_width();
    const int height = ir_left.get_height();
    cv::Mat frame_left = frame2cvmat(ir_left, width, height);
    cv::Mat frame_right = frame2cvmat(ir_right, width, height);
    debug_imshow(frame_left, frame_right);
  }
}

static void motion_handler(const rs2::frame &f, const intel_d435i_node_t &node,
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
    rs_motion_module_t motion{device, node.motion_config};
    rs_stereo_module_t stereo{device, node.stereo_config};

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
        const auto &fs = stereo.waitForFrame();
        stereo_handler(fs, node);
      }
    });

    // Join threads
    motion_thread.join();
    stereo_thread.join();

  } catch (const rs2::error &e) {
    FATAL("[RealSense Exception]: %s", e.what());
  }

  return 0;
}
