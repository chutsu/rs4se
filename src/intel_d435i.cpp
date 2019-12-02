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
  image_transport::Publisher rgb_pub;
  image_transport::Publisher ir0_pub;
  image_transport::Publisher ir1_pub;
  image_transport::Publisher depth_pub;
  ros::Publisher gyro0_pub;
  ros::Publisher accel0_pub;
  ros::Publisher imu0_pub;

  rs_rgb_module_config_t rgb_config;
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
    // -- RGB module
    {
      const std::string ns = "rgb";
      ROS_PARAM(nh, ns + "/global_time", rgb_config.global_time);
      ROS_PARAM(nh, ns + "/format", rgb_config.format);
      ROS_PARAM(nh, ns + "/frame_rate", rgb_config.frame_rate);
      ROS_PARAM(nh, ns + "/width", rgb_config.width);
      ROS_PARAM(nh, ns + "/height", rgb_config.height);
      ROS_PARAM(nh, ns + "/exposure", rgb_config.exposure);
    }
    // -- Stereo module
    {
      const std::string ns = "stereo";
      ROS_PARAM(nh, ns + "/global_time", stereo_config.global_time);
      ROS_PARAM(nh, ns + "/sync_size", stereo_config.sync_size);
      ROS_PARAM(nh, ns + "/enable_depth", stereo_config.enable_depth);
      ROS_PARAM(nh, ns + "/enable_emitter", stereo_config.enable_emitter);
      ROS_PARAM(nh, ns + "/format_stereo", stereo_config.format_stereo);
      if (stereo_config.enable_depth) {
        ROS_PARAM(nh, ns + "/format_depth", stereo_config.format_depth);
      }
      ROS_PARAM(nh, ns + "/frame_rate", stereo_config.frame_rate);
      ROS_PARAM(nh, ns + "/width", stereo_config.width);
      ROS_PARAM(nh, ns + "/height", stereo_config.height);
      ROS_PARAM(nh, ns + "/exposure", stereo_config.exposure);
    }
    // -- Motion module
    {
      const std::string ns = "motion";
      ROS_PARAM(nh, ns + "/global_time", motion_config.global_time);
      ROS_PARAM(nh, ns + "/accel_hz", motion_config.accel_hz);
      ROS_PARAM(nh, ns + "/gyro_hz", motion_config.gyro_hz);
    }

    // Publishers
    // -- RGB module
    image_transport::ImageTransport rgb_it(nh);
    rgb_pub = rgb_it.advertise("rgb/camera0/image", 1);
    // -- Stereo module
    image_transport::ImageTransport stereo_it(nh);
    ir0_pub = stereo_it.advertise("stereo/camera0/image", 1);
    ir1_pub = stereo_it.advertise("stereo/camera1/image", 1);
    if (stereo_config.enable_depth) {
      image_transport::ImageTransport depth_it(nh);
      depth_pub = depth_it.advertise("stereo/depth0/image", 1);
    }
    // -- Motion module
    gyro0_pub = nh.advertise<Vector3StampedMsg>("motion/gyro0", 1);
    accel0_pub = nh.advertise<Vector3StampedMsg>("motion/accel0", 1);
    imu0_pub = nh.advertise<ImuMsg>("motion/imu0", 1);
  }
};

static void rgb_handler(const rs2::frameset &fc, const intel_d435i_node_t &node,
                        const bool debug = false) {
  const auto rgb_frame = fc.get_color_frame();
  const auto msg = create_image_msg(rgb_frame, "rgb/camera0", true);
  node.rgb_pub.publish(msg);

  // Debug
  if (debug) {
    const int width = rgb_frame.get_width();
    const int height = rgb_frame.get_height();
    cv::Mat frame = frame2cvmat(rgb_frame, width, height, true);
    cv::imshow("Image", frame);
  }
}

static void stereo_handler(const rs2::frameset &fs,
                           const intel_d435i_node_t &node,
                           const bool debug = false) {
  // Pre-check
  if (node.stereo_config.enable_depth && fs.size() != 3) {
    return;
  } else if (node.stereo_config.enable_depth == false && fs.size() != 2) {
    return;
  }

  // Get stereo infrared frames and publish
  const auto ir_left = fs.get_infrared_frame(1);
  const auto ir_right = fs.get_infrared_frame(2);
  const auto cam0_msg = create_image_msg(ir_left, "stereo/camera0", false);
  const auto cam1_msg = create_image_msg(ir_right, "stereo/camera1", false);
  node.ir0_pub.publish(cam0_msg);
  node.ir1_pub.publish(cam1_msg);

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
    cv::Mat frame_left = frame2cvmat(ir_left, width, height, false);
    cv::Mat frame_right = frame2cvmat(ir_right, width, height, false);
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
    rs_rgb_module_t rgb{device, node.rgb_config};
    rs_stereo_module_t stereo{device, node.stereo_config};
    rs_motion_module_t motion{device, node.motion_config};

    // Process rgb stream
    std::thread rgb_thread([&]() {
      while (true) {
        const auto &fs = rgb.waitForFrame();
        rgb_handler(fs, node);
      }
    });

    // Process stereo stream
    std::thread stereo_thread([&]() {
      while (true) {
        const auto &fs = stereo.waitForFrame();
        stereo_handler(fs, node);
      }
    });

    // Process IMU stream
    lerp_buf_t lerp_buf;
    std::thread motion_thread([&]() {
      while (true) {
        const auto f = motion.waitForFrame();
        motion_handler(f, node, lerp_buf);
      }
    });

    // Join threads
    rgb_thread.join();
    stereo_thread.join();
    motion_thread.join();

  } catch (const rs2::error &e) {
    FATAL("[RealSense Exception]: %s", e.what());
  }

  return 0;
}
