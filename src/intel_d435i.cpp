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

// Signal handler
bool keep_running = true;
void signal_handler(int sig) {
  UNUSED(sig);
  keep_running = false;
}

std::string ros_node_name(int argc, char *argv[]) {
  for (int i = 1; i < argc; i++) {
    std::string arg(argv[i]);
    if (arg.find("__name:=") != std::string::npos) { return arg.substr(8); }
  }
  FATAL("Failed to find node name?");
}

struct intel_d435i_node_t {
  image_transport::Publisher rgb0_pub;
  image_transport::Publisher ir0_pub;
  image_transport::Publisher ir1_pub;
  image_transport::Publisher depth0_pub;
  ros::Publisher gyro0_pub;
  ros::Publisher accel0_pub;
  ros::Publisher imu0_pub;

  rs_motion_module_config_t motion_config;
  rs_rgbd_module_config_t rgbd_config;

  intel_d435i_node_t(const std::string &nn) {
    ros::NodeHandle nh;

    // ROS params
    // -- RGBD
    ROS_PARAM(nh, nn + "/global_time", rgbd_config.global_time);
    ROS_PARAM(nh, nn + "/enable_rgb", rgbd_config.enable_rgb);
    ROS_PARAM(nh, nn + "/enable_ir", rgbd_config.enable_ir);
    ROS_PARAM(nh, nn + "/enable_depth", rgbd_config.enable_depth);
    ROS_PARAM(nh, nn + "/enable_emitter", rgbd_config.enable_emitter);
    ROS_PARAM(nh, nn + "/rgb_width", rgbd_config.rgb_width);
    ROS_PARAM(nh, nn + "/rgb_height", rgbd_config.rgb_height);
    ROS_PARAM(nh, nn + "/rgb_format", rgbd_config.rgb_format);
    ROS_PARAM(nh, nn + "/rgb_frame_rate", rgbd_config.rgb_frame_rate);
    ROS_PARAM(nh, nn + "/rgb_exposure", rgbd_config.rgb_exposure);
    ROS_PARAM(nh, nn + "/ir_width", rgbd_config.ir_width);
    ROS_PARAM(nh, nn + "/ir_height", rgbd_config.ir_height);
    ROS_PARAM(nh, nn + "/ir_format", rgbd_config.ir_format);
    ROS_PARAM(nh, nn + "/ir_frame_rate", rgbd_config.ir_frame_rate);
    ROS_PARAM(nh, nn + "/ir_exposure", rgbd_config.ir_exposure);
    ROS_PARAM(nh, nn + "/depth_width", rgbd_config.depth_width);
    ROS_PARAM(nh, nn + "/depth_height", rgbd_config.depth_height);
    ROS_PARAM(nh, nn + "/depth_format", rgbd_config.depth_format);
    ROS_PARAM(nh, nn + "/depth_frame_rate", rgbd_config.depth_frame_rate);
    // -- Motion monule
    ROS_PARAM(nh, nn + "/global_time", motion_config.global_time);
    ROS_PARAM(nh, nn + "/enable_motion", motion_config.enable_motion);
    ROS_PARAM(nh, nn + "/accel_hz", motion_config.accel_hz);
    ROS_PARAM(nh, nn + "/gyro_hz", motion_config.gyro_hz);

    // Publishers
    const auto rgb0_topic = nn + "/rgb0/image";
    const auto ir0_topic = nn + "/ir0/image";
    const auto ir1_topic = nn + "/ir1/image";
    const auto depth0_topic = nn + "/depth0/image";
    const auto gyro0_topic = nn + "/gyro0/data";
    const auto accel0_topic = nn + "/accel0/data";
    const auto imu0_topic = nn + "/imu0/data";
    // -- RGB module
    if (rgbd_config.enable_rgb) {
      image_transport::ImageTransport rgb_it(nh);
      rgb0_pub = rgb_it.advertise(rgb0_topic, 1);
    }
    // -- Stereo module
    if (rgbd_config.enable_ir) {
      image_transport::ImageTransport stereo_it(nh);
      ir0_pub = stereo_it.advertise(ir0_topic, 1);
      ir1_pub = stereo_it.advertise(ir1_topic, 1);
    }
    if (rgbd_config.enable_depth) {
      image_transport::ImageTransport depth_it(nh);
      depth0_pub = depth_it.advertise(depth0_topic, 1);
    }
    // -- Motion module
    if (motion_config.enable_motion) {
      gyro0_pub = nh.advertise<Vector3StampedMsg>(gyro0_topic, 1);
      accel0_pub = nh.advertise<Vector3StampedMsg>(accel0_topic, 1);
      imu0_pub = nh.advertise<ImuMsg>(imu0_topic, 1);
    }
  }
};

int main(int argc, char **argv) {
  // ROS init
  const std::string node_name = ros_node_name(argc, argv);
  if (ros::isInitialized() == false) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  try {
    // ROS
    intel_d435i_node_t node(node_name);

    // RGBD Callback
    auto rgbd_cb = [&](const rs2::frame &frame) {
      if (const auto &fs = frame.as<rs2::frameset>()) {
        // IR0 & IR1
        const bool enable_ir = node.rgbd_config.enable_ir;
        const auto ir0 = fs.get_infrared_frame(1);
        const auto ir1 = fs.get_infrared_frame(2);
        if (enable_ir && ir0 && ir1) {
          const auto cam0_msg = create_image_msg(ir0, "rs/ir0", false);
          const auto cam1_msg = create_image_msg(ir1, "rs/ir1", false);
          node.ir0_pub.publish(cam0_msg);
          node.ir1_pub.publish(cam1_msg);
        }

        // Align depth to rgb
        rs2::align align_to_color(RS2_STREAM_COLOR);
        const auto fs_aligned = align_to_color.process(fs);

        // RGB
        const bool enable_rgb = node.rgbd_config.enable_rgb;
        const auto rgb = fs_aligned.get_color_frame();
        if (enable_rgb && rgb) {
          const auto msg = create_image_msg(rgb, "rs/rgb0", true);
          node.rgb0_pub.publish(msg);
        }

        // Depth image
        const bool enable_depth = node.rgbd_config.enable_depth;
        const auto depth = fs_aligned.get_depth_frame();
        if (enable_depth && depth) {
          const auto depth_msg = create_depth_msg(depth, "rs/depth0");
          node.depth0_pub.publish(depth_msg);
        }
      }
    };

    // IMU callback
    lerp_buf_t lerp_buf;
    auto motion_cb = [&](const rs2::frame &frame) {
      const bool enable_motion = node.motion_config.enable_motion;
      const auto mf = frame.as<rs2::motion_frame>();
      if (enable_motion && mf) {
        if (mf && mf.get_profile().stream_type() == RS2_STREAM_ACCEL) {
          const auto msg = create_vec3_msg(mf, "rs/accel0");
          node.accel0_pub.publish(msg);
          lerp_buf.addAccel(msg);

        } else if (mf && mf.get_profile().stream_type() == RS2_STREAM_GYRO) {
          const auto msg = create_vec3_msg(mf, "rs/gyro0");
          node.gyro0_pub.publish(msg);
          lerp_buf.addGyro(msg);
        }

        if (lerp_buf.ready()) {
          lerp_buf.interpolate();
          lerp_buf.publishIMUMessages(node.imu0_pub, "rs/imu0");
        }
      }
    };

    // Connect to Intel RealSense D435i
    rs2::device device = rs2_connect();
    rs_motion_module_t motion_module(device, node.motion_config, motion_cb);
    rs_rgbd_module_t rgbd_module(device, node.rgbd_config, rgbd_cb);

    // Pipelines are threads so we need a blocking loop
    signal(SIGINT, signal_handler);
    while (keep_running) {
      sleep(1);
    }

  } catch (const rs2::error &e) {
    FATAL("[RealSense Exception]: %s", e.what());
  }

  return 0;
}
