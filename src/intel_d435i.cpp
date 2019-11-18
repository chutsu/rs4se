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
  image_transport::Publisher color_pub;
  ros::Publisher gyro0_pub;
  ros::Publisher accel0_pub;
  ros::Publisher imu0_pub;

  bool global_time;

  rs_stereo_module_config_t stereo_config;
  rs_motion_module_config_t motion_config;
  rs_camera_module_config_t color_config;

  intel_d435i_node_t(int argc, char **argv) {
    // Parse args
    std::string node_name;
    for (int i = 1; i < argc; i++) {
      std::string arg(argv[i]);

      // ros node name
      if (arg.find("__name:=") != std::string::npos) {
        node_name = arg.substr(8);
      }
    }

    // Setup ros node
    ros::init(argc, argv, argv[0]);
    ros::NodeHandle nh;

    // ROS params
    const std::string ns = "stereo";
    ROS_GET_PARAM(ns + "/global_time", global_time);
    ROS_GET_PARAM(ns + "/sync_size", stereo_config.sync_size);
    ROS_GET_PARAM(ns + "/enable_emitter", stereo_config.enable_emitter);
    ROS_GET_PARAM(ns + "/frame_rate", stereo_config.frame_rate);
    ROS_GET_PARAM(ns + "/format", stereo_config.format);
    ROS_GET_PARAM(ns + "/width", stereo_config.width);
    ROS_GET_PARAM(ns + "/height", stereo_config.height);
    ROS_GET_PARAM(ns + "/exposure", stereo_config.exposure);

    // added by Saeed Bastani
    const std::string nsc = "camera";
    ROS_GET_PARAM(nsc + "/global_time", global_time);
    ROS_GET_PARAM(nsc + "/sync_size", color_config.sync_size);
    ROS_GET_PARAM(nsc + "/color_fps", color_config.color_fps);
    ROS_GET_PARAM(nsc + "/color_format", color_config.color_format);
    ROS_GET_PARAM(nsc + "/color_width", color_config.color_width);
    ROS_GET_PARAM(nsc + "/color_height", color_config.color_height);



    // Publishers
    // -- Stereo module
    image_transport::ImageTransport it(nh);
    cam0_pub = it.advertise("stereo/camera0/image", 1);
    cam1_pub = it.advertise("stereo/camera1/image", 1);
    // -- Motion module
    gyro0_pub = nh.advertise<Vector3StampedMsg>("motion/gyro0", 1);
    accel0_pub = nh.advertise<Vector3StampedMsg>("motion/accel0", 1);
    imu0_pub = nh.advertise<ImuMsg>("motion/imu0", 1);
    
    // added by saeed bastani
    // -- RGB Camera module
    image_transport::ImageTransport cit(nh);
    color_pub = cit.advertise("camera/color/image_raw", 1);

  }
};

static void stereo_handler(const rs2::frameset &fs,
                           const intel_d435i_node_t &node,
                           const bool debug = false) {
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

//added vy Saeed Bastani
static void color_handler(const rs2::frameset &fc,
                           const intel_d435i_node_t &node,
                           const bool debug = false) {

           for (auto it = fc.begin(); it != fc.end(); ++it)
            {
                auto f = (*it);
                auto stream_type = f.get_profile().stream_type();
                auto stream_index = f.get_profile().stream_index();
                auto stream_format = f.get_profile().format();
                auto stream_unique_id = f.get_profile().unique_id();

               //printf("Frameset contain (%s, %d, %s %d) \n",
               //             rs2_stream_to_string(stream_type), stream_index, rs2_format_to_string(stream_format), stream_unique_id);
                
            }  
  if (fc.size() != 1) {
    printf("received multiple frames in the color handler %d \n", fc.size());
    return;
  }

  // Create cv::Mat image
  const auto color_frame = fc.get_color_frame();
  const int width = color_frame.get_width();
  const int height = color_frame.get_height();
  cv::Mat mFrame = frame2cvmat_color(color_frame, width, height);


  // Build image messages
  const auto color_msg = create_color_image_msg(color_frame, "_camera_color");
  


  // Publish image messages
  node.color_pub.publish(color_msg);
  //printf("received a frame in color stream");

  // Debug
  if (debug) {
    debug_imshow_color(mFrame);
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
    rs_camera_module_t color{device, node.color_config};

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
    // Process color stream
    std::thread color_thread([&]() {
      while (true) {
        const auto fc = color.waitForFrame();
        color_handler(fc, node);
      }
    });
    // Join threads
    motion_thread.join();
    stereo_thread.join();
    color_thread.join();

  } catch (const rs2::error &e) {
    FATAL("[RealSense Exception]: %s", e.what());
  }

  return 0;
}
