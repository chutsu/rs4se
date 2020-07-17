#include <signal.h>
#include <unistd.h>
#include <thread>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <image_transport/image_transport.h>

#include <librealsense2/rs.hpp>

#include "rs4se.hpp"

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

static sensor_msgs::ImagePtr create_image_msg(const rs2::video_frame &vf,
                                              const std::string &frame_id,
                                              bool is_color) {
  // Form msg stamp
  const uint64_t ts_ns = vframe2ts(vf);
  ros::Time msg_stamp;
  msg_stamp.fromNSec(ts_ns);

  // Form msg header
  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = msg_stamp;

  // Image message
  const int width = vf.get_width();
  const int height = vf.get_height();
  if (is_color) {
    cv::Mat cv_frame = frame2cvmat(vf, width, height, CV_8UC3);
    return cv_bridge::CvImage(header, "rgb8", cv_frame).toImageMsg();
  } else {
    cv::Mat cv_frame = frame2cvmat(vf, width, height, CV_8UC1);
    return cv_bridge::CvImage(header, "mono8", cv_frame).toImageMsg();
  }
}

static sensor_msgs::ImagePtr create_depth_msg(const rs2::depth_frame &df,
                                              const std::string &frame_id) {
  // Form msg stamp
  const uint64_t ts_ns = vframe2ts(df);
  // should work fine since depth_frame is derived from video frame
  ros::Time msg_stamp;
  msg_stamp.fromNSec(ts_ns);

  // Form msg header
  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = msg_stamp;

  // Image message
  const int width = df.get_width();
  const int height = df.get_height();
  cv::Mat cv_frame = frame2cvmat(df, width, height, CV_16UC1);
  const auto msg = cv_bridge::CvImage(header, "mono16", cv_frame).toImageMsg();

  return msg;
}

static geometry_msgs::Vector3Stamped
create_vec3_msg(const rs2::motion_frame &f, const std::string &frame_id) {
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
                                       const Eigen::Vector3d &accel,
                                       const std::string &frame_id) {
  sensor_msgs::Imu msg;

  msg.header.frame_id = frame_id;
  msg.header.stamp = ros::Time{ts};
  msg.angular_velocity.x = gyro(0);
  msg.angular_velocity.y = gyro(1);
  msg.angular_velocity.z = gyro(2);
  msg.linear_acceleration.x = accel(0);
  msg.linear_acceleration.y = accel(1);
  msg.linear_acceleration.z = accel(2);

  return msg;
}

struct intel_d435i_node_t {
  image_transport::Publisher rgb0_pub_;
  image_transport::Publisher ir0_pub_;
  image_transport::Publisher ir1_pub_;
  image_transport::Publisher depth0_pub_;
  ros::Publisher gyro0_pub_;
  ros::Publisher accel0_pub_;
  ros::Publisher imu0_pub_;

  rs_motion_module_config_t motion_config_;
  rs_rgbd_module_config_t rgbd_config_;

  intel_d435i_node_t(const std::string &nn) {
    ros::NodeHandle nh;

    // ROS params
    // -- RGBD
    ROS_PARAM(nh, nn + "/global_time", rgbd_config_.global_time);
    ROS_PARAM(nh, nn + "/enable_rgb", rgbd_config_.enable_rgb);
    ROS_PARAM(nh, nn + "/enable_ir", rgbd_config_.enable_ir);
    ROS_PARAM(nh, nn + "/enable_depth", rgbd_config_.enable_depth);
    ROS_PARAM(nh, nn + "/enable_emitter", rgbd_config_.enable_emitter);
    ROS_PARAM(nh, nn + "/rgb_width", rgbd_config_.rgb_width);
    ROS_PARAM(nh, nn + "/rgb_height", rgbd_config_.rgb_height);
    ROS_PARAM(nh, nn + "/rgb_format", rgbd_config_.rgb_format);
    ROS_PARAM(nh, nn + "/rgb_frame_rate", rgbd_config_.rgb_frame_rate);
    ROS_PARAM(nh, nn + "/rgb_exposure", rgbd_config_.rgb_exposure);
    ROS_PARAM(nh, nn + "/ir_width", rgbd_config_.ir_width);
    ROS_PARAM(nh, nn + "/ir_height", rgbd_config_.ir_height);
    ROS_PARAM(nh, nn + "/ir_format", rgbd_config_.ir_format);
    ROS_PARAM(nh, nn + "/ir_frame_rate", rgbd_config_.ir_frame_rate);
    ROS_PARAM(nh, nn + "/ir_exposure", rgbd_config_.ir_exposure);
    ROS_PARAM(nh, nn + "/depth_width", rgbd_config_.depth_width);
    ROS_PARAM(nh, nn + "/depth_height", rgbd_config_.depth_height);
    ROS_PARAM(nh, nn + "/depth_format", rgbd_config_.depth_format);
    ROS_PARAM(nh, nn + "/depth_frame_rate", rgbd_config_.depth_frame_rate);
    // -- Motion monule
    ROS_PARAM(nh, nn + "/global_time", motion_config_.global_time);
    ROS_PARAM(nh, nn + "/enable_motion", motion_config_.enable_motion);
    ROS_PARAM(nh, nn + "/accel_hz", motion_config_.accel_hz);
    ROS_PARAM(nh, nn + "/gyro_hz", motion_config_.gyro_hz);

    // Publishers
    const auto rgb0_topic = nn + "/rgb0/image";
    const auto ir0_topic = nn + "/ir0/image";
    const auto ir1_topic = nn + "/ir1/image";
    const auto depth0_topic = nn + "/depth0/image";
    const auto gyro0_topic = nn + "/gyro0/data";
    const auto accel0_topic = nn + "/accel0/data";
    const auto imu0_topic = nn + "/imu0/data";
    // -- RGB module
    if (rgbd_config_.enable_rgb) {
      image_transport::ImageTransport rgb_it(nh);
      rgb0_pub_ = rgb_it.advertise(rgb0_topic, 100);
    }
    // -- Stereo module
    if (rgbd_config_.enable_ir) {
      image_transport::ImageTransport stereo_it(nh);
      ir0_pub_ = stereo_it.advertise(ir0_topic, 100);
      ir1_pub_ = stereo_it.advertise(ir1_topic, 100);
    }
    if (rgbd_config_.enable_depth) {
      image_transport::ImageTransport depth_it(nh);
      depth0_pub_ = depth_it.advertise(depth0_topic, 100);
    }
    // -- Motion module
    if (motion_config_.enable_motion) {
      gyro0_pub_ = nh.advertise<Vector3StampedMsg>(gyro0_topic, 100);
      accel0_pub_ = nh.advertise<Vector3StampedMsg>(accel0_topic, 100);
      imu0_pub_ = nh.advertise<ImuMsg>(imu0_topic, 100);
    }
  }

  void publish_ir_msgs(const rs2::video_frame &ir0,
                       const rs2::video_frame &ir1) {
    const auto cam0_msg = create_image_msg(ir0, "rs/ir0", false);
    const auto cam1_msg = create_image_msg(ir1, "rs/ir1", false);
    ir0_pub_.publish(cam0_msg);
    ir1_pub_.publish(cam1_msg);
  }

  void publish_rgb0_msg(const rs2::video_frame &rgb) {
    const auto msg = create_image_msg(rgb, "rs/rgb0", true);
    rgb0_pub_.publish(msg);
  }

  void publish_depth0_msg(const rs2::video_frame &depth) {
    const auto msg = create_depth_msg(depth, "rs/depth0");
    depth0_pub_.publish(msg);
  }

  void publish_accel0_msg(const rs2::motion_frame &mf) {
    const auto msg = create_vec3_msg(mf, "rs/accel0");
    accel0_pub_.publish(msg);
  }

  void publish_gyro0_msg(const rs2::motion_frame &mf) {
    const auto msg = create_vec3_msg(mf, "rs/gyro0");
    gyro0_pub_.publish(msg);
  }

  void publish_imu0_msg(lerp_buf_t &buf) {
    while (buf.lerped_gyro_ts_.size()) {
      // Timestamp
      const auto ts = buf.lerped_gyro_ts_.front();
      buf.lerped_gyro_ts_.pop_front();
      buf.lerped_accel_ts_.pop_front();

      // Accel
      const auto accel = buf.lerped_accel_data_.front();
      buf.lerped_accel_data_.pop_front();

      // Gyro
      const auto gyro = buf.lerped_gyro_data_.front();
      buf.lerped_gyro_data_.pop_front();

      // Publish imu messages
      const auto msg = create_imu_msg(ts, gyro, accel, "rs/imu0");
      imu0_pub_.publish(msg);
    }

    buf.clear();
  }

  void stream() {
    // Callback
    lerp_buf_t lerp_buf;
    auto cb = [&](const rs2::frame &frame) {
      // Handle motion frame
      if (auto mf = frame.as<rs2::motion_frame>()) {
        if (mf && mf.get_profile().stream_type() == RS2_STREAM_ACCEL) {
          publish_accel0_msg(mf);

          double ts_s = mf.get_timestamp() * 1e-3;
          const rs2_vector data = mf.get_motion_data();
          lerp_buf.addAccel(ts_s, data.x, data.y, data.z);

        } else if (mf && mf.get_profile().stream_type() == RS2_STREAM_GYRO) {
          publish_gyro0_msg(mf);

          double ts_s = mf.get_timestamp() * 1e-3;
          const rs2_vector data = mf.get_motion_data();
          lerp_buf.addGyro(ts_s, data.x, data.y, data.z);
        }

        if (lerp_buf.ready()) {
          lerp_buf.interpolate();
          publish_imu0_msg(lerp_buf);
        }
        return;
      }

      // Handle camera frames
      if (const auto &fs = frame.as<rs2::frameset>()) {
        // IR0 & IR1
        const bool enable_ir = rgbd_config_.enable_ir;
        const auto ir0 = fs.get_infrared_frame(1);
        const auto ir1 = fs.get_infrared_frame(2);
        if (enable_ir && ir0 && ir1) { publish_ir_msgs(ir0, ir1); }

        // Align depth to rgb
        rs2::align align_to_color(RS2_STREAM_COLOR);
        const auto fs_aligned = align_to_color.process(fs);

        // RGB
        const bool enable_rgb = rgbd_config_.enable_rgb;
        const auto rgb = fs_aligned.get_color_frame();
        if (enable_rgb && rgb) { publish_rgb0_msg(rgb); }

        // Depth image
        const bool enable_depth = rgbd_config_.enable_depth;
        const auto depth = fs_aligned.get_depth_frame();
        if (enable_depth && depth) { publish_depth0_msg(depth); }
      }
    };

    // Connect and stream
    rs2::device device = rs2_connect();
    intel_d435i_t sensor(device, rgbd_config_, motion_config_, cb);

    // Pipelines are threads so we need a blocking loop
    signal(SIGINT, signal_handler);
    while (keep_running) {
      sleep(1);
    }
  }
};

int main(int argc, char **argv) {
  // ROS init
  const std::string node_name = ros_node_name(argc, argv);
  if (ros::isInitialized() == false) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  // Start ROS node
  try {
    intel_d435i_node_t node(node_name);
    node.stream();
  } catch (const rs2::error &e) {
    FATAL("[RealSense Exception]: %s", e.what());
  }

  return 0;
}
