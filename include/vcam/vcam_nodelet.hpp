
#ifndef V_CAM_NODELET_HPP_
#define V_CAM_NODELET_HPP_
#include "vcam/logging_macros.hpp"
#include <boost/thread/mutex.hpp>
#include <cstdlib>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <thread>
#include <vcam/device.h>
#include <vcam/v4l2_cam_driver.h>

#define DEBUG_PRINTOUT_FRAME_GRAB_RATES

namespace vio_cam {
class VCamNodelet : public nodelet::Nodelet {
public:
  constexpr static unsigned int RECONFIGURE_RUNNING = 0;
  constexpr static unsigned int RECONFIGURE_STOP = 1;
  constexpr static unsigned int RECONFIGURE_CLOSE = 3;
  constexpr static int DEFAULT_IMAGE_WIDTH =
      640; // NOTE: these default values do not matter, as they
  constexpr static int DEFAULT_IMAGE_HEIGHT =
      480; // are overwritten by queryCamParams() during connectCam()
  constexpr static double DEFAULT_EXPOSURE = 30.0;
  constexpr static double DEFAULT_FRAME_RATE = 10.0;
  constexpr static int DEFAULT_PIXEL_CLOCK = 25;

  const static std::string DEFAULT_FRAME_NAME;
  const static std::string DEFAULT_CAMERA_NAME;
  const static std::string DEFAULT_CAMERA_TOPIC;
  const static std::string DEFAULT_TIMEOUT_TOPIC;
  const static std::string DEFAULT_COLOR_MODE;
  VCamNodelet();
  ~VCamNodelet();

  virtual void onInit();

protected:
  vio_cam::Camera cam;
  vio_cam::Devices dev;
  std::string cam_name_;
  std::string video_node;
  std::string dev_node_name = "/dev/";

  std::thread frame_grab_thread_;
  bool frame_grab_alive_;
  image_transport::CameraPublisher ros_cam_pub_;
  sensor_msgs::Image ros_image_;
  sensor_msgs::CameraInfo ros_cam_info_;
  unsigned int ros_frame_count_;
  ros::Publisher timeout_pub_;
  unsigned long long int timeout_count_;

  ros::ServiceServer set_cam_info_srv_;
  std::string frame_name_;
  std::string cam_topic_;
  std::string timeout_topic_;
  std::string cam_intr_filename_;
  std::string cam_params_filename_; // should be valid UEye INI file

  ros::Time init_ros_time_; // for processing frames
  uint64_t init_clock_tick_;

  ros::Time init_publish_time_;    // for throttling frames from being published
                                   // (see cfg.output_rate)
  uint64_t prev_output_frame_idx_; // see init_publish_time_
  boost::mutex output_rate_mutex_;

  bool setCamInfo(sensor_msgs::SetCameraInfo::Request &req,
                  sensor_msgs::SetCameraInfo::Response &rsp);
  void frameGrabLoop();
  void startFrameGrabber();
};

} // namespace vio_cam

#endif /* SAMPLE_NODELET_CLASS_SRC_SAMPLE_NODELET_CLASS_H_ */