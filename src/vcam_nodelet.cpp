#include "vcam/vcam_nodelet.hpp"
#include <pluginlib/class_list_macros.h>

namespace vio_cam {
const std::string VCamNodelet::DEFAULT_FRAME_NAME = "base_link";
const std::string VCamNodelet::DEFAULT_CAMERA_TOPIC = "cameratopic";
VCamNodelet::VCamNodelet() { ROS_INFO("VCamNodelet Constructor"); }

VCamNodelet::~VCamNodelet() { ROS_INFO("VCamNodelet Destructor"); }

void VCamNodelet::onInit() {

  NODELET_INFO("VCamNodelet - %s", __FUNCTION__);

  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &local_nh = getPrivateNodeHandle();
  image_transport::ImageTransport it(nh);
  ros_frame_count_ = 0;
  Devices dev = Devices();
  int ret = dev.list_devices();
  if (ret == -1) {
    ROS_ERROR("could not find any camera");
  } else {
    dev.get_camera(0, &cam_name_, &video_node);
    dev_node_name += video_node;
    ROS_INFO("dev_node_name %s", dev_node_name.c_str());
    ret = cam.open_device(dev_node_name.c_str());
    if (ret == 0) {
      cam.cam_init();
      ros_timestamp_sub_ = nh.subscribe("/mavros/cam_imu_sync/cam_imu_stamp", 1,
                                        &VCamNodelet::bufferTimestamp, this);
      ros_cam_pub_ = it.advertiseCamera(cam_name_ + "/" + cam_topic_, 1);
      set_cam_info_srv_ = nh.advertiseService(cam_name_ + "/set_camera_info",
                                              &VCamNodelet::setCamInfo, this);

      std::vector<Camera::CamResFormat> res_list;
      Camera::CamResFormat current_res;
      cam.enum_resolution(&res_list, &current_res);
      ROS_INFO("supproted resolutions:");
      for (auto res_format : res_list) {
        std::cout << "pixelformat: " << res_format.pixelformat
                  << " res: " << res_format.widht << "*" << res_format.height
                  << "max fps:"
                  // << 20
                  << *std::max_element(res_format.fps.begin(),
                                       res_format.fps.end())
                  << std::endl;
      }
      ROS_INFO("current resolution:");
      std::cout << current_res.pixelformat << current_res.widht << "*"
                << current_res.height << std::endl;
      cam_trigger_srv_ =
          nh.serviceClient<std_srvs::SetBool>(cam_name_ + "/trigger_switch");

      // startFrameGrabber();
      // bool trigSuccess = switchCameraTrigger(true);
      // if (!trigSuccess) {
      //   ROS_ERROR("could not switch camera trigger");
      // }
    }
  }
};

bool VCamNodelet::switchCameraTrigger(bool state) {
  std_srvs::SetBool sig;
  sig.request.data = state;
  if (!cam_trigger_srv_.call(sig)) {
    ROS_ERROR("failed to cal ready_for_trigger");
    return false;
  };
  return true;
};

bool VCamNodelet::setCamInfo(sensor_msgs::SetCameraInfo::Request &req,
                             sensor_msgs::SetCameraInfo::Response &rsp) {
  ros_cam_info_ = req.camera_info;
  ros_cam_info_.header.frame_id = frame_name_;
  rsp.success = true;
  // TODO: write calibration file here
  return true;
}

void VCamNodelet::startFrameGrabber() {
  frame_grab_alive_ = true;
  frame_grab_thread_ =
      std::thread(std::bind(&VCamNodelet::frameGrabLoop, this));
}

void VCamNodelet::frameGrabLoop() {
#ifdef DEBUG_PRINTOUT_FRAME_GRAB_RATES
  ros::Time prevStartGrab = ros::Time::now();
  ros::Time prevGrabbedFrame = ros::Time::now();
  ros::Time currStartGrab;
  ros::Time currGrabbedFrame;
  double startGrabSum = 0;
  double grabbedFrameSum = 0;
  double startGrabSumSqrd = 0;
  double grabbedFrameSumSqrd = 0;
  unsigned int startGrabCount = 0;
  unsigned int grabbedFrameCount = 0;
#endif
  DEBUG_STREAM("Starting threaded frame grabber loop for [" << cam_name_
                                                            << "]");
  ros::Rate idleDelay(200);
  int prevNumSubscribers = 0;
  int currNumSubscribers = 0;
  while (frame_grab_alive_ && ros::ok()) {

    ros_image_.is_bigendian = 0;
    ros_image_.header.frame_id = "/" + frame_name_;
    cam.capture(&ros_image_);
    DEBUG_STREAM("(Re-)allocated ROS image buffer for ["
                 << cam_name_ << "]:"
                 << "\n  width: " << ros_image_.width << "\n  height: "
                 << ros_image_.height << "\n  step: " << ros_image_.step
                 << "\n  encoding: " << ros_image_.encoding);

    ros_image_.header.seq = ros_cam_info_.header.seq = ros_frame_count_++;
    ros_image_.header.frame_id = ros_cam_info_.header.frame_id;
    ros_image_.header.stamp = ros::Time::now();
    ros_cam_pub_.publish(ros_image_, ros_cam_info_);
  }
}

void VCamNodelet::bufferTimestamp(const mavros_msgs::CamIMUStamp &msg) {

  timestamp_buffer_.push_back(msg);

  // Check whether buffer has stale stamp and if so throw away oldest
  if (timestamp_buffer_.size() > 1000) {
    timestamp_buffer_.erase(timestamp_buffer_.begin());
    ROS_ERROR_THROTTLE(1, "Dropping timestamp");
  }
}

} // namespace vio_cam

PLUGINLIB_EXPORT_CLASS(vio_cam::VCamNodelet, nodelet::Nodelet)