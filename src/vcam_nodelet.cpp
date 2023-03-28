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
                  << " max fps:"
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

      cam.request_format("GREY", 640, 480, 30, true);
      startFrameGrabber();
      ros::service::waitForService(cam_name_ + "/trigger_switch");
      switchCameraTrigger(false);
      switchCameraTrigger(true);
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

  int prevNumSubscribers = 0;
  int currNumSubscribers = 0;
  while (frame_grab_alive_ && ros::ok()) {

    ros_image_.is_bigendian = 0;
    ros_image_.header.frame_id = "/" + frame_name_;

#ifdef DEBUG_PRINTOUT_FRAME_GRAB_RATES
    startGrabCount++;
    currStartGrab = ros::Time::now();

    if (startGrabCount > 1) {
      startGrabSum += (currStartGrab - prevStartGrab).toSec() * 1000.0;
      startGrabSumSqrd += ((currStartGrab - prevStartGrab).toSec() * 1000.0) *
                          ((currStartGrab - prevStartGrab).toSec() * 1000.0);
    }

    prevStartGrab = currStartGrab;
#endif

    bool capture_succeeded = cam.capture(&ros_image_) == 0;
    if (capture_succeeded) {
      DEBUG_STREAM("(Re-)allocated ROS image buffer for ["
                   << cam_name_ << "]:"
                   << "\n  width: " << ros_image_.width << "\n  height: "
                   << ros_image_.height << "\n  step: " << ros_image_.step
                   << "\n  encoding: " << ros_image_.encoding);
      ros_cam_info_.header.stamp = ros_image_.header.stamp;
      ros_cam_info_.width = ros_image_.width;
      ros_cam_info_.height = ros_image_.height;
      ros_image_.header.seq = ros_cam_info_.header.seq = ros_frame_count_++;
      ros_image_.header.frame_id = ros_cam_info_.header.frame_id;

      image_buffer_.push_back(ros_image_);
      cinfo_buffer_.push_back(ros_cam_info_);

      if (image_buffer_.size() && timestamp_buffer_.size()) {
        unsigned int i;

        for (i = 0; i < image_buffer_.size() && timestamp_buffer_.size() > 0;) {
          i += stampAndPublishImage(i);
        }
      }
      // Check whether buffer has stale data and if so, throw away oldest
      if (image_buffer_.size() > 100) {
        image_buffer_.erase(image_buffer_.begin());
        ROS_ERROR_STREAM("image buffer is full dropping images ");
      }

      if (cinfo_buffer_.size() > 100) {
        cinfo_buffer_.erase(cinfo_buffer_.begin());
      }
    }
    // ros_image_.header.seq = ros_cam_info_.header.seq = ros_frame_count_++;
    // ros_image_.header.frame_id = ros_cam_info_.header.frame_id;
    // // ros_image_.header.stamp = ros::Time::now();
    // ros_cam_pub_.publish(ros_image_, ros_cam_info_);
  }
}

void VCamNodelet::bufferTimestamp(const mavros_msgs::CamIMUStamp &msg) {
  if (!zero_frame_id_set) {
    zero_timestamp_frame_id = msg.frame_seq_id - 1;
    zero_frame_id_set = true;
  }

  timestamp_buffer_.push_back(msg);
  // if (timestamp_buffer_.size() < 100) {
  //   ROS_INFO_STREAM("timestamp frame_seq_id: " << msg.frame_seq_id);
  //   timestamp_buffer_.push_back(msg);
  // }

  // Check whether buffer has stale stamp and if so throw away oldest
  if (timestamp_buffer_.size() > 100) {
    timestamp_buffer_.erase(timestamp_buffer_.begin());
    ROS_ERROR_THROTTLE(1, "Dropping timestamp");
  }
}

int VCamNodelet::findInStampBuffer(unsigned int index) {
  if (image_buffer_.size() < 1) {
    return 0;
  }
  // check which timestamp corresponds to the same frame sequence
  unsigned int k = 0;
  while (k < timestamp_buffer_.size() && ros::ok()) {
    if (image_buffer_.at(index).header.seq + zero_timestamp_frame_id ==
        (uint)timestamp_buffer_.at(k).frame_seq_id) {
      return k;
    } else {
      k += 1;
    }
  }
  return 0;
}
int VCamNodelet::stampAndPublishImage(unsigned int index) {
  int timestamp_index = findInStampBuffer(index);
  if (timestamp_index) {
    sensor_msgs::Image image = image_buffer_.at(timestamp_index);
    sensor_msgs::CameraInfo cinfo = cinfo_buffer_.at(timestamp_index);

    double timestamp =
        image.header.stamp.toSec() +
        timestamp_buffer_.at(timestamp_index).frame_stamp.toSec();

    // image.header.stamp = ros::Time(timestamp);
    image.header.stamp = ros::Time::now();
    cinfo.header = image.header;
    ros_cam_pub_.publish(image, cinfo);
    image_buffer_.erase(image_buffer_.begin() + index);
    cinfo_buffer_.erase(cinfo_buffer_.begin() + index);
    timestamp_buffer_.erase(timestamp_buffer_.begin() + timestamp_index);
    return 0;
  }
  return 1;
}

} // namespace vio_cam

PLUGINLIB_EXPORT_CLASS(vio_cam::VCamNodelet, nodelet::Nodelet)