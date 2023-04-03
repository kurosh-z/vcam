#include "vcam/vcam_nodelet.hpp"
#include <iomanip>
#include <pluginlib/class_list_macros.h>

namespace vio_cam {
const std::string VCamNodelet::DEFAULT_FRAME_NAME = "base_link";
const std::string VCamNodelet::DEFAULT_CAMERA_TOPIC = "cameratopic";
VCamNodelet::VCamNodelet() { ROS_INFO("VCamNodelet Constructor"); }

VCamNodelet::~VCamNodelet() {
  switchCameraTrigger(false);
  ROS_INFO("VCamNodelet Destructor");
}

void VCamNodelet::onInit() {
  ROS_INFO("=================================================================");
  NODELET_INFO("VCamNodelet - %s", __FUNCTION__);

  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &local_nh = getPrivateNodeHandle();
  auto rate = ros::Rate(1);

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
      // cam.set_exposure(30);
      rate.sleep();
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

      // cam.request_format("GREY", 640, 480, 30, true);

      rate.sleep();
      rate.sleep();

      startFrameGrabber();
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
  ROS_INFO("\n=================== Start Frame Grabber =====================\n");
  ros::service::waitForService(cam_name_ + "/trigger_switch");
  switchCameraTrigger(true);
  frame_grab_alive_ = true;
  frame_grab_thread_ =
      std::thread(std::bind(&VCamNodelet::frameGrabLoop, this));
}

void VCamNodelet::frameGrabLoop() {

#ifdef DEBUG_PRINTOUT_FRAME_GRAB_RATES
  ros::Time prevFrameTime = ros::Time::now();
  ros::Time curretnFrameTime = ros::Time::now();
#endif
  DEBUG_STREAM("Starting threaded frame grabber loop for [" << cam_name_
                                                            << "]");
  ros::Rate idleDelay(500);

  ros_frame_count_ = 0;
  while (frame_grab_alive_ && ros::ok()) {

    ros_image_.is_bigendian = 0;
    ros_image_.header.frame_id = "/" + frame_name_;

    bool capture_succeeded = cam.capture(&ros_image_) == 0;
    if (capture_succeeded) {

      // DEBUG_STREAM("(Re-)allocated ROS image buffer for ["
      //              << cam_name_ << "]:"
      //              << "\n  width: " << ros_image_.width << "\n  height: "
      //              << ros_image_.height << "\n  step: " << ros_image_.step
      //              << "\n  encoding: " << ros_image_.encoding);
      ros_image_.header.seq = ros_cam_info_.header.seq = ++ros_frame_count_;
      ros_image_.header.frame_id = ros_cam_info_.header.frame_id;
      ros_cam_info_.header.stamp = ros_image_.header.stamp;
      ros_cam_info_.width = ros_image_.width;
      ros_cam_info_.height = ros_image_.height;
      // ros_image_.header.seq = ros_cam_info_.header.seq = ros_frame_count_++;
      // ros_image_.header.frame_id = ros_cam_info_.header.frame_id;
      // ROS_INFO_STREAM(" " << ros::Time::now().toSec() - start
      //                     << " image recieved" << ros_image_.header.seq);

      image_buffer_.push_back(ros_image_);
      cinfo_buffer_.push_back(ros_cam_info_);
      ROS_INFO_STREAM("[frameGrabLoop] image seq: " << ros_image_.header.seq);

      if (skippFrame && timestamp_buffer_.size() > 2 && ros_frame_count_ > 2) {
        int iLast = image_buffer_.size() - 1;
        int tLast = timestamp_buffer_.size() - 1;
        float frame_interval =
            1000 * (image_buffer_.at(iLast).header.stamp.toSec() -
                    image_buffer_.at(iLast - 1).header.stamp.toSec());

        if (frame_interval > (IDEAL_INTERVAL - 5) && frame_interval < (IDEAL_INTERVAL + 5)) {
          timestamp_buffer_.erase(timestamp_buffer_.begin(),
                                  timestamp_buffer_.begin() + tLast);
          zero_timestamp_frame_id = timestamp_buffer_.at(0).frame_seq_id - 1;
          skippFrame = false;
          ros_frame_count_ = 1;
          image_buffer_.erase(image_buffer_.begin(),
                              image_buffer_.begin() + iLast);
          ROS_INFO_STREAM("synchronized ");
        }
      }
      // if (skippFrame && ros_frame_count_ > 3) {
      //   int last_Tidx = timestamp_buffer_.size() - 1;
      //   int i = 0;
      //   int count = 0;
      //   for (int fIdx = image_buffer_.size() - 1;
      //        fIdx > 0 && (last_Tidx - i) > -1 && skippFrame; fIdx--) {

      //     auto tirgger_stamp = timestamp_buffer_.at(last_Tidx - i);
      //     auto frame = image_buffer_.at(fIdx);
      //     int diff_ms = 1000 * (frame.header.stamp.toSec() -
      //                           tirgger_stamp.frame_stamp.toSec());

      //     ROS_INFO_STREAM("diff_ms: " << std::fixed << std::setprecision(5)
      //                                 << diff_ms);

      //     if (diff_ms > 0) {
      //       count++;
      //     }
      //     if (count > 1) {
      //       ROS_INFO_STREAM(
      //           "timestamp_buffer_ size: " << timestamp_buffer_.size());
      //       ROS_INFO_STREAM("image_buffer_ size: " << image_buffer_.size());
      //       skippFrame = false;
      //       int len = timestamp_buffer_.size();
      //       timestamp_buffer_.erase(timestamp_buffer_.begin() + len - 1);
      //       zero_timestamp_frame_id = timestamp_buffer_.at(0).frame_seq_id -
      //       1; ros_frame_count_ = 1; len = image_buffer_.size();
      //       image_buffer_.erase(image_buffer_.begin() + len - 1);
      //       image_buffer_.at(0).header.seq = 1;
      //       ROS_INFO("frames and timestapmed are synched");
      //     }
      //     i++;
      //   }
      // }
      // if ((ros_image_.header.stamp.toSec() -
      //      timestamp_buffer_.back().frame_stamp.toSec()) > 0 &&
      //     100 * (ros_image_.header.stamp.toSec() -
      //            timestamp_buffer_.back().frame_stamp.toSec()) <
      //         5.0 &&
      //     skippFrame) {

      //   int len = timestamp_buffer_.size();
      //   timestamp_buffer_.erase(timestamp_buffer_.begin() + len - 1);
      //   zero_timestamp_frame_id = timestamp_buffer_.at(0).frame_seq_id - 1;
      //   len = image_buffer_.size();
      //   image_buffer_.erase(image_buffer_.begin() + len - 1);
      //   image_buffer_.at(0).header.seq = 1;
      //   ros_frame_count_ = 1;
      //   skippFrame = false;
      //   ROS_INFO_STREAM("frame and trigger are synched: "
      //                   << std::fixed << std::setprecision(4)
      //                   << 1000 *
      //                          (ros_image_.header.stamp.toSec() -
      //                           timestamp_buffer_.back().frame_stamp.toSec()));
      // }

#ifdef DEBUG_PRINTOUT_FRAME_GRAB_RATES
      prevFrameTime = curretnFrameTime;
      curretnFrameTime = ros::Time::now();
      ROS_INFO_STREAM("frame interval:"
                      << std::fixed << std::setprecision(5)
                      << curretnFrameTime.toSec() - prevFrameTime.toSec()
                      << std ::endl);

#endif

      // TODO: calculate this to make sure trigger signal actually works!
      if (image_buffer_.size() > 1) {
        // ROS_INFO_STREAM("frame interval-> "
        //                 << ros_image_.header.stamp.toSec() -
        //                        image_buffer_.at(image_buffer_.size() - 2)
        //                            .header.stamp.toSec());
      }

      if (image_buffer_.size() && timestamp_buffer_.size() && !skippFrame) {
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
      // ros_image_.header.seq = ros_cam_info_.header.seq = ros_frame_count_++;
      // ros_image_.header.frame_id = ros_cam_info_.header.frame_id;
      // ros_image_.header.stamp = ros::Time::now();
      // ros_cam_pub_.publish(ros_image_, ros_cam_info_);
      // ROS_INFO_STREAM("frame " << ros_image_.header.seq << " published");
      // image_buffer_.erase(image_buffer_.begin());
    }
    idleDelay.sleep();
  }
}

void VCamNodelet::bufferTimestamp(const mavros_msgs::CamIMUStamp &msg) {
  // if (!zero_frame_id_set) {
  //   ROS_INFO_STREAM("first camIMUStamp: " << msg.frame_seq_id);
  //   zero_timestamp_frame_id = msg.frame_seq_id - 1;
  //   zero_frame_id_set = true;
  // }

  timestamp_buffer_.push_back(msg);
  ROS_INFO_STREAM("[bufferTimestamp]: trig seq: "
                  << msg.frame_seq_id - zero_timestamp_frame_id);
#ifdef DEBUG_PRINTOUT_FRAME_GRAB_RATES
  prevTriggerTime = currentTriggerTime;
  currentTriggerTime = ros::Time::now();
  ROS_INFO_STREAM("trigger interval:"
                  << std::fixed << std::setprecision(4)
                  << currentTriggerTime.toSec() - prevTriggerTime.toSec()
                  << std::endl);

#endif
  // uint len = timestamp_buffer_.size();
  // if (len > 1) {

  //   ROS_INFO_STREAM(" tr seq: "
  //                   << msg.frame_seq_id << " time diff: "
  //                   << msg.frame_stamp.toSec() -
  //                          timestamp_buffer_.at(len -
  //                          2).frame_stamp.toSec());
  // }
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
  return -1;
}
int VCamNodelet::stampAndPublishImage(unsigned int index) {
  int timestamp_index = findInStampBuffer(index);
  if (timestamp_index != -1) {
    sensor_msgs::Image image = image_buffer_.at(index);
    sensor_msgs::CameraInfo cinfo = cinfo_buffer_.at(index);

    // std::cout << "im_[s]: " << image.header.stamp.toSec() << " imu[s] "
    //           << image_buffer_.at(timestamp_index).header.stamp.toSec()
    //           << std::endl;

    double timestamp =1.6 / 2  +
        timestamp_buffer_.at(timestamp_index).frame_stamp.toSec();

    // ROS_INFO_STREAM(std::fixed // fix the number of decimal digits
    //                 << std::setprecision(18) << "image stamp sec -> "
    //                 << image.header.stamp.toSec());

    image.header.stamp = ros::Time(timestamp);
    // image.header.stamp = ros::Time::now();
    cinfo.header = image.header;
    ros_cam_pub_.publish(image, cinfo);

    // ROS_INFO_STREAM(std::fixed // fix the number of decimal digits
    //                 << std::setprecision(18) << std::setw(30)
    //                 << "[stampAndPublishImage] published image_seq -> "
    //                 << image.header.seq
    //                 << " time: " << image.header.stamp.toSec());
    image_buffer_.erase(image_buffer_.begin() + index);
    cinfo_buffer_.erase(cinfo_buffer_.begin() + index);
    timestamp_buffer_.erase(timestamp_buffer_.begin() + timestamp_index);
    return 0;
  }
  return 1;
}

} // namespace vio_cam

PLUGINLIB_EXPORT_CLASS(vio_cam::VCamNodelet, nodelet::Nodelet)