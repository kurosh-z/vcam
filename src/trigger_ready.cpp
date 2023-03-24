#include "mavros_msgs/CommandTriggerControl.h"
#include "ros/ros.h"
#include <cstdlib>
#include <std_srvs/Trigger.h>
#include <string>

class TriggerReady {

public:
  TriggerReady() {
    cam0_OK_ = false;
    cam1_OK_ = true;
    framerate_hz_ = 18; // default framerate TODO get this from the ueye node
    triggerClient_ = n_.serviceClient<mavros_msgs::CommandTriggerControl>(
        "/mavros/cmd/trigger_control");
    advertiseService();
  }

  bool servCam0(std_srvs::Trigger::Request &req,
                std_srvs::Trigger::Response &resp) {
    cam0_OK_ = true;
    resp.success = true;
    ROS_INFO_STREAM("camera: See3CAM_20CUG_27049809 is primed for trigger");
    return true;
  }

  bool servCam1(std_srvs::Trigger::Request &req,
                std_srvs::Trigger::Response &resp) {
    cam1_OK_ = true;
    resp.success = true;
    ROS_INFO_STREAM("Camera 1 is primed for trigger");
    return true;
  }

  bool cam0_OK() { return cam0_OK_; }

  bool cam1_OK() { return cam1_OK_; }

  int enableTrigger() {
    // TODO: set trigger cycle in
    // FC:https://github.com/mavlink/mavros/issues/987
    // srv_.request.cycle_time = (1000 / framerate_hz_);
    srv_.request.trigger_enable = true;

    if (triggerClient_.call(srv_)) {
      ROS_INFO("Successfully enabled camera trigger");

    } else {
      ROS_ERROR("Failed to call trigger_control service");
      return 1;
    }

    return 0;
  }

  int disableTrigger() {
    // srv_.request.cycle_time = 0;
    srv_.request.trigger_enable = false;

    if (triggerClient_.call(srv_)) {
      ROS_INFO("Successfully disabled camera trigger");

    } else {
      ROS_ERROR("Failed to call trigger_control service");
      return 1;
    }

    return 0;
  }

  void advertiseService() {
    serverCam0_ = n_.advertiseService("See3CAM_20CUG_27049809/trigger_ready",
                                      &TriggerReady::servCam0, this);
    serverCam1_ = n_.advertiseService("cam1/trigger_ready",
                                      &TriggerReady::servCam1, this);
  }

private:
  bool cam0_OK_;
  bool cam1_OK_;
  int framerate_hz_;

  ros::NodeHandle n_;

  ros::ServiceClient triggerClient_;
  mavros_msgs::CommandTriggerControl srv_;

  ros::ServiceServer serverCam0_;
  ros::ServiceServer serverCam1_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "StartTrigger");
  TriggerReady tr;

  ros::Rate r2(5); // Hz

  while (tr.disableTrigger() && ros::ok()) {
    ROS_INFO_STREAM("Retrying reaching pixhawk");
    r2.sleep();
  }

  // Define time update rate to call callback function if necessary
  ros::Rate r(100); // Hz

  // while (!(tr.cam0_OK() && tr.cam1_OK()) && ros::ok())
  while (!(tr.cam0_OK()) && ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  // Send start trigger command to Pixhawk
  while (tr.enableTrigger() && ros::ok()) {
    ROS_INFO_STREAM("Retrying reaching pixhawk");
    r2.sleep();
  }
}
