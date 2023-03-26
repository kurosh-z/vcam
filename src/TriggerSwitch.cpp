#include "mavros_msgs/CommandTriggerControl.h"
#include "ros/ros.h"
#include <cstdlib>
#include <std_srvs/SetBool.h>
#include <std_srvs/SetBoolRequest.h>
#include <string>

class TriggerSwitch {

public:
  TriggerSwitch() {
    triggerClient_ =
        nodeHandle_.serviceClient<mavros_msgs::CommandTriggerControl>(
            "/mavros/cmd/trigger_control");
    advertiseService();
  }

  bool servCameraTriggerCB(std_srvs::SetBool::RequestType &req,
                           std_srvs::SetBool::Response &resp) {

    if (req.data) {
      mavros_tr_ctl_srv_.request.trigger_enable = true;
    } else {
      mavros_tr_ctl_srv_.request.trigger_enable = false;
    }

    if (triggerClient_.call(mavros_tr_ctl_srv_)) {
      resp.success = true;
      ROS_INFO(req.data ? "Successfully enabled camera trigger"
                        : "Successfully disabled camera trigger");
      resp.success = true;
      return true;
    }
    ROS_ERROR("Failed to call trigger_control service");
    resp.success = false;
    return false;
  }

  void advertiseService() {
    serverCam0_ =
        nodeHandle_.advertiseService("See3CAM_20CUG_27049809/trigger_switch",
                                     &TriggerSwitch::servCameraTriggerCB, this);
  }

private:
  ros::NodeHandle nodeHandle_;

  ros::ServiceClient triggerClient_;
  mavros_msgs::CommandTriggerControl mavros_tr_ctl_srv_;

  ros::ServiceServer serverCam0_;
  ros::ServiceServer serverCam1_;
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "TriggerSwitch");
  TriggerSwitch tr;
  ros::spin();
}
