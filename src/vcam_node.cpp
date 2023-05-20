#include <nodelet/loader.h>
#include <ros/ros.h>
#include <string>

int main(int argc, char **argv) {
  ros::init(argc, argv, "vcam");
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "vcam/VCamNodelet", remap, nargv);
  ros::spin();
}