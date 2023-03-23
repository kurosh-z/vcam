
#ifndef DEVICE_H
#define DEVICE_H
#include <algorithm>
#include <dirent.h>
#include <fcntl.h>
#include <libudev.h>
#include <linux/videodev2.h>
#include <ros/ros.h>
#include <string>
#include <sys/ioctl.h>
#include <vector>
#define USB 1
#define MIPI 2
#define FAILURE -1
#define SUCCESS 0
#define V4L2_CAP_META_CAPTURE 0x00800000

namespace vio_cam {

class Devices {
private:
  std::vector<std::string> device_node_name;
  std::vector<std::string> camera_name;
  bool check_for_valid_videonode(std::string &dev_node);
  int check_camera_type(std::string &dev_node, std::string *productName,
                        int *type);
  bool get_product_name(std::string &dev_node);
  bool isEnumerated(std::string node_name);
  void prepare_topic(std::string *productName);

public:
  int list_devices();
  int get_camera_count();
  bool remove_camera(std::string cam_name);
  bool get_camera(int index, std::string *cam_name, std::string *dev_node_name);
  std::vector<std::string> get_list();
};
} // namespace vio_cam

#endif