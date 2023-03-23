#ifndef V4L2_H
#define V4L2_H
#define PIX_FORMAT 1
#define RESOLUTION 2
#define FPS 3
#define MAX_FRAME_SIZE_SUPPORT 16
#define MAX_PIXELFORMAT_SUPPORT 16
#define MAX_FRMINVAL_SUPPORT 10
#define DISABLE 0
#define ENABLE 1
#define SUCCESS 0
#define FAILURE -1
#define DEV_UNPLUGGED -2
#include <asm/types.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <ros/ros.h>
#include <ros/topic_manager.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <vector>
namespace vio_cam {
class Camera {
private:
  class Buffer {
  public:
    uint8_t *start;
    size_t length;
  }; // end of class Buffer

  class V4L2_structure {
  public:
    struct v4l2_frmsizeenum fsize[MAX_FRAME_SIZE_SUPPORT];
    struct v4l2_fmtdesc ffmt[MAX_PIXELFORMAT_SUPPORT];
    struct v4l2_frmivalenum frminterval;
    struct v4l2_fract desc_frm_inval[MAX_FRMINVAL_SUPPORT];
    struct v4l2_format stream_fmt;
    struct v4l2_streamparm cam_parm;
    struct v4l2_requestbuffers reqbuf;
    struct v4l2_capability cap;
    struct v4l2_buffer buffer;
  }; // end of class Format
  int cam_fd;
  // int subscribers_count;
  int width, height;
  int numerator, denominator;
  bool streamon; // flag to indicate camera is streaming
  bool init;     // flag to indicate initialisation
  std::string pixelformat;
  std::vector<Buffer> buffers;
  std::string camera_name;
  std::string video_node;

  // private member functions
  void enqueue(int index);
  // public of class Camera
public:
  V4L2_structure v4l2; // obj of class Format
  // Member function of class Camera
  Camera();

  int open_device(const std::string &device);

  void cam_init();

  void enum_pixelformat(std::vector<std::string> *list, std::string *str);

  void enum_resolution(std::string pixelformat, std::vector<std::string> *list,
                       std::string *str);

  void enum_framerate(std::string pixelformat, int width, int height,
                      std::vector<std::string> *list, std::string *str);

  int set_format(const std::string &fourcc, int input_width, int input_height);

  void get_format(std::string *PixelFormat, int *Height, int *Width,
                  int *Numerator, int *Denominator, int type);

  int set_framerate(unsigned input_numerator, unsigned input_denominator);

  int request_buffer();

  int stream_on();

  int capture(sensor_msgs::Image *image);

  int stream_off();

  int clean_buffer();

  void munmap_buffers();

  void close_device();

  bool onInit();

  // void set_init(bool value);

  // int sub_count();

  bool isStreamOn();

  int xioctl(unsigned cmd, void *arg);

  void get_four_character_code(int32_t pix_fmt, std::string *pixelformat);

  void set_camera_name(std::string name, std::string node_name);

  std::string get_camera_name();

  void get_node_name(std::string *dev_node_name);

}; // end of class Camera

} // namespace vio_cam
#endif // V4L2_H
