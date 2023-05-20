
#include "vcam/v4l2_cam_driver.h"

namespace vio_cam {

// Constructor of class Camera
Camera::Camera() {
  streamon = false;
  cam_fd = -1;
  init = true;
  memset(&queryctrl, 0, sizeof queryctrl);

  // subscribers_count = 0;
}

/*****************************************************************************
 *  Name	:	open_device.
 *  Parameter1 : const std::string &device - Name of the device node,
 *                                           For ex. /dev/video0
 *  Returns	:	Function result depends on return value of child
 *functions and condition available in the functions based on this return value
 *will
 *
 *  			SUCCESS	- all the condition executed properly
 *  			FAILURE	- Failed to perform specified operation
 *  Description	:   This function is to open the camera file descriptor.
 ****************************************************************************/
int Camera::open_device(const std::string &device) {
  // checking if camera is not already open.
  if (cam_fd == -1) {
    cam_fd = open(device.c_str(), O_RDWR | O_NONBLOCK);
  }
  // if open camera failed cam_fd will be equal to -1
  if (cam_fd < 0) {
    ROS_INFO("Open device Failed");
    return FAILURE;
  }
  memset(&v4l2.cap, 0, sizeof(v4l2.cap));
  ioctl(cam_fd, VIDIOC_QUERYCAP, &v4l2.cap);

  if (!(v4l2.cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    ROS_INFO("Device does not support video capture");
    return FAILURE;
  }

  if (!(v4l2.cap.capabilities & V4L2_CAP_STREAMING)) {
    ROS_INFO("Device does not support streaming I/O");
    return FAILURE;
  }

  return SUCCESS;
}

/*****************************************************************************
 *  Name	:	cam_init.
 *  Description	:   This function is to setup the camera to streamon,
 *                  once it is selected.
 ****************************************************************************/
void Camera::cam_init() {
  memset(&v4l2.stream_fmt, 0, sizeof(v4l2.stream_fmt));
  v4l2.stream_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  v4l2.stream_fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
  if (ioctl(cam_fd, VIDIOC_G_FMT, &v4l2.stream_fmt) < 0) {
    ROS_INFO("VIDIOC_S_FMT failed");
    return;
  }

  get_four_character_code(v4l2.stream_fmt.fmt.pix.pixelformat, &pixelformat);
  set_format(pixelformat, v4l2.stream_fmt.fmt.pix.width,
             v4l2.stream_fmt.fmt.pix.height);

  memset(&v4l2.cam_parm, 0, sizeof(v4l2.cam_parm));
  v4l2.cam_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  ioctl(cam_fd, VIDIOC_G_PARM, &v4l2.cam_parm);
  if (!(v4l2.cam_parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)) {
    ROS_WARN("V4L2_CAP_TIMEPERFRAME not supported by driver");
    ROS_WARN("Leaving frame rate unchanged");
    return;
  }
  numerator = v4l2.cam_parm.parm.capture.timeperframe.numerator;
  denominator = v4l2.cam_parm.parm.capture.timeperframe.denominator;
  request_buffer();
  stream_on();
  qeury_controls();
  init = true;
}

void Camera::enum_pixelformat(std::vector<std::string> *list,
                              std::string *str) {

  for (int cnt = 0; cnt < MAX_PIXELFORMAT_SUPPORT; cnt++) {
    memset(&v4l2.ffmt[cnt], 0, sizeof(v4l2.ffmt[cnt]));
    v4l2.ffmt[cnt].index = cnt;
    v4l2.ffmt[cnt].type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (0 == xioctl(VIDIOC_ENUM_FMT, &v4l2.ffmt[cnt])) {
      get_four_character_code(v4l2.ffmt[cnt].pixelformat, str);
      list->push_back(*str);
    } else {
      break;
    }
  }
  memset(&v4l2.stream_fmt, 0, sizeof(v4l2.stream_fmt));
  v4l2.stream_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  v4l2.stream_fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
  if (0 == xioctl(VIDIOC_G_FMT, &v4l2.stream_fmt)) {
    get_four_character_code(v4l2.stream_fmt.fmt.pix.pixelformat, str);
  }
}

void Camera::enum_resolution(std::vector<CamResFormat> *list,
                             CamResFormat *current_res_format) {

  for (int cnt = 0; cnt < MAX_FRAME_SIZE_SUPPORT; cnt++) {

    memset(&v4l2.fsize[cnt], 0, sizeof(v4l2.fsize[cnt]));

    v4l2.fsize[cnt].index = cnt;
    v4l2.fsize[cnt].pixel_format = v4l2_fourcc(pixelformat[0], pixelformat[1],
                                               pixelformat[2], pixelformat[3]);

    if (0 == xioctl(VIDIOC_ENUM_FRAMESIZES, &v4l2.fsize[cnt])) {
      CamResFormat temp_resformat;
      temp_resformat.pixelformat = pixelformat;
      temp_resformat.widht = v4l2.fsize[cnt].discrete.width;
      temp_resformat.height = v4l2.fsize[cnt].discrete.height;

      enum_framerate(temp_resformat.pixelformat, temp_resformat.widht,
                     temp_resformat.height, &temp_resformat.fps);
      list->push_back(temp_resformat);

    } else {
      break;
    }
  }
  memset(&v4l2.stream_fmt, 0, sizeof(v4l2.stream_fmt));

  v4l2.stream_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  v4l2.stream_fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
  if (0 == xioctl(VIDIOC_G_FMT, &v4l2.stream_fmt)) {

    current_res_format->widht = v4l2.stream_fmt.fmt.pix.width;
    current_res_format->height = v4l2.stream_fmt.fmt.pix.height;
    current_res_format->pixelformat = pixelformat;

  } else {
    ROS_ERROR("ERROR getting the current resolution ");
  }
}

void Camera::enum_framerate(std::string pixelformat, int width, int height,
                            std::vector<std::uint32_t> *list) {

  for (int cnt = 0; cnt < MAX_FRMINVAL_SUPPORT; cnt++) {
    memset(&v4l2.frminterval, 0, sizeof(v4l2.frminterval));
    v4l2.frminterval.index = cnt;
    v4l2.frminterval.width = width;
    v4l2.frminterval.height = height;
    v4l2.frminterval.pixel_format = v4l2_fourcc(pixelformat[0], pixelformat[1],
                                                pixelformat[2], pixelformat[3]);

    if (0 == xioctl(VIDIOC_ENUM_FRAMEINTERVALS, &v4l2.frminterval)) {
      if (v4l2.frminterval.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
        v4l2.desc_frm_inval[cnt].numerator =
            v4l2.frminterval.discrete.numerator;
        v4l2.desc_frm_inval[cnt].denominator =
            v4l2.frminterval.discrete.denominator;
      }
      /*dividing denominator by numerator.
        for ex: (1) den-30, num- 1
                30/1 = 30 FPS
                (2) den-15, num -2
                15/2 = 7.5 FPS
      */

      list->push_back((double)v4l2.desc_frm_inval[cnt].denominator /
                      v4l2.desc_frm_inval[cnt].numerator);
    } else {
      break;
    }
  }
}
/*****************************************************************************
 *  Name	:	set_format.
 *  Parameter1 : const std::string& fourcc - Name of the format,
 *                                           For ex: UYVY,MJPG
 *  Parameter2: int input_width- Width of the frame
 *  Parameter3: int input_height- Height of the frame
 *  Returns	:	Function result depends on return value of child
 *functions and condition available in the functions based on this return value
 *will
 *
 *  			SUCCESS	- all the condition executed properly
 *  			FAILURE	- Failed to perform specified operation
 *  Description	:   This function is to set the resolution and format.
 ****************************************************************************/
int Camera::set_format(const std::string &fourcc, int input_width,
                       int input_height) {
  memset(&v4l2.stream_fmt, 0, sizeof(v4l2.stream_fmt));

  v4l2.stream_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  v4l2.stream_fmt.fmt.pix.width = input_width;
  v4l2.stream_fmt.fmt.pix.height = input_height;
  v4l2.stream_fmt.fmt.pix.pixelformat =
      v4l2_fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);

  v4l2.stream_fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
  if (ioctl(cam_fd, VIDIOC_S_FMT, &v4l2.stream_fmt) < 0) {
    ROS_INFO("VIDIOC_S_FMT failed");
    return FAILURE;
  }
  get_four_character_code(v4l2.stream_fmt.fmt.pix.pixelformat, &pixelformat);
  height = input_height;
  width = input_width;
  return SUCCESS;
}

/*****************************************************************************
 *  Name	:	get_format.
 *  Parameter1 : const std::string * PixelFormat - Name of the format,
                                                   For ex: UYVY,MJPG
 *  Parameter2: int *height- Height of the frame
 *  Parameter3: int *width- Width of the frame
 *  Parameter4: int *Numerator- Numerator of framerate
 *  Parameter5: int *Denominator- Denominator of framerate
 *  Parameter6: int type - This denotes when the function is called.
                 The possible values are:
                 PIX_FORMAT                    1
                 RESOLUTION                    2
                 FPS                           3
 *  Description	: This function is to get the format,resolution and framerate.
 ****************************************************************************/
void Camera::get_format(std::string *PixelFormat, int *Height, int *Width,
                        int *Numerator, int *Denominator, int type) {
  *PixelFormat = pixelformat;
  *Height = height;
  *Width = width;
  *Numerator = numerator;
  *Denominator = denominator;
  if (type == FPS) {
    init = false;
  }
}

/*****************************************************************************
 *  Name	:	set_framerate.
 *  Parameter1: unsigned input_numerator - Numerator of the frame interval
 *  Parameter2: unsigned input_denominator - denominator of the frame interval
 *  Returns	:	Function result depends on return value of child
 *functions and condition available in the functions based on this return value
 *will
 *
 *  			SUCCESS	- all the condition executed properly
 *  			FAILURE	- Failed to perform specified operation
 *  Description	:   This function is to set the framerate.
 ****************************************************************************/
int Camera::set_framerate(unsigned input_numerator,
                          unsigned input_denominator) {
  memset(&v4l2.cam_parm, 0, sizeof(v4l2.cam_parm));
  v4l2.cam_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  ioctl(cam_fd, VIDIOC_G_PARM, &v4l2.cam_parm);
  if (!(v4l2.cam_parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)) {
    ROS_WARN("V4L2_CAP_TIMEPERFRAME not supported by driver");
    ROS_WARN("Leaving frame rate unchanged");

    return FAILURE;
  }
  v4l2.cam_parm.parm.capture.timeperframe.numerator = input_numerator;
  v4l2.cam_parm.parm.capture.timeperframe.denominator = input_denominator;
  ioctl(cam_fd, VIDIOC_S_PARM, &v4l2.cam_parm);
  if (v4l2.cam_parm.parm.capture.timeperframe.numerator != input_numerator ||
      v4l2.cam_parm.parm.capture.timeperframe.denominator !=
          input_denominator) {
    ROS_WARN_STREAM("Driver rejected "
                    << input_numerator << "/" << input_denominator
                    << "s per frame and used "
                    << v4l2.cam_parm.parm.capture.timeperframe.numerator << "/"
                    << v4l2.cam_parm.parm.capture.timeperframe.denominator
                    << "s instead");
  }
  numerator = v4l2.cam_parm.parm.capture.timeperframe.numerator;
  denominator = v4l2.cam_parm.parm.capture.timeperframe.denominator;
  return SUCCESS;
}

/*****************************************************************************
 *  Name	:	request_buffer.
 *  Returns	:	Function result depends on return value of child
 *functions and condition available in the functions based on this return value
 *will
 *
 *  			SUCCESS	- all the condition executed properly
 *  			FAILURE	- Failed to perform specified operation
 *  Description	:   This function will request buffer from the camera,
 *                  map the buffers and also enqueue the buffers.
 ****************************************************************************/
int Camera::request_buffer() {
  memset(&v4l2.reqbuf, 0, sizeof(v4l2.reqbuf));

  v4l2.reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  v4l2.reqbuf.memory = V4L2_MEMORY_MMAP;
  v4l2.reqbuf.count = 2;

  if (ioctl(cam_fd, VIDIOC_REQBUFS, &v4l2.reqbuf) < 0) {
    ROS_INFO("VIDIOC_REQBUFS failed");
    return FAILURE;
  }

  buffers.resize(v4l2.reqbuf.count);
  for (unsigned i = 0; i < v4l2.reqbuf.count; i++) {

    memset(&v4l2.buffer, 0, sizeof(v4l2.buffer));

    v4l2.buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2.buffer.memory = V4L2_MEMORY_MMAP;
    v4l2.buffer.index = i;

    if (ioctl(cam_fd, VIDIOC_QUERYBUF, &v4l2.buffer) < 0) {
      ROS_INFO("VIDIOC_QUERYBUF failed");
      return FAILURE;
    }
    if (ioctl(cam_fd, VIDIOC_QBUF, &v4l2.buffer) < 0) {
      ROS_INFO("VIDIOC_QBUF failed");
      return FAILURE;
    }
    buffers[i].length = v4l2.buffer.length;
    buffers[i].start = NULL;
    buffers[i].start = static_cast<uint8_t *>(
        mmap(NULL, v4l2.buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED,
             cam_fd, v4l2.buffer.m.offset));

    if (NULL == buffers[i].start)
      ROS_INFO("MAP_FAILED");
  }
  return SUCCESS;
}

/*****************************************************************************
 *  Name	:	stream_on.
 *  Returns	:	Function result depends on return value of child
 *functions and condition available in the functions based on this return value
 *will
 *
 *  			SUCCESS	- all the condition executed properly
 *  			FAILURE	- Failed to perform specified operation
 *  Description	:   This function will stream on the camera.
 ****************************************************************************/
int Camera::stream_on() {
  const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(cam_fd, VIDIOC_STREAMON, &type) < 0) {
    ROS_INFO("VIDIOC_STREAMON failed");
    return FAILURE;
  } else {
    ROS_INFO("VIDIOC_STREAMON is ON");
    streamon = true;
  }
  return SUCCESS;
}

/****************************************************************************
 *  Name	:	enqueue.
 *  Parameter1: int index- The buffer index in which frame will be stored.
 *  Description	:   This function will enqueue the buffer
 ****************************************************************************/
void Camera::enqueue(int index) {
  memset(&v4l2.buffer, 0, sizeof(v4l2.buffer));
  v4l2.buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  v4l2.buffer.memory = V4L2_MEMORY_MMAP;
  v4l2.buffer.index = index;
  ioctl(cam_fd, VIDIOC_QBUF, &v4l2.buffer);
}

/*****************************************************************************
 *  Name	:	capture.
 *  Parameter1: vcam::image *image-The image in which the frame is stored.
 *  Returns	:	Function result depends on return value of child
 *functions and condition available in the functions based on this return value
 *will
 *
 *  			SUCCESS	- all the condition executed properly
 *  			FAILURE	- Failed to perform specified operation
 *        DEV_UNPLUGGED - If the device is unplugged.
 *  Description	: This function will dequeue the buffer and create image data.
 ****************************************************************************/
int Camera::capture(sensor_msgs::Image *image) {
  memset(&v4l2.buffer, 0, sizeof(v4l2.buffer));
  v4l2.buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  v4l2.buffer.memory = V4L2_MEMORY_MMAP;
  if (ioctl(cam_fd, VIDIOC_DQBUF, &v4l2.buffer) < 0) {
    if (errno == ENODEV) {
      image->encoding = "unplugged";
      image->data.reserve(1);
      image->data.resize(1);
      ROS_WARN("Device Unplugged");
      return DEV_UNPLUGGED;
    }
    return FAILURE;
  }
  if (v4l2.buffer.flags & V4L2_BUF_FLAG_ERROR) {
    enqueue(v4l2.buffer.index);
    ROS_WARN("Camera driver notified buffer error, ignoring frame");
    return FAILURE;
  } else {
    image->width = v4l2.stream_fmt.fmt.pix.width;
    image->height = v4l2.stream_fmt.fmt.pix.height;
    // image->length = v4l2.buffer.bytesused;

    // TODO: seperate raw image from other types
    if (v4l2.stream_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG) {

      image->encoding = "mjpg";
      image->data.reserve(v4l2.stream_fmt.fmt.pix.height *
                          v4l2.stream_fmt.fmt.pix.width);
      image->data.resize(v4l2.stream_fmt.fmt.pix.width *
                         v4l2.stream_fmt.fmt.pix.height);
      memcpy(&image->data[0], buffers[v4l2.buffer.index].start,
             v4l2.stream_fmt.fmt.pix.width * v4l2.stream_fmt.fmt.pix.height);

    } else if (v4l2.stream_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY) {
      if (v4l2.buffer.bytesused !=
          v4l2.stream_fmt.fmt.pix.height * v4l2.stream_fmt.fmt.pix.width) {
        enqueue(v4l2.buffer.index);
        return FAILURE;
      }
      image->header.stamp = ros::Time::now();
      image->encoding = sensor_msgs::image_encodings::MONO8;
      image->width = v4l2.stream_fmt.fmt.pix.width;
      image->height = v4l2.stream_fmt.fmt.pix.height;
      image->step =
          image->width *
          static_cast<unsigned int>(
              sensor_msgs::image_encodings::numChannels(image->encoding)) *
          static_cast<unsigned int>(
              sensor_msgs::image_encodings::bitDepth(image->encoding)) /
          8;
      image->data.reserve(image->height * image->step);
      image->data.resize(image->height * image->step);
      memcpy(&image->data[0], buffers[v4l2.buffer.index].start,
             v4l2.stream_fmt.fmt.pix.width * v4l2.stream_fmt.fmt.pix.height);
      // ROS_INFO("image captured mono8");

    } else if (v4l2.stream_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_Y16) {
      if (v4l2.buffer.bytesused !=
          v4l2.stream_fmt.fmt.pix.height * v4l2.stream_fmt.fmt.pix.width * 2) {
        enqueue(v4l2.buffer.index);
        return FAILURE;
      }
      // TODO: BUG:here you should copy data according to step
      image->encoding = sensor_msgs::image_encodings::MONO16;
      image->width = v4l2.stream_fmt.fmt.pix.width;
      image->height = v4l2.stream_fmt.fmt.pix.height;
      image->step =
          image->width *
          static_cast<unsigned int>(
              sensor_msgs::image_encodings::numChannels(image->encoding)) *
          static_cast<unsigned int>(
              sensor_msgs::image_encodings::bitDepth(image->encoding)) /
          8;
      image->data.reserve(image->height * image->step);
      image->data.resize(image->height * image->step);
      memcpy(&image->data[0], buffers[v4l2.buffer.index].start,
             2 * v4l2.stream_fmt.fmt.pix.width *
                 v4l2.stream_fmt.fmt.pix.height);

    } else if (v4l2.stream_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_UYVY) {
      if (v4l2.buffer.bytesused !=
          v4l2.stream_fmt.fmt.pix.height * v4l2.stream_fmt.fmt.pix.width * 2) {
        enqueue(v4l2.buffer.index);
        return FAILURE;
      }
      image->encoding = "uyvy";
      image->data.reserve(2 * v4l2.stream_fmt.fmt.pix.height *
                          v4l2.stream_fmt.fmt.pix.width);
      image->data.resize(2 * v4l2.stream_fmt.fmt.pix.width *
                         v4l2.stream_fmt.fmt.pix.height);
      memcpy(&image->data[0], buffers[v4l2.buffer.index].start,
             2 * v4l2.stream_fmt.fmt.pix.width *
                 v4l2.stream_fmt.fmt.pix.height);

    } else if (v4l2.stream_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV) {
      if (v4l2.buffer.bytesused !=
          v4l2.stream_fmt.fmt.pix.height * v4l2.stream_fmt.fmt.pix.width * 2) {
        enqueue(v4l2.buffer.index);
        return FAILURE;
      }
      image->encoding = "yuyv";
      image->data.reserve(2 * v4l2.stream_fmt.fmt.pix.height *
                          v4l2.stream_fmt.fmt.pix.width);
      image->data.resize(2 * v4l2.stream_fmt.fmt.pix.width *
                         v4l2.stream_fmt.fmt.pix.height);
      memcpy(&image->data[0], buffers[v4l2.buffer.index].start,
             2 * v4l2.stream_fmt.fmt.pix.width *
                 v4l2.stream_fmt.fmt.pix.height);

    } else if (v4l2.stream_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_Y12) {
      if (v4l2.buffer.bytesused != v4l2.stream_fmt.fmt.pix.height *
                                       v4l2.stream_fmt.fmt.pix.width * 1.5) {
        enqueue(v4l2.buffer.index);
        return FAILURE;
      }
      image->encoding = "mono12";
      image->data.reserve(1.5 * v4l2.stream_fmt.fmt.pix.height *
                          v4l2.stream_fmt.fmt.pix.width);
      image->data.resize(1.5 * v4l2.stream_fmt.fmt.pix.width *
                         v4l2.stream_fmt.fmt.pix.height);
      memcpy(&image->data[0], buffers[v4l2.buffer.index].start,
             1.5 * v4l2.stream_fmt.fmt.pix.width *
                 v4l2.stream_fmt.fmt.pix.height);

    } else if (v4l2.stream_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_SBGGR8) {
      if (v4l2.buffer.bytesused !=
          v4l2.stream_fmt.fmt.pix.height * v4l2.stream_fmt.fmt.pix.width) {
        enqueue(v4l2.buffer.index);
        return FAILURE;
      }
      image->encoding = "ba81";
      image->data.reserve(v4l2.stream_fmt.fmt.pix.height *
                          v4l2.stream_fmt.fmt.pix.width);
      image->data.resize(v4l2.stream_fmt.fmt.pix.width *
                         v4l2.stream_fmt.fmt.pix.height);
      memcpy(&image->data[0], buffers[v4l2.buffer.index].start,
             v4l2.stream_fmt.fmt.pix.width * v4l2.stream_fmt.fmt.pix.height);
    }
  }
  enqueue(v4l2.buffer.index);
  return SUCCESS;
}

/*****************************************************************************
 *  Name	:	stream_off.
 *  Returns	:	Function result depends on return value of child
 *functions and condition available in the functions based on this return value
 *will
 *
 *  			SUCCESS	- all the condition executed properly
 *  			FAILURE	- Failed to perform specified operation
 *  Description	:   This function will stream off the camera.
 ****************************************************************************/
int Camera::stream_off() {
  const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(cam_fd, VIDIOC_STREAMOFF, &type) < 0) {
    ROS_INFO("VIDIOC_STREAMOFF failed");
    return FAILURE;
  } else {
    streamon = false;
  }
  return SUCCESS;
}

/*****************************************************************************
 *  Name	:	clean_buffer.
 *  Returns	:	Function result depends on return value of child
 *functions and condition available in the functions based on this return value
 *will
 *
 *  			SUCCESS	- all the condition executed properly
 *  			FAILURE	- Failed to perform specified operation
 *  Description	:   This function will clean the buffer in the camera.
 *****************************************************************************/
int Camera::clean_buffer() {
  munmap_buffers();
  memset(&v4l2.reqbuf, 0, sizeof(v4l2.reqbuf));
  v4l2.reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  v4l2.reqbuf.memory = V4L2_MEMORY_MMAP;
  v4l2.reqbuf.count = 0;
  if (ioctl(cam_fd, VIDIOC_REQBUFS, &v4l2.reqbuf) < 0) {
    ROS_INFO("VIDIOC_REQBUFS failed");
    return FAILURE;
  } else {
    return SUCCESS;
  }
}

/*****************************************************************************
 *  Name	:	munmap_buffers.
 *  Description	:   This function will unmaap the allocated buffer.
 *****************************************************************************/
void Camera::munmap_buffers() {
  __u32 index;
  if (buffers.size() != 0) {
    for (index = 0; index < v4l2.reqbuf.count; index++) {
      if (buffers[index].start != NULL) {
        if (munmap(buffers[index].start, buffers[index].length) == SUCCESS) {
          buffers[index].start = NULL;
        }
      }
    }
  }
}

/*****************************************************************************
 *  Name	:	close_device.
 *  Description	:   This function is to close the camera file descriptor
 ****************************************************************************/
void Camera::close_device() {
  // checking if camera is already opened
  if (cam_fd > -1) {
    close(cam_fd);
    cam_fd = -1;
  }
}

/*****************************************************************************
 *  Name	:	onInit.
 *  Returns	:
 *  			True	- If the camera is initialized
 *       False - if the camera is not initialized
 *  Description	: This function is to check whether camera
 *                 is already initialized or not.
 *****************************************************************************/
bool Camera::onInit() { return init; }

// /*****************************************************************************
// *  Name	:	set_init.
// *  Description	:   This function is to set the init value
// *                   and initialize subscribers count.
// *****************************************************************************/
// void Camera::set_init(bool value)
// {
//   init=value;
//   if(subscribers_count<0){
//     subscribers_count = 0;
//   }
//   subscribers_count++;
// }

/*****************************************************************************
 *  Name	:	sub_count.
 *  Returns	: int	- number of subscribers.
 *  Description	:   This function is to get the subscribers count.
 *****************************************************************************/
// int Camera::sub_count()
// {
//   subscribers_count--;
//   return subscribers_count;
// }

/*****************************************************************************
 *  Name	:	isStreamOn.
 *  Returns	:
 *  			True	- If the camera is streamed on
 *       False - if the camera is not streamed off.
 *  Description:This function is to check whether camera is streamed on or off.
 *****************************************************************************/
bool Camera::isStreamOn() { return streamon; }

/*****************************************************************************
 *  Name	:	set_camera_name.
 *  Parameter1: std::string name - Name of the camera.
 *  Parameter2 : std::string node_name - Node name of the camera. (Ex: video0)
 *  Description	:   This function is to set the camera name and the node name.
 ****************************************************************************/
void Camera::set_camera_name(std::string name, std::string node_name) {
  camera_name = name;
  video_node = node_name;
}

/*****************************************************************************
 *  Name	:	get_camera_name.
 *  Returns	: std::string - Name of the camera.
 *  Description	:   This function is to get the camera name.
 ****************************************************************************/
std::string Camera::get_camera_name() { return camera_name; }

/*****************************************************************************
 *  Name	:	get_camera_name.
 *  Returns	: std::string - Node name of the camera.
 *  Description	:   This function is to get the video node name.
 ****************************************************************************/
void Camera::get_node_name(std::string *dev_node_name) {
  *dev_node_name += video_node;
}

/*****************************************************************************
 *  Name	:	xioctl.
 *  Returns	:	Function result depends on return value of child
 *functions and condition available in the functions based on this return value
 *will
 *
 *  			SUCCESS	- all the condition executed properly
 *  			errno	- Failed to perform specified operation
 *  Description	:   This function is to make the ioctl call.
 *****************************************************************************/
int Camera::xioctl(unsigned cmd, void *arg) {
  if (ioctl(cam_fd, cmd, arg) < 0) {
    return errno;
  }
  return SUCCESS;
}

/*****************************************************************************
 *  Name	:	get_four_character_code.
 *  Parameter1: int32_t pix_fmt - pixel_format value.
 *  Parameter2 : std::string *pixelformat - Name of the format,
 *                                          For ex: UYVY,MJPG
 *  Description	:   This function is to get the four character code as
 *                  computed by the v4l2_fourcc() macro.
 ****************************************************************************/
void Camera::get_four_character_code(int32_t pix_fmt,
                                     std::string *pixelformat) {
  switch (pix_fmt) {
  case V4L2_PIX_FMT_MJPEG:
    *pixelformat = "MJPG";
    break;
  case V4L2_PIX_FMT_UYVY:
    *pixelformat = "UYVY";
    break;
  case V4L2_PIX_FMT_YUYV:
    *pixelformat = "YUYV";
    break;
  case V4L2_PIX_FMT_GREY:
    *pixelformat = "GREY";
    break;
  case V4L2_PIX_FMT_Y16:
    *pixelformat = "Y16 ";
    break;
  case V4L2_PIX_FMT_Y12:
    *pixelformat = "Y12 ";
    break;
  case V4L2_PIX_FMT_SBGGR8:
    *pixelformat = "BA81";
  default:
    break;
  }
}

/*****************************************************************************
 *  Name	:	check_valid_control
 *  Parameter1 : int controlid - control id
 *  Returns	:
 *  			True	- If the control name  is a valid.
 *       False - If the control name  is a invalid.
 *  Description	: This function is to check whether the control name is valid.
 *****************************************************************************/
bool Camera::check_valid_control(int controlid) {
  if (controlid == V4L2_CID_USER_CLASS || controlid == V4L2_CID_CAMERA_CLASS ||
      controlid == V4L2_CID_ROI_EXPOSURE ||
      controlid == V4L2_CID_ROI_WINDOW_SIZE ||
      controlid >= V4L2_CID_CUSTOM_CONTROLS) {
    return false;
  }
  return true;
}

bool Camera::request_format(const std::string &encoding, unsigned width,
                            unsigned height, uint32_t fps, bool set_steam_on) {
  stream_off();
  clean_buffer();
  int ret = set_format(encoding, width, height);
  if (ret != 0) {
    return false;
  }
  // set_framerate(1, fps);
  ret = request_buffer();
  if (set_steam_on && ret == 0) {
    return stream_on() == 0;
  }
  return ret == 0;
}

void Camera::qeury_controls() {
  memset(&queryctrl, 0, sizeof queryctrl);
  camera_contorl_list.clear();

  queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
  while (xioctl(VIDIOC_QUERYCTRL, &queryctrl) == 0) {

    if (!(check_valid_control(queryctrl.id))) {
      queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
      continue;
    }

    CameraCtrl temp_ctrl;
    temp_ctrl.id = queryctrl.id;
    temp_ctrl.type = queryctrl.type;
    temp_ctrl.name = std::string((char *)queryctrl.name);
    temp_ctrl.default_value = queryctrl.default_value;
    if (queryctrl.type == V4L2_CTRL_TYPE_INTEGER) {
      temp_ctrl.minimum = queryctrl.minimum;
      temp_ctrl.maximum = queryctrl.maximum;
      temp_ctrl.step = queryctrl.step;
    }
    std::transform(temp_ctrl.name.begin(), temp_ctrl.name.end(),
                   temp_ctrl.name.begin(), ::toupper);
    camera_contorl_list.push_back(temp_ctrl);
    if (temp_ctrl.name.find("EXPOSURE") != std::string::npos) {
      exposure_ctrl_index = camera_contorl_list.size() - 1;
    } else if (temp_ctrl.name.find("BRIGHTNESS") != std::string::npos) {
      exposure_ctrl_index = camera_contorl_list.size() - 1;
    }

    // getting the current value:
    struct v4l2_control ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = temp_ctrl.id;

    if (xioctl(VIDIOC_G_CTRL, &ctrl) == 0) {
      temp_ctrl.cur_value = ctrl.value;
      ROS_INFO_STREAM("current value of " << temp_ctrl.name << ": "
                                          << temp_ctrl.cur_value);
    }

    else {
      ROS_ERROR("Getting current value for %s control is failed",
                (char *)queryctrl.name);
    }

    // trying next id
    queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
  }
  ROS_INFO_STREAM("size of camera controls: " << camera_contorl_list.size());
}

bool Camera::set_exposure(uint32_t val) {
  if (exposure_ctrl_index == -1) {
    ROS_ERROR("camera exposure contorl could not be found!");
    return false;
  }
  struct v4l2_control ctrl;
  ctrl.id = camera_contorl_list.at(exposure_ctrl_index).id;
  ctrl.value = val;

  if (0 == xioctl(VIDIOC_S_CTRL, &ctrl)) {
    ROS_INFO_STREAM("[v4l2_cam_driver.cpp] exposure set to: " << val);
    return true;
  }

  else {
    ROS_ERROR("error setting camera exposure contorl!");
    return false;
  }
}

} // namespace vio_cam
