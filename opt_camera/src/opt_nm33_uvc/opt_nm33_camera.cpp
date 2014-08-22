/*                                                            */
/* This code is based on luvcview_wr_100823 from Opt corp.    */
/*                                                            */
/*                  2010/09/15    k-okada@jsk.t.u-tokyo.ac.jp */

#include "opt_nm33_camera.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

//
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#define CLEAR(x) memset (&(x), 0, sizeof (x))

/* convert from mjpeg to rgb24 */
static bool
mjpeg_to_rgb24 (int width, int height,
		unsigned char *src, int length,
		unsigned char *dst)
{
  cv::Mat temp=cv::imdecode(cv::Mat(std::vector<uchar>(src, src + length)), 1);
  if( !temp.data || temp.cols != width || temp.rows != height )
    return false;
  memcpy(dst, temp.data, width*height*3);
  return true;
}

void OptNM3xCamera::device_open(int camera_index)
{
  // open_device
  char dev_name[128];
  sprintf(dev_name, "/dev/video%1d", camera_index);
  fprintf(stderr, "Opening device '%s'\n", dev_name);

  width  = 640;
  height = 480;

 try_again:
  fd = open(dev_name, O_RDWR, 0);

  if (fd == -1) {
    fprintf(stderr, "Cannot open '%s': %d, %s\n",
	    dev_name, errno, strerror(errno));
    exit(EXIT_FAILURE);
  }

  // init_device
  struct v4l2_capability cap;
  struct v4l2_format fmt;

  if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s is no V4L2 device\n", dev_name);
    }
    perror("VIDIOC_QUERYCAP");
    exit(EXIT_FAILURE);
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    fprintf(stderr, "%s is no video capture device\n", dev_name);
    exit(EXIT_FAILURE);
  }

  if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
    fprintf(stderr, "%s does not support streaming i/o\n", dev_name);
    exit(EXIT_FAILURE);
  }

  CLEAR(fmt);

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = width;
  fmt.fmt.pix.height = height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

  if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
    perror("VIDIOC_S_FMT");
    exit(EXIT_FAILURE);
  }

  // init mmap
  struct v4l2_requestbuffers req;

  CLEAR(req);

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
    perror("VIDIOC_REQBUFS");
    exit(EXIT_FAILURE);
  }

  if (req.count < 2) {
    fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
    exit(EXIT_FAILURE);
  }

  buffers = (buffer *)calloc(req.count, sizeof(*buffers));

  if (!buffers) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }

  for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers;

    if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
      perror("VIDIOC_QUERYBUF");
      exit(EXIT_FAILURE);
    }

    buffers[n_buffers].length = buf.length;
    buffers[n_buffers].start = mmap(NULL /* start anywhere */ ,
				    buf.length, PROT_READ | PROT_WRITE
				    /* required */ ,
				    MAP_SHARED /* recommended */ ,
				    fd, buf.m.offset);

    if (buffers[n_buffers].start == MAP_FAILED) {
      perror("mmap");
      exit(EXIT_FAILURE);
    }
  }

  // start capturing
  unsigned int i;
  enum v4l2_buf_type type;

  for (i = 0; i < n_buffers; ++i) {
    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
      if ( width == 640 && height == 480 ) {
	device_close();
	width = 320; height = 240;
	goto try_again;
      }
      perror("VIDIOC_QBUF");
      exit(EXIT_FAILURE);
    }
  }

  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
    if ( width == 640 && height == 480 ) {
      device_close();
      width = 320; height = 240;
      goto try_again;
    }
    perror("VIDIOC_STREAMON");
    exit(EXIT_FAILURE);
  }
  //
  fprintf(stderr, "video capabilities\n");
  fprintf(stderr, "cap.driver        =  %s\n", cap.driver);
  fprintf(stderr, "cap.card          =  %s\n", cap.card);
  fprintf(stderr, "cap.buf_info      =  %s\n", cap.bus_info);
  fprintf(stderr, "cap.version       =  %d\n", cap.version);
  fprintf(stderr, "cap.capabilities  =  0x%08x ", cap.capabilities);
  if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
    fprintf(stderr, " VIDEO_CAPTURE");
  if (cap.capabilities & V4L2_CAP_VIDEO_OUTPUT)
    fprintf(stderr, " VIDEO_OUTPUT");
  if (cap.capabilities & V4L2_CAP_VIDEO_OVERLAY)
    fprintf(stderr, " VIDEO_OVERLAY");
  if (cap.capabilities & V4L2_CAP_VBI_CAPTURE)
    fprintf(stderr, " VBI_CAPTURE");
  if (cap.capabilities & V4L2_CAP_VBI_OUTPUT)
    fprintf(stderr, " VBI_OUTPUT");
#ifdef V4L2_CAP_SLICED_VBI_CAPTURE
  if (cap.capabilities & V4L2_CAP_SLICED_VBI_CAPTURE)
    fprintf(stderr, " SLICED_VBI_CAPTURE");
#endif
#ifdef V4L2_CAP_SLICED_VBI_OUTPUT
  if (cap.capabilities & V4L2_CAP_SLICED_VBI_OUTPUT)
    fprintf(stderr, " VBI_SLICED_OUTPUT");
#endif
  if (cap.capabilities & V4L2_CAP_RDS_CAPTURE)
    fprintf(stderr, " RDS_CAPTURE");
#if V4L2_CAP_VIDEO_OUTPUT_OVERLAY
  if (cap.capabilities & V4L2_CAP_VIDEO_OUTPUT_OVERLAY)
    fprintf(stderr, " VIDEO_OUTPUT_OVERLAY");
#endif
  if (cap.capabilities & V4L2_CAP_TUNER)
    fprintf(stderr, " TUNER");
  if (cap.capabilities & V4L2_CAP_AUDIO)
    fprintf(stderr, " AUDIO");
  if (cap.capabilities & V4L2_CAP_RADIO)
    fprintf(stderr, " RADIO");
  if (cap.capabilities & V4L2_CAP_READWRITE)
    fprintf(stderr, " READWRITE");
  if (cap.capabilities & V4L2_CAP_ASYNCIO)
    fprintf(stderr, " ASYNCIO");
  if (cap.capabilities & V4L2_CAP_STREAMING)
    fprintf(stderr, " STREAMING");
  fprintf(stderr, "\n");
  fprintf(stderr, "cap.width         =  %d\n", width);
  fprintf(stderr, "cap.height        =  %d\n", height);

  /* Set up Image data */
  frame = (IplImage *)malloc(sizeof(IplImage));
  cvInitImageHeader( frame,
		     cvSize( width, height ),
		     IPL_DEPTH_8U, 3, IPL_ORIGIN_TL, 4 );
  /* Allocate space for RGBA data */
  frame->imageData = (char *)cvAlloc(frame->imageSize);
}

void OptNM3xCamera::device_close()
{
  // stop capturing
  enum v4l2_buf_type type;

  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (ioctl(fd, VIDIOC_STREAMOFF, &type) == -1) {
    perror("VIDIOC_STREAMOFF");
    exit(EXIT_FAILURE);
  }

  // uninit_mmap
  unsigned int i;

  for (i = 0; i < n_buffers; ++i) {
    if (munmap(buffers[i].start, buffers[i].length) == -1) {
      perror("munmap");
      exit(EXIT_FAILURE);
    }
  }

  // uninit_device
  free(buffers);

  // close_device
  if (close(fd) == -1) {
    perror("close");
    exit(EXIT_FAILURE);
  }
  fd = -1;
}

IplImage *OptNM3xCamera::read_frame ()
{
  // read frame
  struct v4l2_buffer buf;

  CLEAR(buf);

  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;

  if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
    perror("VIDIOC_DQBUF");
    return NULL;
  }
  assert(buf.index < n_buffers);

  //process_image(buffers[buf.index].start);
  if (!mjpeg_to_rgb24(width, height,
		      (unsigned char*)(buffers[buf.index].start),
		      buffers[buf.index].length,
		      (unsigned char*)frame->imageData)) {
    perror("mjpeg_to_rgb24");
    return NULL;
  }

  if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
    perror("VIDIOC_QBUF");
    return NULL;
  }

  return frame;
}

OptNM3xCamera::OptNM3xCamera(int camera_index)
{
  device_open(camera_index);

  setMode(4);
  if ( strcmp(getFirmwareVersion().c_str(), "") == 0 ) {
    fprintf(stderr, "ERROR : try sudo -E ./bin/init_xu_register\n");
    exit(1);
  }
  fprintf(stderr, "firmwareVersion -> %s\n", getFirmwareVersion().c_str());
  fprintf(stderr, "serialId -> %s\n", getSerialID().c_str());

  frame_omni = frame_wide = frame_middle = frame_narrow = NULL;
}

OptNM3xCamera::~OptNM3xCamera()
{
  device_close();
}

// obtaining images
IplImage *OptNM3xCamera::queryFrame ()
{
  if ( read_frame()==NULL ) {
    fprintf(stderr, "ERROR : read_frame returns NULL\n");
    return NULL;
  }


  int height = frame->height, width = frame->width;
  if ( ! ( frame_omni && (frame_omni->width  == height/2 &&
                          frame_omni->height == height/2 ) ) )
    frame_omni = cvCreateImage(cvSize(height/2, height/2), frame->depth, frame->nChannels);
  if ( ! ( frame_wide && (frame_wide->width  == (width-height/2) &&
                          frame_wide->height == height/2 ) ) )
    frame_wide = cvCreateImage(cvSize(width-height/2, height/2), frame->depth, frame->nChannels);
  if ( ! ( frame_middle && (frame_middle->width  == width/2 &&
                            frame_middle->height == height/2 ) ) )
    frame_middle = cvCreateImage(cvSize(width/2, height/2), frame->depth, frame->nChannels);
  if ( ! ( frame_narrow && (frame_narrow->width  == width/2 &&
                            frame_narrow->height == height/2 ) ) )
    frame_narrow = cvCreateImage(cvSize(width/2, height/2), frame->depth, frame->nChannels);
  return frame;
}

void OptNM3xCamera::getOmniImage (IplImage *frame, CvMat &subframe) {
  int height = frame->height;
  cvGetSubRect(frame, &subframe, cvRect(0, 0, height/2, height/2));
}

void OptNM3xCamera::getWideImage (IplImage *frame, CvMat &subframe) {
  int width = frame->width, height = frame->height;
  cvGetSubRect(frame, &subframe, cvRect(height/2, 0, width-height/2, height/2));
}

void OptNM3xCamera::getMiddleImage (IplImage *frame, CvMat &subframe) {
  int width = frame->width, height = frame->height;
  cvGetSubRect(frame, &subframe, cvRect(0, height/2, width/2, height/2));
}

void OptNM3xCamera::getNarrowImage (IplImage *frame, CvMat &subframe) {
  int width = frame->width, height = frame->height;
  cvGetSubRect(frame, &subframe, cvRect(width/2, height/2, width/2, height/2));
}


IplImage *OptNM3xCamera::queryOmniFrame () {
  int height = frame->height;
  cvSetImageROI(frame, cvRect(0, 0, height/2, height/2));
  cvCopy(frame, frame_omni);
  cvResetImageROI(frame);
  return frame_omni;
}

IplImage *OptNM3xCamera::queryWideFrame () {
  int width = frame->width, height = frame->height;
  cvSetImageROI(frame, cvRect(height/2, 0, width-height/2, height/2));
  cvCopy(frame, frame_wide);
  cvResetImageROI(frame);
  return frame_wide;
}

IplImage *OptNM3xCamera::queryMiddleFrame () {
  int width = frame->width, height = frame->height;
  cvSetImageROI(frame, cvRect(0, height/2, width/2, height/2));
  cvCopy(frame, frame_middle);
  cvResetImageROI(frame);
  return frame_middle;
}

IplImage *OptNM3xCamera::queryNarrowFrame () {
  int width = frame->width, height = frame->height;
  cvSetImageROI(frame, cvRect(width/2, height/2, width/2, height/2));
  cvCopy(frame, frame_narrow);
  cvResetImageROI(frame);
  return frame_narrow;
}

// set commands
bool OptNM3xCamera::setMode (int mode) {
  return (ioctl(fd, VIDIOC_S_INPUT, &mode)==0);
}

// ct command
bool OptNM3xCamera::setAutoExposure(bool mode) {
  v4l2_set_ioctl(V4L2_CID_EXPOSURE_AUTO, mode?1:2);
  return true;
}
bool OptNM3xCamera::setExposure (int value) {
  v4l2_set_ioctl(V4L2_CID_EXPOSURE_ABSOLUTE, value);
  return true;
}
bool OptNM3xCamera::setIris(int value) {
  v4l2_set_ioctl(V4L2_CID_IRIS_ABSOLUTE, value);
  return true;
}
bool OptNM3xCamera::setBrightness(int value) {
  v4l2_set_ioctl(V4L2_CID_BRIGHTNESS, value);
  return true;
}
bool OptNM3xCamera::setSharpness(int value) {
  v4l2_set_ioctl(V4L2_CID_SHARPNESS, value);
  return true;
}
bool OptNM3xCamera::setWhitebalance(int value) {
  v4l2_set_ioctl(V4L2_CID_WHITE_BALANCE_TEMPERATURE, value);
  return true;
}
bool OptNM3xCamera::setAutoWhitebalance(bool mode) {
  v4l2_set_ioctl(V4L2_CID_AUTO_WHITE_BALANCE, mode?1:0);
  return true;
}
bool OptNM3xCamera::setPanAbsolute(double value) {
  return v4l2_set_ioctl(V4L2_CID_PAN_ABSOLUTE,  (int)(value*3600));
}
bool OptNM3xCamera::setTiltAbsolute(double value) {
  return v4l2_set_ioctl(V4L2_CID_TILT_ABSOLUTE, (int)(value*3600));
}
bool OptNM3xCamera::setRollAbsolute(double value) {
  return v4l2_set_ioctl(V4L2_CID_ROLL_ABSOLUTE, (int)(value*1));
}
bool OptNM3xCamera::setZoomAbsolute(double value) {
  return v4l2_set_ioctl(V4L2_CID_ZOOM_ABSOLUTE, (int)value);
}
bool OptNM3xCamera::setPanAbsolute(int value) {
  return v4l2_set_ioctl(V4L2_CID_PAN_ABSOLUTE,  value);
}
bool OptNM3xCamera::setTiltAbsolute(int value) {
  return v4l2_set_ioctl(V4L2_CID_TILT_ABSOLUTE, value);
}
bool OptNM3xCamera::setRollAbsolute(int value) {
  return v4l2_set_ioctl(V4L2_CID_ROLL_ABSOLUTE, value);
}
bool OptNM3xCamera::setZoomAbsolute(int value) {
  return v4l2_set_ioctl(V4L2_CID_ZOOM_ABSOLUTE, value);
}

bool OptNM3xCamera::v4l2_set_ioctl(int selector, int value) {
  v4l2_queryctrl q;
  q.id = selector;
  if ( ioctl(fd, VIDIOC_QUERYCTRL, &q) < 0 )
    {
      perror("unable to query control");
      return false;
    }

  v4l2_control  c;
  c.id = selector;
  c.value = value;
  fprintf(stderr, "VIDIOC_S_CTRL: sel = 0x%08x, data =0x%08x\n",
          selector, value);
  if (ioctl(fd, VIDIOC_S_CTRL, &c) < 0)
    {
      perror("unable to set control");
      return false;
    }
  return true;
}

int OptNM3xCamera::v4l2_get_ioctl(int selector) {
  v4l2_control c;
  c.id = selector;
  fprintf(stderr, "VIDIOC_S_CTRL: sel = 0x%02x\n", selector);
  if (ioctl(fd, VIDIOC_G_CTRL, &c) == 0)
    {
      perror("unable to get control");
      return -1;
    }
  return c.value;
}

// xu command
std::string OptNM3xCamera::getFirmwareVersion() {
#ifdef NON_STD_UVC
  std::string str(255,0);
#else
  std::string str(32,0);
#endif
  getXuValue(XU_FIRMWARE_VERSION_CONTROL, str.c_str());
  return str.substr(0,str.find_first_of('\0'));
}
bool OptNM3xCamera::setFlipScreen(char value) {
  return setXuValue(XU_FLIP_SCREEN_CONTROL,value);
}
bool OptNM3xCamera::setSmallHemisphere(char value) {
  return setXuValue(XU_SMALL_HEMISPHERE_CONTROL,value);
}
bool OptNM3xCamera::setMedianFilter(bool mode) {
  return setXuValue(XU_MEDIAN_FILTER_CONTROL, (char)(mode?1:0));
}
bool OptNM3xCamera::setJpegQuality(char value) {
  return setXuValue(XU_JPEG_QUALITY_CONTROL,value);
}
std::string OptNM3xCamera::getSerialID() {
  std::string str(16,0);
  getXuValue(XU_SERIAL_ID_CONTROL, str.c_str());
  return str.substr(0,str.find_first_of('\0'));
}
bool OptNM3xCamera::setInfoDisplay(bool mode) {
  return setXuValue(XU_INFO_DISPLAY_CONTROL, mode?1:0);
}
bool OptNM3xCamera::setCaptureFPS(short value) {
  return setXuValue(XU_CAPTURE_FPS_CONTROL,value);
}
short OptNM3xCamera::getActualFPS() {
  short value;
  getXuValue(XU_CAPTURE_FPS_CONTROL, &value);
  return value;
}
bool OptNM3xCamera::setLensType(char value) {
  return setXuValue(XU_LENS_TYPE_CONTROL,value);
}
bool OptNM3xCamera::setPanAbsolute(int no, int value) {
  return setXuValue(XU_PAN_ABSOLUTE_CONTROL,  no, value);
}
bool OptNM3xCamera::setTiltAbsolute(int no, int value) {
  return setXuValue(XU_TILT_ABSOLUTE_CONTROL,  no, value);
}
bool OptNM3xCamera::setRollAbsolute(int no, int value) {
  return setXuValue(XU_ROLL_ABSOLUTE_CONTROL,  no, value);
}
bool OptNM3xCamera::setZoomAbsolute(int no, int value) {
  return setXuValue(XU_ZOOM_ABSOLUTE_CONTROL,  no, value);
}
bool OptNM3xCamera::setLocationAbsolute(int no, int pan, int tilt, int roll, int zoom) {
  return setXuValue(XU_LOCATION_ABSOLUTE_CONTROL, no, pan, tilt, roll, zoom);
}
//
bool OptNM3xCamera::getXuValue(int selector, const char *str) {
  return xu_ioctl(selector, UVC_GET_CUR, (void *)str);
}
bool OptNM3xCamera::getXuValue(int selector, short *value) {
  return xu_ioctl(selector, UVC_GET_CUR, (void *)value);
}
bool OptNM3xCamera::setXuValue(int selector, char value) {
  return xu_ioctl(selector, UVC_SET_CUR, (void *)&value);
}
bool OptNM3xCamera::setXuValue(int selector, short v1, short v2) {
  struct { short v1, v2;} value;
  value.v1 = v1;
  value.v2 = v2;
  return xu_ioctl(selector, UVC_SET_CUR, (void *)&value);
}
bool OptNM3xCamera::setXuValue(int selector, short v1, short v2, short v3, short v4, short v5) {
  struct { short v1, v2, v3, v4, v5;} value;
  value.v1 = v1; value.v2 = v2; value.v3 = v3; value.v4 = v4; value.v5 = v5;
  return xu_ioctl(selector, UVC_SET_CUR, (void *)&value);
}

bool OptNM3xCamera::xu_ioctl(int selector, int ctrl, void* value) {
  for (int i = 0; i < uvc_xu_tbl_cnt; i++) {
    if (xu_control_tbl[i].selector == selector) {
      struct uvc_xu_control_query xctrl;
      xctrl.unit     = 0x60;
      xctrl.selector = xu_control_tbl[i].selector;
      xctrl.size     = xu_control_tbl[i].size;
      xctrl.data     = (__u8*)value;
      xctrl.query    = ctrl;
      fprintf(stderr, "%s: name = %s, sel = 0x%02x, size = %d, data =",
              (ctrl == UVC_SET_CUR) ? "UVC_SET_CUR" : "UVC_GET_CUR",
              xu_control_tbl[i].name,
              xctrl.selector,
              xctrl.size);
      if (ioctl(fd, UVCIOC_CTRL_QUERY, &xctrl) != 0) {
        xctrl.data[0] = 0;
        fprintf(stderr, "\nioctl error %s\n", strerror(errno));
        return false;
      } else {
        for (int j = 0; j < xctrl.size; j++)
          fprintf(stderr, " 0x%02x", *(xctrl.data + j));
        fprintf(stderr, "\n");
        if (xu_control_tbl[i].size == 255) xctrl.data[xctrl.size] = 0;
        return true;
      }
    }
  }
  return false;
}
