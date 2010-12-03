/*                                                            */
/* This code is based on luvcview_wr_100823 from Opt corp.    */
/*                                                            */
/*                  2010/09/15    k-okada@jsk.t.u-tokyo.ac.jp */

#include "opt_nm33_camera.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

OptNM3xCamera::OptNM3xCamera(int camera_index)
{
  if((capture=cvCaptureFromCAM(camera_index)) == NULL) {
    fprintf(stderr, "ERROR : cvCaptureFromCAM");
  }

  typedef struct CvCaptureCAM_V4L {
    int* tmp;
    int* deviceHandle;
  } CvCaptureCAM_V4L;

  CvCaptureCAM_V4L *cap = (CvCaptureCAM_V4L *)capture;
  fd = *(cap->deviceHandle);

  setMode(4);
  if ( strcmp(getFirmwareVersion().c_str(), "") == 0 ) {
    fprintf(stderr, "ERROR : try sudo -E ./bin/init_xu_register\n");
    exit(1);
  }
  fprintf(stderr, "firmwareVersion -> %s\n", getFirmwareVersion().c_str());
  fprintf(stderr, "serialId -> %s\n", getSerialID().c_str());
}

OptNM3xCamera::~OptNM3xCamera()
{
  cvReleaseCapture(&capture);
}

// obtaining images
IplImage *OptNM3xCamera::queryFrame ()
{
  return cvQueryFrame(capture);
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
  return xu_ioctl(selector, UVCIOC_CTRL_GET, (void *)str);
}
bool OptNM3xCamera::getXuValue(int selector, short *value) {
  return xu_ioctl(selector, UVCIOC_CTRL_GET, (void *)value);
}
bool OptNM3xCamera::setXuValue(int selector, char value) {
  return xu_ioctl(selector, UVCIOC_CTRL_SET, (void *)&value);
}
bool OptNM3xCamera::setXuValue(int selector, short v1, short v2) {
  struct { short v1, v2;} value;
  value.v1 = v1;
  value.v2 = v2;
  return xu_ioctl(selector, UVCIOC_CTRL_SET, (void *)&value);
}
bool OptNM3xCamera::setXuValue(int selector, short v1, short v2, short v3, short v4, short v5) {
  struct { short v1, v2, v3, v4, v5;} value;
  value.v1 = v1; value.v2 = v2; value.v3 = v3; value.v4 = v4; value.v5 = v5;
  return xu_ioctl(selector, UVCIOC_CTRL_SET, (void *)&value);
}
bool OptNM3xCamera::xu_ioctl(int selector, int ctrl, void* value) {
  for (int i = 0; i < uvc_xu_tbl_cnt; i++){
    if ( xu_control_tbl[i].selector == selector ) {
      struct uvc_xu_control xctrl;
      xctrl.unit      = 0x60;
      xctrl.selector = xu_control_tbl[i].selector;
      xctrl.size     = xu_control_tbl[i].size;
      xctrl.data     = (__u8*)value;
      fprintf(stderr, "%s: name = %s, sel = 0x%02x, size = %d, data =",
              (ctrl==UVCIOC_CTRL_SET)?"UVCIOC_CTRL_SET":"UVCIOC_CTRL_GET",
              xu_control_tbl[i].name,
              xctrl.selector,
              xctrl.size);
      if (  ioctl(fd, ctrl, &xctrl) != 0 ) {
        xctrl.data[0]=0;
        fprintf(stderr, "\nioctl error %s\n", strerror(errno));
        return false;
      } else {
        for ( int j = 0; j  < xctrl.size; j++ )
          fprintf(stderr, " 0x%02x", *(xctrl.data + j));
        fprintf(stderr, "\n");
        if ( xu_control_tbl[i].size==255 ) xctrl.data[xctrl.size] = 0;
        return true;
      }
    }
  }
  return false;
}

