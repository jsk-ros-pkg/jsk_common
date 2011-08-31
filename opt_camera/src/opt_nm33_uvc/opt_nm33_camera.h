/*                                                            */
/* This code is based on luvcview_wr_100823 from Opt corp.    */
/*                                                            */
/*                  2010/09/15    k-okada@jsk.t.u-tokyo.ac.jp */

#include "opt_nm33_uvc.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

class OptNM3xCamera
{
private:
  struct buffer {
    void *start;
    size_t length;
  } *buffers;
  unsigned int n_buffers;
  IplImage *frame, *frame_omni, *frame_wide, *frame_middle, *frame_narrow;
  int width, height;
  int fd;

  bool v4l2_set_ioctl(int selector, int value);
  int v4l2_get_ioctl(int selector);

  bool getXuValue(int selector, const char *str);
  bool getXuValue(int selector, short *value);
  bool setXuValue(int selector, char value);
  bool setXuValue(int selector, short v1, short v2);
  bool setXuValue(int selector, short v1, short v2, short v3, short v4, short v5);
  bool xu_ioctl(int selector, int ctrl, void* value);

public:
  OptNM3xCamera(int camera_index);
  ~OptNM3xCamera();

  void device_open(int camera_index);
  void device_close();
  IplImage *read_frame();

  // obtaining images
  IplImage *queryFrame ();
  IplImage *queryOmniFrame ();
  IplImage *queryWideFrame ();
  IplImage *queryMiddleFrame ();
  IplImage *queryNarrowFrame ();

  void getOmniImage (IplImage *frame, CvMat &subframe);
  void getWideImage (IplImage *frame, CvMat &subframe);
  void getMiddleImage (IplImage *frame, CvMat &subframe);
  void getNarrowImage (IplImage *frame, CvMat &subframe);

  // set commands
  bool setMode (int mode);

  // ct command
  bool setAutoExposure(bool mode);
  bool setExposure (int value);
  bool setIris(int value);
  bool setBrightness(int value);
  bool setSharpness(int value);
  bool setWhitebalance(int value);
  bool setAutoWhitebalance(bool mode);
  bool setPanAbsolute(double value);
  bool setTiltAbsolute(double value);
  bool setRollAbsolute(double value);
  bool setZoomAbsolute(double value);
  bool setPanAbsolute(int value);
  bool setTiltAbsolute(int value);
  bool setRollAbsolute(int value);
  bool setZoomAbsolute(int value);

  // xu command
  std::string getFirmwareVersion();
  bool setFlipScreen(char value);
  bool setSmallHemisphere(char value);
  bool setMedianFilter(bool mode);
  bool setJpegQuality(char value);
  std::string getSerialID();
  bool setInfoDisplay(bool mode);
  bool setCaptureFPS(short value);
  short getActualFPS();
  bool setLensType(char value);
  bool setPanAbsolute(int no, int value);
  bool setTiltAbsolute(int no, int value);
  bool setRollAbsolute(int no, int value);
  bool setZoomAbsolute(int no, int value);
  bool setLocationAbsolute(int no, int pan, int tilt, int roll, int zoom);
  //
};
