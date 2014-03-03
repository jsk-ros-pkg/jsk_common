#include "opt_nm33_camera.h"
#include <highgui.h>

int main(int argc, char *argv[]) {
  int camera_index = 1;
  OptNM3xCamera *camera;
  IplImage *frame;
  CvMat subframe;
  int count = 0;

  if ( argc > 1 ) {
    camera_index = atoi(argv[1]);
    fprintf(stderr, "set camera_index to %d\n", camera_index);
  }

  camera = new OptNM3xCamera(camera_index);
  camera->setSmallHemisphere(1);
  camera->setLocationAbsolute(0, 0, 0, 0,   0);
  camera->setLocationAbsolute(1, 0, 0, 0,  40);
  camera->setLocationAbsolute(2, 0, 0, 0, 120);

  cvNamedWindow("Opt NM33 Camera 0", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("Opt NM33 Camera 1", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("Opt NM33 Camera 2", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("Opt NM33 Camera 3", CV_WINDOW_AUTOSIZE);

  //cvSetMouseCallback ("Opt NM33 Camera 0", on_mouse);

  while (1)  {
    frame = camera->queryFrame();
    fprintf(stderr, "get frame: channels=%d, depth=%d, width=%d, height=%d\n",
            frame->nChannels, frame->depth, frame->width, frame->height);

    cvShowImage ("Opt NM33 Camera 0", camera->queryOmniFrame());

    cvShowImage ("Opt NM33 Camera 1", camera->queryWideFrame());

    cvShowImage ("Opt NM33 Camera 2", camera->queryMiddleFrame());

    cvShowImage ("Opt NM33 Camera 3", camera->queryNarrowFrame());

    float f = (float)count/20.0;
    camera->setLocationAbsolute(2, 180*sin(f), 45*sin(f+M_PI/3)+45, 180*sin(f+M_PI/2), 50*sin(f*2)+100);

    char c = cvWaitKey (2);
    if (c == '\x1b')
      break;
    count++;
  }
}
