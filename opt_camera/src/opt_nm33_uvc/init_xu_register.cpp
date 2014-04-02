/*                                                            */
/* This code is based on luvcview_wr_100823 from Opt corp.    */
/*                                                            */
/*                  2010/09/15    k-okada@jsk.t.u-tokyo.ac.jp */

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "opt_nm33_uvc.h"

int main (int argc, char* argv[]) {
  char dev[256];
  int fd;
  if ( getuid() != 0 ) {
    fprintf(stderr, "\033[31m### you must run this program as a root : sudo %s\033[0m\n", argv[0]);
    exit(1);
  }
  for (int i = 0; i < 16; i++ ) {
    sprintf(dev, "/dev/video%d", i);
    if ( ( fd = open((const char *)dev,O_RDWR))  > 0 ) {
      fprintf(stderr, ";; video capabilities %s(%d)\n", dev, fd);

      struct v4l2_capability vd;
      memset(&vd, 0, sizeof(struct v4l2_capability));
      if(ioctl(fd, VIDIOC_QUERYCAP, &vd) < 0) {
        perror("ioctl(VIDIOC_QUERYCAP)");
        return false;
      }
      fprintf(stderr, ";; vd.driver         = %s\n", vd.driver);
      fprintf(stderr, ";; vd.card           = %s\n", vd.card);
      fprintf(stderr, ";; vd.buf_info       = %s\n", vd.bus_info);
      fprintf(stderr, ";; vd.version        = %d\n", vd.version);

      if (strcmp("NM30 Ultra Wideview Camera", (const char*)vd.card) == 0 ) {
        set_XU_control(fd);
      }
    }
  }
}

