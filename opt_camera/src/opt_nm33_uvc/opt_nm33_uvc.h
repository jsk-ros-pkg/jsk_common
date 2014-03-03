/*                                                            */
/* This code is based on luvcview_wr_100823 from Opt corp.    */
/*                                                            */
/*                  2010/09/15    k-okada@jsk.t.u-tokyo.ac.jp */
#ifndef _OPT_NM33_UVC_H_
#define _OPT_NM33_UVC_H_

//#define NON_STD_UVC = 1
//
// 2010/8/23 Opt Corporation.
//
//  add xu_control_tbl
//  add set_xu_control()

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include "uvcvideo.h"

typedef struct uvc_xu_tbl_info {
	__u8 name[32];
	__u8 selector;
	__u16 size;
	__u8 offset;
	__u32 flag;
	enum v4l2_ctrl_type v4l2_type;
	__u32 data_type;
}uvc_xu_tbl_info;

extern uvc_xu_tbl_info xu_control_tbl[];
extern int uvc_xu_tbl_cnt;

int set_XU_control(int vd);

#endif // _OPT_NM33_UVC_H_
