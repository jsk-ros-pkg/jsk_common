/*                                                            */
/* This code is based on luvcview_wr_100823 from Opt corp.    */
/*                                                            */
/*                  2010/09/15    k-okada@jsk.t.u-tokyo.ac.jp */

//
// 2010/8/23 Opt Corporation.
//
//  add xu_control_tbl
//  add set_xu_control()


#include "opt_nm33_uvc.h"

__u8 gui_extension[16] = UVC_GUID_UVC_EXTENSION;

#define XU_GET UVC_CONTROL_GET_CUR
#define XU_SET UVC_CONTROL_SET_CUR

#define V4L2_XU_ID_BASE 0x0A046D01
uvc_xu_tbl_info xu_control_tbl[] = {
#ifdef NON_STD_UVC
  {"Firmware Version", XU_FIRMWARE_VERSION_CONTROL, 0xFF, 0, XU_GET,
        V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_RAW},
#else
  {"Firmware Version", XU_FIRMWARE_VERSION_CONTROL, 0x20, 0, XU_GET,
        V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_RAW},
#endif
  {"Flash Parameter", XU_FLASH_PARAMETER_CONTROL, 1, 0, XU_SET | XU_GET,
        V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_UNSIGNED},
  {"Flip Screen", XU_FLIP_SCREEN_CONTROL, 1, 0, XU_SET | XU_GET,
        V4L2_CTRL_TYPE_MENU, UVC_CTRL_DATA_TYPE_BITMASK},
  {"small Hemisphere", XU_SMALL_HEMISPHERE_CONTROL, 1, 0, XU_SET | XU_GET,
        V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_UNSIGNED},
  {"Analog Video", XU_ANALOG_VIDEO_STANDARD_CONTROL, 1, 0, XU_SET | XU_GET,
        V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_UNSIGNED},
  {"Median Filter", XU_MEDIAN_FILTER_CONTROL, 1, 0, XU_SET | XU_GET,
        V4L2_CTRL_TYPE_BOOLEAN, UVC_CTRL_DATA_TYPE_BOOLEAN},
  {"Push Button", XU_PUSH_BUTTON_CONTROL, 2, 0, XU_SET | XU_GET,
        V4L2_CTRL_TYPE_MENU, UVC_CTRL_DATA_TYPE_BITMASK},
  {"Pending PTZR", XU_PENDING_PTZR_CONTROL, 1, 0, XU_SET | XU_GET,
        V4L2_CTRL_TYPE_BOOLEAN, UVC_CTRL_DATA_TYPE_BOOLEAN},
  {"Auto Pan", XU_AUTO_PAN_SPEED_CONTROL, 1, 0, XU_SET | XU_GET,
        V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_SIGNED},
  {"JPEG Quality", XU_JPEG_QUALITY_CONTROL, 1, 0, XU_SET | XU_GET,
        V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_SIGNED},
#ifdef NON_STD_UVC
  {"Serial ID", XU_SERIAL_ID_CONTROL, 16, 0, XU_GET,
        V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_RAW},
#else
  {"Serial ID", XU_SERIAL_ID_CONTROL, 16, 0, XU_SET | XU_GET,
        V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_RAW},
#endif
  {"Info Display", XU_INFO_DISPLAY_CONTROL, 1, 0, XU_SET | XU_GET,
        V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_UNSIGNED},
  {"Capture Fps", XU_CAPTURE_FPS_CONTROL, 2, 0, XU_SET | XU_GET,
        V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_UNSIGNED},
  {"Actual Fps", XU_ACTUAL_FPS_CONTROL, 2, 0, XU_GET,
        V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_UNSIGNED},
  {"Lens Type", XU_LENS_TYPE_CONTROL, 1, 0, XU_SET | XU_GET,
        V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_UNSIGNED},
  {"XPan Absolute", XU_PAN_ABSOLUTE_CONTROL, 6, 0, XU_SET | XU_GET,
        V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_RAW},
  {"XTilt Absolute", XU_TILT_ABSOLUTE_CONTROL, 6, 0, XU_SET | XU_GET,
        V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_RAW},
  {"XRoll Absolute", XU_ROLL_ABSOLUTE_CONTROL, 6, 0, XU_SET | XU_GET,
        V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_RAW},
  {"XZoom Absolute", XU_ZOOM_ABSOLUTE_CONTROL, 6, 0, XU_SET | XU_GET,
        V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_RAW},
  {"Location Absolute", XU_LOCATION_ABSOLUTE_CONTROL, 10, 0, XU_SET,
        V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_RAW},
  {"XGain Absolute", XU_GAIN_ABSOLUTE_CONTROL, 2, 0, XU_SET | XU_GET,
        V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_RAW},
};

int uvc_xu_tbl_cnt = sizeof(xu_control_tbl)/sizeof(uvc_xu_tbl_info);

int set_XU_control(int vd)
{
  int i = 0;
  int value = 0;
  int ret = 0;
  struct uvc_xu_control_info info = {UVC_GUID_UVC_EXTENSION,0,0,0,0};
  struct uvc_xu_control_mapping map = {0,"",UVC_GUID_UVC_EXTENSION,0,0,0,(v4l2_ctrl_type)0,0};

  printf("********XU registration************\n");
  for(i = 0; i < uvc_xu_tbl_cnt; i++){
    info.index = i + 1 ;
    info.selector = xu_control_tbl[i].selector;
    info.size = xu_control_tbl[i].size;
    info.flags = xu_control_tbl[i].flag;
    if ((value = ioctl(vd, UVCIOC_CTRL_ADD, &info)) != -1){
      map.id = V4L2_XU_ID_BASE + i;
      memcpy(map.name, xu_control_tbl[i].name, 32);
      map.selector = xu_control_tbl[i].selector;
      if(xu_control_tbl[i].size == 0xFF)
	map.size = xu_control_tbl[i].size;
      else
	map.size = xu_control_tbl[i].size*8;
      map.offset = xu_control_tbl[i].offset;
      map.v4l2_type = xu_control_tbl[i].v4l2_type;
      map.data_type = xu_control_tbl[i].data_type;
      if ((value = ioctl(vd, UVCIOC_CTRL_MAP, &map)) < 0){
	ret = 1;
	printf("XU mapping error:%s\n", xu_control_tbl[i].name);
      }
    }
    else {
      if(errno == EEXIST){
	printf("XU add already:%s\n", xu_control_tbl[i].name);
      }
      else{
	ret = 1;
	printf("XU add error:%s\n", xu_control_tbl[i].name);
      }
    }
  }

  return 0;
}

