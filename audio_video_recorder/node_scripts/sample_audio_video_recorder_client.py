#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from audio_video_recorder.audio_video_recorder_client import AudioVideoRecorderClient
from sensor_msgs.msg import Image
from audio_common_msgs.msg import AudioData


def main():

    rospy.init_node('audio_video_recorder_client_demo')

    destination = rospy.get_param('~destination','/tmp/')
    file_name_01 = destination + 'audio_video_recorder_server_demo_01.avi'
    file_name_02 = destination + 'audio_video_recorder_server_demo_02.avi'

    try:
        msg_image = rospy.wait_for_message('/usb_cam_node/image_raw', Image, timeout=rospy.Duration(10))
        msg_audio = rospy.wait_for_message('/audio', AudioData, timeout=rospy.Duration(10))
    except rospy.ROSException as e:
        rospy.logerr(e)
        return

    client = AudioVideoRecorderClient()
    rospy.loginfo('Start recording')
    client.start_record('/audio','/usb_cam_node/image_raw',file_name_01,30)
    rospy.sleep(5)
    client.start_record('/audio','/usb_cam_node/image_raw',file_name_02,30)
    rospy.sleep(5)
    rospy.loginfo('Stop recording')
    client.stop_record(file_name_01)
    rospy.sleep(5)
    client.stop_record(file_name_02)


if __name__ == '__main__':
    main()
