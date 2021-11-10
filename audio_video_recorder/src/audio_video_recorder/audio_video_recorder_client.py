# -*- encoding: utf-8 -*-

import rospy
import roslaunch

from sensor_msgs.msg import Image
from audio_video_recorder.msg import RecordTask, RecordTaskArray
from audio_video_recorder.srv import StartRecord, StartRecordRequest, StartRecordResponse
from audio_video_recorder.srv import StopRecord, StopRecordRequest, StopRecordResponse

import threading


class AudioVideoRecorderClient:

    def __init__(self):

        self.record_task_array = RecordTaskArray()

        self.client_start_record = rospy.ServiceProxy(
                '/audio_video_recorder_server/start_record',
                StartRecord)
        self.client_stop_record = rospy.ServiceProxy(
                '/audio_video_recorder_server/stop_record',
                StopRecord)
        self.sub_record_task_array = rospy.Subscriber(
                '/audio_video_recorder_server/record_tasks',
                RecordTaskArray,
                self.__callback
                )

    def __callback(self, msg):
        self.record_task_array = msg

    def start_record(self,
                    audio_topic_name,
                    image_topic_name,
                    file_name,
                    video_framerate,
                    queue_size=100,
                    file_format='avi',
                    audio_format='mp3',
                    audio_sample_format='S16LE',
                    audio_channels=1,
                    audio_depth=16,
                    audio_sample_rate=16000,
                    video_encoding='RGB',
                    video_height=None,
                    video_width=None
                    ):
        # Get width and height from message
        if video_height is None or video_width is None:
            try:
                msg_image = rospy.wait_for_message(image_topic_name,Image,timeout=rospy.Duration(5))
                video_height = msg_image.height
                video_width = msg_image.width
            except rospy.ROSException as e:
                return False, 'Image topic not published.'
        #
        req = StartRecordRequest()
        req.task.audio_topic_name = audio_topic_name
        req.task.image_topic_name = image_topic_name
        req.task.queue_size = queue_size
        req.task.file_name = file_name
        req.task.file_format = file_format
        req.task.audio_format = audio_format
        req.task.audio_sample_format = audio_sample_format
        req.task.audio_channels = audio_channels
        req.task.audio_depth = audio_depth
        req.task.audio_sample_rate = audio_sample_rate
        req.task.video_encoding = video_encoding
        req.task.video_height = video_height
        req.task.video_width = video_width
        req.task.video_framerate = video_framerate
        #
        res = self.client_start_record(req)
        return res.success, res.message

    def get_record_task_array(self):
        return self.record_task_array

    def stop_record(self, file_name):
        #
        req = StopRecordRequest()
        req.file_name = file_name
        #
        res = self.client_stop_record(req)
        return res.success, res.message
