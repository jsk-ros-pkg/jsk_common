# -*- encoding: utf-8 -*-

import rospy
import roslaunch

from audio_video_recorder_server.msg import RecordTask, RecordTaskArray
from audio_video_recorder_server.srv import StartRecord, StopRecord

import threading


def thread_launch_recorder( record_task ):

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch_path = rospkg.RosPack().get_path('audio_video_recorder_server') +\
        '/launch/audio_video_recorder.launch'
    roslaunch_cli_args = [roslaunch_path]
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(
        roslaunch_cli_args)
    roslaunch_parent = roslaunch.parent.ROSLaunchParent(
        uuid,
        roslaunch_file
    )
    roslaunch_parent.start()


class AudioVideoRecorderServer:

    def __init__(self):

        self.pub_record_task_array = rospy.Publisher('~record_tasks', RecordTaskArray, queue_size=1)

        self.list_record_task_and_launch = {}
        self.lock_for_list = threading.Lock()

        roslaunch.pmon._init_signal_handlers()

        self.srv_start_record = rospy.Service('~start_record', StartRecord, self.handler_start_record)
        self.srv_stop_record = rospy.Service('~stop_record', StopRecord, self.handler_stop_record)


    def __start_record(self, record_task):

        self.lock_for_list.acquire()

        if record_task.file_name in self.list_record_task_and_launch:
            self.lock_for_list.release()
            return False, 'There is already a recording task with the same name.'

        # start roslaunch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch_path = rospkg.RosPack().get_path('audio_video_recorder_server') +\
            '/launch/audio_video_recorder.launch'
        roslaunch_cli_args = [
                roslaunch_path,
                'audio_topic_name:={}'.format(record_task.audio_topic_name).
                'image_topic_name:={}'.format(record_task.image_topic_name),
                'queue_size:={}'.format(record_task.queue_size),
                'file_name:={}'.format(record_task.flie_name),
                'file_format:={}'.format(record_task.file_format),
                'audio_format:={}'.format(record_task.audio_format),
                'audio_sample_format:={}'.format(record_task.audio_sample_format),
                'audio_channels:={}'.format(record_task.audio_channels),
                ]
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(
            roslaunch_cli_args)
        roslaunch_parent = roslaunch.parent.ROSLaunchParent(
            uuid,
            roslaunch_file
        )
        roslaunch_parent.start()

        # Add task to list
        self.list_record_task_and_launch[record_task.file_name] = {
                'task': record_task,
                'launch_handler': roslaunch_parent,
                }

        self.lock_for_list.release()
        return True, 'Success'

    def handler_start_record(self, req):

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch_path = rospkg.RosPack().get_path('audio_video_recorder_server') +\
            '/launch/audio_video_recorder.launch'
        roslaunch_cli_args = [roslaunch_path]
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(
            roslaunch_cli_args)
        self.roslaunch_parent = roslaunch.parent.ROSLaunchParent(
            uuid,
            roslaunch_file
        )
        self.roslaunch_parent.start()
