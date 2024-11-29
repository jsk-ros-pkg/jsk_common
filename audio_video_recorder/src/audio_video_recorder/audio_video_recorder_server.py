# -*- encoding: utf-8 -*-

import rospy
import rospkg
import roslaunch

from audio_video_recorder.msg import RecordTask, RecordTaskArray
from audio_video_recorder.srv import StartRecord, StartRecordRequest, StartRecordResponse
from audio_video_recorder.srv import StopRecord, StopRecordRequest, StopRecordResponse

import threading


class AudioVideoRecorderServer:

    def __init__(self):

        self.pub_record_task_array = rospy.Publisher('~record_tasks', RecordTaskArray, queue_size=1)

        self.list_record_task_and_launch = {}
        self.lock_for_list = threading.Lock()

        roslaunch.pmon._init_signal_handlers()

        self.srv_start_record = rospy.Service('~start_record', StartRecord, self.handler_start_record)
        self.srv_stop_record = rospy.Service('~stop_record', StopRecord, self.handler_stop_record)

    def __publish_tasks(self):

        self.lock_for_list.acquire()
        msg = RecordTaskArray()
        for key, item in self.list_record_task_and_launch.items():
            msg.array.append(item['task'])
        self.pub_record_task_array.publish(msg)
        self.lock_for_list.release()

    def __start_record(self, record_task):

        if not isinstance(record_task, RecordTask):
            return False, 'Argument is not an instance of RecordTask'
        self.lock_for_list.acquire()
        if record_task.file_name in self.list_record_task_and_launch:
            self.lock_for_list.release()
            return False, 'There is already a recording task with the same name.'
        # start roslaunch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch_path = rospkg.RosPack().get_path('audio_video_recorder') + '/launch/audio_video_recorder.launch'
        roslaunch_args = [ \
                'audio_topic_name:={}'.format(record_task.audio_topic_name),
                'image_topic_name:={}'.format(record_task.image_topic_name),
                'queue_size:={}'.format(record_task.queue_size),
                'file_name:={}'.format(record_task.file_name),
                'file_format:={}'.format(record_task.file_format),
                'audio_format:={}'.format(record_task.audio_format),
                'audio_sample_format:={}'.format(record_task.audio_sample_format),
                'audio_channels:={}'.format(record_task.audio_channels),
                'audio_depth:={}'.format(record_task.audio_depth),
                'audio_sample_rate:={}'.format(record_task.audio_sample_rate),
                'video_encoding:={}'.format(record_task.video_encoding),
                'video_height:={}'.format(record_task.video_height),
                'video_width:={}'.format(record_task.video_width),
                'video_framerate:={}'.format(record_task.video_framerate)
                ]
        roslaunch_file = [(
                    roslaunch.rlutil.resolve_launch_arguments([roslaunch_path])[0],
                    roslaunch_args)]
        roslaunch_parent = roslaunch.parent.ROSLaunchParent(
            uuid,
            roslaunch_file,
            is_core=False
        )
        roslaunch_parent.start()
        # Add task to list
        self.list_record_task_and_launch[record_task.file_name] = {
                'task': record_task,
                'launch_handler': roslaunch_parent,
                }

        self.lock_for_list.release()
        return True, 'Success'

    def __stop_record(self, file_name):

        if not isinstance(file_name, str):
            return False, 'Argument is not an instance of str'
        self.lock_for_list.acquire()
        if file_name not in self.list_record_task_and_launch:
            self.lock_for_list.release()
            return False, 'There is no recording task with the specified name.'
        self.list_record_task_and_launch[file_name]['launch_handler'].shutdown()
        del self.list_record_task_and_launch[file_name]
        self.lock_for_list.release()
        return True, 'Success'

    def handler_start_record(self, req):

        success, message = self.__start_record(req.task)
        if success:
            rospy.loginfo('Start recoding to {}: {}'.format(req.task.file_name, message))
        else:
            rospy.logerr('Failed to start recoding to {}: {}'.format(req.task.file_name, message))
        response = StartRecordResponse()
        response.success = success
        response.message = message
        return response

    def handler_stop_record(self, req):

        success, message = self.__stop_record(req.file_name)
        if success:
            rospy.loginfo('Stop recoding to {}: {}'.format(req.file_name, message))
        else:
            rospy.logerr('Failed to stop recoding to {}: {}'.format(req.file_name, message))
        response = StopRecordResponse()
        response.success = success
        response.message = message
        return response

    def spin(self):

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            self.__publish_tasks()
