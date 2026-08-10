import distutils.spawn
import os
import rospy

# check if ffmpeg is found
if distutils.spawn.find_executable('ffmpeg'):
    try:
        from jsk_rosbag_tools.bag_to_video import bag_to_video
    except ImportError as e:
        rospy.logerr('{}'.format(e))
        rospy.logerr('Skip using rosbag_video_converter_plugin')
        bag_to_video = None
else:
    rospy.logerr('ffpmeg is not found.')
    rospy.logerr('Skip using rosbag_video_converter_plugin')
    bag_to_video = None

from app_manager import AppManagerPlugin


class RosbagVideoConverterPlugin(AppManagerPlugin):
    def __init__(self):
        super(RosbagVideoConverterPlugin, self).__init__()

    @classmethod
    def app_manager_stop_plugin(cls, app, ctx, plugin_args):
        if bag_to_video:
            rosbag_file_path = os.path.join(
                str(plugin_args['rosbag_path']),
                str(plugin_args['rosbag_title']))
            kwargs = {'output_filepath': str(plugin_args['video_path']),
                      'image_topic': str(plugin_args['image_topic_name']),
                      'fps': int(plugin_args['image_fps'])}
            # If audio_topic_name is given, video with audio is converted
            if 'audio_topic_name' in plugin_args:
                audio_kwargs = {
                    'audio_topic': str(plugin_args['audio_topic_name']),
                    'samplerate': int(plugin_args['audio_sample_rate']),
                    'channels': int(plugin_args['audio_channels'])}
                kwargs.update(audio_kwargs)
            try:
                bag_to_video(rosbag_file_path, **kwargs)
            except ValueError as e:
                # topic is not included in bagfile
                rospy.logerr('{}'.format(e))
        else:
            rospy.logerr('Skipping rosbag_video_converter_plugin')
        return ctx
