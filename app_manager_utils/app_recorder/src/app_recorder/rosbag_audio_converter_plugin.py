import distutils.spawn
import os
import rospy

from app_manager import AppManagerPlugin

# check if ffmpeg is found
if distutils.spawn.find_executable('ffmpeg'):
    try:
        from jsk_rosbag_tools.bag_to_video import bag_to_audio
    except ImportError as e:
        rospy.logerr('{}'.format(e))
        rospy.logerr('Skip using rosbag_audio_converter_plugin')
        bag_to_audio = None
else:
    rospy.logerr('ffpmeg is not found.')
    rospy.logerr('Skip using rosbag_audio_converter_plugin')
    bag_to_audio = None


class RosbagAudioConverterPlugin(AppManagerPlugin):
    def __init__(self):
        super(RosbagAudioConverterPlugin, self).__init__()

    @classmethod
    def app_manager_stop_plugin(cls, app, ctx, plugin_args):
        if bag_to_audio:
            rosbag_file_path = os.path.join(
                str(plugin_args['rosbag_path']),
                str(plugin_args['rosbag_title']))
            kwargs = {'wav_outpath': str(plugin_args['audio_path']),
                      'topic_name': str(plugin_args['audio_topic_name']),
                      'samplerate': int(plugin_args['audio_sample_rate']),
                      'channels': int(plugin_args['audio_channels'])}
            try:
                bag_to_audio(rosbag_file_path, **kwargs)
            except ValueError as e:
                # topic is not included in bagfile
                rospy.logerr('{}'.format(e))
        else:
            rospy.logerr('Skipping rosbag_audio_converter_plugin')
        return ctx
