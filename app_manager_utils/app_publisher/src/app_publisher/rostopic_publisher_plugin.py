import importlib

from app_manager import AppManagerPlugin
import rospy
from rospy_message_converter import message_converter


class RostopicPublisherPlugin(AppManagerPlugin):
    def __init__(self):
        super(RostopicPublisherPlugin, self).__init__()

    def _check_condition(self, topic_cond, ctx):
        exit_code = ctx['exit_code'] if 'exit_code' in ctx else None
        stopped = ctx['stopped'] if 'stopped' in ctx else None
        timeout = ctx['timeout'] if 'timeout' in ctx else None
        if topic_cond == 'success':
            return exit_code == 0
        elif topic_cond == 'failure':
            return exit_code != 0
        elif topic_cond == 'stop':
            if stopped is None:
                rospy.logerr(
                    'stopped is not set in app_manager plugin ctx.')
                rospy.logerr(
                    'Please check app_manager version.')
                rospy.logerr('Skipping rostopic_publisher_plugin')
                return False
            else:
                return stopped
        elif topic_cond == 'timeout':
            if stopped is None:
                rospy.logerr(
                    'stopped is not set in app_manager plugin ctx.'
                    'Please check app_manager version.')
                rospy.logerr('Skipping rostopic_publisher_plugin')
                return False
            elif timeout is None:
                rospy.logerr(
                    'timeout is not set in app_manager plugin ctx.'
                    'Please check app_manager version.')
                rospy.logerr('Skipping rostopic_publisher_plugin')
                return False
            else:
                return stopped and timeout
        else:
            rospy.logerr('Invalid topic cond: {}'.format(topic_cond))
            return False

    def _publish_topic(self, topic, ctx, check_cond=False):
        if check_cond and 'cond' in topic:
            conditions = topic['cond']
            if not isinstance(conditions, list):
                conditions = [conditions]
            do_publish = False
            for cond in conditions:
                do_publish = do_publish or self._check_condition(cond, ctx)
            if do_publish is False:
                return
        msg = getattr(
            importlib.import_module(
                '{}.msg'.format(topic['pkg'])), topic['type'])
        pub = rospy.Publisher(topic['name'], msg, queue_size=1)
        rospy.sleep(1)
        if 'field' in topic:
            pub_msg = message_converter.convert_dictionary_to_ros_message(
                '{}/{}'.format(topic['pkg'], topic['type']),
                topic['field'])
        else:
            pub_msg = msg()
        pub.publish(pub_msg)

    def app_manager_start_plugin(self, app, ctx, plugin_args):
        if 'start_topics' not in plugin_args:
            return
        topics = plugin_args['start_topics']
        for topic in topics:
            self._publish_topic(topic, ctx, check_cond=False)

    def app_manager_stop_plugin(self, app, ctx, plugin_args):
        if 'stop_topics' not in plugin_args:
            return
        topics = plugin_args['stop_topics']
        for topic in topics:
            self._publish_topic(topic, ctx, check_cond=True)
