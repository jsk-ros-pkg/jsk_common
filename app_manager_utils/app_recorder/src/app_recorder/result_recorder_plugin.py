import os
import rospy
import yaml

from app_manager import AppManagerPlugin


class ResultRecorderPlugin(AppManagerPlugin):
    def __init__(self):
        super(ResultRecorderPlugin, self).__init__()

    @classmethod
    def app_manager_stop_plugin(cls, app, ctx, plugin_args):
        result_path = '/tmp'
        if 'result_path' in plugin_args:
            result_path = plugin_args['result_path']
        result_title = 'result.yaml'
        if 'result_title' in plugin_args:
            result_title = plugin_args['result_title']
        try:
            with open(os.path.join(result_path, result_title), 'w') as f:
                yaml.safe_dump(ctx, f)
        except Exception as e:
            rospy.logerr(
                'failed to write result in {}: {}'.format(
                    os.path.join(result_path, result_title), e))
        return ctx
