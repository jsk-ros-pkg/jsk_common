import actionlib
from app_manager import AppManagerPlugin
import rospy

from app_notifier.util import check_timestamp_before_start
from app_notifier.util import get_notification_json_paths
from app_notifier.util import load_notification_jsons
from app_notifier.util import parse_context
from app_notifier.util import tweet

from rostwitter.msg import TweetAction


class TweetNotifierPlugin(AppManagerPlugin):
    def __init__(self):
        super(TweetNotifierPlugin, self).__init__()
        self.client = None

    def app_manager_start_plugin(self, app, ctx, plugin_args):
        self.start_time = rospy.Time.now()
        client_name = plugin_args['client_name']
        image = False
        if 'image' in plugin_args:
            image = plugin_args['image']
        if image and 'image_topic_name':
            image_topic_name = plugin_args['image_topic_name']

        if 'warning' in plugin_args:
            warning = plugin_args['warning']
        else:
            warning = False

        display_name = app.display_name
        username = rospy.get_param('/app_manager/running_user_name', None)
        tweet_text = None
        if username:
            tweet_text = "{} is starting {} app".format(username, display_name)
        elif warning:
            tweet_text = "Unknown user is starting {} app".format(display_name)

        if tweet_text is not None:
            client = actionlib.SimpleActionClient(
                client_name, TweetAction)
            tweet(
                client, tweet_text, image=image,
                image_topic_name=image_topic_name)
        return ctx

    def app_manager_stop_plugin(self, app, ctx, plugin_args):
        client_name = plugin_args['client_name']
        image = False
        if 'image' in plugin_args:
            image = plugin_args['image']
        if image and 'image_topic_name':
            image_topic_name = plugin_args['image_topic_name']

        exit_code, stopped, timeout, upload_successes, _, _ = \
            parse_context(ctx)

        display_name = app.display_name
        client = actionlib.SimpleActionClient(client_name, TweetAction)
        if exit_code == 0 and not stopped:
            tweet_text = "I succeeded in doing {} app.".format(display_name)
        elif stopped:
            tweet_text = "I stopped doing {} app".format(display_name)
            if timeout:
                tweet_text += " because of timeout."
            tweet_text += "."
        else:
            tweet_text = "I failed to do {} app.".format(display_name)
        if upload_successes:
            if all(upload_successes):
                tweet_text += " I succeeded to upload data."
            else:
                tweet_text += " I failed to upload data."

        # only tweet about object recognition
        json_paths = get_notification_json_paths()
        notification = load_notification_jsons(json_paths)
        if 'object recognition' in notification:
            for event in notification['object recognition']:
                if check_timestamp_before_start(
                        event['date'], self.start_time):
                    continue
                time = event['date'].split('T')[1]
                tweet_text += " At {}, {} in {}.".format(
                    time, event['message'], event['location'])

        tweet(
            client, tweet_text[:280], image=image,
            image_topic_name=image_topic_name)
        return ctx
