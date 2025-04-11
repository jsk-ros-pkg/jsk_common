import actionlib
from app_manager import AppManagerPlugin
import rospy

from app_notifier.util import check_timestamp_before_start
from app_notifier.util import get_notification_json_paths
from app_notifier.util import load_notification_jsons
from app_notifier.util import parse_context
from app_notifier.util import speak

from sound_play.msg import SoundRequestAction


class SpeechNotifierPlugin(AppManagerPlugin):
    def __init__(self):
        super(SpeechNotifierPlugin, self).__init__()
        self.client = None

    def app_manager_start_plugin(self, app, ctx, plugin_args):
        self.start_time = rospy.Time.now()
        client_name = plugin_args['client_name']
        lang = None
        if 'lang' in plugin_args:
            lang = plugin_args['lang']

        display_name = app.display_name
        display_name = display_name.replace('_', ' ')
        display_name = display_name.replace('-', ' ')
        client = actionlib.SimpleActionClient(client_name, SoundRequestAction)
        speech_text = "I'm starting {} app.".format(display_name)
        speak(client, speech_text, lang=lang)
        return ctx

    def app_manager_stop_plugin(self, app, ctx, plugin_args):
        client_name = plugin_args['client_name']
        lang = None
        if 'lang' in plugin_args:
            lang = plugin_args['lang']

        exit_code, stopped, timeout, upload_successes, _, _ = \
            parse_context(ctx)

        display_name = app.display_name
        display_name = display_name.replace('_', ' ')
        display_name = display_name.replace('-', ' ')
        client = actionlib.SimpleActionClient(client_name, SoundRequestAction)
        if exit_code == 0 and not stopped:
            speech_text = "I succeeded in doing {} app.".format(display_name)
        elif stopped:
            speech_text = "I stopped doing {} app".format(display_name)
            if timeout:
                speech_text += " because of timeout"
            speech_text += "."
        else:
            speech_text = "I failed to do {} app.".format(display_name)
        if upload_successes:
            if all(upload_successes):
                speech_text += " I succeeded to upload data."
            else:
                speech_text += " I failed to upload data."

        # only speak about object recognition
        json_paths = get_notification_json_paths()
        notification = load_notification_jsons(json_paths)
        if 'object recognition' in notification:
            for event in notification['object recognition']:
                if check_timestamp_before_start(
                        event['date'], self.start_time):
                    continue
                time = event['date'].split('T')[1]
                speech_text += " At {}, {} in {}.".format(
                    time, event['message'], event['location'])

        speak(client, speech_text, lang=lang)
        return ctx
