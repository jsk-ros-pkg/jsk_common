import actionlib
from app_manager import AppManagerPlugin
import rospy

from app_notifier.util import speak

from sound_play.msg import SoundRequestAction


class UserSpeechNotifierPlugin(AppManagerPlugin):
    def __init__(self):
        super(UserSpeechNotifierPlugin, self).__init__()
        self.client = None
        self.username = rospy.get_param('/app_manager/running_user_name', None)
        if self.username is not None:
            self.username = self.username.replace('_', ' ')
            self.username = self.username.replace('-', ' ')

    def app_manager_start_plugin(self, app, ctx, plugin_args):
        client_name = plugin_args['client_name']
        if 'warning' in plugin_args:
            warning = plugin_args['warning']
        else:
            warning = False
        display_name = app.display_name
        display_name = display_name.replace('_', ' ')
        display_name = display_name.replace('-', ' ')
        speech_text = None
        if self.username:
            speech_text = "{} is starting {} app".format(
                self.username, display_name)
        elif warning:
            speech_text = "Unknown user is starting {} app".format(
                display_name)

        if speech_text is not None:
            lang = None
            if 'lang' in plugin_args:
                lang = plugin_args['lang']
            client = actionlib.SimpleActionClient(
                client_name, SoundRequestAction)
            speak(client, speech_text, lang=lang)
        return ctx

    def app_manager_stop_plugin(self, app, ctx, plugin_args):
        client_name = plugin_args['client_name']
        if 'warning' in plugin_args:
            warning = plugin_args['warning']
        else:
            warning = False

        display_name = app.display_name
        display_name = display_name.replace('_', ' ')
        display_name = display_name.replace('-', ' ')
        speech_text = None
        if self.username:
            speech_text = "{} is stopping {} app".format(
                self.username, display_name)
        elif warning:
            speech_text = "Unknown user is stopping {} app".format(
                display_name)

        if speech_text is not None:
            lang = None
            if 'lang' in plugin_args:
                lang = plugin_args['lang']
            client = actionlib.SimpleActionClient(
                client_name, SoundRequestAction)
            speak(client, speech_text, lang=lang)
        return ctx
