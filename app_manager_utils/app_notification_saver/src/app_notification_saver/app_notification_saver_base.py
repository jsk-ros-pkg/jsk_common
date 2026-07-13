import datetime
import json
import os
import rospy


class AppNotificationSaver(object):
    def __init__(self):
        self.json_path = rospy.get_param(
            '~json_path', '/tmp/app_notification.json')

    def save_app_notification(self, title, stamp, location, message):
        """
        Save app notification to json file.

        Args:
            title (str)   : Notification title (e.g. object detection, navigation faliure ...) # NOQA
            stamp (float) : UNIX time when the event occurred
            location (str): The location where the event occurred
            message (str) : Notification message

        Returns:
            Result of whether the json was saved. (bool)
        """
        # Load notification json
        if os.path.exists(self.json_path):
            with open(self.json_path, 'r') as j:
                notification = json.load(j)
        else:
            notification = {}
        # Append notification
        stamp = datetime.datetime.fromtimestamp(stamp)
        new_notification = {'date': stamp.isoformat(),
                            'location': location,
                            'message': message}
        if title in notification:
            notification[title].append(new_notification)
        else:
            notification[title] = [new_notification]
        # Dump json
        with open(self.json_path, 'w') as j:
            json.dump(notification, j, indent=4)
        rospy.loginfo(json.dumps(notification, indent=4))
