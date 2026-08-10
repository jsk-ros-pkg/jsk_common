import os
import rospy

from app_notification_saver import AppNotificationSaver

from app_notification_saver.srv import SaveAppNotification
from app_notification_saver.srv import SaveAppNotificationResponse
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse


class ServiceNotificationSaver(AppNotificationSaver):
    def __init__(self):
        super(ServiceNotificationSaver, self).__init__()
        rospy.Service(
            '~save_app_notification',
            SaveAppNotification,
            self.save_service_notification_cb)
        rospy.Service(
            '~clear_app_notification',
            Empty,
            self.clear_app_notification_cb)

    def save_service_notification_cb(self, req):
        self.save_app_notification(
            req.title, float(req.stamp.secs), req.location, req.message)
        return SaveAppNotificationResponse(True)

    def clear_app_notification_cb(self, req):
        if os.path.exists(self.json_path):
            os.remove(self.json_path)
            rospy.loginfo('Remove file {}'.format(self.json_path))
        return EmptyResponse()
