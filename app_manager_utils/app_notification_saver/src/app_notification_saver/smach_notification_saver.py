import rospy

from app_notification_saver import AppNotificationSaver

from smach_msgs.msg import SmachContainerStatus


class SmachNotificationSaver(AppNotificationSaver):
    def __init__(self):
        super(SmachNotificationSaver, self).__init__()
        rospy.Subscriber(
            "~smach/container_status", SmachContainerStatus, self.smach_cb)

    def smach_cb(self, msg):
        if len(msg.active_states) == 0:
            return
        states_str = "Active states is "
        states_str += ', '.join(msg.active_states)
        self.save_app_notification(
            "smach", float(msg.header.stamp.secs), "", states_str)
