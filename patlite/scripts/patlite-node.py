#!/usr/bin/env python
import roslib; roslib.load_manifest('patlite')

import rospy
from std_msgs.msg import Int8

def main():
    rospy.init_node('patlite_node')
    rospy.Subscriber("set/red", Int8, callback_r)
    rospy.Subscriber("set/yellow", Int8, callback_y)
    rospy.Subscriber("set/green", Int8, callback_g)
    rospy.Subscriber("set/blue", Int8, callback_b)
    rospy.Subscriber("set/white", Int8, callback_w)
    patlite_ip = "10.68.0.10"
    p = Patlite(patlite_ip)
    while not rospy.is_shutdown():
        with p:
            rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

