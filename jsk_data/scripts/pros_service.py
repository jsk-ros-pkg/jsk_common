#!/usr/bin/env python
import roslib; roslib.load_manifest('polled_camera'); roslib.load_manifest('sensor_msgs')
import sys
import rospy
from polled_camera.srv import GetPolledImage
from sensor_msgs.msg import RegionOfInterest

def pros_call_client():
    while not rospy.is_shutdown():
        rospy.wait_for_service('/prosilica/request_image')
        try:
            print "Request Prosilica"
            proc_call = rospy.ServiceProxy('/prosilica/request_image', GetPolledImage)
            resp1=proc_call("rgb", rospy.Duration(0, 0), 0 , 0,RegionOfInterest(0, 0, 1500, 2000, 0))
            if resp1.success:
                print "succeeded"
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return
if __name__ == "__main__":
    try:
        pros_call_client()
    except KeyboardInterrupt:
        print "Ctrl+C was pressed"
    except rospy.ROSInterruptException:
        pass
