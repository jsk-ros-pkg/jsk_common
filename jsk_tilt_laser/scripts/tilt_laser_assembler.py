#!/usr/bin/env python
import rospy
try:
    from laser_assembler.srv import *
except:
    import roslib; roslib.load_manifest("laser_assembler");
    from laser_assembler.srv import *

class AssembleCaller:
    cloud_pub = None
    joint_sub = None
    assemble_srv = None
    command_pub = None
    prev_angle = None
    max_angle = None
    min_angle = None
    lower_threshold = None
    upper_threshold = None
    scan_time = None ###

    def init(self):
        rospy.init_node('tilt_laser_assembler')
        self.joint_name = rospy.get_param("~tilt_joint_name", "tilt_laser_joint")
        self.cloud_pub = rospy.Publisher('assemble_cloud', sensor_msgs.msg.PointCloud2)
        self.command_pub = rospy.Publisher('/tilt_controller/command', std_msgs.msg.Float64)
        self.joint_sub = rospy.Subscriber('joint_states', sensor_msgs.msg.JointState, self.joint_callback)
        self.assemble_srv = rospy.ServiceProxy('assemble_scans2', AssembleScans2)

        if not self.max_angle:
            self.max_angle = rospy.get_param('~tilt_joint_max_angle', default=0.70)

        if not self.min_angle:
            self.min_angle = rospy.get_param('~tilt_joint_min_angle',default=-0.95)
        if not self.scan_time:
            self.scan_time = 8.0
        if not self.lower_threshold:
            self.lower_threshold = self.min_angle
        if not self.upper_threshold:
            self.upper_threshold = self.max_angle

    def move_to_angle(self, angle):
        self.command_pub.publish(std_msgs.msg.Float64(angle))

    def joint_callback(self, msg):
        pos = None
        try:
            pos = msg.position[msg.name.index(self.joint_name)] ### 
            rospy.logdebug('pos = %f'%pos)
        except:
            # do nothing
            rospy.logdebug('exept')

        if pos:
            if not self.prev_angle:
                self.prev_angle = pos
                if pos - self.min_angle > self.max_angle - pos:
                    self.prev_time = msg.header.stamp
                    self.move_to_angle(self.max_angle + 0.01)
                else:
                    self.prev_time = msg.header.stamp
                    self.move_to_angle(self.min_angle - 0.01)
                return
            if pos > self.max_angle:
                self.move_to_angle(self.min_angle - 0.01)
            if self.prev_angle < self.upper_threshold and pos > self.upper_threshold:
                self.scan_and_publish(self.prev_time, msg.header.stamp)

            if self.prev_angle > self.lower_threshold and pos < self.lower_threshold:
                self.scan_and_publish(self.prev_time, msg.header.stamp)

            if pos < self.min_angle:
                self.move_to_angle(self.max_angle + 0.01)
            # update timestamp
            if pos < self.min_angle or pos > self.max_angle:
                self.prev_time = msg.header.stamp
            self.prev_angle = pos

    def scan_and_publish(self, begin, end):
        req = AssembleScans2Request()
        req.begin = begin
        req.end = end
        try:
            ret = self.assemble_srv(req.begin, req.end)
            if ret:
                rospy.logdebug('publish')
                self.cloud_pub.publish(ret.cloud)
        except:
            # do nothing
            rospy.logerr('error calling service')

    def spin(self):
        rospy.spin()

if __name__=='__main__':
    a = AssembleCaller()
    a.init()
    a.spin()
