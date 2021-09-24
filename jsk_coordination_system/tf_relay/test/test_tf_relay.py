 #!/usr/bin/env python
PKG = 'tf_relay'

import unittest

import rospy
import tf2_ros

class TestTFRelay(unittest.TestCase):
    ## test 1 == 1
    def test_one_equals_one(self): # only functions with 'test_'-prefix will be run!
        self.assertEquals(1, 1, "1!=1")

#    def test_lookup_transform(self):
#        rospy.init_node('test_tf_relay')
#
#        tf_buffer = tf2_ros.Buffer()
#        tf_listener = tf2_ros.TransformListener(tf_buffer)
#
#        transform_a_to_c = tf_buffer.lookup_transform(
#                            'frame_a',
#                            'frame_c',
#                            rospy.Time(),
#                            rospy.Duration(5)
#                            )
#        transform_a_to_c_relayed = tf_buffer.lookup_transform(
#                            'frame_a',
#                            'frame_c_relayed',
#                            rospy.Time(),
#                            rospy.Duration(5)
#                            )
#
#        self.assertEqual(transform_a_to_c.header.frame_id, 'frame_a')
#        self.assertEqual(transform_a_to_c.child_frame_id, 'frame_c')
#
#        self.assertEqual(transform_a_to_c_relayed.header.frame_id, 'frame_a')
#        self.assertEqual(transform_a_to_c_relayed.child_frame_id, 'frame_c_relayed')
#
#        self.assertEqual(transform_a_to_c.transform.translation.x, 
#                         transform_a_to_c_relayed.transform.translation.x)
#        self.assertEqual(transform_a_to_c.transform.translation.y, 
#                         transform_a_to_c_relayed.transform.translation.y)
#        self.assertEqual(transform_a_to_c.transform.translation.z, 
#                         transform_a_to_c_relayed.transform.translation.z)
#        self.assertEqual(transform_a_to_c.transform.rotation.x, 
#                         transform_a_to_c_relayed.transform.rotation.x)
#        self.assertEqual(transform_a_to_c.transform.rotation.y, 
#                         transform_a_to_c_relayed.transform.rotation.y)
#        self.assertEqual(transform_a_to_c.transform.rotation.z, 
#                         transform_a_to_c_relayed.transform.rotation.z)
#        self.assertEqual(transform_a_to_c.transform.rotation.w, 
#                         transform_a_to_c_relayed.transform.rotation.w)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_tf_relay', TestTFRelay)
