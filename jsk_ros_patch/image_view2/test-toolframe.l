#!/usr/bin/env roseus

(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "image_view2")
;;;
;;;
(ros::roseus "imageview2-client")

(ros::advertise "image_marker" image_view2::ImageMarker2 1)

(ros::rate 10)
(while (ros::ok)
  (let ((mrk (instance image_view2::ImageMarker2 :init)))
    (send mrk :header :stamp (ros::time-now))
    (send mrk :type image_view2::FRAMES)
    (send mrk :frames (list "/r_gripper_tool_frame" "/l_gripper_tool_frame"))
    (ros::publish "image_marker" mrk)
    (print (list (send mrk :header :stamp) (send mrk :frames)))
    (ros::spin-once)
    (ros::sleep)))


