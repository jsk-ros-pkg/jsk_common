#!/usr/bin/env roseus

#|
;;
;; imageview2 sample program
;;
;; before invoke this script, first launch following programs
;;
roslaunch roseus_tutorials usb-camera.launch
roslaunch rosues_tutorials image-view.launch
rosrun tf static_transform_publisher 0 0 1 0 0 0 /camera /target 10
|#

(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "image_view2")
;;;
;;;
(ros::roseus "imageview2-client")

(ros::advertise "image_marker" image_view2::ImageMarker2 1)

(setq i 0)
(ros::rate 10)
(while (ros::ok)
  (let ((mrk (instance image_view2::ImageMarker2 :init)))
    (case (mod (/ i 10) 8)
          (0
           (send mrk :type image_view2::ImageMarker2::*CIRCLE*)
           (send mrk :id 5)
           (send mrk :position (instance geometry_msgs::Point :init :x 320 :y 240))
           (send mrk :outline_color (instance std_msgs::ColorRGBA :init :r 0.0 :g 1.0 :b 0.0 :a 1.0))
	   (send mrk :lifetime (ros::Time 10))
           (send mrk :scale 100.0)
           )
          (1
           (send mrk :type image_view2::ImageMarker2::*LINE_STRIP*)
           (send mrk :points (list (instance geometry_msgs::Point :init :x 200 :y 150)
                                   (instance geometry_msgs::Point :init :x 440 :y 150)
                                   (instance geometry_msgs::Point :init :x 440 :y 330)
                                   (instance geometry_msgs::Point :init :x 200 :y 330)))
           (send mrk :outline_colors (list (instance std_msgs::ColorRGBA :init :r 0.0 :g 1.0 :b 0.0 :a 1.0)
					   (instance std_msgs::ColorRGBA :init :r 0.0 :g 0.0 :b 1.0 :a 1.0)))
           )
          (2
           (send mrk :type image_view2::ImageMarker2::*LINE_LIST*)
           (send mrk :points (list (instance geometry_msgs::Point :init :x 200 :y 150)
                                   (instance geometry_msgs::Point :init :x 440 :y 150)
                                   (instance geometry_msgs::Point :init :x 440 :y 330)
                                   (instance geometry_msgs::Point :init :x 200 :y 330)))
           )
          (3
           (send mrk :type image_view2::ImageMarker2::*POLYGON*)
           (send mrk :points (list (instance geometry_msgs::Point :init :x 200 :y 150)
                                   (instance geometry_msgs::Point :init :x 440 :y 150)
                                   (instance geometry_msgs::Point :init :x 440 :y 330)
                                   (instance geometry_msgs::Point :init :x 200 :y 330)))
           )
          (4
           (send mrk :type image_view2::ImageMarker2::*POINTS*)
           (send mrk :points (list (instance geometry_msgs::Point :init :x 320 :y 240)))
	   (send mrk :scale 10.0)
           )
          (5
           (send mrk :action image_view2::ImageMarker2::*REMOVE*)
           (send mrk :id -1)
	   )
          (6
           (send mrk :type image_view2::ImageMarker2::*FRAMES*)
           (send mrk :frames (list "/target"))
	   )
          (7
           (send mrk :type image_view2::ImageMarker2::*TEXT*)
           (send mrk :position (instance geometry_msgs::Point :init :x 320 :y 240))
	   (send mrk :scale 0.5)
	   (send mrk :text "Hello World!")
	   )
          (t
           (warn "unknwon case")))
    (ros::publish "image_marker" mrk)
    (ros::ros-info "type ~A id ~A" (send mrk :type) (send mrk :id))
    (ros::spin-once)
    (ros::sleep)
    (incf i)))
