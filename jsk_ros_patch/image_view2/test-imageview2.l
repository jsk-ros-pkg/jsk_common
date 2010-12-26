#!/usr/bin/env roseus

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
    (case (mod (/ i 10) 6)
          (0
           (send mrk :type image_view2::CIRCLE)
           (send mrk :id 5)
           (send mrk :position (instance geometry_msgs::Point :init :x 320 :y 240))
           (send mrk :outline_color (instance std_msgs::ColorRGBA :init :r 0.0 :g 1.0 :b 0.0 :a 1.0))
	   (send mrk :lifetime (ros::Time 2))
           (send mrk :scale 100.0)
           )
          (1
           (send mrk :type image_view2::LINE_STRIP)
           (send mrk :points (list (instance geometry_msgs::Point :init :x 200 :y 150)
                                   (instance geometry_msgs::Point :init :x 440 :y 150)
                                   (instance geometry_msgs::Point :init :x 440 :y 330)
                                   (instance geometry_msgs::Point :init :x 200 :y 330)))
           (send mrk :outline_colors (list (instance std_msgs::ColorRGBA :init :r 0.0 :g 1.0 :b 0.0 :a 1.0)
					   (instance std_msgs::ColorRGBA :init :r 0.0 :g 0.0 :b 1.0 :a 1.0)))
           )
          (2
           (send mrk :type image_view2::LINE_LIST)
           (send mrk :points (list (instance geometry_msgs::Point :init :x 200 :y 150)
                                   (instance geometry_msgs::Point :init :x 440 :y 150)
                                   (instance geometry_msgs::Point :init :x 440 :y 330)
                                   (instance geometry_msgs::Point :init :x 200 :y 330)))
           )
          (3
           (send mrk :type image_view2::POLYGON)
           (send mrk :points (list (instance geometry_msgs::Point :init :x 200 :y 150)
                                   (instance geometry_msgs::Point :init :x 440 :y 150)
                                   (instance geometry_msgs::Point :init :x 440 :y 330)
                                   (instance geometry_msgs::Point :init :x 200 :y 330)))
           )
          (4
           (send mrk :type image_view2::POINTS)
           (send mrk :points (list (instance geometry_msgs::Point :init :x 320 :y 240)))
           )
          (5
           (send mrk :type image_view2::FRAMES)
           (send mrk :frames (list "/target"))
           )
          (t
           (warn "unknwon case")))
    (ros::publish "image_marker" mrk)
    (print mrk)
    (ros::spin-once)
    (ros::sleep)
    (incf i)))
