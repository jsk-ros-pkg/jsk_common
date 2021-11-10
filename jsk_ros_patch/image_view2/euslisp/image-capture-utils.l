(ros::load-ros-manifest "image_view2")

(defun capture-image (image-topic file-name)
  (let (res (req (instance image_view2::CaptureRequest :init)))
    (send req :image_topic image-topic)
    (send req :file_name file-name)
    (setq res (ros::service-call "/image_capture_server/capture" req t))
    (send res :success)
    ))
