(ros::load-ros-manifest "jsk_image_capture")

(defparameter *jsk-image-capture-service-name* "/image_capture_server/capture")

(defun capture-image (image-topic file-name)
  (let (res (req (instance jsk_image_capture::CaptureRequest :init)))
    (send req :image_topic image-topic)
    (send req :file_name file-name)
    (setq res (ros::service-call *jsk-image-capture-service-name* req t))
    (send res :success)
    ))
