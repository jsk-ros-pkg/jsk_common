#!/usr/bin/env roseus
(require :compressed-angle-vector "package://jsk_network_tools/euslisp/angle-vector-compress.l")
(ros::roseus-add-msgs "sensor_msgs")

(ros::roseus "joint_state_decompressor")


(setq *msg-type-string* (ros::get-param "~message_type"
                          "jsk_network_tools/CompressedAngleVectorPR2"))
(unless *msg-type-string*
  (error "Please specify ~~message_type"))

;; msg-type-string should be like "pkg_name/MessageType"
;; euslisp string operation is poor, so we use pathname utilities to parse it
(setq *message-package* (car (pathname-directory (pathname *msg-type-string*))))

(ros::roseus-add-msgs *message-package*)

(setq *message-type* (symbol-value
                      (intern (string-upcase (send (pathname *msg-type-string*) :name))
                              (string-upcase *message-package*))))
;; load robot model

(setq *robot-name* (ros::get-param "~robot" "pr2"))
(unless *robot-name*
  (error "Please specify ~~robot_name"))
;; load robot files... it's not so good
(setq *robot-model-files*
      (list "package://pr2eus/pr2.l"
            "package://hrpsys_ros_bridge_tutorials/models/hrp2jsk.l"
            "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknt.l"
            "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknts.l"
            "package://hrpsys_ros_bridge_tutorials/models/staro.l"
            "package://hrpsys_ros_bridge_tutorials/models/urataleg.l"
            "package://hrpsys_ros_bridge_tutorials/models/samplerobot.l"
            "package://hrpsys_ros_bridge_tutorials/models/hrp2w.l"))

(dolist (f *robot-model-files*)
  (if (probe-file (ros::resolve-ros-path f))
      (load f)))

(setq *robot* (make-robot-model-from-name *robot-name*))

(defun joint-state-compressed-callback (inmsg)
  (let ((msg (instance sensor_msgs::JointState :init)))
    (send msg :name (send-all (send *robot* :joint-list) :name))
    (let ((avs (coerce (decompress-angle-vector
                        *robot* (send inmsg :angles))
                       cons)))
      (send msg :position (mapcar #'(lambda (v j)
                                      (if (derivedp j linear-joint)
                                          (* 0.001 v)
                                        (deg2rad v)))
                                  avs (send *robot* :joint-list))))
    (send msg :header :stamp (ros::time-now))
    (ros::publish "/joint_states_decompressed" msg)))

(ros::advertise "/joint_states_decompressed" sensor_msgs::JointState)
(ros::subscribe "/joint_states_compressed" *message-type*
  #'joint-state-compressed-callback)

(ros::spin)
