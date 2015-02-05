#!/usr/bin/env roseus
(require :compressed-angle-vector "package://jsk_network_tools/euslisp/angle-vector-compress.l")
(ros::roseus-add-msgs "sensor_msgs")

(ros::roseus "joint_state_compressor")


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

(defun joint-state-callback (inmsg)
  (let ((msg (instance *message-type* :init))
        (avs nil))
    ;; update robot-model
    (dolist (j (send *robot* :joint-list))
      ;; lookup msg
      (dotimes (i (length (send inmsg :name)))
        (let ((n (elt (send inmsg :name) i)))
          (if (string= n (send j :name))
              (setq avs (append avs (list
                                     (if (derivedp j linear-joint)
                                         (* 1000 (elt (send inmsg :position) i))
                                       (rad2deg (elt (send inmsg :position) i))))))))))
    (send msg :angles (compress-angle-vector *robot* avs))
    (ros::publish "/joint_states_compressed" msg)
    ))
(ros::advertise "/joint_states_compressed" *message-type*)
(ros::subscribe "/joint_states" sensor_msgs::JointState #'joint-state-callback)

(ros::spin)
