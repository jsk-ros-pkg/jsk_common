#!/usr/bin/env runeus
;;;
;;; euslisp version of ros_tutorials/rospy_tutorials/001_talker_listener
;;;
(load (format nil "~A/src/eus-pddl.l"
              (read-line (piped-fork "rospack find pddl_planner"))))
;;(setq ros::*compile-message* t) ;; comment out if you want to compile msg and srv
;; when msgs and srvs are compiled, all msgs and srvs should be loaded.
(roseus-add-msgs "roslib")
(roseus-add-msgs "pddl_msgs")

;;(send (instance pddl_msgs::PDDLhoge :init :a (list "a" "b")) :serialize)
;;(describe (send (instance pddl_msgs::PDDLhoge :init) :deserialize
;;                (send (instance pddl_msgs::PDDLhoge :init :a (list "c" "d")) :serialize)))

#|
(roseus "listener")

(defclass string-cb-class
  :super propertied-object
  :slots ())
(defmethod string-cb-class
  (:init () (ros::subscribe "chatter" pddl_msgs::PDDLhoge
              #'send self :string-cb))
  (:string-cb (msg) (print (list 'cb self (send msg :a)))))
(setq m (instance string-cb-class :init))

(do-until-key
 (ros::spin-once)
 ;;(sys::gc)
 )
#|