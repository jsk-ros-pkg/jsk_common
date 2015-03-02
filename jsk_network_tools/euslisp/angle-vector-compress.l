(defun compress-angle-vector (robot av &optional tms)
  "Convert angle vector (float-vector) into uchar vector.
   0 means equal to min-angle, and 255 means equal to max-angle."
  (if tms
      (error "time is not impremented!"))
  (coerce (mapcar #'(lambda (v jt)
                      (max 0.0 
                           (min 255.0
                                (if (or (derivedp jt linear-joint)
                                        (not (infinite-joint-p jt)))
                                    ;; 255 * (v - min) / (max - min)
                                    (round (* (/ (- v (send jt :min-angle))
                                                 (- (send jt :max-angle)
                                                    (send jt :min-angle)))
                                              255))
                                  (round (* (/ (360-mod v) 360.0) 255))))))
                  (coerce av cons) (send robot :joint-list))
          integer-vector))


(defun decompress-angle-vector (robot av)
  "Decompress angle vector written in uchar vector to float-vector"
  (coerce (mapcar #'(lambda (v jt)
                      (if (or (derivedp jt linear-joint)
                              (not (infinite-joint-p jt)))
                          ;; v * (max - min) / 255 + min
                          (+ (send jt :min-angle)
                             (* (/ v 255.0)
                                (- (send jt :max-angle)
                                   (send jt :min-angle))))
                        (* (/ v 255.0) 360.0)))
                  (subseq (coerce av cons) 0 (length (send robot :joint-list)))
                  (send robot :joint-list))
          float-vector))

(defun infinite-joint-p (jt)
  (or (> (send jt :max-angle) 360)
      (< (send jt :min-angle) -360)))

(defun 360-mod (v)
  (if (> v 0)
      (mod v 360)
    (+ 360 (mod v 360)))))

(defun load-robot-model (robot-name)
  (let ((lrobot-name (string-downcase robot-name)))
    (load (cond ((string= lrobot-name "pr2")
                 "package://pr2eus/pr2.l")
                ((string= lrobot-name "hrp2jsk")
                 "package://hrpsys_ros_bridge_tutorials/models/hrp2jsk.l")
                ((string= lrobot-name "hrp2jsknt")
                 "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknt.l")
                ((string= lrobot-name "hrp2jsknts")
                 "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknts.l")
                ((string= lrobot-name "staro")
                 "package://hrpsys_ros_bridge_tutorials/models/staro.l")
                ((string= lrobot-name "urataleg")
                 "package://hrpsys_ros_bridge_tutorials/models/urataleg.l")
                ((string= lrobot-name "samplerobot")
                 "package://hrpsys_ros_bridge_tutorials/models/samplerobot.l")
                ((string= lrobot-name "hrp2w")
                 "package://hrpsys_ros_bridge_tutorials/models/hrp2w.l")))))

(provide :compressed-angle-vector)
