(defun compress-angle-vector (robot av &optional tms)
  "Convert angle vector (float-vector) into uchar vector.
   0 means equal to min-angle, and 255 means equal to max-angle."
  (if tms
      (error "time is not impremented!"))
  (coerce (mapcar #'(lambda (v jt)
                      (if (or (derivedp jt linear-joint)
                              (not (infinite-joint-p jt)))
                          (round (* (/ (- v (send jt :min-angle))
                                       (- (send jt :max-angle)
                                          (send jt :min-angle)))
                                    255))
                        (round (* (/ (360-mod v) 360.0) 255))))
                  (coerce av cons) (send robot :joint-list))
          integer-vector))


(defun decompress-angle-vector (robot av)
  "Decompress angle vector written in uchar vector to float-vector"
  (coerce (mapcar #'(lambda (v jt)
                      (if (or (derivedp jt linear-joint)
                              (not (infinite-joint-p jt)))
                          (+ (send jt :min-angle)
                             (* (/ v 255.0)
                                (- (send jt :max-angle)
                                   (send jt :min-angle))))
                        (* (/ v 255.0) 360.0)))
                  (coerce av cons)
                  (send robot :joint-list))
          float-vector))

(defun infinite-joint-p (jt)
  (or (> (send jt :max-angle) 360)
      (< (send jt :min-angle) -360)))

(defun 360-mod (v)
  (if (> v 0)
      (mod v 360)
    (+ 360 (mod v 360)))))

(provide :compressed-angle-vector)
