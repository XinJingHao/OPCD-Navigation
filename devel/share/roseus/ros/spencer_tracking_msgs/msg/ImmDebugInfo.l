;; Auto-generated. Do not edit!


(when (boundp 'spencer_tracking_msgs::ImmDebugInfo)
  (if (not (find-package "SPENCER_TRACKING_MSGS"))
    (make-package "SPENCER_TRACKING_MSGS"))
  (shadow 'ImmDebugInfo (find-package "SPENCER_TRACKING_MSGS")))
(unless (find-package "SPENCER_TRACKING_MSGS::IMMDEBUGINFO")
  (make-package "SPENCER_TRACKING_MSGS::IMMDEBUGINFO"))

(in-package "ROS")
;;//! \htmlinclude ImmDebugInfo.msg.html


(defclass spencer_tracking_msgs::ImmDebugInfo
  :super ros::object
  :slots (_track_id _innovation _CpXX _CpYY _CXX _CYY _modeProbabilities ))

(defmethod spencer_tracking_msgs::ImmDebugInfo
  (:init
   (&key
    ((:track_id __track_id) 0)
    ((:innovation __innovation) 0.0)
    ((:CpXX __CpXX) 0.0)
    ((:CpYY __CpYY) 0.0)
    ((:CXX __CXX) 0.0)
    ((:CYY __CYY) 0.0)
    ((:modeProbabilities __modeProbabilities) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _track_id (round __track_id))
   (setq _innovation (float __innovation))
   (setq _CpXX (float __CpXX))
   (setq _CpYY (float __CpYY))
   (setq _CXX (float __CXX))
   (setq _CYY (float __CYY))
   (setq _modeProbabilities __modeProbabilities)
   self)
  (:track_id
   (&optional __track_id)
   (if __track_id (setq _track_id __track_id)) _track_id)
  (:innovation
   (&optional __innovation)
   (if __innovation (setq _innovation __innovation)) _innovation)
  (:CpXX
   (&optional __CpXX)
   (if __CpXX (setq _CpXX __CpXX)) _CpXX)
  (:CpYY
   (&optional __CpYY)
   (if __CpYY (setq _CpYY __CpYY)) _CpYY)
  (:CXX
   (&optional __CXX)
   (if __CXX (setq _CXX __CXX)) _CXX)
  (:CYY
   (&optional __CYY)
   (if __CYY (setq _CYY __CYY)) _CYY)
  (:modeProbabilities
   (&optional __modeProbabilities)
   (if __modeProbabilities (setq _modeProbabilities __modeProbabilities)) _modeProbabilities)
  (:serialization-length
   ()
   (+
    ;; uint64 _track_id
    8
    ;; float64 _innovation
    8
    ;; float64 _CpXX
    8
    ;; float64 _CpYY
    8
    ;; float64 _CXX
    8
    ;; float64 _CYY
    8
    ;; float64[] _modeProbabilities
    (* 8    (length _modeProbabilities)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint64 _track_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _track_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _track_id) (= (length (_track_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_track_id . bv) 0) 0) s)
              (write-long (ash (elt (_track_id . bv) 1) -1) s))
             ((and (class _track_id) (= (length (_track_id . bv)) 1)) ;; big1
              (write-long (elt (_track_id . bv) 0) s)
              (write-long (if (>= _track_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _track_id s)(write-long (if (>= _track_id 0) 0 #xffffffff) s)))
     ;; float64 _innovation
       (sys::poke _innovation (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _CpXX
       (sys::poke _CpXX (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _CpYY
       (sys::poke _CpYY (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _CXX
       (sys::poke _CXX (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _CYY
       (sys::poke _CYY (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _modeProbabilities
     (write-long (length _modeProbabilities) s)
     (dotimes (i (length _modeProbabilities))
       (sys::poke (elt _modeProbabilities i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint64 _track_id
     
#+(or :alpha :irix6 :x86_64)
      (setf _track_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _track_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _innovation
     (setq _innovation (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _CpXX
     (setq _CpXX (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _CpYY
     (setq _CpYY (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _CXX
     (setq _CXX (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _CYY
     (setq _CYY (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _modeProbabilities
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _modeProbabilities (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _modeProbabilities i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get spencer_tracking_msgs::ImmDebugInfo :md5sum-) "824cd837fd158664a6f185fb8316da53")
(setf (get spencer_tracking_msgs::ImmDebugInfo :datatype-) "spencer_tracking_msgs/ImmDebugInfo")
(setf (get spencer_tracking_msgs::ImmDebugInfo :definition-)
      "# Message for passing debug information of filter performance
#

uint64      track_id        # unique identifier of the target, consistent over time
float64      innovation      # innovation of prediction and associated observation
float64      CpXX            # variance of prediction acc. to x
float64      CpYY            # variance of prediction acc. to y
float64      CXX             # variance of state acc. to x
float64      CYY             # variance of state acc. to y
float64[]    modeProbabilities# array containing mode probabilities
")



(provide :spencer_tracking_msgs/ImmDebugInfo "824cd837fd158664a6f185fb8316da53")


