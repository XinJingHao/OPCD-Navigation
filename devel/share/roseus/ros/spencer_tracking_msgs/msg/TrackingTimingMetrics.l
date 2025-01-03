;; Auto-generated. Do not edit!


(when (boundp 'spencer_tracking_msgs::TrackingTimingMetrics)
  (if (not (find-package "SPENCER_TRACKING_MSGS"))
    (make-package "SPENCER_TRACKING_MSGS"))
  (shadow 'TrackingTimingMetrics (find-package "SPENCER_TRACKING_MSGS")))
(unless (find-package "SPENCER_TRACKING_MSGS::TRACKINGTIMINGMETRICS")
  (make-package "SPENCER_TRACKING_MSGS::TRACKINGTIMINGMETRICS"))

(in-package "ROS")
;;//! \htmlinclude TrackingTimingMetrics.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass spencer_tracking_msgs::TrackingTimingMetrics
  :super ros::object
  :slots (_header _track_count _cycle_no _average_cycle_time _cycle_time _average_processing_rate _cpu_load _average_cpu_load _elapsed_time _elapsed_cpu_time ))

(defmethod spencer_tracking_msgs::TrackingTimingMetrics
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:track_count __track_count) 0)
    ((:cycle_no __cycle_no) 0)
    ((:average_cycle_time __average_cycle_time) 0.0)
    ((:cycle_time __cycle_time) 0.0)
    ((:average_processing_rate __average_processing_rate) 0.0)
    ((:cpu_load __cpu_load) 0.0)
    ((:average_cpu_load __average_cpu_load) 0.0)
    ((:elapsed_time __elapsed_time) 0.0)
    ((:elapsed_cpu_time __elapsed_cpu_time) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _track_count (round __track_count))
   (setq _cycle_no (round __cycle_no))
   (setq _average_cycle_time (float __average_cycle_time))
   (setq _cycle_time (float __cycle_time))
   (setq _average_processing_rate (float __average_processing_rate))
   (setq _cpu_load (float __cpu_load))
   (setq _average_cpu_load (float __average_cpu_load))
   (setq _elapsed_time (float __elapsed_time))
   (setq _elapsed_cpu_time (float __elapsed_cpu_time))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:track_count
   (&optional __track_count)
   (if __track_count (setq _track_count __track_count)) _track_count)
  (:cycle_no
   (&optional __cycle_no)
   (if __cycle_no (setq _cycle_no __cycle_no)) _cycle_no)
  (:average_cycle_time
   (&optional __average_cycle_time)
   (if __average_cycle_time (setq _average_cycle_time __average_cycle_time)) _average_cycle_time)
  (:cycle_time
   (&optional __cycle_time)
   (if __cycle_time (setq _cycle_time __cycle_time)) _cycle_time)
  (:average_processing_rate
   (&optional __average_processing_rate)
   (if __average_processing_rate (setq _average_processing_rate __average_processing_rate)) _average_processing_rate)
  (:cpu_load
   (&optional __cpu_load)
   (if __cpu_load (setq _cpu_load __cpu_load)) _cpu_load)
  (:average_cpu_load
   (&optional __average_cpu_load)
   (if __average_cpu_load (setq _average_cpu_load __average_cpu_load)) _average_cpu_load)
  (:elapsed_time
   (&optional __elapsed_time)
   (if __elapsed_time (setq _elapsed_time __elapsed_time)) _elapsed_time)
  (:elapsed_cpu_time
   (&optional __elapsed_cpu_time)
   (if __elapsed_cpu_time (setq _elapsed_cpu_time __elapsed_cpu_time)) _elapsed_cpu_time)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint64 _track_count
    8
    ;; uint64 _cycle_no
    8
    ;; float32 _average_cycle_time
    4
    ;; float32 _cycle_time
    4
    ;; float32 _average_processing_rate
    4
    ;; float32 _cpu_load
    4
    ;; float32 _average_cpu_load
    4
    ;; float32 _elapsed_time
    4
    ;; float32 _elapsed_cpu_time
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint64 _track_count
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _track_count (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _track_count) (= (length (_track_count . bv)) 2)) ;; bignum
              (write-long (ash (elt (_track_count . bv) 0) 0) s)
              (write-long (ash (elt (_track_count . bv) 1) -1) s))
             ((and (class _track_count) (= (length (_track_count . bv)) 1)) ;; big1
              (write-long (elt (_track_count . bv) 0) s)
              (write-long (if (>= _track_count 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _track_count s)(write-long (if (>= _track_count 0) 0 #xffffffff) s)))
     ;; uint64 _cycle_no
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _cycle_no (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _cycle_no) (= (length (_cycle_no . bv)) 2)) ;; bignum
              (write-long (ash (elt (_cycle_no . bv) 0) 0) s)
              (write-long (ash (elt (_cycle_no . bv) 1) -1) s))
             ((and (class _cycle_no) (= (length (_cycle_no . bv)) 1)) ;; big1
              (write-long (elt (_cycle_no . bv) 0) s)
              (write-long (if (>= _cycle_no 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _cycle_no s)(write-long (if (>= _cycle_no 0) 0 #xffffffff) s)))
     ;; float32 _average_cycle_time
       (sys::poke _average_cycle_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cycle_time
       (sys::poke _cycle_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _average_processing_rate
       (sys::poke _average_processing_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cpu_load
       (sys::poke _cpu_load (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _average_cpu_load
       (sys::poke _average_cpu_load (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _elapsed_time
       (sys::poke _elapsed_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _elapsed_cpu_time
       (sys::poke _elapsed_cpu_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint64 _track_count
     
#+(or :alpha :irix6 :x86_64)
      (setf _track_count (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _track_count (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint64 _cycle_no
     
#+(or :alpha :irix6 :x86_64)
      (setf _cycle_no (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _cycle_no (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float32 _average_cycle_time
     (setq _average_cycle_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cycle_time
     (setq _cycle_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _average_processing_rate
     (setq _average_processing_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cpu_load
     (setq _cpu_load (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _average_cpu_load
     (setq _average_cpu_load (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _elapsed_time
     (setq _elapsed_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _elapsed_cpu_time
     (setq _elapsed_cpu_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get spencer_tracking_msgs::TrackingTimingMetrics :md5sum-) "e5e4499959c0302da5d8864744a1d65a")
(setf (get spencer_tracking_msgs::TrackingTimingMetrics :datatype-) "spencer_tracking_msgs/TrackingTimingMetrics")
(setf (get spencer_tracking_msgs::TrackingTimingMetrics :definition-)
      "# Message with timing values for analyzing the efficency 
#

Header              header                  # Header containing timestamp etc. of this message
uint64              track_count             # number of person currentl tracked
uint64              cycle_no                # incremented number of cycles        
float32             average_cycle_time      # average time for one cycle of tracker
float32             cycle_time              # cycle time of current cycle
float32             average_processing_rate # average frequency of processing data
float32             cpu_load                # current cpu load
float32             average_cpu_load        # average cpu load
float32             elapsed_time            # elapsed seconds since start
float32             elapsed_cpu_time        # elapsed cpu time since start
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :spencer_tracking_msgs/TrackingTimingMetrics "e5e4499959c0302da5d8864744a1d65a")


