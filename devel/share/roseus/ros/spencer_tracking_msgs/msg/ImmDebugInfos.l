;; Auto-generated. Do not edit!


(when (boundp 'spencer_tracking_msgs::ImmDebugInfos)
  (if (not (find-package "SPENCER_TRACKING_MSGS"))
    (make-package "SPENCER_TRACKING_MSGS"))
  (shadow 'ImmDebugInfos (find-package "SPENCER_TRACKING_MSGS")))
(unless (find-package "SPENCER_TRACKING_MSGS::IMMDEBUGINFOS")
  (make-package "SPENCER_TRACKING_MSGS::IMMDEBUGINFOS"))

(in-package "ROS")
;;//! \htmlinclude ImmDebugInfos.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass spencer_tracking_msgs::ImmDebugInfos
  :super ros::object
  :slots (_header _infos ))

(defmethod spencer_tracking_msgs::ImmDebugInfos
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:infos __infos) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _infos __infos)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:infos
   (&rest __infos)
   (if (keywordp (car __infos))
       (send* _infos __infos)
     (progn
       (if __infos (setq _infos (car __infos)))
       _infos)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; spencer_tracking_msgs/ImmDebugInfo[] _infos
    (apply #'+ (send-all _infos :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; spencer_tracking_msgs/ImmDebugInfo[] _infos
     (write-long (length _infos) s)
     (dolist (elem _infos)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; spencer_tracking_msgs/ImmDebugInfo[] _infos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _infos (let (r) (dotimes (i n) (push (instance spencer_tracking_msgs::ImmDebugInfo :init) r)) r))
     (dolist (elem- _infos)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get spencer_tracking_msgs::ImmDebugInfos :md5sum-) "ce7fa675b582455db7386ac3eaa36b5b")
(setf (get spencer_tracking_msgs::ImmDebugInfos :datatype-) "spencer_tracking_msgs/ImmDebugInfos")
(setf (get spencer_tracking_msgs::ImmDebugInfos :definition-)
      "# Message with all debug infos per frame
#

Header              header      # Header containing timestamp etc. of this message
ImmDebugInfo[]      infos      # All persons that are currently being tracked
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

================================================================================
MSG: spencer_tracking_msgs/ImmDebugInfo
# Message for passing debug information of filter performance
#

uint64      track_id        # unique identifier of the target, consistent over time
float64      innovation      # innovation of prediction and associated observation
float64      CpXX            # variance of prediction acc. to x
float64      CpYY            # variance of prediction acc. to y
float64      CXX             # variance of state acc. to x
float64      CYY             # variance of state acc. to y
float64[]    modeProbabilities# array containing mode probabilities
")



(provide :spencer_tracking_msgs/ImmDebugInfos "ce7fa675b582455db7386ac3eaa36b5b")


