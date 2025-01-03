;; Auto-generated. Do not edit!


(when (boundp 'pedsim_msgs::Waypoints)
  (if (not (find-package "PEDSIM_MSGS"))
    (make-package "PEDSIM_MSGS"))
  (shadow 'Waypoints (find-package "PEDSIM_MSGS")))
(unless (find-package "PEDSIM_MSGS::WAYPOINTS")
  (make-package "PEDSIM_MSGS::WAYPOINTS"))

(in-package "ROS")
;;//! \htmlinclude Waypoints.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pedsim_msgs::Waypoints
  :super ros::object
  :slots (_header _waypoints ))

(defmethod pedsim_msgs::Waypoints
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:waypoints __waypoints) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _waypoints __waypoints)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:waypoints
   (&rest __waypoints)
   (if (keywordp (car __waypoints))
       (send* _waypoints __waypoints)
     (progn
       (if __waypoints (setq _waypoints (car __waypoints)))
       _waypoints)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; pedsim_msgs/Waypoint[] _waypoints
    (apply #'+ (send-all _waypoints :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; pedsim_msgs/Waypoint[] _waypoints
     (write-long (length _waypoints) s)
     (dolist (elem _waypoints)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; pedsim_msgs/Waypoint[] _waypoints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _waypoints (let (r) (dotimes (i n) (push (instance pedsim_msgs::Waypoint :init) r)) r))
     (dolist (elem- _waypoints)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get pedsim_msgs::Waypoints :md5sum-) "ce5223455655616b49596a8c857a4c8b")
(setf (get pedsim_msgs::Waypoints :datatype-) "pedsim_msgs/Waypoints")
(setf (get pedsim_msgs::Waypoints :definition-)
      "Header header
pedsim_msgs/Waypoint[] waypoints

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
MSG: pedsim_msgs/Waypoint
int8 BHV_SIMPLE = 0
int8 BHV_SOURCE = 1
int8 BHV_SINK = 2

string name
int8 behavior
geometry_msgs/Point position
float32 radius

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :pedsim_msgs/Waypoints "ce5223455655616b49596a8c857a4c8b")


