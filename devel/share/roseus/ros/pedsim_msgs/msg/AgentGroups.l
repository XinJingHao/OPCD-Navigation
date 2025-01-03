;; Auto-generated. Do not edit!


(when (boundp 'pedsim_msgs::AgentGroups)
  (if (not (find-package "PEDSIM_MSGS"))
    (make-package "PEDSIM_MSGS"))
  (shadow 'AgentGroups (find-package "PEDSIM_MSGS")))
(unless (find-package "PEDSIM_MSGS::AGENTGROUPS")
  (make-package "PEDSIM_MSGS::AGENTGROUPS"))

(in-package "ROS")
;;//! \htmlinclude AgentGroups.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pedsim_msgs::AgentGroups
  :super ros::object
  :slots (_header _groups ))

(defmethod pedsim_msgs::AgentGroups
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:groups __groups) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _groups __groups)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:groups
   (&rest __groups)
   (if (keywordp (car __groups))
       (send* _groups __groups)
     (progn
       (if __groups (setq _groups (car __groups)))
       _groups)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; pedsim_msgs/AgentGroup[] _groups
    (apply #'+ (send-all _groups :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; pedsim_msgs/AgentGroup[] _groups
     (write-long (length _groups) s)
     (dolist (elem _groups)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; pedsim_msgs/AgentGroup[] _groups
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _groups (let (r) (dotimes (i n) (push (instance pedsim_msgs::AgentGroup :init) r)) r))
     (dolist (elem- _groups)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get pedsim_msgs::AgentGroups :md5sum-) "6166f52c402612b904d0fda690f00b1c")
(setf (get pedsim_msgs::AgentGroups :datatype-) "pedsim_msgs/AgentGroups")
(setf (get pedsim_msgs::AgentGroups :definition-)
      "Header header
pedsim_msgs/AgentGroup[] groups

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
MSG: pedsim_msgs/AgentGroup
Header header
uint16 group_id
float64 age
uint64[] members
geometry_msgs/Pose center_of_mass

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :pedsim_msgs/AgentGroups "6166f52c402612b904d0fda690f00b1c")


