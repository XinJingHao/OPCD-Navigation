;; Auto-generated. Do not edit!


(when (boundp 'pedsim_msgs::TrackedGroups)
  (if (not (find-package "PEDSIM_MSGS"))
    (make-package "PEDSIM_MSGS"))
  (shadow 'TrackedGroups (find-package "PEDSIM_MSGS")))
(unless (find-package "PEDSIM_MSGS::TRACKEDGROUPS")
  (make-package "PEDSIM_MSGS::TRACKEDGROUPS"))

(in-package "ROS")
;;//! \htmlinclude TrackedGroups.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pedsim_msgs::TrackedGroups
  :super ros::object
  :slots (_header _groups ))

(defmethod pedsim_msgs::TrackedGroups
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
    ;; pedsim_msgs/TrackedGroup[] _groups
    (apply #'+ (send-all _groups :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; pedsim_msgs/TrackedGroup[] _groups
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
   ;; pedsim_msgs/TrackedGroup[] _groups
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _groups (let (r) (dotimes (i n) (push (instance pedsim_msgs::TrackedGroup :init) r)) r))
     (dolist (elem- _groups)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get pedsim_msgs::TrackedGroups :md5sum-) "44229e5f365e63f958cbe69d06c05cc4")
(setf (get pedsim_msgs::TrackedGroups :datatype-) "pedsim_msgs/TrackedGroups")
(setf (get pedsim_msgs::TrackedGroups :definition-)
      "# Message with all currently tracked groups
#

Header              header      # Header containing timestamp etc. of this message
pedsim_msgs/TrackedGroup[]      groups      # All groups that are currently being tracked

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
MSG: pedsim_msgs/TrackedGroup
# Message defining a tracked group
#

uint64      group_id        # unique identifier of the target, consistent over time

duration    age             # age of the group

geometry_msgs/PoseWithCovariance    centerOfGravity   # mean and covariance of the group (calculated from its person tracks)

uint64[]    track_ids       # IDs of the tracked persons in this group. See srl_tracking_msgs/TrackedPersons
================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

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



(provide :pedsim_msgs/TrackedGroups "44229e5f365e63f958cbe69d06c05cc4")


