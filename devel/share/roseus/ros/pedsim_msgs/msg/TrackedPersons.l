;; Auto-generated. Do not edit!


(when (boundp 'pedsim_msgs::TrackedPersons)
  (if (not (find-package "PEDSIM_MSGS"))
    (make-package "PEDSIM_MSGS"))
  (shadow 'TrackedPersons (find-package "PEDSIM_MSGS")))
(unless (find-package "PEDSIM_MSGS::TRACKEDPERSONS")
  (make-package "PEDSIM_MSGS::TRACKEDPERSONS"))

(in-package "ROS")
;;//! \htmlinclude TrackedPersons.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pedsim_msgs::TrackedPersons
  :super ros::object
  :slots (_header _tracks ))

(defmethod pedsim_msgs::TrackedPersons
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:tracks __tracks) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _tracks __tracks)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:tracks
   (&rest __tracks)
   (if (keywordp (car __tracks))
       (send* _tracks __tracks)
     (progn
       (if __tracks (setq _tracks (car __tracks)))
       _tracks)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; pedsim_msgs/TrackedPerson[] _tracks
    (apply #'+ (send-all _tracks :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; pedsim_msgs/TrackedPerson[] _tracks
     (write-long (length _tracks) s)
     (dolist (elem _tracks)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; pedsim_msgs/TrackedPerson[] _tracks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tracks (let (r) (dotimes (i n) (push (instance pedsim_msgs::TrackedPerson :init) r)) r))
     (dolist (elem- _tracks)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get pedsim_msgs::TrackedPersons :md5sum-) "21c0b1a57c4933e68f39aa3802861828")
(setf (get pedsim_msgs::TrackedPersons :datatype-) "pedsim_msgs/TrackedPersons")
(setf (get pedsim_msgs::TrackedPersons :definition-)
      "# Message with all currently tracked persons 
#

Header              header      # Header containing timestamp etc. of this message
TrackedPerson[]     tracks      # All persons that are currently being tracked
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
MSG: pedsim_msgs/TrackedPerson
# Message defining a tracked person
#

uint64      track_id        # unique identifier of the target, consistent over time
bool        is_occluded     # if the track is currently not observable in a physical way
bool        is_matched      # if the track is currently matched by a detection
uint64      detection_id    # id of the corresponding detection in the current cycle (undefined if occluded)
duration    age             # age of the track

# The following fields are extracted from the Kalman state x and its covariance C

geometry_msgs/PoseWithCovariance       pose   # pose of the track (z value and orientation might not be set, check if corresponding variance on diagonal is > 99999)

geometry_msgs/TwistWithCovariance   twist     # velocity of the track (z value and rotational velocities might not be set, check if corresponding variance on diagonal is > 99999)

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

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :pedsim_msgs/TrackedPersons "21c0b1a57c4933e68f39aa3802861828")


