; Auto-generated. Do not edit!


(cl:in-package spencer_tracking_msgs-msg)


;//! \htmlinclude TrackedGroups.msg.html

(cl:defclass <TrackedGroups> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (groups
    :reader groups
    :initarg :groups
    :type (cl:vector spencer_tracking_msgs-msg:TrackedGroup)
   :initform (cl:make-array 0 :element-type 'spencer_tracking_msgs-msg:TrackedGroup :initial-element (cl:make-instance 'spencer_tracking_msgs-msg:TrackedGroup))))
)

(cl:defclass TrackedGroups (<TrackedGroups>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackedGroups>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackedGroups)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_tracking_msgs-msg:<TrackedGroups> is deprecated: use spencer_tracking_msgs-msg:TrackedGroups instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TrackedGroups>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:header-val is deprecated.  Use spencer_tracking_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'groups-val :lambda-list '(m))
(cl:defmethod groups-val ((m <TrackedGroups>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:groups-val is deprecated.  Use spencer_tracking_msgs-msg:groups instead.")
  (groups m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackedGroups>) ostream)
  "Serializes a message object of type '<TrackedGroups>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'groups))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'groups))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackedGroups>) istream)
  "Deserializes a message object of type '<TrackedGroups>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'groups) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'groups)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'spencer_tracking_msgs-msg:TrackedGroup))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackedGroups>)))
  "Returns string type for a message object of type '<TrackedGroups>"
  "spencer_tracking_msgs/TrackedGroups")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackedGroups)))
  "Returns string type for a message object of type 'TrackedGroups"
  "spencer_tracking_msgs/TrackedGroups")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackedGroups>)))
  "Returns md5sum for a message object of type '<TrackedGroups>"
  "44229e5f365e63f958cbe69d06c05cc4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackedGroups)))
  "Returns md5sum for a message object of type 'TrackedGroups"
  "44229e5f365e63f958cbe69d06c05cc4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackedGroups>)))
  "Returns full string definition for message of type '<TrackedGroups>"
  (cl:format cl:nil "# Message with all currently tracked groups ~%#~%~%Header              header      # Header containing timestamp etc. of this message~%TrackedGroup[]      groups      # All groups that are currently being tracked~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: spencer_tracking_msgs/TrackedGroup~%# Message defining a tracked group~%#~%~%uint64      group_id        # unique identifier of the target, consistent over time~%~%duration    age             # age of the group~%~%geometry_msgs/PoseWithCovariance    centerOfGravity   # mean and covariance of the group (calculated from its person tracks)~%~%uint64[]    track_ids       # IDs of the tracked persons in this group. See srl_tracking_msgs/TrackedPersons~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackedGroups)))
  "Returns full string definition for message of type 'TrackedGroups"
  (cl:format cl:nil "# Message with all currently tracked groups ~%#~%~%Header              header      # Header containing timestamp etc. of this message~%TrackedGroup[]      groups      # All groups that are currently being tracked~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: spencer_tracking_msgs/TrackedGroup~%# Message defining a tracked group~%#~%~%uint64      group_id        # unique identifier of the target, consistent over time~%~%duration    age             # age of the group~%~%geometry_msgs/PoseWithCovariance    centerOfGravity   # mean and covariance of the group (calculated from its person tracks)~%~%uint64[]    track_ids       # IDs of the tracked persons in this group. See srl_tracking_msgs/TrackedPersons~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackedGroups>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'groups) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackedGroups>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackedGroups
    (cl:cons ':header (header msg))
    (cl:cons ':groups (groups msg))
))
