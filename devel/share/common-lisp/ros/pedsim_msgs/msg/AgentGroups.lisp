; Auto-generated. Do not edit!


(cl:in-package pedsim_msgs-msg)


;//! \htmlinclude AgentGroups.msg.html

(cl:defclass <AgentGroups> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (groups
    :reader groups
    :initarg :groups
    :type (cl:vector pedsim_msgs-msg:AgentGroup)
   :initform (cl:make-array 0 :element-type 'pedsim_msgs-msg:AgentGroup :initial-element (cl:make-instance 'pedsim_msgs-msg:AgentGroup))))
)

(cl:defclass AgentGroups (<AgentGroups>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AgentGroups>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AgentGroups)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pedsim_msgs-msg:<AgentGroups> is deprecated: use pedsim_msgs-msg:AgentGroups instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AgentGroups>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:header-val is deprecated.  Use pedsim_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'groups-val :lambda-list '(m))
(cl:defmethod groups-val ((m <AgentGroups>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:groups-val is deprecated.  Use pedsim_msgs-msg:groups instead.")
  (groups m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AgentGroups>) ostream)
  "Serializes a message object of type '<AgentGroups>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'groups))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'groups))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AgentGroups>) istream)
  "Deserializes a message object of type '<AgentGroups>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'groups) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'groups)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'pedsim_msgs-msg:AgentGroup))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AgentGroups>)))
  "Returns string type for a message object of type '<AgentGroups>"
  "pedsim_msgs/AgentGroups")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AgentGroups)))
  "Returns string type for a message object of type 'AgentGroups"
  "pedsim_msgs/AgentGroups")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AgentGroups>)))
  "Returns md5sum for a message object of type '<AgentGroups>"
  "6166f52c402612b904d0fda690f00b1c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AgentGroups)))
  "Returns md5sum for a message object of type 'AgentGroups"
  "6166f52c402612b904d0fda690f00b1c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AgentGroups>)))
  "Returns full string definition for message of type '<AgentGroups>"
  (cl:format cl:nil "Header header~%pedsim_msgs/AgentGroup[] groups~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: pedsim_msgs/AgentGroup~%Header header~%uint16 group_id~%float64 age~%uint64[] members~%geometry_msgs/Pose center_of_mass~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AgentGroups)))
  "Returns full string definition for message of type 'AgentGroups"
  (cl:format cl:nil "Header header~%pedsim_msgs/AgentGroup[] groups~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: pedsim_msgs/AgentGroup~%Header header~%uint16 group_id~%float64 age~%uint64[] members~%geometry_msgs/Pose center_of_mass~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AgentGroups>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'groups) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AgentGroups>))
  "Converts a ROS message object to a list"
  (cl:list 'AgentGroups
    (cl:cons ':header (header msg))
    (cl:cons ':groups (groups msg))
))
