; Auto-generated. Do not edit!


(cl:in-package pedsim_msgs-msg)


;//! \htmlinclude AgentState.msg.html

(cl:defclass <AgentState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (social_state
    :reader social_state
    :initarg :social_state
    :type cl:string
    :initform "")
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (twist
    :reader twist
    :initarg :twist
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (forces
    :reader forces
    :initarg :forces
    :type pedsim_msgs-msg:AgentForce
    :initform (cl:make-instance 'pedsim_msgs-msg:AgentForce)))
)

(cl:defclass AgentState (<AgentState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AgentState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AgentState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pedsim_msgs-msg:<AgentState> is deprecated: use pedsim_msgs-msg:AgentState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AgentState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:header-val is deprecated.  Use pedsim_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <AgentState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:id-val is deprecated.  Use pedsim_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <AgentState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:type-val is deprecated.  Use pedsim_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'social_state-val :lambda-list '(m))
(cl:defmethod social_state-val ((m <AgentState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:social_state-val is deprecated.  Use pedsim_msgs-msg:social_state instead.")
  (social_state m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <AgentState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:pose-val is deprecated.  Use pedsim_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'twist-val :lambda-list '(m))
(cl:defmethod twist-val ((m <AgentState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:twist-val is deprecated.  Use pedsim_msgs-msg:twist instead.")
  (twist m))

(cl:ensure-generic-function 'forces-val :lambda-list '(m))
(cl:defmethod forces-val ((m <AgentState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:forces-val is deprecated.  Use pedsim_msgs-msg:forces instead.")
  (forces m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<AgentState>)))
    "Constants for message type '<AgentState>"
  '((:TYPE_STANDING . "\"standing\"")
    (:TYPE_INDIVIDUAL_MOVING . "\"individual_moving\"")
    (:TYPE_WAITING_IN_QUEUE . "\"waiting_in_queue\"")
    (:TYPE_GROUP_MOVING . "\"group_moving\""))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'AgentState)))
    "Constants for message type 'AgentState"
  '((:TYPE_STANDING . "\"standing\"")
    (:TYPE_INDIVIDUAL_MOVING . "\"individual_moving\"")
    (:TYPE_WAITING_IN_QUEUE . "\"waiting_in_queue\"")
    (:TYPE_GROUP_MOVING . "\"group_moving\""))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AgentState>) ostream)
  "Serializes a message object of type '<AgentState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'type)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'social_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'social_state))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'twist) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'forces) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AgentState>) istream)
  "Deserializes a message object of type '<AgentState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'social_state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'social_state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'twist) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'forces) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AgentState>)))
  "Returns string type for a message object of type '<AgentState>"
  "pedsim_msgs/AgentState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AgentState)))
  "Returns string type for a message object of type 'AgentState"
  "pedsim_msgs/AgentState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AgentState>)))
  "Returns md5sum for a message object of type '<AgentState>"
  "b47b93bd58ccba0e6f058fd6a71a4915")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AgentState)))
  "Returns md5sum for a message object of type 'AgentState"
  "b47b93bd58ccba0e6f058fd6a71a4915")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AgentState>)))
  "Returns full string definition for message of type '<AgentState>"
  (cl:format cl:nil "Header header~%uint64 id~%uint16 type~%string social_state~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%pedsim_msgs/AgentForce forces~%~%# Use sensors package to control observability~%~%# Social State string constants~%string      TYPE_STANDING = \"standing\"~%string      TYPE_INDIVIDUAL_MOVING = \"individual_moving\"~%string      TYPE_WAITING_IN_QUEUE = \"waiting_in_queue\"~%string      TYPE_GROUP_MOVING = \"group_moving\"~%~%~%# Agent types~%# 0, 1 -> ordinary agents~%# 2 -> Robot~%# 3 -> standing/elderly agents~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: pedsim_msgs/AgentForce~%# Forces acting on an agent.~%~%# Basic SFM forces.~%geometry_msgs/Vector3 desired_force~%geometry_msgs/Vector3 obstacle_force~%geometry_msgs/Vector3 social_force~%~%# Additional Group Forces~%geometry_msgs/Vector3 group_coherence_force~%geometry_msgs/Vector3 group_gaze_force~%geometry_msgs/Vector3 group_repulsion_force~%~%# Extra stabilization/custom forces.~%geometry_msgs/Vector3 random_force~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AgentState)))
  "Returns full string definition for message of type 'AgentState"
  (cl:format cl:nil "Header header~%uint64 id~%uint16 type~%string social_state~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%pedsim_msgs/AgentForce forces~%~%# Use sensors package to control observability~%~%# Social State string constants~%string      TYPE_STANDING = \"standing\"~%string      TYPE_INDIVIDUAL_MOVING = \"individual_moving\"~%string      TYPE_WAITING_IN_QUEUE = \"waiting_in_queue\"~%string      TYPE_GROUP_MOVING = \"group_moving\"~%~%~%# Agent types~%# 0, 1 -> ordinary agents~%# 2 -> Robot~%# 3 -> standing/elderly agents~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: pedsim_msgs/AgentForce~%# Forces acting on an agent.~%~%# Basic SFM forces.~%geometry_msgs/Vector3 desired_force~%geometry_msgs/Vector3 obstacle_force~%geometry_msgs/Vector3 social_force~%~%# Additional Group Forces~%geometry_msgs/Vector3 group_coherence_force~%geometry_msgs/Vector3 group_gaze_force~%geometry_msgs/Vector3 group_repulsion_force~%~%# Extra stabilization/custom forces.~%geometry_msgs/Vector3 random_force~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AgentState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     2
     4 (cl:length (cl:slot-value msg 'social_state))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'twist))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'forces))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AgentState>))
  "Converts a ROS message object to a list"
  (cl:list 'AgentState
    (cl:cons ':header (header msg))
    (cl:cons ':id (id msg))
    (cl:cons ':type (type msg))
    (cl:cons ':social_state (social_state msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':twist (twist msg))
    (cl:cons ':forces (forces msg))
))
