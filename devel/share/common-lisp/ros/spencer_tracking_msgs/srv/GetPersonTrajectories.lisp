; Auto-generated. Do not edit!


(cl:in-package spencer_tracking_msgs-srv)


;//! \htmlinclude GetPersonTrajectories-request.msg.html

(cl:defclass <GetPersonTrajectories-request> (roslisp-msg-protocol:ros-message)
  ((requested_ids
    :reader requested_ids
    :initarg :requested_ids
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (max_age
    :reader max_age
    :initarg :max_age
    :type cl:real
    :initform 0))
)

(cl:defclass GetPersonTrajectories-request (<GetPersonTrajectories-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPersonTrajectories-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPersonTrajectories-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_tracking_msgs-srv:<GetPersonTrajectories-request> is deprecated: use spencer_tracking_msgs-srv:GetPersonTrajectories-request instead.")))

(cl:ensure-generic-function 'requested_ids-val :lambda-list '(m))
(cl:defmethod requested_ids-val ((m <GetPersonTrajectories-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-srv:requested_ids-val is deprecated.  Use spencer_tracking_msgs-srv:requested_ids instead.")
  (requested_ids m))

(cl:ensure-generic-function 'max_age-val :lambda-list '(m))
(cl:defmethod max_age-val ((m <GetPersonTrajectories-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-srv:max_age-val is deprecated.  Use spencer_tracking_msgs-srv:max_age instead.")
  (max_age m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPersonTrajectories-request>) ostream)
  "Serializes a message object of type '<GetPersonTrajectories-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'requested_ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) ele) ostream))
   (cl:slot-value msg 'requested_ids))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'max_age)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'max_age) (cl:floor (cl:slot-value msg 'max_age)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPersonTrajectories-request>) istream)
  "Deserializes a message object of type '<GetPersonTrajectories-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'requested_ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'requested_ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:aref vals i)) (cl:read-byte istream)))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'max_age) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPersonTrajectories-request>)))
  "Returns string type for a service object of type '<GetPersonTrajectories-request>"
  "spencer_tracking_msgs/GetPersonTrajectoriesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPersonTrajectories-request)))
  "Returns string type for a service object of type 'GetPersonTrajectories-request"
  "spencer_tracking_msgs/GetPersonTrajectoriesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPersonTrajectories-request>)))
  "Returns md5sum for a message object of type '<GetPersonTrajectories-request>"
  "e0e3d63b99808526b112847474001abe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPersonTrajectories-request)))
  "Returns md5sum for a message object of type 'GetPersonTrajectories-request"
  "e0e3d63b99808526b112847474001abe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPersonTrajectories-request>)))
  "Returns full string definition for message of type '<GetPersonTrajectories-request>"
  (cl:format cl:nil "uint64[] requested_ids           # The IDs of the tracks you are interested in getting the trajectories of. An empty array means all available tracks.~%duration max_age                 # The maximum age of a trajectory you want to get. A duration of 0 means \"since the beginning of times.\"~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPersonTrajectories-request)))
  "Returns full string definition for message of type 'GetPersonTrajectories-request"
  (cl:format cl:nil "uint64[] requested_ids           # The IDs of the tracks you are interested in getting the trajectories of. An empty array means all available tracks.~%duration max_age                 # The maximum age of a trajectory you want to get. A duration of 0 means \"since the beginning of times.\"~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPersonTrajectories-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'requested_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPersonTrajectories-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPersonTrajectories-request
    (cl:cons ':requested_ids (requested_ids msg))
    (cl:cons ':max_age (max_age msg))
))
;//! \htmlinclude GetPersonTrajectories-response.msg.html

(cl:defclass <GetPersonTrajectories-response> (roslisp-msg-protocol:ros-message)
  ((trajectories
    :reader trajectories
    :initarg :trajectories
    :type (cl:vector spencer_tracking_msgs-msg:PersonTrajectory)
   :initform (cl:make-array 0 :element-type 'spencer_tracking_msgs-msg:PersonTrajectory :initial-element (cl:make-instance 'spencer_tracking_msgs-msg:PersonTrajectory))))
)

(cl:defclass GetPersonTrajectories-response (<GetPersonTrajectories-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPersonTrajectories-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPersonTrajectories-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_tracking_msgs-srv:<GetPersonTrajectories-response> is deprecated: use spencer_tracking_msgs-srv:GetPersonTrajectories-response instead.")))

(cl:ensure-generic-function 'trajectories-val :lambda-list '(m))
(cl:defmethod trajectories-val ((m <GetPersonTrajectories-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-srv:trajectories-val is deprecated.  Use spencer_tracking_msgs-srv:trajectories instead.")
  (trajectories m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPersonTrajectories-response>) ostream)
  "Serializes a message object of type '<GetPersonTrajectories-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'trajectories))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'trajectories))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPersonTrajectories-response>) istream)
  "Deserializes a message object of type '<GetPersonTrajectories-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'trajectories) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'trajectories)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'spencer_tracking_msgs-msg:PersonTrajectory))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPersonTrajectories-response>)))
  "Returns string type for a service object of type '<GetPersonTrajectories-response>"
  "spencer_tracking_msgs/GetPersonTrajectoriesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPersonTrajectories-response)))
  "Returns string type for a service object of type 'GetPersonTrajectories-response"
  "spencer_tracking_msgs/GetPersonTrajectoriesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPersonTrajectories-response>)))
  "Returns md5sum for a message object of type '<GetPersonTrajectories-response>"
  "e0e3d63b99808526b112847474001abe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPersonTrajectories-response)))
  "Returns md5sum for a message object of type 'GetPersonTrajectories-response"
  "e0e3d63b99808526b112847474001abe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPersonTrajectories-response>)))
  "Returns full string definition for message of type '<GetPersonTrajectories-response>"
  (cl:format cl:nil "PersonTrajectory[] trajectories  # The trajectories of the tracks that have been asked for in requested_ids, in the same order.~%~%~%================================================================================~%MSG: spencer_tracking_msgs/PersonTrajectory~%# Message defining the trajectory of a tracked person.~%#~%# The distinction between track and trajectory is that, depending on the~%# implementation of the tracker, a single track (i.e. tracked person) might~%# change the trajectory if at some point a new trajectory \"fits\" that track (person)~%# better.~%#~%~%uint64                   track_id   # Unique identifier of the tracked person.~%PersonTrajectoryEntry[]  trajectory # All states of the last T frames of the most likely trajectory of that tracked person.~%~%================================================================================~%MSG: spencer_tracking_msgs/PersonTrajectoryEntry~%# Message defining an entry of a person trajectory.~%#~%~%time     stamp           # age of the track~%bool     is_occluded     # if the track is currently not matched by a detection~%uint64   detection_id    # id of the corresponding detection in the current cycle (undefined if occluded)~%~%# The following fields are extracted from the Kalman state x and its covariance C~%~%geometry_msgs/PoseWithCovariance    pose   # pose of the track (z value and orientation might not be set, check if corresponding variance on diagonal is > 99999)~%geometry_msgs/TwistWithCovariance   twist  # velocity of the track (z value and rotational velocities might not be set, check if corresponding variance on diagonal is > 99999)~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPersonTrajectories-response)))
  "Returns full string definition for message of type 'GetPersonTrajectories-response"
  (cl:format cl:nil "PersonTrajectory[] trajectories  # The trajectories of the tracks that have been asked for in requested_ids, in the same order.~%~%~%================================================================================~%MSG: spencer_tracking_msgs/PersonTrajectory~%# Message defining the trajectory of a tracked person.~%#~%# The distinction between track and trajectory is that, depending on the~%# implementation of the tracker, a single track (i.e. tracked person) might~%# change the trajectory if at some point a new trajectory \"fits\" that track (person)~%# better.~%#~%~%uint64                   track_id   # Unique identifier of the tracked person.~%PersonTrajectoryEntry[]  trajectory # All states of the last T frames of the most likely trajectory of that tracked person.~%~%================================================================================~%MSG: spencer_tracking_msgs/PersonTrajectoryEntry~%# Message defining an entry of a person trajectory.~%#~%~%time     stamp           # age of the track~%bool     is_occluded     # if the track is currently not matched by a detection~%uint64   detection_id    # id of the corresponding detection in the current cycle (undefined if occluded)~%~%# The following fields are extracted from the Kalman state x and its covariance C~%~%geometry_msgs/PoseWithCovariance    pose   # pose of the track (z value and orientation might not be set, check if corresponding variance on diagonal is > 99999)~%geometry_msgs/TwistWithCovariance   twist  # velocity of the track (z value and rotational velocities might not be set, check if corresponding variance on diagonal is > 99999)~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPersonTrajectories-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'trajectories) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPersonTrajectories-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPersonTrajectories-response
    (cl:cons ':trajectories (trajectories msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetPersonTrajectories)))
  'GetPersonTrajectories-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetPersonTrajectories)))
  'GetPersonTrajectories-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPersonTrajectories)))
  "Returns string type for a service object of type '<GetPersonTrajectories>"
  "spencer_tracking_msgs/GetPersonTrajectories")