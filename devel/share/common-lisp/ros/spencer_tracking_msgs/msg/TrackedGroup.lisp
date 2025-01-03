; Auto-generated. Do not edit!


(cl:in-package spencer_tracking_msgs-msg)


;//! \htmlinclude TrackedGroup.msg.html

(cl:defclass <TrackedGroup> (roslisp-msg-protocol:ros-message)
  ((group_id
    :reader group_id
    :initarg :group_id
    :type cl:integer
    :initform 0)
   (age
    :reader age
    :initarg :age
    :type cl:real
    :initform 0)
   (centerOfGravity
    :reader centerOfGravity
    :initarg :centerOfGravity
    :type geometry_msgs-msg:PoseWithCovariance
    :initform (cl:make-instance 'geometry_msgs-msg:PoseWithCovariance))
   (track_ids
    :reader track_ids
    :initarg :track_ids
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass TrackedGroup (<TrackedGroup>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackedGroup>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackedGroup)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_tracking_msgs-msg:<TrackedGroup> is deprecated: use spencer_tracking_msgs-msg:TrackedGroup instead.")))

(cl:ensure-generic-function 'group_id-val :lambda-list '(m))
(cl:defmethod group_id-val ((m <TrackedGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:group_id-val is deprecated.  Use spencer_tracking_msgs-msg:group_id instead.")
  (group_id m))

(cl:ensure-generic-function 'age-val :lambda-list '(m))
(cl:defmethod age-val ((m <TrackedGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:age-val is deprecated.  Use spencer_tracking_msgs-msg:age instead.")
  (age m))

(cl:ensure-generic-function 'centerOfGravity-val :lambda-list '(m))
(cl:defmethod centerOfGravity-val ((m <TrackedGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:centerOfGravity-val is deprecated.  Use spencer_tracking_msgs-msg:centerOfGravity instead.")
  (centerOfGravity m))

(cl:ensure-generic-function 'track_ids-val :lambda-list '(m))
(cl:defmethod track_ids-val ((m <TrackedGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:track_ids-val is deprecated.  Use spencer_tracking_msgs-msg:track_ids instead.")
  (track_ids m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackedGroup>) ostream)
  "Serializes a message object of type '<TrackedGroup>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'group_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'group_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'group_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'group_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'group_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'group_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'group_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'group_id)) ostream)
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'age)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'age) (cl:floor (cl:slot-value msg 'age)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'centerOfGravity) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'track_ids))))
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
   (cl:slot-value msg 'track_ids))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackedGroup>) istream)
  "Deserializes a message object of type '<TrackedGroup>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'group_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'group_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'group_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'group_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'group_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'group_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'group_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'group_id)) (cl:read-byte istream))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'age) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'centerOfGravity) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'track_ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'track_ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackedGroup>)))
  "Returns string type for a message object of type '<TrackedGroup>"
  "spencer_tracking_msgs/TrackedGroup")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackedGroup)))
  "Returns string type for a message object of type 'TrackedGroup"
  "spencer_tracking_msgs/TrackedGroup")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackedGroup>)))
  "Returns md5sum for a message object of type '<TrackedGroup>"
  "6a5318bfb8e49948a4dc15c1267f7e54")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackedGroup)))
  "Returns md5sum for a message object of type 'TrackedGroup"
  "6a5318bfb8e49948a4dc15c1267f7e54")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackedGroup>)))
  "Returns full string definition for message of type '<TrackedGroup>"
  (cl:format cl:nil "# Message defining a tracked group~%#~%~%uint64      group_id        # unique identifier of the target, consistent over time~%~%duration    age             # age of the group~%~%geometry_msgs/PoseWithCovariance    centerOfGravity   # mean and covariance of the group (calculated from its person tracks)~%~%uint64[]    track_ids       # IDs of the tracked persons in this group. See srl_tracking_msgs/TrackedPersons~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackedGroup)))
  "Returns full string definition for message of type 'TrackedGroup"
  (cl:format cl:nil "# Message defining a tracked group~%#~%~%uint64      group_id        # unique identifier of the target, consistent over time~%~%duration    age             # age of the group~%~%geometry_msgs/PoseWithCovariance    centerOfGravity   # mean and covariance of the group (calculated from its person tracks)~%~%uint64[]    track_ids       # IDs of the tracked persons in this group. See srl_tracking_msgs/TrackedPersons~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackedGroup>))
  (cl:+ 0
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'centerOfGravity))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'track_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackedGroup>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackedGroup
    (cl:cons ':group_id (group_id msg))
    (cl:cons ':age (age msg))
    (cl:cons ':centerOfGravity (centerOfGravity msg))
    (cl:cons ':track_ids (track_ids msg))
))
