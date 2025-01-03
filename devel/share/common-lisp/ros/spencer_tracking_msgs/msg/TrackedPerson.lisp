; Auto-generated. Do not edit!


(cl:in-package spencer_tracking_msgs-msg)


;//! \htmlinclude TrackedPerson.msg.html

(cl:defclass <TrackedPerson> (roslisp-msg-protocol:ros-message)
  ((track_id
    :reader track_id
    :initarg :track_id
    :type cl:integer
    :initform 0)
   (is_occluded
    :reader is_occluded
    :initarg :is_occluded
    :type cl:boolean
    :initform cl:nil)
   (is_matched
    :reader is_matched
    :initarg :is_matched
    :type cl:boolean
    :initform cl:nil)
   (detection_id
    :reader detection_id
    :initarg :detection_id
    :type cl:integer
    :initform 0)
   (age
    :reader age
    :initarg :age
    :type cl:real
    :initform 0)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:PoseWithCovariance
    :initform (cl:make-instance 'geometry_msgs-msg:PoseWithCovariance))
   (twist
    :reader twist
    :initarg :twist
    :type geometry_msgs-msg:TwistWithCovariance
    :initform (cl:make-instance 'geometry_msgs-msg:TwistWithCovariance)))
)

(cl:defclass TrackedPerson (<TrackedPerson>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackedPerson>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackedPerson)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_tracking_msgs-msg:<TrackedPerson> is deprecated: use spencer_tracking_msgs-msg:TrackedPerson instead.")))

(cl:ensure-generic-function 'track_id-val :lambda-list '(m))
(cl:defmethod track_id-val ((m <TrackedPerson>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:track_id-val is deprecated.  Use spencer_tracking_msgs-msg:track_id instead.")
  (track_id m))

(cl:ensure-generic-function 'is_occluded-val :lambda-list '(m))
(cl:defmethod is_occluded-val ((m <TrackedPerson>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:is_occluded-val is deprecated.  Use spencer_tracking_msgs-msg:is_occluded instead.")
  (is_occluded m))

(cl:ensure-generic-function 'is_matched-val :lambda-list '(m))
(cl:defmethod is_matched-val ((m <TrackedPerson>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:is_matched-val is deprecated.  Use spencer_tracking_msgs-msg:is_matched instead.")
  (is_matched m))

(cl:ensure-generic-function 'detection_id-val :lambda-list '(m))
(cl:defmethod detection_id-val ((m <TrackedPerson>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:detection_id-val is deprecated.  Use spencer_tracking_msgs-msg:detection_id instead.")
  (detection_id m))

(cl:ensure-generic-function 'age-val :lambda-list '(m))
(cl:defmethod age-val ((m <TrackedPerson>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:age-val is deprecated.  Use spencer_tracking_msgs-msg:age instead.")
  (age m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <TrackedPerson>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:pose-val is deprecated.  Use spencer_tracking_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'twist-val :lambda-list '(m))
(cl:defmethod twist-val ((m <TrackedPerson>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:twist-val is deprecated.  Use spencer_tracking_msgs-msg:twist instead.")
  (twist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackedPerson>) ostream)
  "Serializes a message object of type '<TrackedPerson>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_occluded) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_matched) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'detection_id)) ostream)
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
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'twist) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackedPerson>) istream)
  "Deserializes a message object of type '<TrackedPerson>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'track_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'track_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'track_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'track_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'track_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'track_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'track_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'track_id)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'is_occluded) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_matched) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
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
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'twist) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackedPerson>)))
  "Returns string type for a message object of type '<TrackedPerson>"
  "spencer_tracking_msgs/TrackedPerson")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackedPerson)))
  "Returns string type for a message object of type 'TrackedPerson"
  "spencer_tracking_msgs/TrackedPerson")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackedPerson>)))
  "Returns md5sum for a message object of type '<TrackedPerson>"
  "28bdd0d6d6551c668e4fde8aecdf1885")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackedPerson)))
  "Returns md5sum for a message object of type 'TrackedPerson"
  "28bdd0d6d6551c668e4fde8aecdf1885")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackedPerson>)))
  "Returns full string definition for message of type '<TrackedPerson>"
  (cl:format cl:nil "# Message defining a tracked person~%#~%~%uint64      track_id        # unique identifier of the target, consistent over time~%bool        is_occluded     # if the track is currently not observable in a physical way~%bool        is_matched      # if the track is currently matched by a detection~%uint64      detection_id    # id of the corresponding detection in the current cycle (undefined if occluded)~%duration    age             # age of the track~%~%# The following fields are extracted from the Kalman state x and its covariance C~%~%geometry_msgs/PoseWithCovariance       pose   # pose of the track (z value and orientation might not be set, check if corresponding variance on diagonal is > 99999)~%~%geometry_msgs/TwistWithCovariance   twist     # velocity of the track (z value and rotational velocities might not be set, check if corresponding variance on diagonal is > 99999)~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackedPerson)))
  "Returns full string definition for message of type 'TrackedPerson"
  (cl:format cl:nil "# Message defining a tracked person~%#~%~%uint64      track_id        # unique identifier of the target, consistent over time~%bool        is_occluded     # if the track is currently not observable in a physical way~%bool        is_matched      # if the track is currently matched by a detection~%uint64      detection_id    # id of the corresponding detection in the current cycle (undefined if occluded)~%duration    age             # age of the track~%~%# The following fields are extracted from the Kalman state x and its covariance C~%~%geometry_msgs/PoseWithCovariance       pose   # pose of the track (z value and orientation might not be set, check if corresponding variance on diagonal is > 99999)~%~%geometry_msgs/TwistWithCovariance   twist     # velocity of the track (z value and rotational velocities might not be set, check if corresponding variance on diagonal is > 99999)~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackedPerson>))
  (cl:+ 0
     8
     1
     1
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'twist))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackedPerson>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackedPerson
    (cl:cons ':track_id (track_id msg))
    (cl:cons ':is_occluded (is_occluded msg))
    (cl:cons ':is_matched (is_matched msg))
    (cl:cons ':detection_id (detection_id msg))
    (cl:cons ':age (age msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':twist (twist msg))
))
