; Auto-generated. Do not edit!


(cl:in-package spencer_tracking_msgs-msg)


;//! \htmlinclude DetectedPerson.msg.html

(cl:defclass <DetectedPerson> (roslisp-msg-protocol:ros-message)
  ((detection_id
    :reader detection_id
    :initarg :detection_id
    :type cl:integer
    :initform 0)
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:PoseWithCovariance
    :initform (cl:make-instance 'geometry_msgs-msg:PoseWithCovariance))
   (modality
    :reader modality
    :initarg :modality
    :type cl:string
    :initform ""))
)

(cl:defclass DetectedPerson (<DetectedPerson>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectedPerson>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectedPerson)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_tracking_msgs-msg:<DetectedPerson> is deprecated: use spencer_tracking_msgs-msg:DetectedPerson instead.")))

(cl:ensure-generic-function 'detection_id-val :lambda-list '(m))
(cl:defmethod detection_id-val ((m <DetectedPerson>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:detection_id-val is deprecated.  Use spencer_tracking_msgs-msg:detection_id instead.")
  (detection_id m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <DetectedPerson>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:confidence-val is deprecated.  Use spencer_tracking_msgs-msg:confidence instead.")
  (confidence m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <DetectedPerson>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:pose-val is deprecated.  Use spencer_tracking_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'modality-val :lambda-list '(m))
(cl:defmethod modality-val ((m <DetectedPerson>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:modality-val is deprecated.  Use spencer_tracking_msgs-msg:modality instead.")
  (modality m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<DetectedPerson>)))
    "Constants for message type '<DetectedPerson>"
  '((:MODALITY_GENERIC_LASER_2D . "laser2d")
    (:MODALITY_GENERIC_LASER_3D . "laser3d")
    (:MODALITY_GENERIC_MONOCULAR_VISION . "mono")
    (:MODALITY_GENERIC_STEREO_VISION . "stereo")
    (:MODALITY_GENERIC_RGBD . "rgbd"))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'DetectedPerson)))
    "Constants for message type 'DetectedPerson"
  '((:MODALITY_GENERIC_LASER_2D . "laser2d")
    (:MODALITY_GENERIC_LASER_3D . "laser3d")
    (:MODALITY_GENERIC_MONOCULAR_VISION . "mono")
    (:MODALITY_GENERIC_STEREO_VISION . "stereo")
    (:MODALITY_GENERIC_RGBD . "rgbd"))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectedPerson>) ostream)
  "Serializes a message object of type '<DetectedPerson>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'detection_id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'modality))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'modality))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectedPerson>) istream)
  "Deserializes a message object of type '<DetectedPerson>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'modality) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'modality) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectedPerson>)))
  "Returns string type for a message object of type '<DetectedPerson>"
  "spencer_tracking_msgs/DetectedPerson")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectedPerson)))
  "Returns string type for a message object of type 'DetectedPerson"
  "spencer_tracking_msgs/DetectedPerson")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectedPerson>)))
  "Returns md5sum for a message object of type '<DetectedPerson>"
  "62855d424a3d5f142c0e8f5f63be52fe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectedPerson)))
  "Returns md5sum for a message object of type 'DetectedPerson"
  "62855d424a3d5f142c0e8f5f63be52fe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectedPerson>)))
  "Returns full string definition for message of type '<DetectedPerson>"
  (cl:format cl:nil "# Message describing a detection of a person~%#~%~%# Unique id of the detection, monotonically increasing over time~%uint64          detection_id~%~%# (Pseudo-)probabilistic value between 0.0 and 1.0 describing the detector's confidence in the detection~%float64         confidence~%~%# 3D pose (position + orientation) of the *center* of the detection~%# check covariance to see which dimensions are actually set! unset dimensions shall have a covariance > 9999~%geometry_msgs/PoseWithCovariance    pose    ~%~%# Sensor modality / detector type, see example constants below. ~%# used e.g. later in tracking to check which tracks have been visually confirmed~%string          modality            ~%~%                                    ~%string          MODALITY_GENERIC_LASER_2D = laser2d~%string          MODALITY_GENERIC_LASER_3D = laser3d~%string          MODALITY_GENERIC_MONOCULAR_VISION = mono~%string          MODALITY_GENERIC_STEREO_VISION = stereo~%string          MODALITY_GENERIC_RGBD = rgbd~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectedPerson)))
  "Returns full string definition for message of type 'DetectedPerson"
  (cl:format cl:nil "# Message describing a detection of a person~%#~%~%# Unique id of the detection, monotonically increasing over time~%uint64          detection_id~%~%# (Pseudo-)probabilistic value between 0.0 and 1.0 describing the detector's confidence in the detection~%float64         confidence~%~%# 3D pose (position + orientation) of the *center* of the detection~%# check covariance to see which dimensions are actually set! unset dimensions shall have a covariance > 9999~%geometry_msgs/PoseWithCovariance    pose    ~%~%# Sensor modality / detector type, see example constants below. ~%# used e.g. later in tracking to check which tracks have been visually confirmed~%string          modality            ~%~%                                    ~%string          MODALITY_GENERIC_LASER_2D = laser2d~%string          MODALITY_GENERIC_LASER_3D = laser3d~%string          MODALITY_GENERIC_MONOCULAR_VISION = mono~%string          MODALITY_GENERIC_STEREO_VISION = stereo~%string          MODALITY_GENERIC_RGBD = rgbd~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectedPerson>))
  (cl:+ 0
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     4 (cl:length (cl:slot-value msg 'modality))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectedPerson>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectedPerson
    (cl:cons ':detection_id (detection_id msg))
    (cl:cons ':confidence (confidence msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':modality (modality msg))
))
