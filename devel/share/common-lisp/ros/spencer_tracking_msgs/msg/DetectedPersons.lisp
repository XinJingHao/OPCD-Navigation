; Auto-generated. Do not edit!


(cl:in-package spencer_tracking_msgs-msg)


;//! \htmlinclude DetectedPersons.msg.html

(cl:defclass <DetectedPersons> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (detections
    :reader detections
    :initarg :detections
    :type (cl:vector spencer_tracking_msgs-msg:DetectedPerson)
   :initform (cl:make-array 0 :element-type 'spencer_tracking_msgs-msg:DetectedPerson :initial-element (cl:make-instance 'spencer_tracking_msgs-msg:DetectedPerson))))
)

(cl:defclass DetectedPersons (<DetectedPersons>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectedPersons>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectedPersons)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_tracking_msgs-msg:<DetectedPersons> is deprecated: use spencer_tracking_msgs-msg:DetectedPersons instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DetectedPersons>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:header-val is deprecated.  Use spencer_tracking_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'detections-val :lambda-list '(m))
(cl:defmethod detections-val ((m <DetectedPersons>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:detections-val is deprecated.  Use spencer_tracking_msgs-msg:detections instead.")
  (detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectedPersons>) ostream)
  "Serializes a message object of type '<DetectedPersons>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'detections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectedPersons>) istream)
  "Deserializes a message object of type '<DetectedPersons>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'spencer_tracking_msgs-msg:DetectedPerson))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectedPersons>)))
  "Returns string type for a message object of type '<DetectedPersons>"
  "spencer_tracking_msgs/DetectedPersons")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectedPersons)))
  "Returns string type for a message object of type 'DetectedPersons"
  "spencer_tracking_msgs/DetectedPersons")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectedPersons>)))
  "Returns md5sum for a message object of type '<DetectedPersons>"
  "38e416cebb5dfd7363bde91113e7f3bd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectedPersons)))
  "Returns md5sum for a message object of type 'DetectedPersons"
  "38e416cebb5dfd7363bde91113e7f3bd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectedPersons>)))
  "Returns full string definition for message of type '<DetectedPersons>"
  (cl:format cl:nil "# Message with all currently detected persons~%#~%~%Header                  header          # Header containing timestamp etc. of this message~%DetectedPerson[]        detections      # All persons that are currently being detected~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: spencer_tracking_msgs/DetectedPerson~%# Message describing a detection of a person~%#~%~%# Unique id of the detection, monotonically increasing over time~%uint64          detection_id~%~%# (Pseudo-)probabilistic value between 0.0 and 1.0 describing the detector's confidence in the detection~%float64         confidence~%~%# 3D pose (position + orientation) of the *center* of the detection~%# check covariance to see which dimensions are actually set! unset dimensions shall have a covariance > 9999~%geometry_msgs/PoseWithCovariance    pose    ~%~%# Sensor modality / detector type, see example constants below. ~%# used e.g. later in tracking to check which tracks have been visually confirmed~%string          modality            ~%~%                                    ~%string          MODALITY_GENERIC_LASER_2D = laser2d~%string          MODALITY_GENERIC_LASER_3D = laser3d~%string          MODALITY_GENERIC_MONOCULAR_VISION = mono~%string          MODALITY_GENERIC_STEREO_VISION = stereo~%string          MODALITY_GENERIC_RGBD = rgbd~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectedPersons)))
  "Returns full string definition for message of type 'DetectedPersons"
  (cl:format cl:nil "# Message with all currently detected persons~%#~%~%Header                  header          # Header containing timestamp etc. of this message~%DetectedPerson[]        detections      # All persons that are currently being detected~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: spencer_tracking_msgs/DetectedPerson~%# Message describing a detection of a person~%#~%~%# Unique id of the detection, monotonically increasing over time~%uint64          detection_id~%~%# (Pseudo-)probabilistic value between 0.0 and 1.0 describing the detector's confidence in the detection~%float64         confidence~%~%# 3D pose (position + orientation) of the *center* of the detection~%# check covariance to see which dimensions are actually set! unset dimensions shall have a covariance > 9999~%geometry_msgs/PoseWithCovariance    pose    ~%~%# Sensor modality / detector type, see example constants below. ~%# used e.g. later in tracking to check which tracks have been visually confirmed~%string          modality            ~%~%                                    ~%string          MODALITY_GENERIC_LASER_2D = laser2d~%string          MODALITY_GENERIC_LASER_3D = laser3d~%string          MODALITY_GENERIC_MONOCULAR_VISION = mono~%string          MODALITY_GENERIC_STEREO_VISION = stereo~%string          MODALITY_GENERIC_RGBD = rgbd~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectedPersons>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectedPersons>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectedPersons
    (cl:cons ':header (header msg))
    (cl:cons ':detections (detections msg))
))
