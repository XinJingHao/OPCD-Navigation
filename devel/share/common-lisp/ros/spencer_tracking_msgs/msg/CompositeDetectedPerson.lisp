; Auto-generated. Do not edit!


(cl:in-package spencer_tracking_msgs-msg)


;//! \htmlinclude CompositeDetectedPerson.msg.html

(cl:defclass <CompositeDetectedPerson> (roslisp-msg-protocol:ros-message)
  ((composite_detection_id
    :reader composite_detection_id
    :initarg :composite_detection_id
    :type cl:integer
    :initform 0)
   (mean_confidence
    :reader mean_confidence
    :initarg :mean_confidence
    :type cl:float
    :initform 0.0)
   (max_confidence
    :reader max_confidence
    :initarg :max_confidence
    :type cl:float
    :initform 0.0)
   (min_confidence
    :reader min_confidence
    :initarg :min_confidence
    :type cl:float
    :initform 0.0)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:PoseWithCovariance
    :initform (cl:make-instance 'geometry_msgs-msg:PoseWithCovariance))
   (original_detections
    :reader original_detections
    :initarg :original_detections
    :type (cl:vector spencer_tracking_msgs-msg:DetectedPerson)
   :initform (cl:make-array 0 :element-type 'spencer_tracking_msgs-msg:DetectedPerson :initial-element (cl:make-instance 'spencer_tracking_msgs-msg:DetectedPerson))))
)

(cl:defclass CompositeDetectedPerson (<CompositeDetectedPerson>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CompositeDetectedPerson>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CompositeDetectedPerson)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_tracking_msgs-msg:<CompositeDetectedPerson> is deprecated: use spencer_tracking_msgs-msg:CompositeDetectedPerson instead.")))

(cl:ensure-generic-function 'composite_detection_id-val :lambda-list '(m))
(cl:defmethod composite_detection_id-val ((m <CompositeDetectedPerson>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:composite_detection_id-val is deprecated.  Use spencer_tracking_msgs-msg:composite_detection_id instead.")
  (composite_detection_id m))

(cl:ensure-generic-function 'mean_confidence-val :lambda-list '(m))
(cl:defmethod mean_confidence-val ((m <CompositeDetectedPerson>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:mean_confidence-val is deprecated.  Use spencer_tracking_msgs-msg:mean_confidence instead.")
  (mean_confidence m))

(cl:ensure-generic-function 'max_confidence-val :lambda-list '(m))
(cl:defmethod max_confidence-val ((m <CompositeDetectedPerson>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:max_confidence-val is deprecated.  Use spencer_tracking_msgs-msg:max_confidence instead.")
  (max_confidence m))

(cl:ensure-generic-function 'min_confidence-val :lambda-list '(m))
(cl:defmethod min_confidence-val ((m <CompositeDetectedPerson>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:min_confidence-val is deprecated.  Use spencer_tracking_msgs-msg:min_confidence instead.")
  (min_confidence m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <CompositeDetectedPerson>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:pose-val is deprecated.  Use spencer_tracking_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'original_detections-val :lambda-list '(m))
(cl:defmethod original_detections-val ((m <CompositeDetectedPerson>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:original_detections-val is deprecated.  Use spencer_tracking_msgs-msg:original_detections instead.")
  (original_detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CompositeDetectedPerson>) ostream)
  "Serializes a message object of type '<CompositeDetectedPerson>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'composite_detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'composite_detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'composite_detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'composite_detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'composite_detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'composite_detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'composite_detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'composite_detection_id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'mean_confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max_confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'min_confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'original_detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'original_detections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CompositeDetectedPerson>) istream)
  "Deserializes a message object of type '<CompositeDetectedPerson>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'composite_detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'composite_detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'composite_detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'composite_detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'composite_detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'composite_detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'composite_detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'composite_detection_id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mean_confidence) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_confidence) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_confidence) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'original_detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'original_detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'spencer_tracking_msgs-msg:DetectedPerson))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CompositeDetectedPerson>)))
  "Returns string type for a message object of type '<CompositeDetectedPerson>"
  "spencer_tracking_msgs/CompositeDetectedPerson")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CompositeDetectedPerson)))
  "Returns string type for a message object of type 'CompositeDetectedPerson"
  "spencer_tracking_msgs/CompositeDetectedPerson")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CompositeDetectedPerson>)))
  "Returns md5sum for a message object of type '<CompositeDetectedPerson>"
  "10e83f06a9bfbf6da1ae6f0fcdbe2cc4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CompositeDetectedPerson)))
  "Returns md5sum for a message object of type 'CompositeDetectedPerson"
  "10e83f06a9bfbf6da1ae6f0fcdbe2cc4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CompositeDetectedPerson>)))
  "Returns full string definition for message of type '<CompositeDetectedPerson>"
  (cl:format cl:nil "# Specifies which detected persons have been merged into a composite detection by the spencer_detected_person_association module.~%~%# TODO: Do we need a composite person-specific timestamp (or even a full message header including frame ID)?~%# Having a separate timestamp per person could be useful if the timestamps of the merged DetectedPersons messages vary a lot,~%# and some persons are only seen by a single sensor (so averaging over all input timestamps would have a detrimental effect). ~%~%uint64      composite_detection_id          # ID of the fused detection~%~%float64     mean_confidence                 # mean of the confidences of the original detections~%float64     max_confidence                  # maximum confidence of original detections~%float64     min_confidence                  # minimum confidence of original detections~%~%~%geometry_msgs/PoseWithCovariance    pose    # Merged 3D pose (position + orientation) of the detection center~%                                            # check covariance to see which dimensions are actually set!~%                                            # unset dimensions shall have a covariance > 9999~%~%DetectedPerson[] original_detections        # The original detections from individual sensor-specific detectors that have been combined into a composite detection~%                                            # We are copying the entire DetectedPersons messages, *with poses transformed into the target frame*, such that subscribers~%                                            # do not have to subscribe to all the original DetectedPersons topics.~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: spencer_tracking_msgs/DetectedPerson~%# Message describing a detection of a person~%#~%~%# Unique id of the detection, monotonically increasing over time~%uint64          detection_id~%~%# (Pseudo-)probabilistic value between 0.0 and 1.0 describing the detector's confidence in the detection~%float64         confidence~%~%# 3D pose (position + orientation) of the *center* of the detection~%# check covariance to see which dimensions are actually set! unset dimensions shall have a covariance > 9999~%geometry_msgs/PoseWithCovariance    pose    ~%~%# Sensor modality / detector type, see example constants below. ~%# used e.g. later in tracking to check which tracks have been visually confirmed~%string          modality            ~%~%                                    ~%string          MODALITY_GENERIC_LASER_2D = laser2d~%string          MODALITY_GENERIC_LASER_3D = laser3d~%string          MODALITY_GENERIC_MONOCULAR_VISION = mono~%string          MODALITY_GENERIC_STEREO_VISION = stereo~%string          MODALITY_GENERIC_RGBD = rgbd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CompositeDetectedPerson)))
  "Returns full string definition for message of type 'CompositeDetectedPerson"
  (cl:format cl:nil "# Specifies which detected persons have been merged into a composite detection by the spencer_detected_person_association module.~%~%# TODO: Do we need a composite person-specific timestamp (or even a full message header including frame ID)?~%# Having a separate timestamp per person could be useful if the timestamps of the merged DetectedPersons messages vary a lot,~%# and some persons are only seen by a single sensor (so averaging over all input timestamps would have a detrimental effect). ~%~%uint64      composite_detection_id          # ID of the fused detection~%~%float64     mean_confidence                 # mean of the confidences of the original detections~%float64     max_confidence                  # maximum confidence of original detections~%float64     min_confidence                  # minimum confidence of original detections~%~%~%geometry_msgs/PoseWithCovariance    pose    # Merged 3D pose (position + orientation) of the detection center~%                                            # check covariance to see which dimensions are actually set!~%                                            # unset dimensions shall have a covariance > 9999~%~%DetectedPerson[] original_detections        # The original detections from individual sensor-specific detectors that have been combined into a composite detection~%                                            # We are copying the entire DetectedPersons messages, *with poses transformed into the target frame*, such that subscribers~%                                            # do not have to subscribe to all the original DetectedPersons topics.~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: spencer_tracking_msgs/DetectedPerson~%# Message describing a detection of a person~%#~%~%# Unique id of the detection, monotonically increasing over time~%uint64          detection_id~%~%# (Pseudo-)probabilistic value between 0.0 and 1.0 describing the detector's confidence in the detection~%float64         confidence~%~%# 3D pose (position + orientation) of the *center* of the detection~%# check covariance to see which dimensions are actually set! unset dimensions shall have a covariance > 9999~%geometry_msgs/PoseWithCovariance    pose    ~%~%# Sensor modality / detector type, see example constants below. ~%# used e.g. later in tracking to check which tracks have been visually confirmed~%string          modality            ~%~%                                    ~%string          MODALITY_GENERIC_LASER_2D = laser2d~%string          MODALITY_GENERIC_LASER_3D = laser3d~%string          MODALITY_GENERIC_MONOCULAR_VISION = mono~%string          MODALITY_GENERIC_STEREO_VISION = stereo~%string          MODALITY_GENERIC_RGBD = rgbd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CompositeDetectedPerson>))
  (cl:+ 0
     8
     8
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'original_detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CompositeDetectedPerson>))
  "Converts a ROS message object to a list"
  (cl:list 'CompositeDetectedPerson
    (cl:cons ':composite_detection_id (composite_detection_id msg))
    (cl:cons ':mean_confidence (mean_confidence msg))
    (cl:cons ':max_confidence (max_confidence msg))
    (cl:cons ':min_confidence (min_confidence msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':original_detections (original_detections msg))
))
