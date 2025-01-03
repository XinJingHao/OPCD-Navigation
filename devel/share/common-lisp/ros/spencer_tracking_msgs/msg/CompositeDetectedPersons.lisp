; Auto-generated. Do not edit!


(cl:in-package spencer_tracking_msgs-msg)


;//! \htmlinclude CompositeDetectedPersons.msg.html

(cl:defclass <CompositeDetectedPersons> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (elements
    :reader elements
    :initarg :elements
    :type (cl:vector spencer_tracking_msgs-msg:CompositeDetectedPerson)
   :initform (cl:make-array 0 :element-type 'spencer_tracking_msgs-msg:CompositeDetectedPerson :initial-element (cl:make-instance 'spencer_tracking_msgs-msg:CompositeDetectedPerson))))
)

(cl:defclass CompositeDetectedPersons (<CompositeDetectedPersons>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CompositeDetectedPersons>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CompositeDetectedPersons)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_tracking_msgs-msg:<CompositeDetectedPersons> is deprecated: use spencer_tracking_msgs-msg:CompositeDetectedPersons instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CompositeDetectedPersons>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:header-val is deprecated.  Use spencer_tracking_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'elements-val :lambda-list '(m))
(cl:defmethod elements-val ((m <CompositeDetectedPersons>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:elements-val is deprecated.  Use spencer_tracking_msgs-msg:elements instead.")
  (elements m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CompositeDetectedPersons>) ostream)
  "Serializes a message object of type '<CompositeDetectedPersons>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'elements))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'elements))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CompositeDetectedPersons>) istream)
  "Deserializes a message object of type '<CompositeDetectedPersons>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'elements) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'elements)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'spencer_tracking_msgs-msg:CompositeDetectedPerson))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CompositeDetectedPersons>)))
  "Returns string type for a message object of type '<CompositeDetectedPersons>"
  "spencer_tracking_msgs/CompositeDetectedPersons")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CompositeDetectedPersons)))
  "Returns string type for a message object of type 'CompositeDetectedPersons"
  "spencer_tracking_msgs/CompositeDetectedPersons")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CompositeDetectedPersons>)))
  "Returns md5sum for a message object of type '<CompositeDetectedPersons>"
  "c4f9f34f5cb712f010de12fc027da3e6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CompositeDetectedPersons)))
  "Returns md5sum for a message object of type 'CompositeDetectedPersons"
  "c4f9f34f5cb712f010de12fc027da3e6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CompositeDetectedPersons>)))
  "Returns full string definition for message of type '<CompositeDetectedPersons>"
  (cl:format cl:nil "# Message specifying which original detected persons (from all kinds of sensors) have been merged into one fused detection before being processed by the person tracker, in the current time step.~%#~%# This information is processed by the spencer_detected_person_association module, such that other perception components can associate their results (e.g. person age, gender)~%# with a particular spencer_tracking_msgs/TrackedPerson (which contains a reference to a composite person detection ID).~%~%Header                      header          # Header timestamp is set to the *latest* timestamp of all fused DetectedPerson messages.~%                                            # Before fusion, all detections are transformed into a common coordinate frame (usually base_footprint).~%CompositeDetectedPerson[]   elements        # Fused (merged) detected persons~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: spencer_tracking_msgs/CompositeDetectedPerson~%# Specifies which detected persons have been merged into a composite detection by the spencer_detected_person_association module.~%~%# TODO: Do we need a composite person-specific timestamp (or even a full message header including frame ID)?~%# Having a separate timestamp per person could be useful if the timestamps of the merged DetectedPersons messages vary a lot,~%# and some persons are only seen by a single sensor (so averaging over all input timestamps would have a detrimental effect). ~%~%uint64      composite_detection_id          # ID of the fused detection~%~%float64     mean_confidence                 # mean of the confidences of the original detections~%float64     max_confidence                  # maximum confidence of original detections~%float64     min_confidence                  # minimum confidence of original detections~%~%~%geometry_msgs/PoseWithCovariance    pose    # Merged 3D pose (position + orientation) of the detection center~%                                            # check covariance to see which dimensions are actually set!~%                                            # unset dimensions shall have a covariance > 9999~%~%DetectedPerson[] original_detections        # The original detections from individual sensor-specific detectors that have been combined into a composite detection~%                                            # We are copying the entire DetectedPersons messages, *with poses transformed into the target frame*, such that subscribers~%                                            # do not have to subscribe to all the original DetectedPersons topics.~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: spencer_tracking_msgs/DetectedPerson~%# Message describing a detection of a person~%#~%~%# Unique id of the detection, monotonically increasing over time~%uint64          detection_id~%~%# (Pseudo-)probabilistic value between 0.0 and 1.0 describing the detector's confidence in the detection~%float64         confidence~%~%# 3D pose (position + orientation) of the *center* of the detection~%# check covariance to see which dimensions are actually set! unset dimensions shall have a covariance > 9999~%geometry_msgs/PoseWithCovariance    pose    ~%~%# Sensor modality / detector type, see example constants below. ~%# used e.g. later in tracking to check which tracks have been visually confirmed~%string          modality            ~%~%                                    ~%string          MODALITY_GENERIC_LASER_2D = laser2d~%string          MODALITY_GENERIC_LASER_3D = laser3d~%string          MODALITY_GENERIC_MONOCULAR_VISION = mono~%string          MODALITY_GENERIC_STEREO_VISION = stereo~%string          MODALITY_GENERIC_RGBD = rgbd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CompositeDetectedPersons)))
  "Returns full string definition for message of type 'CompositeDetectedPersons"
  (cl:format cl:nil "# Message specifying which original detected persons (from all kinds of sensors) have been merged into one fused detection before being processed by the person tracker, in the current time step.~%#~%# This information is processed by the spencer_detected_person_association module, such that other perception components can associate their results (e.g. person age, gender)~%# with a particular spencer_tracking_msgs/TrackedPerson (which contains a reference to a composite person detection ID).~%~%Header                      header          # Header timestamp is set to the *latest* timestamp of all fused DetectedPerson messages.~%                                            # Before fusion, all detections are transformed into a common coordinate frame (usually base_footprint).~%CompositeDetectedPerson[]   elements        # Fused (merged) detected persons~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: spencer_tracking_msgs/CompositeDetectedPerson~%# Specifies which detected persons have been merged into a composite detection by the spencer_detected_person_association module.~%~%# TODO: Do we need a composite person-specific timestamp (or even a full message header including frame ID)?~%# Having a separate timestamp per person could be useful if the timestamps of the merged DetectedPersons messages vary a lot,~%# and some persons are only seen by a single sensor (so averaging over all input timestamps would have a detrimental effect). ~%~%uint64      composite_detection_id          # ID of the fused detection~%~%float64     mean_confidence                 # mean of the confidences of the original detections~%float64     max_confidence                  # maximum confidence of original detections~%float64     min_confidence                  # minimum confidence of original detections~%~%~%geometry_msgs/PoseWithCovariance    pose    # Merged 3D pose (position + orientation) of the detection center~%                                            # check covariance to see which dimensions are actually set!~%                                            # unset dimensions shall have a covariance > 9999~%~%DetectedPerson[] original_detections        # The original detections from individual sensor-specific detectors that have been combined into a composite detection~%                                            # We are copying the entire DetectedPersons messages, *with poses transformed into the target frame*, such that subscribers~%                                            # do not have to subscribe to all the original DetectedPersons topics.~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: spencer_tracking_msgs/DetectedPerson~%# Message describing a detection of a person~%#~%~%# Unique id of the detection, monotonically increasing over time~%uint64          detection_id~%~%# (Pseudo-)probabilistic value between 0.0 and 1.0 describing the detector's confidence in the detection~%float64         confidence~%~%# 3D pose (position + orientation) of the *center* of the detection~%# check covariance to see which dimensions are actually set! unset dimensions shall have a covariance > 9999~%geometry_msgs/PoseWithCovariance    pose    ~%~%# Sensor modality / detector type, see example constants below. ~%# used e.g. later in tracking to check which tracks have been visually confirmed~%string          modality            ~%~%                                    ~%string          MODALITY_GENERIC_LASER_2D = laser2d~%string          MODALITY_GENERIC_LASER_3D = laser3d~%string          MODALITY_GENERIC_MONOCULAR_VISION = mono~%string          MODALITY_GENERIC_STEREO_VISION = stereo~%string          MODALITY_GENERIC_RGBD = rgbd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CompositeDetectedPersons>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'elements) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CompositeDetectedPersons>))
  "Converts a ROS message object to a list"
  (cl:list 'CompositeDetectedPersons
    (cl:cons ':header (header msg))
    (cl:cons ':elements (elements msg))
))
