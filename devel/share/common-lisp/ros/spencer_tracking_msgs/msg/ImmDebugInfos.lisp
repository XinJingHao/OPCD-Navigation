; Auto-generated. Do not edit!


(cl:in-package spencer_tracking_msgs-msg)


;//! \htmlinclude ImmDebugInfos.msg.html

(cl:defclass <ImmDebugInfos> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (infos
    :reader infos
    :initarg :infos
    :type (cl:vector spencer_tracking_msgs-msg:ImmDebugInfo)
   :initform (cl:make-array 0 :element-type 'spencer_tracking_msgs-msg:ImmDebugInfo :initial-element (cl:make-instance 'spencer_tracking_msgs-msg:ImmDebugInfo))))
)

(cl:defclass ImmDebugInfos (<ImmDebugInfos>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImmDebugInfos>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImmDebugInfos)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_tracking_msgs-msg:<ImmDebugInfos> is deprecated: use spencer_tracking_msgs-msg:ImmDebugInfos instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ImmDebugInfos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:header-val is deprecated.  Use spencer_tracking_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'infos-val :lambda-list '(m))
(cl:defmethod infos-val ((m <ImmDebugInfos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:infos-val is deprecated.  Use spencer_tracking_msgs-msg:infos instead.")
  (infos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImmDebugInfos>) ostream)
  "Serializes a message object of type '<ImmDebugInfos>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'infos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'infos))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImmDebugInfos>) istream)
  "Deserializes a message object of type '<ImmDebugInfos>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'infos) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'infos)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'spencer_tracking_msgs-msg:ImmDebugInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImmDebugInfos>)))
  "Returns string type for a message object of type '<ImmDebugInfos>"
  "spencer_tracking_msgs/ImmDebugInfos")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImmDebugInfos)))
  "Returns string type for a message object of type 'ImmDebugInfos"
  "spencer_tracking_msgs/ImmDebugInfos")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImmDebugInfos>)))
  "Returns md5sum for a message object of type '<ImmDebugInfos>"
  "ce7fa675b582455db7386ac3eaa36b5b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImmDebugInfos)))
  "Returns md5sum for a message object of type 'ImmDebugInfos"
  "ce7fa675b582455db7386ac3eaa36b5b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImmDebugInfos>)))
  "Returns full string definition for message of type '<ImmDebugInfos>"
  (cl:format cl:nil "# Message with all debug infos per frame~%#~%~%Header              header      # Header containing timestamp etc. of this message~%ImmDebugInfo[]      infos      # All persons that are currently being tracked~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: spencer_tracking_msgs/ImmDebugInfo~%# Message for passing debug information of filter performance~%#~%~%uint64      track_id        # unique identifier of the target, consistent over time~%float64      innovation      # innovation of prediction and associated observation~%float64      CpXX            # variance of prediction acc. to x~%float64      CpYY            # variance of prediction acc. to y~%float64      CXX             # variance of state acc. to x~%float64      CYY             # variance of state acc. to y~%float64[]    modeProbabilities# array containing mode probabilities~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImmDebugInfos)))
  "Returns full string definition for message of type 'ImmDebugInfos"
  (cl:format cl:nil "# Message with all debug infos per frame~%#~%~%Header              header      # Header containing timestamp etc. of this message~%ImmDebugInfo[]      infos      # All persons that are currently being tracked~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: spencer_tracking_msgs/ImmDebugInfo~%# Message for passing debug information of filter performance~%#~%~%uint64      track_id        # unique identifier of the target, consistent over time~%float64      innovation      # innovation of prediction and associated observation~%float64      CpXX            # variance of prediction acc. to x~%float64      CpYY            # variance of prediction acc. to y~%float64      CXX             # variance of state acc. to x~%float64      CYY             # variance of state acc. to y~%float64[]    modeProbabilities# array containing mode probabilities~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImmDebugInfos>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'infos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImmDebugInfos>))
  "Converts a ROS message object to a list"
  (cl:list 'ImmDebugInfos
    (cl:cons ':header (header msg))
    (cl:cons ':infos (infos msg))
))
