; Auto-generated. Do not edit!


(cl:in-package spencer_tracking_msgs-msg)


;//! \htmlinclude TrackedPersons2d.msg.html

(cl:defclass <TrackedPersons2d> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (boxes
    :reader boxes
    :initarg :boxes
    :type (cl:vector spencer_tracking_msgs-msg:TrackedPerson2d)
   :initform (cl:make-array 0 :element-type 'spencer_tracking_msgs-msg:TrackedPerson2d :initial-element (cl:make-instance 'spencer_tracking_msgs-msg:TrackedPerson2d))))
)

(cl:defclass TrackedPersons2d (<TrackedPersons2d>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackedPersons2d>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackedPersons2d)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_tracking_msgs-msg:<TrackedPersons2d> is deprecated: use spencer_tracking_msgs-msg:TrackedPersons2d instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TrackedPersons2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:header-val is deprecated.  Use spencer_tracking_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'boxes-val :lambda-list '(m))
(cl:defmethod boxes-val ((m <TrackedPersons2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:boxes-val is deprecated.  Use spencer_tracking_msgs-msg:boxes instead.")
  (boxes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackedPersons2d>) ostream)
  "Serializes a message object of type '<TrackedPersons2d>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'boxes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'boxes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackedPersons2d>) istream)
  "Deserializes a message object of type '<TrackedPersons2d>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'boxes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'boxes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'spencer_tracking_msgs-msg:TrackedPerson2d))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackedPersons2d>)))
  "Returns string type for a message object of type '<TrackedPersons2d>"
  "spencer_tracking_msgs/TrackedPersons2d")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackedPersons2d)))
  "Returns string type for a message object of type 'TrackedPersons2d"
  "spencer_tracking_msgs/TrackedPersons2d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackedPersons2d>)))
  "Returns md5sum for a message object of type '<TrackedPersons2d>"
  "972b7d693ce31c3b18f092f43387621c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackedPersons2d)))
  "Returns md5sum for a message object of type 'TrackedPersons2d"
  "972b7d693ce31c3b18f092f43387621c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackedPersons2d>)))
  "Returns full string definition for message of type '<TrackedPersons2d>"
  (cl:format cl:nil "# Message with all 2d bbox in image of currently tracked persons ~%#~%~%Header                header      # Header containing timestamp etc. of this message~%TrackedPerson2d[]     boxes       # All persons that are currently being tracked (2d image bbox)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: spencer_tracking_msgs/TrackedPerson2d~%# Message defining a 2d image bbox of a tracked person~%#~%~%uint64      track_id        # unique identifier of the target, consistent over time~%float32     person_height   # 3d height of person in m~%int32       x               # top left corner x of 2d image bbox~%int32       y               # top left corner y of 2d image bbox~%uint32      w               # width of 2d image bbox~%uint32      h               # height of 2d image bbox~%float32     depth           # distance from the camera in m~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackedPersons2d)))
  "Returns full string definition for message of type 'TrackedPersons2d"
  (cl:format cl:nil "# Message with all 2d bbox in image of currently tracked persons ~%#~%~%Header                header      # Header containing timestamp etc. of this message~%TrackedPerson2d[]     boxes       # All persons that are currently being tracked (2d image bbox)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: spencer_tracking_msgs/TrackedPerson2d~%# Message defining a 2d image bbox of a tracked person~%#~%~%uint64      track_id        # unique identifier of the target, consistent over time~%float32     person_height   # 3d height of person in m~%int32       x               # top left corner x of 2d image bbox~%int32       y               # top left corner y of 2d image bbox~%uint32      w               # width of 2d image bbox~%uint32      h               # height of 2d image bbox~%float32     depth           # distance from the camera in m~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackedPersons2d>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'boxes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackedPersons2d>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackedPersons2d
    (cl:cons ':header (header msg))
    (cl:cons ':boxes (boxes msg))
))
