; Auto-generated. Do not edit!


(cl:in-package pedsim_msgs-msg)


;//! \htmlinclude SocialRelations.msg.html

(cl:defclass <SocialRelations> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (elements
    :reader elements
    :initarg :elements
    :type (cl:vector pedsim_msgs-msg:SocialRelation)
   :initform (cl:make-array 0 :element-type 'pedsim_msgs-msg:SocialRelation :initial-element (cl:make-instance 'pedsim_msgs-msg:SocialRelation))))
)

(cl:defclass SocialRelations (<SocialRelations>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SocialRelations>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SocialRelations)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pedsim_msgs-msg:<SocialRelations> is deprecated: use pedsim_msgs-msg:SocialRelations instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SocialRelations>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:header-val is deprecated.  Use pedsim_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'elements-val :lambda-list '(m))
(cl:defmethod elements-val ((m <SocialRelations>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:elements-val is deprecated.  Use pedsim_msgs-msg:elements instead.")
  (elements m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SocialRelations>) ostream)
  "Serializes a message object of type '<SocialRelations>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'elements))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'elements))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SocialRelations>) istream)
  "Deserializes a message object of type '<SocialRelations>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'elements) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'elements)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'pedsim_msgs-msg:SocialRelation))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SocialRelations>)))
  "Returns string type for a message object of type '<SocialRelations>"
  "pedsim_msgs/SocialRelations")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SocialRelations)))
  "Returns string type for a message object of type 'SocialRelations"
  "pedsim_msgs/SocialRelations")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SocialRelations>)))
  "Returns md5sum for a message object of type '<SocialRelations>"
  "433b8d0d73396f7d640c03dcb80bb4fe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SocialRelations)))
  "Returns md5sum for a message object of type 'SocialRelations"
  "433b8d0d73396f7d640c03dcb80bb4fe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SocialRelations>)))
  "Returns full string definition for message of type '<SocialRelations>"
  (cl:format cl:nil "std_msgs/Header     header~%~%# All social relations of all tracks in the current time step.~%# There might be multiple social relations per pair of tracks.~%SocialRelation[]    elements~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: pedsim_msgs/SocialRelation~%string      type        # e.g. mother-son relationship, romantic relationship, etc.~%float32     strength    # relationship strength between 0.0 and 1.0~%~%uint32      track1_id~%uint32      track2_id~%~%~%# Constants for type (just examples at the moment)~%string      TYPE_SPATIAL  = \"spatial\"~%string      TYPE_ROMANTIC = \"romantic\"~%string      TYPE_PARENT_CHILD = \"parent_child\"~%string      TYPE_FRIENDSHIP = \"friendship\"~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SocialRelations)))
  "Returns full string definition for message of type 'SocialRelations"
  (cl:format cl:nil "std_msgs/Header     header~%~%# All social relations of all tracks in the current time step.~%# There might be multiple social relations per pair of tracks.~%SocialRelation[]    elements~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: pedsim_msgs/SocialRelation~%string      type        # e.g. mother-son relationship, romantic relationship, etc.~%float32     strength    # relationship strength between 0.0 and 1.0~%~%uint32      track1_id~%uint32      track2_id~%~%~%# Constants for type (just examples at the moment)~%string      TYPE_SPATIAL  = \"spatial\"~%string      TYPE_ROMANTIC = \"romantic\"~%string      TYPE_PARENT_CHILD = \"parent_child\"~%string      TYPE_FRIENDSHIP = \"friendship\"~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SocialRelations>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'elements) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SocialRelations>))
  "Converts a ROS message object to a list"
  (cl:list 'SocialRelations
    (cl:cons ':header (header msg))
    (cl:cons ':elements (elements msg))
))
