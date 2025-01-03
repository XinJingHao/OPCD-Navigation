; Auto-generated. Do not edit!


(cl:in-package pedsim_msgs-msg)


;//! \htmlinclude SocialRelation.msg.html

(cl:defclass <SocialRelation> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:string
    :initform "")
   (strength
    :reader strength
    :initarg :strength
    :type cl:float
    :initform 0.0)
   (track1_id
    :reader track1_id
    :initarg :track1_id
    :type cl:integer
    :initform 0)
   (track2_id
    :reader track2_id
    :initarg :track2_id
    :type cl:integer
    :initform 0))
)

(cl:defclass SocialRelation (<SocialRelation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SocialRelation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SocialRelation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pedsim_msgs-msg:<SocialRelation> is deprecated: use pedsim_msgs-msg:SocialRelation instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <SocialRelation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:type-val is deprecated.  Use pedsim_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'strength-val :lambda-list '(m))
(cl:defmethod strength-val ((m <SocialRelation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:strength-val is deprecated.  Use pedsim_msgs-msg:strength instead.")
  (strength m))

(cl:ensure-generic-function 'track1_id-val :lambda-list '(m))
(cl:defmethod track1_id-val ((m <SocialRelation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:track1_id-val is deprecated.  Use pedsim_msgs-msg:track1_id instead.")
  (track1_id m))

(cl:ensure-generic-function 'track2_id-val :lambda-list '(m))
(cl:defmethod track2_id-val ((m <SocialRelation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:track2_id-val is deprecated.  Use pedsim_msgs-msg:track2_id instead.")
  (track2_id m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SocialRelation>)))
    "Constants for message type '<SocialRelation>"
  '((:TYPE_SPATIAL . "\"spatial\"")
    (:TYPE_ROMANTIC . "\"romantic\"")
    (:TYPE_PARENT_CHILD . "\"parent_child\"")
    (:TYPE_FRIENDSHIP . "\"friendship\""))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SocialRelation)))
    "Constants for message type 'SocialRelation"
  '((:TYPE_SPATIAL . "\"spatial\"")
    (:TYPE_ROMANTIC . "\"romantic\"")
    (:TYPE_PARENT_CHILD . "\"parent_child\"")
    (:TYPE_FRIENDSHIP . "\"friendship\""))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SocialRelation>) ostream)
  "Serializes a message object of type '<SocialRelation>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'strength))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'track1_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'track1_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'track1_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'track1_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'track2_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'track2_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'track2_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'track2_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SocialRelation>) istream)
  "Deserializes a message object of type '<SocialRelation>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'strength) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'track1_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'track1_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'track1_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'track1_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'track2_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'track2_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'track2_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'track2_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SocialRelation>)))
  "Returns string type for a message object of type '<SocialRelation>"
  "pedsim_msgs/SocialRelation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SocialRelation)))
  "Returns string type for a message object of type 'SocialRelation"
  "pedsim_msgs/SocialRelation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SocialRelation>)))
  "Returns md5sum for a message object of type '<SocialRelation>"
  "004e9c919342c749d66dbc051dba51f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SocialRelation)))
  "Returns md5sum for a message object of type 'SocialRelation"
  "004e9c919342c749d66dbc051dba51f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SocialRelation>)))
  "Returns full string definition for message of type '<SocialRelation>"
  (cl:format cl:nil "string      type        # e.g. mother-son relationship, romantic relationship, etc.~%float32     strength    # relationship strength between 0.0 and 1.0~%~%uint32      track1_id~%uint32      track2_id~%~%~%# Constants for type (just examples at the moment)~%string      TYPE_SPATIAL  = \"spatial\"~%string      TYPE_ROMANTIC = \"romantic\"~%string      TYPE_PARENT_CHILD = \"parent_child\"~%string      TYPE_FRIENDSHIP = \"friendship\"~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SocialRelation)))
  "Returns full string definition for message of type 'SocialRelation"
  (cl:format cl:nil "string      type        # e.g. mother-son relationship, romantic relationship, etc.~%float32     strength    # relationship strength between 0.0 and 1.0~%~%uint32      track1_id~%uint32      track2_id~%~%~%# Constants for type (just examples at the moment)~%string      TYPE_SPATIAL  = \"spatial\"~%string      TYPE_ROMANTIC = \"romantic\"~%string      TYPE_PARENT_CHILD = \"parent_child\"~%string      TYPE_FRIENDSHIP = \"friendship\"~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SocialRelation>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'type))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SocialRelation>))
  "Converts a ROS message object to a list"
  (cl:list 'SocialRelation
    (cl:cons ':type (type msg))
    (cl:cons ':strength (strength msg))
    (cl:cons ':track1_id (track1_id msg))
    (cl:cons ':track2_id (track2_id msg))
))
