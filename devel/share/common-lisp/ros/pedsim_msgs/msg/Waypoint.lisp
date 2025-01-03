; Auto-generated. Do not edit!


(cl:in-package pedsim_msgs-msg)


;//! \htmlinclude Waypoint.msg.html

(cl:defclass <Waypoint> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (behavior
    :reader behavior
    :initarg :behavior
    :type cl:fixnum
    :initform 0)
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (radius
    :reader radius
    :initarg :radius
    :type cl:float
    :initform 0.0))
)

(cl:defclass Waypoint (<Waypoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Waypoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Waypoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pedsim_msgs-msg:<Waypoint> is deprecated: use pedsim_msgs-msg:Waypoint instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <Waypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:name-val is deprecated.  Use pedsim_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'behavior-val :lambda-list '(m))
(cl:defmethod behavior-val ((m <Waypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:behavior-val is deprecated.  Use pedsim_msgs-msg:behavior instead.")
  (behavior m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <Waypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:position-val is deprecated.  Use pedsim_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <Waypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:radius-val is deprecated.  Use pedsim_msgs-msg:radius instead.")
  (radius m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Waypoint>)))
    "Constants for message type '<Waypoint>"
  '((:BHV_SIMPLE . 0)
    (:BHV_SOURCE . 1)
    (:BHV_SINK . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Waypoint)))
    "Constants for message type 'Waypoint"
  '((:BHV_SIMPLE . 0)
    (:BHV_SOURCE . 1)
    (:BHV_SINK . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Waypoint>) ostream)
  "Serializes a message object of type '<Waypoint>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let* ((signed (cl:slot-value msg 'behavior)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Waypoint>) istream)
  "Deserializes a message object of type '<Waypoint>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'behavior) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'radius) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Waypoint>)))
  "Returns string type for a message object of type '<Waypoint>"
  "pedsim_msgs/Waypoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Waypoint)))
  "Returns string type for a message object of type 'Waypoint"
  "pedsim_msgs/Waypoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Waypoint>)))
  "Returns md5sum for a message object of type '<Waypoint>"
  "7fe6c11b241f6ddddc1c756dacf4a21f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Waypoint)))
  "Returns md5sum for a message object of type 'Waypoint"
  "7fe6c11b241f6ddddc1c756dacf4a21f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Waypoint>)))
  "Returns full string definition for message of type '<Waypoint>"
  (cl:format cl:nil "int8 BHV_SIMPLE = 0~%int8 BHV_SOURCE = 1~%int8 BHV_SINK = 2~%~%string name~%int8 behavior~%geometry_msgs/Point position~%float32 radius~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Waypoint)))
  "Returns full string definition for message of type 'Waypoint"
  (cl:format cl:nil "int8 BHV_SIMPLE = 0~%int8 BHV_SOURCE = 1~%int8 BHV_SINK = 2~%~%string name~%int8 behavior~%geometry_msgs/Point position~%float32 radius~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Waypoint>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Waypoint>))
  "Converts a ROS message object to a list"
  (cl:list 'Waypoint
    (cl:cons ':name (name msg))
    (cl:cons ':behavior (behavior msg))
    (cl:cons ':position (position msg))
    (cl:cons ':radius (radius msg))
))
