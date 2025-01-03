; Auto-generated. Do not edit!


(cl:in-package spencer_control_msgs-msg)


;//! \htmlinclude ComponentStatus.msg.html

(cl:defclass <ComponentStatus> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (alive
    :reader alive
    :initarg :alive
    :type cl:boolean
    :initform cl:nil)
   (detail
    :reader detail
    :initarg :detail
    :type cl:string
    :initform ""))
)

(cl:defclass ComponentStatus (<ComponentStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ComponentStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ComponentStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_control_msgs-msg:<ComponentStatus> is deprecated: use spencer_control_msgs-msg:ComponentStatus instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ComponentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_control_msgs-msg:name-val is deprecated.  Use spencer_control_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'alive-val :lambda-list '(m))
(cl:defmethod alive-val ((m <ComponentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_control_msgs-msg:alive-val is deprecated.  Use spencer_control_msgs-msg:alive instead.")
  (alive m))

(cl:ensure-generic-function 'detail-val :lambda-list '(m))
(cl:defmethod detail-val ((m <ComponentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_control_msgs-msg:detail-val is deprecated.  Use spencer_control_msgs-msg:detail instead.")
  (detail m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ComponentStatus>) ostream)
  "Serializes a message object of type '<ComponentStatus>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'alive) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'detail))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'detail))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ComponentStatus>) istream)
  "Deserializes a message object of type '<ComponentStatus>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'alive) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'detail) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'detail) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ComponentStatus>)))
  "Returns string type for a message object of type '<ComponentStatus>"
  "spencer_control_msgs/ComponentStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ComponentStatus)))
  "Returns string type for a message object of type 'ComponentStatus"
  "spencer_control_msgs/ComponentStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ComponentStatus>)))
  "Returns md5sum for a message object of type '<ComponentStatus>"
  "b570ce0ae5316c3ad53df1d245486902")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ComponentStatus)))
  "Returns md5sum for a message object of type 'ComponentStatus"
  "b570ce0ae5316c3ad53df1d245486902")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ComponentStatus>)))
  "Returns full string definition for message of type '<ComponentStatus>"
  (cl:format cl:nil "string name~%bool alive~%string detail~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ComponentStatus)))
  "Returns full string definition for message of type 'ComponentStatus"
  (cl:format cl:nil "string name~%bool alive~%string detail~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ComponentStatus>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     1
     4 (cl:length (cl:slot-value msg 'detail))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ComponentStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'ComponentStatus
    (cl:cons ':name (name msg))
    (cl:cons ':alive (alive msg))
    (cl:cons ':detail (detail msg))
))
