; Auto-generated. Do not edit!


(cl:in-package gazebo_sfm_plugin-srv)


;//! \htmlinclude ped_state-request.msg.html

(cl:defclass <ped_state-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass ped_state-request (<ped_state-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ped_state-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ped_state-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gazebo_sfm_plugin-srv:<ped_state-request> is deprecated: use gazebo_sfm_plugin-srv:ped_state-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ped_state-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_sfm_plugin-srv:name-val is deprecated.  Use gazebo_sfm_plugin-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ped_state-request>) ostream)
  "Serializes a message object of type '<ped_state-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ped_state-request>) istream)
  "Deserializes a message object of type '<ped_state-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ped_state-request>)))
  "Returns string type for a service object of type '<ped_state-request>"
  "gazebo_sfm_plugin/ped_stateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ped_state-request)))
  "Returns string type for a service object of type 'ped_state-request"
  "gazebo_sfm_plugin/ped_stateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ped_state-request>)))
  "Returns md5sum for a message object of type '<ped_state-request>"
  "05beeb87454630bfc7ea88a89fd9b876")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ped_state-request)))
  "Returns md5sum for a message object of type 'ped_state-request"
  "05beeb87454630bfc7ea88a89fd9b876")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ped_state-request>)))
  "Returns full string definition for message of type '<ped_state-request>"
  (cl:format cl:nil "# client~%string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ped_state-request)))
  "Returns full string definition for message of type 'ped_state-request"
  (cl:format cl:nil "# client~%string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ped_state-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ped_state-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ped_state-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude ped_state-response.msg.html

(cl:defclass <ped_state-response> (roslisp-msg-protocol:ros-message)
  ((px
    :reader px
    :initarg :px
    :type cl:float
    :initform 0.0)
   (py
    :reader py
    :initarg :py
    :type cl:float
    :initform 0.0)
   (pz
    :reader pz
    :initarg :pz
    :type cl:float
    :initform 0.0)
   (vx
    :reader vx
    :initarg :vx
    :type cl:float
    :initform 0.0)
   (vy
    :reader vy
    :initarg :vy
    :type cl:float
    :initform 0.0)
   (theta
    :reader theta
    :initarg :theta
    :type cl:float
    :initform 0.0))
)

(cl:defclass ped_state-response (<ped_state-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ped_state-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ped_state-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gazebo_sfm_plugin-srv:<ped_state-response> is deprecated: use gazebo_sfm_plugin-srv:ped_state-response instead.")))

(cl:ensure-generic-function 'px-val :lambda-list '(m))
(cl:defmethod px-val ((m <ped_state-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_sfm_plugin-srv:px-val is deprecated.  Use gazebo_sfm_plugin-srv:px instead.")
  (px m))

(cl:ensure-generic-function 'py-val :lambda-list '(m))
(cl:defmethod py-val ((m <ped_state-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_sfm_plugin-srv:py-val is deprecated.  Use gazebo_sfm_plugin-srv:py instead.")
  (py m))

(cl:ensure-generic-function 'pz-val :lambda-list '(m))
(cl:defmethod pz-val ((m <ped_state-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_sfm_plugin-srv:pz-val is deprecated.  Use gazebo_sfm_plugin-srv:pz instead.")
  (pz m))

(cl:ensure-generic-function 'vx-val :lambda-list '(m))
(cl:defmethod vx-val ((m <ped_state-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_sfm_plugin-srv:vx-val is deprecated.  Use gazebo_sfm_plugin-srv:vx instead.")
  (vx m))

(cl:ensure-generic-function 'vy-val :lambda-list '(m))
(cl:defmethod vy-val ((m <ped_state-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_sfm_plugin-srv:vy-val is deprecated.  Use gazebo_sfm_plugin-srv:vy instead.")
  (vy m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <ped_state-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_sfm_plugin-srv:theta-val is deprecated.  Use gazebo_sfm_plugin-srv:theta instead.")
  (theta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ped_state-response>) ostream)
  "Serializes a message object of type '<ped_state-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'px))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'py))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'vx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'vy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ped_state-response>) istream)
  "Deserializes a message object of type '<ped_state-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'px) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'py) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pz) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vx) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vy) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ped_state-response>)))
  "Returns string type for a service object of type '<ped_state-response>"
  "gazebo_sfm_plugin/ped_stateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ped_state-response)))
  "Returns string type for a service object of type 'ped_state-response"
  "gazebo_sfm_plugin/ped_stateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ped_state-response>)))
  "Returns md5sum for a message object of type '<ped_state-response>"
  "05beeb87454630bfc7ea88a89fd9b876")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ped_state-response)))
  "Returns md5sum for a message object of type 'ped_state-response"
  "05beeb87454630bfc7ea88a89fd9b876")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ped_state-response>)))
  "Returns full string definition for message of type '<ped_state-response>"
  (cl:format cl:nil "# server~%float64 px~%float64 py~%float64 pz~%float64 vx~%float64 vy~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ped_state-response)))
  "Returns full string definition for message of type 'ped_state-response"
  (cl:format cl:nil "# server~%float64 px~%float64 py~%float64 pz~%float64 vx~%float64 vy~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ped_state-response>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ped_state-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ped_state-response
    (cl:cons ':px (px msg))
    (cl:cons ':py (py msg))
    (cl:cons ':pz (pz msg))
    (cl:cons ':vx (vx msg))
    (cl:cons ':vy (vy msg))
    (cl:cons ':theta (theta msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ped_state)))
  'ped_state-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ped_state)))
  'ped_state-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ped_state)))
  "Returns string type for a service object of type '<ped_state>"
  "gazebo_sfm_plugin/ped_state")