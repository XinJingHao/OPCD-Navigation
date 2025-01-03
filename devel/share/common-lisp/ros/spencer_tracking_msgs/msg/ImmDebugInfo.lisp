; Auto-generated. Do not edit!


(cl:in-package spencer_tracking_msgs-msg)


;//! \htmlinclude ImmDebugInfo.msg.html

(cl:defclass <ImmDebugInfo> (roslisp-msg-protocol:ros-message)
  ((track_id
    :reader track_id
    :initarg :track_id
    :type cl:integer
    :initform 0)
   (innovation
    :reader innovation
    :initarg :innovation
    :type cl:float
    :initform 0.0)
   (CpXX
    :reader CpXX
    :initarg :CpXX
    :type cl:float
    :initform 0.0)
   (CpYY
    :reader CpYY
    :initarg :CpYY
    :type cl:float
    :initform 0.0)
   (CXX
    :reader CXX
    :initarg :CXX
    :type cl:float
    :initform 0.0)
   (CYY
    :reader CYY
    :initarg :CYY
    :type cl:float
    :initform 0.0)
   (modeProbabilities
    :reader modeProbabilities
    :initarg :modeProbabilities
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ImmDebugInfo (<ImmDebugInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImmDebugInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImmDebugInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_tracking_msgs-msg:<ImmDebugInfo> is deprecated: use spencer_tracking_msgs-msg:ImmDebugInfo instead.")))

(cl:ensure-generic-function 'track_id-val :lambda-list '(m))
(cl:defmethod track_id-val ((m <ImmDebugInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:track_id-val is deprecated.  Use spencer_tracking_msgs-msg:track_id instead.")
  (track_id m))

(cl:ensure-generic-function 'innovation-val :lambda-list '(m))
(cl:defmethod innovation-val ((m <ImmDebugInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:innovation-val is deprecated.  Use spencer_tracking_msgs-msg:innovation instead.")
  (innovation m))

(cl:ensure-generic-function 'CpXX-val :lambda-list '(m))
(cl:defmethod CpXX-val ((m <ImmDebugInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:CpXX-val is deprecated.  Use spencer_tracking_msgs-msg:CpXX instead.")
  (CpXX m))

(cl:ensure-generic-function 'CpYY-val :lambda-list '(m))
(cl:defmethod CpYY-val ((m <ImmDebugInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:CpYY-val is deprecated.  Use spencer_tracking_msgs-msg:CpYY instead.")
  (CpYY m))

(cl:ensure-generic-function 'CXX-val :lambda-list '(m))
(cl:defmethod CXX-val ((m <ImmDebugInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:CXX-val is deprecated.  Use spencer_tracking_msgs-msg:CXX instead.")
  (CXX m))

(cl:ensure-generic-function 'CYY-val :lambda-list '(m))
(cl:defmethod CYY-val ((m <ImmDebugInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:CYY-val is deprecated.  Use spencer_tracking_msgs-msg:CYY instead.")
  (CYY m))

(cl:ensure-generic-function 'modeProbabilities-val :lambda-list '(m))
(cl:defmethod modeProbabilities-val ((m <ImmDebugInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:modeProbabilities-val is deprecated.  Use spencer_tracking_msgs-msg:modeProbabilities instead.")
  (modeProbabilities m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImmDebugInfo>) ostream)
  "Serializes a message object of type '<ImmDebugInfo>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'track_id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'innovation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'CpXX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'CpYY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'CXX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'CYY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'modeProbabilities))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'modeProbabilities))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImmDebugInfo>) istream)
  "Deserializes a message object of type '<ImmDebugInfo>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'track_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'track_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'track_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'track_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'track_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'track_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'track_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'track_id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'innovation) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'CpXX) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'CpYY) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'CXX) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'CYY) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'modeProbabilities) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'modeProbabilities)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImmDebugInfo>)))
  "Returns string type for a message object of type '<ImmDebugInfo>"
  "spencer_tracking_msgs/ImmDebugInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImmDebugInfo)))
  "Returns string type for a message object of type 'ImmDebugInfo"
  "spencer_tracking_msgs/ImmDebugInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImmDebugInfo>)))
  "Returns md5sum for a message object of type '<ImmDebugInfo>"
  "824cd837fd158664a6f185fb8316da53")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImmDebugInfo)))
  "Returns md5sum for a message object of type 'ImmDebugInfo"
  "824cd837fd158664a6f185fb8316da53")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImmDebugInfo>)))
  "Returns full string definition for message of type '<ImmDebugInfo>"
  (cl:format cl:nil "# Message for passing debug information of filter performance~%#~%~%uint64      track_id        # unique identifier of the target, consistent over time~%float64      innovation      # innovation of prediction and associated observation~%float64      CpXX            # variance of prediction acc. to x~%float64      CpYY            # variance of prediction acc. to y~%float64      CXX             # variance of state acc. to x~%float64      CYY             # variance of state acc. to y~%float64[]    modeProbabilities# array containing mode probabilities~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImmDebugInfo)))
  "Returns full string definition for message of type 'ImmDebugInfo"
  (cl:format cl:nil "# Message for passing debug information of filter performance~%#~%~%uint64      track_id        # unique identifier of the target, consistent over time~%float64      innovation      # innovation of prediction and associated observation~%float64      CpXX            # variance of prediction acc. to x~%float64      CpYY            # variance of prediction acc. to y~%float64      CXX             # variance of state acc. to x~%float64      CYY             # variance of state acc. to y~%float64[]    modeProbabilities# array containing mode probabilities~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImmDebugInfo>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'modeProbabilities) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImmDebugInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'ImmDebugInfo
    (cl:cons ':track_id (track_id msg))
    (cl:cons ':innovation (innovation msg))
    (cl:cons ':CpXX (CpXX msg))
    (cl:cons ':CpYY (CpYY msg))
    (cl:cons ':CXX (CXX msg))
    (cl:cons ':CYY (CYY msg))
    (cl:cons ':modeProbabilities (modeProbabilities msg))
))
