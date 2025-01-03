; Auto-generated. Do not edit!


(cl:in-package spencer_tracking_msgs-msg)


;//! \htmlinclude TrackedPerson2d.msg.html

(cl:defclass <TrackedPerson2d> (roslisp-msg-protocol:ros-message)
  ((track_id
    :reader track_id
    :initarg :track_id
    :type cl:integer
    :initform 0)
   (person_height
    :reader person_height
    :initarg :person_height
    :type cl:float
    :initform 0.0)
   (x
    :reader x
    :initarg :x
    :type cl:integer
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:integer
    :initform 0)
   (w
    :reader w
    :initarg :w
    :type cl:integer
    :initform 0)
   (h
    :reader h
    :initarg :h
    :type cl:integer
    :initform 0)
   (depth
    :reader depth
    :initarg :depth
    :type cl:float
    :initform 0.0))
)

(cl:defclass TrackedPerson2d (<TrackedPerson2d>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackedPerson2d>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackedPerson2d)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_tracking_msgs-msg:<TrackedPerson2d> is deprecated: use spencer_tracking_msgs-msg:TrackedPerson2d instead.")))

(cl:ensure-generic-function 'track_id-val :lambda-list '(m))
(cl:defmethod track_id-val ((m <TrackedPerson2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:track_id-val is deprecated.  Use spencer_tracking_msgs-msg:track_id instead.")
  (track_id m))

(cl:ensure-generic-function 'person_height-val :lambda-list '(m))
(cl:defmethod person_height-val ((m <TrackedPerson2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:person_height-val is deprecated.  Use spencer_tracking_msgs-msg:person_height instead.")
  (person_height m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <TrackedPerson2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:x-val is deprecated.  Use spencer_tracking_msgs-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <TrackedPerson2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:y-val is deprecated.  Use spencer_tracking_msgs-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'w-val :lambda-list '(m))
(cl:defmethod w-val ((m <TrackedPerson2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:w-val is deprecated.  Use spencer_tracking_msgs-msg:w instead.")
  (w m))

(cl:ensure-generic-function 'h-val :lambda-list '(m))
(cl:defmethod h-val ((m <TrackedPerson2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:h-val is deprecated.  Use spencer_tracking_msgs-msg:h instead.")
  (h m))

(cl:ensure-generic-function 'depth-val :lambda-list '(m))
(cl:defmethod depth-val ((m <TrackedPerson2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:depth-val is deprecated.  Use spencer_tracking_msgs-msg:depth instead.")
  (depth m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackedPerson2d>) ostream)
  "Serializes a message object of type '<TrackedPerson2d>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'track_id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'person_height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'w)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'w)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'w)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'w)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'h)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'h)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'h)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'h)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'depth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackedPerson2d>) istream)
  "Deserializes a message object of type '<TrackedPerson2d>"
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
    (cl:setf (cl:slot-value msg 'person_height) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'w)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'w)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'w)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'w)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'h)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'h)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'h)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'h)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'depth) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackedPerson2d>)))
  "Returns string type for a message object of type '<TrackedPerson2d>"
  "spencer_tracking_msgs/TrackedPerson2d")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackedPerson2d)))
  "Returns string type for a message object of type 'TrackedPerson2d"
  "spencer_tracking_msgs/TrackedPerson2d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackedPerson2d>)))
  "Returns md5sum for a message object of type '<TrackedPerson2d>"
  "12df4823c658df0d660d2a5c68ae4aea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackedPerson2d)))
  "Returns md5sum for a message object of type 'TrackedPerson2d"
  "12df4823c658df0d660d2a5c68ae4aea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackedPerson2d>)))
  "Returns full string definition for message of type '<TrackedPerson2d>"
  (cl:format cl:nil "# Message defining a 2d image bbox of a tracked person~%#~%~%uint64      track_id        # unique identifier of the target, consistent over time~%float32     person_height   # 3d height of person in m~%int32       x               # top left corner x of 2d image bbox~%int32       y               # top left corner y of 2d image bbox~%uint32      w               # width of 2d image bbox~%uint32      h               # height of 2d image bbox~%float32     depth           # distance from the camera in m~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackedPerson2d)))
  "Returns full string definition for message of type 'TrackedPerson2d"
  (cl:format cl:nil "# Message defining a 2d image bbox of a tracked person~%#~%~%uint64      track_id        # unique identifier of the target, consistent over time~%float32     person_height   # 3d height of person in m~%int32       x               # top left corner x of 2d image bbox~%int32       y               # top left corner y of 2d image bbox~%uint32      w               # width of 2d image bbox~%uint32      h               # height of 2d image bbox~%float32     depth           # distance from the camera in m~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackedPerson2d>))
  (cl:+ 0
     8
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackedPerson2d>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackedPerson2d
    (cl:cons ':track_id (track_id msg))
    (cl:cons ':person_height (person_height msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':w (w msg))
    (cl:cons ':h (h msg))
    (cl:cons ':depth (depth msg))
))
