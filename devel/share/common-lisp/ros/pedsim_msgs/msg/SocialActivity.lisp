; Auto-generated. Do not edit!


(cl:in-package pedsim_msgs-msg)


;//! \htmlinclude SocialActivity.msg.html

(cl:defclass <SocialActivity> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:string
    :initform "")
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0)
   (track_ids
    :reader track_ids
    :initarg :track_ids
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass SocialActivity (<SocialActivity>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SocialActivity>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SocialActivity)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pedsim_msgs-msg:<SocialActivity> is deprecated: use pedsim_msgs-msg:SocialActivity instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <SocialActivity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:type-val is deprecated.  Use pedsim_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <SocialActivity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:confidence-val is deprecated.  Use pedsim_msgs-msg:confidence instead.")
  (confidence m))

(cl:ensure-generic-function 'track_ids-val :lambda-list '(m))
(cl:defmethod track_ids-val ((m <SocialActivity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:track_ids-val is deprecated.  Use pedsim_msgs-msg:track_ids instead.")
  (track_ids m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SocialActivity>)))
    "Constants for message type '<SocialActivity>"
  '((:TYPE_SHOPPING . "shopping")
    (:TYPE_STANDING . "standing")
    (:TYPE_INDIVIDUAL_MOVING . "individual_moving")
    (:TYPE_WAITING_IN_QUEUE . "waiting_in_queue")
    (:TYPE_LOOKING_AT_INFORMATION_SCREEN . "looking_at_information_screen")
    (:TYPE_LOOKING_AT_KIOSK . "looking_at_kiosk")
    (:TYPE_GROUP_ASSEMBLING . "group_assembling")
    (:TYPE_GROUP_MOVING . "group_moving")
    (:TYPE_FLOW_WITH_ROBOT . "flow")
    (:TYPE_ANTIFLOW_AGAINST_ROBOT . "antiflow")
    (:TYPE_WAITING_FOR_OTHERS . "waiting_for_others"))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SocialActivity)))
    "Constants for message type 'SocialActivity"
  '((:TYPE_SHOPPING . "shopping")
    (:TYPE_STANDING . "standing")
    (:TYPE_INDIVIDUAL_MOVING . "individual_moving")
    (:TYPE_WAITING_IN_QUEUE . "waiting_in_queue")
    (:TYPE_LOOKING_AT_INFORMATION_SCREEN . "looking_at_information_screen")
    (:TYPE_LOOKING_AT_KIOSK . "looking_at_kiosk")
    (:TYPE_GROUP_ASSEMBLING . "group_assembling")
    (:TYPE_GROUP_MOVING . "group_moving")
    (:TYPE_FLOW_WITH_ROBOT . "flow")
    (:TYPE_ANTIFLOW_AGAINST_ROBOT . "antiflow")
    (:TYPE_WAITING_FOR_OTHERS . "waiting_for_others"))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SocialActivity>) ostream)
  "Serializes a message object of type '<SocialActivity>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'track_ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) ele) ostream))
   (cl:slot-value msg 'track_ids))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SocialActivity>) istream)
  "Deserializes a message object of type '<SocialActivity>"
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
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'track_ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'track_ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SocialActivity>)))
  "Returns string type for a message object of type '<SocialActivity>"
  "pedsim_msgs/SocialActivity")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SocialActivity)))
  "Returns string type for a message object of type 'SocialActivity"
  "pedsim_msgs/SocialActivity")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SocialActivity>)))
  "Returns md5sum for a message object of type '<SocialActivity>"
  "064aac12909022edb4df5d568c180144")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SocialActivity)))
  "Returns md5sum for a message object of type 'SocialActivity"
  "064aac12909022edb4df5d568c180144")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SocialActivity>)))
  "Returns full string definition for message of type '<SocialActivity>"
  (cl:format cl:nil "string      type        # see constants below~%float32     confidence  # detection confidence~%~%uint64[]      track_ids   # IDs of all person tracks involved in the activity, might be one or multiple~%~%~%# Constants for social activity type (just examples at the moment)~%string      TYPE_SHOPPING = shopping~%string      TYPE_STANDING = standing~%string      TYPE_INDIVIDUAL_MOVING = individual_moving~%string      TYPE_WAITING_IN_QUEUE = waiting_in_queue~%string      TYPE_LOOKING_AT_INFORMATION_SCREEN = looking_at_information_screen~%string      TYPE_LOOKING_AT_KIOSK = looking_at_kiosk~%string      TYPE_GROUP_ASSEMBLING = group_assembling~%string      TYPE_GROUP_MOVING = group_moving~%string      TYPE_FLOW_WITH_ROBOT = flow~%string      TYPE_ANTIFLOW_AGAINST_ROBOT = antiflow~%string      TYPE_WAITING_FOR_OTHERS = waiting_for_others~%~%#string      TYPE_COMMUNICATING = communicating~%#string      TYPE_TAKING_PHOTOGRAPH = taking_photograph~%#string      TYPE_TALKING_ON_PHONE = talking_on_phone~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SocialActivity)))
  "Returns full string definition for message of type 'SocialActivity"
  (cl:format cl:nil "string      type        # see constants below~%float32     confidence  # detection confidence~%~%uint64[]      track_ids   # IDs of all person tracks involved in the activity, might be one or multiple~%~%~%# Constants for social activity type (just examples at the moment)~%string      TYPE_SHOPPING = shopping~%string      TYPE_STANDING = standing~%string      TYPE_INDIVIDUAL_MOVING = individual_moving~%string      TYPE_WAITING_IN_QUEUE = waiting_in_queue~%string      TYPE_LOOKING_AT_INFORMATION_SCREEN = looking_at_information_screen~%string      TYPE_LOOKING_AT_KIOSK = looking_at_kiosk~%string      TYPE_GROUP_ASSEMBLING = group_assembling~%string      TYPE_GROUP_MOVING = group_moving~%string      TYPE_FLOW_WITH_ROBOT = flow~%string      TYPE_ANTIFLOW_AGAINST_ROBOT = antiflow~%string      TYPE_WAITING_FOR_OTHERS = waiting_for_others~%~%#string      TYPE_COMMUNICATING = communicating~%#string      TYPE_TAKING_PHOTOGRAPH = taking_photograph~%#string      TYPE_TALKING_ON_PHONE = talking_on_phone~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SocialActivity>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'type))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'track_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SocialActivity>))
  "Converts a ROS message object to a list"
  (cl:list 'SocialActivity
    (cl:cons ':type (type msg))
    (cl:cons ':confidence (confidence msg))
    (cl:cons ':track_ids (track_ids msg))
))
