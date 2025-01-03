; Auto-generated. Do not edit!


(cl:in-package spencer_human_attribute_msgs-msg)


;//! \htmlinclude ScalarAttribute.msg.html

(cl:defclass <ScalarAttribute> (roslisp-msg-protocol:ros-message)
  ((subject_id
    :reader subject_id
    :initarg :subject_id
    :type cl:integer
    :initform 0)
   (type
    :reader type
    :initarg :type
    :type cl:string
    :initform "")
   (values
    :reader values
    :initarg :values
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (confidences
    :reader confidences
    :initarg :confidences
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ScalarAttribute (<ScalarAttribute>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ScalarAttribute>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ScalarAttribute)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_human_attribute_msgs-msg:<ScalarAttribute> is deprecated: use spencer_human_attribute_msgs-msg:ScalarAttribute instead.")))

(cl:ensure-generic-function 'subject_id-val :lambda-list '(m))
(cl:defmethod subject_id-val ((m <ScalarAttribute>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_human_attribute_msgs-msg:subject_id-val is deprecated.  Use spencer_human_attribute_msgs-msg:subject_id instead.")
  (subject_id m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <ScalarAttribute>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_human_attribute_msgs-msg:type-val is deprecated.  Use spencer_human_attribute_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'values-val :lambda-list '(m))
(cl:defmethod values-val ((m <ScalarAttribute>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_human_attribute_msgs-msg:values-val is deprecated.  Use spencer_human_attribute_msgs-msg:values instead.")
  (values m))

(cl:ensure-generic-function 'confidences-val :lambda-list '(m))
(cl:defmethod confidences-val ((m <ScalarAttribute>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_human_attribute_msgs-msg:confidences-val is deprecated.  Use spencer_human_attribute_msgs-msg:confidences instead.")
  (confidences m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ScalarAttribute>)))
    "Constants for message type '<ScalarAttribute>"
  '((:PERSON_HEIGHT . "person_height"))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ScalarAttribute)))
    "Constants for message type 'ScalarAttribute"
  '((:PERSON_HEIGHT . "person_height"))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ScalarAttribute>) ostream)
  "Serializes a message object of type '<ScalarAttribute>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'subject_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'subject_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'subject_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'subject_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'subject_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'subject_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'subject_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'subject_id)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'values))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'values))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'confidences))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'confidences))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ScalarAttribute>) istream)
  "Deserializes a message object of type '<ScalarAttribute>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'subject_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'subject_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'subject_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'subject_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'subject_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'subject_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'subject_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'subject_id)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'values) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'values)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'confidences) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'confidences)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ScalarAttribute>)))
  "Returns string type for a message object of type '<ScalarAttribute>"
  "spencer_human_attribute_msgs/ScalarAttribute")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ScalarAttribute)))
  "Returns string type for a message object of type 'ScalarAttribute"
  "spencer_human_attribute_msgs/ScalarAttribute")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ScalarAttribute>)))
  "Returns md5sum for a message object of type '<ScalarAttribute>"
  "e10ac15d3bace9d3787523d1dd728fe0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ScalarAttribute)))
  "Returns md5sum for a message object of type 'ScalarAttribute"
  "e10ac15d3bace9d3787523d1dd728fe0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ScalarAttribute>)))
  "Returns full string definition for message of type '<ScalarAttribute>"
  (cl:format cl:nil "uint64                  subject_id              # either an observation ID or a track ID (if information has been integrated over time); should be encoded in topic name~%string                  type                    # type of the attribute, see constants below~%~%float32[]               values                  # values, each value also should have a confidence; highest-confidence value comes first!~%float32[]               confidences             # corresponding confidences should sum up to 1.0~%~%~%###########################################~%### Constants for scalar attribute types. #~%###########################################~%~%string      PERSON_HEIGHT        = person_height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ScalarAttribute)))
  "Returns full string definition for message of type 'ScalarAttribute"
  (cl:format cl:nil "uint64                  subject_id              # either an observation ID or a track ID (if information has been integrated over time); should be encoded in topic name~%string                  type                    # type of the attribute, see constants below~%~%float32[]               values                  # values, each value also should have a confidence; highest-confidence value comes first!~%float32[]               confidences             # corresponding confidences should sum up to 1.0~%~%~%###########################################~%### Constants for scalar attribute types. #~%###########################################~%~%string      PERSON_HEIGHT        = person_height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ScalarAttribute>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'type))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'confidences) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ScalarAttribute>))
  "Converts a ROS message object to a list"
  (cl:list 'ScalarAttribute
    (cl:cons ':subject_id (subject_id msg))
    (cl:cons ':type (type msg))
    (cl:cons ':values (values msg))
    (cl:cons ':confidences (confidences msg))
))
