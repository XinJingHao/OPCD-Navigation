; Auto-generated. Do not edit!


(cl:in-package spencer_human_attribute_msgs-msg)


;//! \htmlinclude CategoricalAttribute.msg.html

(cl:defclass <CategoricalAttribute> (roslisp-msg-protocol:ros-message)
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
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (confidences
    :reader confidences
    :initarg :confidences
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass CategoricalAttribute (<CategoricalAttribute>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CategoricalAttribute>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CategoricalAttribute)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_human_attribute_msgs-msg:<CategoricalAttribute> is deprecated: use spencer_human_attribute_msgs-msg:CategoricalAttribute instead.")))

(cl:ensure-generic-function 'subject_id-val :lambda-list '(m))
(cl:defmethod subject_id-val ((m <CategoricalAttribute>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_human_attribute_msgs-msg:subject_id-val is deprecated.  Use spencer_human_attribute_msgs-msg:subject_id instead.")
  (subject_id m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <CategoricalAttribute>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_human_attribute_msgs-msg:type-val is deprecated.  Use spencer_human_attribute_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'values-val :lambda-list '(m))
(cl:defmethod values-val ((m <CategoricalAttribute>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_human_attribute_msgs-msg:values-val is deprecated.  Use spencer_human_attribute_msgs-msg:values instead.")
  (values m))

(cl:ensure-generic-function 'confidences-val :lambda-list '(m))
(cl:defmethod confidences-val ((m <CategoricalAttribute>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_human_attribute_msgs-msg:confidences-val is deprecated.  Use spencer_human_attribute_msgs-msg:confidences instead.")
  (confidences m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CategoricalAttribute>)))
    "Constants for message type '<CategoricalAttribute>"
  '((:GENDER . "gender")
    (:AGE_GROUP . "age_group")
    (:GENDER_MALE . "male")
    (:GENDER_FEMALE . "female")
    (:AGE_GROUP_0_TO_2 . "0-2")
    (:AGE_GROUP_3_TO_7 . "3-7")
    (:AGE_GROUP_8_TO_12 . "8-12")
    (:AGE_GROUP_13_TO_19 . "13-19")
    (:AGE_GROUP_20_TO_36 . "20-36")
    (:AGE_GROUP_37_TO_65 . "37-65")
    (:AGE_GROUP_66_TO_99 . "66-99"))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CategoricalAttribute)))
    "Constants for message type 'CategoricalAttribute"
  '((:GENDER . "gender")
    (:AGE_GROUP . "age_group")
    (:GENDER_MALE . "male")
    (:GENDER_FEMALE . "female")
    (:AGE_GROUP_0_TO_2 . "0-2")
    (:AGE_GROUP_3_TO_7 . "3-7")
    (:AGE_GROUP_8_TO_12 . "8-12")
    (:AGE_GROUP_13_TO_19 . "13-19")
    (:AGE_GROUP_20_TO_36 . "20-36")
    (:AGE_GROUP_37_TO_65 . "37-65")
    (:AGE_GROUP_66_TO_99 . "66-99"))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CategoricalAttribute>) ostream)
  "Serializes a message object of type '<CategoricalAttribute>"
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
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CategoricalAttribute>) istream)
  "Deserializes a message object of type '<CategoricalAttribute>"
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
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CategoricalAttribute>)))
  "Returns string type for a message object of type '<CategoricalAttribute>"
  "spencer_human_attribute_msgs/CategoricalAttribute")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CategoricalAttribute)))
  "Returns string type for a message object of type 'CategoricalAttribute"
  "spencer_human_attribute_msgs/CategoricalAttribute")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CategoricalAttribute>)))
  "Returns md5sum for a message object of type '<CategoricalAttribute>"
  "022dc607a1c6f3404ecf625be18f25f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CategoricalAttribute)))
  "Returns md5sum for a message object of type 'CategoricalAttribute"
  "022dc607a1c6f3404ecf625be18f25f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CategoricalAttribute>)))
  "Returns full string definition for message of type '<CategoricalAttribute>"
  (cl:format cl:nil "uint64                  subject_id              # either an observation ID or a track ID (if information has been integrated over time); should be encoded in topic name~%string                  type                    # type of the attribute, see constants below~%~%string[]                values                  # values, each value also should have a confidence, highest-confidence attribute should come first~%float32[]               confidences             # corresponding confidences should sum up to 1.0, highest confidence comes first~%~%~%##################################################~%### Constants for categorical attribute types. ###~%##################################################~%~%string      GENDER        = gender~%string      AGE_GROUP     = age_group~%~%###################################################~%### Constants for categorical attribute values. ###~%###################################################~%~%string      GENDER_MALE             = male~%string      GENDER_FEMALE           = female~%~%# Age groups are based upon the categories from the \"Images Of Groups\" RGB database~%string      AGE_GROUP_0_TO_2        = 0-2~%string      AGE_GROUP_3_TO_7        = 3-7~%string      AGE_GROUP_8_TO_12       = 8-12~%string      AGE_GROUP_13_TO_19      = 13-19~%string      AGE_GROUP_20_TO_36      = 20-36~%string      AGE_GROUP_37_TO_65      = 37-65~%string      AGE_GROUP_66_TO_99      = 66-99~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CategoricalAttribute)))
  "Returns full string definition for message of type 'CategoricalAttribute"
  (cl:format cl:nil "uint64                  subject_id              # either an observation ID or a track ID (if information has been integrated over time); should be encoded in topic name~%string                  type                    # type of the attribute, see constants below~%~%string[]                values                  # values, each value also should have a confidence, highest-confidence attribute should come first~%float32[]               confidences             # corresponding confidences should sum up to 1.0, highest confidence comes first~%~%~%##################################################~%### Constants for categorical attribute types. ###~%##################################################~%~%string      GENDER        = gender~%string      AGE_GROUP     = age_group~%~%###################################################~%### Constants for categorical attribute values. ###~%###################################################~%~%string      GENDER_MALE             = male~%string      GENDER_FEMALE           = female~%~%# Age groups are based upon the categories from the \"Images Of Groups\" RGB database~%string      AGE_GROUP_0_TO_2        = 0-2~%string      AGE_GROUP_3_TO_7        = 3-7~%string      AGE_GROUP_8_TO_12       = 8-12~%string      AGE_GROUP_13_TO_19      = 13-19~%string      AGE_GROUP_20_TO_36      = 20-36~%string      AGE_GROUP_37_TO_65      = 37-65~%string      AGE_GROUP_66_TO_99      = 66-99~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CategoricalAttribute>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'type))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'confidences) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CategoricalAttribute>))
  "Converts a ROS message object to a list"
  (cl:list 'CategoricalAttribute
    (cl:cons ':subject_id (subject_id msg))
    (cl:cons ':type (type msg))
    (cl:cons ':values (values msg))
    (cl:cons ':confidences (confidences msg))
))
