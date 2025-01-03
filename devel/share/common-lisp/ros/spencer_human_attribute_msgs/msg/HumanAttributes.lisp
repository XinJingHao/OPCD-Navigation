; Auto-generated. Do not edit!


(cl:in-package spencer_human_attribute_msgs-msg)


;//! \htmlinclude HumanAttributes.msg.html

(cl:defclass <HumanAttributes> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (categoricalAttributes
    :reader categoricalAttributes
    :initarg :categoricalAttributes
    :type (cl:vector spencer_human_attribute_msgs-msg:CategoricalAttribute)
   :initform (cl:make-array 0 :element-type 'spencer_human_attribute_msgs-msg:CategoricalAttribute :initial-element (cl:make-instance 'spencer_human_attribute_msgs-msg:CategoricalAttribute)))
   (scalarAttributes
    :reader scalarAttributes
    :initarg :scalarAttributes
    :type (cl:vector spencer_human_attribute_msgs-msg:ScalarAttribute)
   :initform (cl:make-array 0 :element-type 'spencer_human_attribute_msgs-msg:ScalarAttribute :initial-element (cl:make-instance 'spencer_human_attribute_msgs-msg:ScalarAttribute))))
)

(cl:defclass HumanAttributes (<HumanAttributes>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HumanAttributes>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HumanAttributes)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_human_attribute_msgs-msg:<HumanAttributes> is deprecated: use spencer_human_attribute_msgs-msg:HumanAttributes instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HumanAttributes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_human_attribute_msgs-msg:header-val is deprecated.  Use spencer_human_attribute_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'categoricalAttributes-val :lambda-list '(m))
(cl:defmethod categoricalAttributes-val ((m <HumanAttributes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_human_attribute_msgs-msg:categoricalAttributes-val is deprecated.  Use spencer_human_attribute_msgs-msg:categoricalAttributes instead.")
  (categoricalAttributes m))

(cl:ensure-generic-function 'scalarAttributes-val :lambda-list '(m))
(cl:defmethod scalarAttributes-val ((m <HumanAttributes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_human_attribute_msgs-msg:scalarAttributes-val is deprecated.  Use spencer_human_attribute_msgs-msg:scalarAttributes instead.")
  (scalarAttributes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HumanAttributes>) ostream)
  "Serializes a message object of type '<HumanAttributes>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'categoricalAttributes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'categoricalAttributes))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'scalarAttributes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'scalarAttributes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HumanAttributes>) istream)
  "Deserializes a message object of type '<HumanAttributes>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'categoricalAttributes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'categoricalAttributes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'spencer_human_attribute_msgs-msg:CategoricalAttribute))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'scalarAttributes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'scalarAttributes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'spencer_human_attribute_msgs-msg:ScalarAttribute))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HumanAttributes>)))
  "Returns string type for a message object of type '<HumanAttributes>"
  "spencer_human_attribute_msgs/HumanAttributes")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HumanAttributes)))
  "Returns string type for a message object of type 'HumanAttributes"
  "spencer_human_attribute_msgs/HumanAttributes")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HumanAttributes>)))
  "Returns md5sum for a message object of type '<HumanAttributes>"
  "0ce67039f788378180e089c471174c58")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HumanAttributes)))
  "Returns md5sum for a message object of type 'HumanAttributes"
  "0ce67039f788378180e089c471174c58")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HumanAttributes>)))
  "Returns full string definition for message of type '<HumanAttributes>"
  (cl:format cl:nil "std_msgs/Header header~%~%# One entry per attribute type per person~%CategoricalAttribute[]   categoricalAttributes~%ScalarAttribute[]       scalarAttributes~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: spencer_human_attribute_msgs/CategoricalAttribute~%uint64                  subject_id              # either an observation ID or a track ID (if information has been integrated over time); should be encoded in topic name~%string                  type                    # type of the attribute, see constants below~%~%string[]                values                  # values, each value also should have a confidence, highest-confidence attribute should come first~%float32[]               confidences             # corresponding confidences should sum up to 1.0, highest confidence comes first~%~%~%##################################################~%### Constants for categorical attribute types. ###~%##################################################~%~%string      GENDER        = gender~%string      AGE_GROUP     = age_group~%~%###################################################~%### Constants for categorical attribute values. ###~%###################################################~%~%string      GENDER_MALE             = male~%string      GENDER_FEMALE           = female~%~%# Age groups are based upon the categories from the \"Images Of Groups\" RGB database~%string      AGE_GROUP_0_TO_2        = 0-2~%string      AGE_GROUP_3_TO_7        = 3-7~%string      AGE_GROUP_8_TO_12       = 8-12~%string      AGE_GROUP_13_TO_19      = 13-19~%string      AGE_GROUP_20_TO_36      = 20-36~%string      AGE_GROUP_37_TO_65      = 37-65~%string      AGE_GROUP_66_TO_99      = 66-99~%~%~%~%~%================================================================================~%MSG: spencer_human_attribute_msgs/ScalarAttribute~%uint64                  subject_id              # either an observation ID or a track ID (if information has been integrated over time); should be encoded in topic name~%string                  type                    # type of the attribute, see constants below~%~%float32[]               values                  # values, each value also should have a confidence; highest-confidence value comes first!~%float32[]               confidences             # corresponding confidences should sum up to 1.0~%~%~%###########################################~%### Constants for scalar attribute types. #~%###########################################~%~%string      PERSON_HEIGHT        = person_height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HumanAttributes)))
  "Returns full string definition for message of type 'HumanAttributes"
  (cl:format cl:nil "std_msgs/Header header~%~%# One entry per attribute type per person~%CategoricalAttribute[]   categoricalAttributes~%ScalarAttribute[]       scalarAttributes~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: spencer_human_attribute_msgs/CategoricalAttribute~%uint64                  subject_id              # either an observation ID or a track ID (if information has been integrated over time); should be encoded in topic name~%string                  type                    # type of the attribute, see constants below~%~%string[]                values                  # values, each value also should have a confidence, highest-confidence attribute should come first~%float32[]               confidences             # corresponding confidences should sum up to 1.0, highest confidence comes first~%~%~%##################################################~%### Constants for categorical attribute types. ###~%##################################################~%~%string      GENDER        = gender~%string      AGE_GROUP     = age_group~%~%###################################################~%### Constants for categorical attribute values. ###~%###################################################~%~%string      GENDER_MALE             = male~%string      GENDER_FEMALE           = female~%~%# Age groups are based upon the categories from the \"Images Of Groups\" RGB database~%string      AGE_GROUP_0_TO_2        = 0-2~%string      AGE_GROUP_3_TO_7        = 3-7~%string      AGE_GROUP_8_TO_12       = 8-12~%string      AGE_GROUP_13_TO_19      = 13-19~%string      AGE_GROUP_20_TO_36      = 20-36~%string      AGE_GROUP_37_TO_65      = 37-65~%string      AGE_GROUP_66_TO_99      = 66-99~%~%~%~%~%================================================================================~%MSG: spencer_human_attribute_msgs/ScalarAttribute~%uint64                  subject_id              # either an observation ID or a track ID (if information has been integrated over time); should be encoded in topic name~%string                  type                    # type of the attribute, see constants below~%~%float32[]               values                  # values, each value also should have a confidence; highest-confidence value comes first!~%float32[]               confidences             # corresponding confidences should sum up to 1.0~%~%~%###########################################~%### Constants for scalar attribute types. #~%###########################################~%~%string      PERSON_HEIGHT        = person_height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HumanAttributes>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'categoricalAttributes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'scalarAttributes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HumanAttributes>))
  "Converts a ROS message object to a list"
  (cl:list 'HumanAttributes
    (cl:cons ':header (header msg))
    (cl:cons ':categoricalAttributes (categoricalAttributes msg))
    (cl:cons ':scalarAttributes (scalarAttributes msg))
))
