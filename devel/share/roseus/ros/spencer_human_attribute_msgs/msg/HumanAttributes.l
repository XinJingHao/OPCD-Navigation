;; Auto-generated. Do not edit!


(when (boundp 'spencer_human_attribute_msgs::HumanAttributes)
  (if (not (find-package "SPENCER_HUMAN_ATTRIBUTE_MSGS"))
    (make-package "SPENCER_HUMAN_ATTRIBUTE_MSGS"))
  (shadow 'HumanAttributes (find-package "SPENCER_HUMAN_ATTRIBUTE_MSGS")))
(unless (find-package "SPENCER_HUMAN_ATTRIBUTE_MSGS::HUMANATTRIBUTES")
  (make-package "SPENCER_HUMAN_ATTRIBUTE_MSGS::HUMANATTRIBUTES"))

(in-package "ROS")
;;//! \htmlinclude HumanAttributes.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass spencer_human_attribute_msgs::HumanAttributes
  :super ros::object
  :slots (_header _categoricalAttributes _scalarAttributes ))

(defmethod spencer_human_attribute_msgs::HumanAttributes
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:categoricalAttributes __categoricalAttributes) ())
    ((:scalarAttributes __scalarAttributes) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _categoricalAttributes __categoricalAttributes)
   (setq _scalarAttributes __scalarAttributes)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:categoricalAttributes
   (&rest __categoricalAttributes)
   (if (keywordp (car __categoricalAttributes))
       (send* _categoricalAttributes __categoricalAttributes)
     (progn
       (if __categoricalAttributes (setq _categoricalAttributes (car __categoricalAttributes)))
       _categoricalAttributes)))
  (:scalarAttributes
   (&rest __scalarAttributes)
   (if (keywordp (car __scalarAttributes))
       (send* _scalarAttributes __scalarAttributes)
     (progn
       (if __scalarAttributes (setq _scalarAttributes (car __scalarAttributes)))
       _scalarAttributes)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; spencer_human_attribute_msgs/CategoricalAttribute[] _categoricalAttributes
    (apply #'+ (send-all _categoricalAttributes :serialization-length)) 4
    ;; spencer_human_attribute_msgs/ScalarAttribute[] _scalarAttributes
    (apply #'+ (send-all _scalarAttributes :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; spencer_human_attribute_msgs/CategoricalAttribute[] _categoricalAttributes
     (write-long (length _categoricalAttributes) s)
     (dolist (elem _categoricalAttributes)
       (send elem :serialize s)
       )
     ;; spencer_human_attribute_msgs/ScalarAttribute[] _scalarAttributes
     (write-long (length _scalarAttributes) s)
     (dolist (elem _scalarAttributes)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; spencer_human_attribute_msgs/CategoricalAttribute[] _categoricalAttributes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _categoricalAttributes (let (r) (dotimes (i n) (push (instance spencer_human_attribute_msgs::CategoricalAttribute :init) r)) r))
     (dolist (elem- _categoricalAttributes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; spencer_human_attribute_msgs/ScalarAttribute[] _scalarAttributes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _scalarAttributes (let (r) (dotimes (i n) (push (instance spencer_human_attribute_msgs::ScalarAttribute :init) r)) r))
     (dolist (elem- _scalarAttributes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get spencer_human_attribute_msgs::HumanAttributes :md5sum-) "0ce67039f788378180e089c471174c58")
(setf (get spencer_human_attribute_msgs::HumanAttributes :datatype-) "spencer_human_attribute_msgs/HumanAttributes")
(setf (get spencer_human_attribute_msgs::HumanAttributes :definition-)
      "std_msgs/Header header

# One entry per attribute type per person
CategoricalAttribute[]   categoricalAttributes
ScalarAttribute[]       scalarAttributes
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: spencer_human_attribute_msgs/CategoricalAttribute
uint64                  subject_id              # either an observation ID or a track ID (if information has been integrated over time); should be encoded in topic name
string                  type                    # type of the attribute, see constants below

string[]                values                  # values, each value also should have a confidence, highest-confidence attribute should come first
float32[]               confidences             # corresponding confidences should sum up to 1.0, highest confidence comes first


##################################################
### Constants for categorical attribute types. ###
##################################################

string      GENDER        = gender
string      AGE_GROUP     = age_group

###################################################
### Constants for categorical attribute values. ###
###################################################

string      GENDER_MALE             = male
string      GENDER_FEMALE           = female

# Age groups are based upon the categories from the \"Images Of Groups\" RGB database
string      AGE_GROUP_0_TO_2        = 0-2
string      AGE_GROUP_3_TO_7        = 3-7
string      AGE_GROUP_8_TO_12       = 8-12
string      AGE_GROUP_13_TO_19      = 13-19
string      AGE_GROUP_20_TO_36      = 20-36
string      AGE_GROUP_37_TO_65      = 37-65
string      AGE_GROUP_66_TO_99      = 66-99




================================================================================
MSG: spencer_human_attribute_msgs/ScalarAttribute
uint64                  subject_id              # either an observation ID or a track ID (if information has been integrated over time); should be encoded in topic name
string                  type                    # type of the attribute, see constants below

float32[]               values                  # values, each value also should have a confidence; highest-confidence value comes first!
float32[]               confidences             # corresponding confidences should sum up to 1.0


###########################################
### Constants for scalar attribute types. #
###########################################

string      PERSON_HEIGHT        = person_height

")



(provide :spencer_human_attribute_msgs/HumanAttributes "0ce67039f788378180e089c471174c58")


