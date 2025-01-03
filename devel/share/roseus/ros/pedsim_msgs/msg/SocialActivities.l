;; Auto-generated. Do not edit!


(when (boundp 'pedsim_msgs::SocialActivities)
  (if (not (find-package "PEDSIM_MSGS"))
    (make-package "PEDSIM_MSGS"))
  (shadow 'SocialActivities (find-package "PEDSIM_MSGS")))
(unless (find-package "PEDSIM_MSGS::SOCIALACTIVITIES")
  (make-package "PEDSIM_MSGS::SOCIALACTIVITIES"))

(in-package "ROS")
;;//! \htmlinclude SocialActivities.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pedsim_msgs::SocialActivities
  :super ros::object
  :slots (_header _elements ))

(defmethod pedsim_msgs::SocialActivities
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:elements __elements) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _elements __elements)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:elements
   (&rest __elements)
   (if (keywordp (car __elements))
       (send* _elements __elements)
     (progn
       (if __elements (setq _elements (car __elements)))
       _elements)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; pedsim_msgs/SocialActivity[] _elements
    (apply #'+ (send-all _elements :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; pedsim_msgs/SocialActivity[] _elements
     (write-long (length _elements) s)
     (dolist (elem _elements)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; pedsim_msgs/SocialActivity[] _elements
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _elements (let (r) (dotimes (i n) (push (instance pedsim_msgs::SocialActivity :init) r)) r))
     (dolist (elem- _elements)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get pedsim_msgs::SocialActivities :md5sum-) "ca56b183969705615c620702275b430f")
(setf (get pedsim_msgs::SocialActivities :datatype-) "pedsim_msgs/SocialActivities")
(setf (get pedsim_msgs::SocialActivities :definition-)
      "std_msgs/Header     header

# All social activities that have been detected in the current time step,
# within sensor range of the robot.
SocialActivity[]    elements
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
MSG: pedsim_msgs/SocialActivity
string      type        # see constants below
float32     confidence  # detection confidence

uint64[]      track_ids   # IDs of all person tracks involved in the activity, might be one or multiple


# Constants for social activity type (just examples at the moment)
string      TYPE_SHOPPING = shopping
string      TYPE_STANDING = standing
string      TYPE_INDIVIDUAL_MOVING = individual_moving
string      TYPE_WAITING_IN_QUEUE = waiting_in_queue
string      TYPE_LOOKING_AT_INFORMATION_SCREEN = looking_at_information_screen
string      TYPE_LOOKING_AT_KIOSK = looking_at_kiosk
string      TYPE_GROUP_ASSEMBLING = group_assembling
string      TYPE_GROUP_MOVING = group_moving
string      TYPE_FLOW_WITH_ROBOT = flow
string      TYPE_ANTIFLOW_AGAINST_ROBOT = antiflow
string      TYPE_WAITING_FOR_OTHERS = waiting_for_others

#string      TYPE_COMMUNICATING = communicating
#string      TYPE_TAKING_PHOTOGRAPH = taking_photograph
#string      TYPE_TALKING_ON_PHONE = talking_on_phone

")



(provide :pedsim_msgs/SocialActivities "ca56b183969705615c620702275b430f")


