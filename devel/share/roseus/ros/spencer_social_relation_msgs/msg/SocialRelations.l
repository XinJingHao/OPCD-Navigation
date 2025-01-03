;; Auto-generated. Do not edit!


(when (boundp 'spencer_social_relation_msgs::SocialRelations)
  (if (not (find-package "SPENCER_SOCIAL_RELATION_MSGS"))
    (make-package "SPENCER_SOCIAL_RELATION_MSGS"))
  (shadow 'SocialRelations (find-package "SPENCER_SOCIAL_RELATION_MSGS")))
(unless (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALRELATIONS")
  (make-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALRELATIONS"))

(in-package "ROS")
;;//! \htmlinclude SocialRelations.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass spencer_social_relation_msgs::SocialRelations
  :super ros::object
  :slots (_header _elements ))

(defmethod spencer_social_relation_msgs::SocialRelations
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
    ;; spencer_social_relation_msgs/SocialRelation[] _elements
    (apply #'+ (send-all _elements :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; spencer_social_relation_msgs/SocialRelation[] _elements
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
   ;; spencer_social_relation_msgs/SocialRelation[] _elements
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _elements (let (r) (dotimes (i n) (push (instance spencer_social_relation_msgs::SocialRelation :init) r)) r))
     (dolist (elem- _elements)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get spencer_social_relation_msgs::SocialRelations :md5sum-) "433b8d0d73396f7d640c03dcb80bb4fe")
(setf (get spencer_social_relation_msgs::SocialRelations :datatype-) "spencer_social_relation_msgs/SocialRelations")
(setf (get spencer_social_relation_msgs::SocialRelations :definition-)
      "std_msgs/Header     header

# All social relations of all tracks in the current time step.
# There might be multiple social relations per pair of tracks.
SocialRelation[]    elements
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
MSG: spencer_social_relation_msgs/SocialRelation
string      type        # e.g. mother-son relationship, romantic relationship, etc.
float32     strength    # relationship strength between 0.0 and 1.0

uint32      track1_id
uint32      track2_id


# Constants for type (just examples at the moment)
string      TYPE_SPATIAL  = \"spatial\"
string      TYPE_ROMANTIC = \"romantic\"
string      TYPE_PARENT_CHILD = \"parent_child\"
string      TYPE_FRIENDSHIP = \"friendship\"
")



(provide :spencer_social_relation_msgs/SocialRelations "433b8d0d73396f7d640c03dcb80bb4fe")


