;; Auto-generated. Do not edit!


(when (boundp 'spencer_social_relation_msgs::SocialRelation)
  (if (not (find-package "SPENCER_SOCIAL_RELATION_MSGS"))
    (make-package "SPENCER_SOCIAL_RELATION_MSGS"))
  (shadow 'SocialRelation (find-package "SPENCER_SOCIAL_RELATION_MSGS")))
(unless (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALRELATION")
  (make-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALRELATION"))

(in-package "ROS")
;;//! \htmlinclude SocialRelation.msg.html


(intern "*TYPE_SPATIAL*" (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALRELATION"))
(shadow '*TYPE_SPATIAL* (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALRELATION"))
(defconstant spencer_social_relation_msgs::SocialRelation::*TYPE_SPATIAL* "\"spatial\"")
(intern "*TYPE_ROMANTIC*" (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALRELATION"))
(shadow '*TYPE_ROMANTIC* (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALRELATION"))
(defconstant spencer_social_relation_msgs::SocialRelation::*TYPE_ROMANTIC* "\"romantic\"")
(intern "*TYPE_PARENT_CHILD*" (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALRELATION"))
(shadow '*TYPE_PARENT_CHILD* (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALRELATION"))
(defconstant spencer_social_relation_msgs::SocialRelation::*TYPE_PARENT_CHILD* "\"parent_child\"")
(intern "*TYPE_FRIENDSHIP*" (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALRELATION"))
(shadow '*TYPE_FRIENDSHIP* (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALRELATION"))
(defconstant spencer_social_relation_msgs::SocialRelation::*TYPE_FRIENDSHIP* "\"friendship\"")

(defun spencer_social_relation_msgs::SocialRelation-to-symbol (const)
  (cond
        (t nil)))

(defclass spencer_social_relation_msgs::SocialRelation
  :super ros::object
  :slots (_type _strength _track1_id _track2_id ))

(defmethod spencer_social_relation_msgs::SocialRelation
  (:init
   (&key
    ((:type __type) "")
    ((:strength __strength) 0.0)
    ((:track1_id __track1_id) 0)
    ((:track2_id __track2_id) 0)
    )
   (send-super :init)
   (setq _type (string __type))
   (setq _strength (float __strength))
   (setq _track1_id (round __track1_id))
   (setq _track2_id (round __track2_id))
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:strength
   (&optional __strength)
   (if __strength (setq _strength __strength)) _strength)
  (:track1_id
   (&optional __track1_id)
   (if __track1_id (setq _track1_id __track1_id)) _track1_id)
  (:track2_id
   (&optional __track2_id)
   (if __track2_id (setq _track2_id __track2_id)) _track2_id)
  (:serialization-length
   ()
   (+
    ;; string _type
    4 (length _type)
    ;; float32 _strength
    4
    ;; uint32 _track1_id
    4
    ;; uint32 _track2_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _type
       (write-long (length _type) s) (princ _type s)
     ;; float32 _strength
       (sys::poke _strength (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _track1_id
       (write-long _track1_id s)
     ;; uint32 _track2_id
       (write-long _track2_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _strength
     (setq _strength (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _track1_id
     (setq _track1_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _track2_id
     (setq _track2_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get spencer_social_relation_msgs::SocialRelation :md5sum-) "004e9c919342c749d66dbc051dba51f6")
(setf (get spencer_social_relation_msgs::SocialRelation :datatype-) "spencer_social_relation_msgs/SocialRelation")
(setf (get spencer_social_relation_msgs::SocialRelation :definition-)
      "string      type        # e.g. mother-son relationship, romantic relationship, etc.
float32     strength    # relationship strength between 0.0 and 1.0

uint32      track1_id
uint32      track2_id


# Constants for type (just examples at the moment)
string      TYPE_SPATIAL  = \"spatial\"
string      TYPE_ROMANTIC = \"romantic\"
string      TYPE_PARENT_CHILD = \"parent_child\"
string      TYPE_FRIENDSHIP = \"friendship\"
")



(provide :spencer_social_relation_msgs/SocialRelation "004e9c919342c749d66dbc051dba51f6")


