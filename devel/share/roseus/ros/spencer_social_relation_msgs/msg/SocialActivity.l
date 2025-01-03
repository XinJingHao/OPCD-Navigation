;; Auto-generated. Do not edit!


(when (boundp 'spencer_social_relation_msgs::SocialActivity)
  (if (not (find-package "SPENCER_SOCIAL_RELATION_MSGS"))
    (make-package "SPENCER_SOCIAL_RELATION_MSGS"))
  (shadow 'SocialActivity (find-package "SPENCER_SOCIAL_RELATION_MSGS")))
(unless (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY")
  (make-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))

(in-package "ROS")
;;//! \htmlinclude SocialActivity.msg.html


(intern "*TYPE_SHOPPING*" (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(shadow '*TYPE_SHOPPING* (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(defconstant spencer_social_relation_msgs::SocialActivity::*TYPE_SHOPPING* "shopping")
(intern "*TYPE_STANDING*" (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(shadow '*TYPE_STANDING* (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(defconstant spencer_social_relation_msgs::SocialActivity::*TYPE_STANDING* "standing")
(intern "*TYPE_INDIVIDUAL_MOVING*" (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(shadow '*TYPE_INDIVIDUAL_MOVING* (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(defconstant spencer_social_relation_msgs::SocialActivity::*TYPE_INDIVIDUAL_MOVING* "individual_moving")
(intern "*TYPE_WAITING_IN_QUEUE*" (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(shadow '*TYPE_WAITING_IN_QUEUE* (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(defconstant spencer_social_relation_msgs::SocialActivity::*TYPE_WAITING_IN_QUEUE* "waiting_in_queue")
(intern "*TYPE_LOOKING_AT_INFORMATION_SCREEN*" (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(shadow '*TYPE_LOOKING_AT_INFORMATION_SCREEN* (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(defconstant spencer_social_relation_msgs::SocialActivity::*TYPE_LOOKING_AT_INFORMATION_SCREEN* "looking_at_information_screen")
(intern "*TYPE_LOOKING_AT_KIOSK*" (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(shadow '*TYPE_LOOKING_AT_KIOSK* (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(defconstant spencer_social_relation_msgs::SocialActivity::*TYPE_LOOKING_AT_KIOSK* "looking_at_kiosk")
(intern "*TYPE_GROUP_ASSEMBLING*" (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(shadow '*TYPE_GROUP_ASSEMBLING* (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(defconstant spencer_social_relation_msgs::SocialActivity::*TYPE_GROUP_ASSEMBLING* "group_assembling")
(intern "*TYPE_GROUP_MOVING*" (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(shadow '*TYPE_GROUP_MOVING* (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(defconstant spencer_social_relation_msgs::SocialActivity::*TYPE_GROUP_MOVING* "group_moving")
(intern "*TYPE_FLOW_WITH_ROBOT*" (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(shadow '*TYPE_FLOW_WITH_ROBOT* (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(defconstant spencer_social_relation_msgs::SocialActivity::*TYPE_FLOW_WITH_ROBOT* "flow")
(intern "*TYPE_ANTIFLOW_AGAINST_ROBOT*" (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(shadow '*TYPE_ANTIFLOW_AGAINST_ROBOT* (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(defconstant spencer_social_relation_msgs::SocialActivity::*TYPE_ANTIFLOW_AGAINST_ROBOT* "antiflow")
(intern "*TYPE_WAITING_FOR_OTHERS*" (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(shadow '*TYPE_WAITING_FOR_OTHERS* (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(defconstant spencer_social_relation_msgs::SocialActivity::*TYPE_WAITING_FOR_OTHERS* "waiting_for_others")
(intern "*TYPE_LOOKING_FOR_HELP*" (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(shadow '*TYPE_LOOKING_FOR_HELP* (find-package "SPENCER_SOCIAL_RELATION_MSGS::SOCIALACTIVITY"))
(defconstant spencer_social_relation_msgs::SocialActivity::*TYPE_LOOKING_FOR_HELP* "looking_for_help")

(defun spencer_social_relation_msgs::SocialActivity-to-symbol (const)
  (cond
        (t nil)))

(defclass spencer_social_relation_msgs::SocialActivity
  :super ros::object
  :slots (_type _confidence _track_ids ))

(defmethod spencer_social_relation_msgs::SocialActivity
  (:init
   (&key
    ((:type __type) "")
    ((:confidence __confidence) 0.0)
    ((:track_ids __track_ids) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _type (string __type))
   (setq _confidence (float __confidence))
   (setq _track_ids __track_ids)
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:track_ids
   (&optional __track_ids)
   (if __track_ids (setq _track_ids __track_ids)) _track_ids)
  (:serialization-length
   ()
   (+
    ;; string _type
    4 (length _type)
    ;; float32 _confidence
    4
    ;; uint64[] _track_ids
    (* 8    (length _track_ids)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _type
       (write-long (length _type) s) (princ _type s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint64[] _track_ids
     (write-long (length _track_ids) s)
     (dotimes (i (length _track_ids))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _track_ids i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _track_ids i)) (= (length ((elt _track_ids i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _track_ids i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _track_ids i) . bv) 1) -1) s))
             ((and (class (elt _track_ids i)) (= (length ((elt _track_ids i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _track_ids i) . bv) 0) s)
              (write-long (if (>= (elt _track_ids i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _track_ids i) s)(write-long (if (>= (elt _track_ids i) 0) 0 #xffffffff) s)))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint64[] _track_ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _track_ids (instantiate integer-vector n))
     (dotimes (i n)
     
#+(or :alpha :irix6 :x86_64)
      (setf (elt _track_ids i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _track_ids i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;;
   self)
  )

(setf (get spencer_social_relation_msgs::SocialActivity :md5sum-) "b2c2f6733618189901fea0ddf041096b")
(setf (get spencer_social_relation_msgs::SocialActivity :datatype-) "spencer_social_relation_msgs/SocialActivity")
(setf (get spencer_social_relation_msgs::SocialActivity :definition-)
      "string      type        # see constants below
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
string      TYPE_LOOKING_FOR_HELP = looking_for_help


#string      TYPE_COMMUNICATING = communicating
#string      TYPE_TAKING_PHOTOGRAPH = taking_photograph
#string      TYPE_TALKING_ON_PHONE = talking_on_phone

")



(provide :spencer_social_relation_msgs/SocialActivity "b2c2f6733618189901fea0ddf041096b")


