;; Auto-generated. Do not edit!


(when (boundp 'pedsim_msgs::AgentGroup)
  (if (not (find-package "PEDSIM_MSGS"))
    (make-package "PEDSIM_MSGS"))
  (shadow 'AgentGroup (find-package "PEDSIM_MSGS")))
(unless (find-package "PEDSIM_MSGS::AGENTGROUP")
  (make-package "PEDSIM_MSGS::AGENTGROUP"))

(in-package "ROS")
;;//! \htmlinclude AgentGroup.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pedsim_msgs::AgentGroup
  :super ros::object
  :slots (_header _group_id _age _members _center_of_mass ))

(defmethod pedsim_msgs::AgentGroup
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:group_id __group_id) 0)
    ((:age __age) 0.0)
    ((:members __members) (make-array 0 :initial-element 0 :element-type :integer))
    ((:center_of_mass __center_of_mass) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _group_id (round __group_id))
   (setq _age (float __age))
   (setq _members __members)
   (setq _center_of_mass __center_of_mass)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:group_id
   (&optional __group_id)
   (if __group_id (setq _group_id __group_id)) _group_id)
  (:age
   (&optional __age)
   (if __age (setq _age __age)) _age)
  (:members
   (&optional __members)
   (if __members (setq _members __members)) _members)
  (:center_of_mass
   (&rest __center_of_mass)
   (if (keywordp (car __center_of_mass))
       (send* _center_of_mass __center_of_mass)
     (progn
       (if __center_of_mass (setq _center_of_mass (car __center_of_mass)))
       _center_of_mass)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint16 _group_id
    2
    ;; float64 _age
    8
    ;; uint64[] _members
    (* 8    (length _members)) 4
    ;; geometry_msgs/Pose _center_of_mass
    (send _center_of_mass :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint16 _group_id
       (write-word _group_id s)
     ;; float64 _age
       (sys::poke _age (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint64[] _members
     (write-long (length _members) s)
     (dotimes (i (length _members))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _members i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _members i)) (= (length ((elt _members i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _members i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _members i) . bv) 1) -1) s))
             ((and (class (elt _members i)) (= (length ((elt _members i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _members i) . bv) 0) s)
              (write-long (if (>= (elt _members i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _members i) s)(write-long (if (>= (elt _members i) 0) 0 #xffffffff) s)))
       )
     ;; geometry_msgs/Pose _center_of_mass
       (send _center_of_mass :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint16 _group_id
     (setq _group_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _age
     (setq _age (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint64[] _members
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _members (instantiate integer-vector n))
     (dotimes (i n)
     
#+(or :alpha :irix6 :x86_64)
      (setf (elt _members i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _members i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;; geometry_msgs/Pose _center_of_mass
     (send _center_of_mass :deserialize buf ptr-) (incf ptr- (send _center_of_mass :serialization-length))
   ;;
   self)
  )

(setf (get pedsim_msgs::AgentGroup :md5sum-) "5fd22ef81e8e7ee63e1028ae9b798458")
(setf (get pedsim_msgs::AgentGroup :datatype-) "pedsim_msgs/AgentGroup")
(setf (get pedsim_msgs::AgentGroup :definition-)
      "Header header
uint16 group_id
float64 age
uint64[] members
geometry_msgs/Pose center_of_mass

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :pedsim_msgs/AgentGroup "5fd22ef81e8e7ee63e1028ae9b798458")


