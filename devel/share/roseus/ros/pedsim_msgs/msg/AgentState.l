;; Auto-generated. Do not edit!


(when (boundp 'pedsim_msgs::AgentState)
  (if (not (find-package "PEDSIM_MSGS"))
    (make-package "PEDSIM_MSGS"))
  (shadow 'AgentState (find-package "PEDSIM_MSGS")))
(unless (find-package "PEDSIM_MSGS::AGENTSTATE")
  (make-package "PEDSIM_MSGS::AGENTSTATE"))

(in-package "ROS")
;;//! \htmlinclude AgentState.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*TYPE_STANDING*" (find-package "PEDSIM_MSGS::AGENTSTATE"))
(shadow '*TYPE_STANDING* (find-package "PEDSIM_MSGS::AGENTSTATE"))
(defconstant pedsim_msgs::AgentState::*TYPE_STANDING* "\"standing\"")
(intern "*TYPE_INDIVIDUAL_MOVING*" (find-package "PEDSIM_MSGS::AGENTSTATE"))
(shadow '*TYPE_INDIVIDUAL_MOVING* (find-package "PEDSIM_MSGS::AGENTSTATE"))
(defconstant pedsim_msgs::AgentState::*TYPE_INDIVIDUAL_MOVING* "\"individual_moving\"")
(intern "*TYPE_WAITING_IN_QUEUE*" (find-package "PEDSIM_MSGS::AGENTSTATE"))
(shadow '*TYPE_WAITING_IN_QUEUE* (find-package "PEDSIM_MSGS::AGENTSTATE"))
(defconstant pedsim_msgs::AgentState::*TYPE_WAITING_IN_QUEUE* "\"waiting_in_queue\"")
(intern "*TYPE_GROUP_MOVING*" (find-package "PEDSIM_MSGS::AGENTSTATE"))
(shadow '*TYPE_GROUP_MOVING* (find-package "PEDSIM_MSGS::AGENTSTATE"))
(defconstant pedsim_msgs::AgentState::*TYPE_GROUP_MOVING* "\"group_moving\"")

(defun pedsim_msgs::AgentState-to-symbol (const)
  (cond
        (t nil)))

(defclass pedsim_msgs::AgentState
  :super ros::object
  :slots (_header _id _type _social_state _pose _twist _forces ))

(defmethod pedsim_msgs::AgentState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:id __id) 0)
    ((:type __type) 0)
    ((:social_state __social_state) "")
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:twist __twist) (instance geometry_msgs::Twist :init))
    ((:forces __forces) (instance pedsim_msgs::AgentForce :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _id (round __id))
   (setq _type (round __type))
   (setq _social_state (string __social_state))
   (setq _pose __pose)
   (setq _twist __twist)
   (setq _forces __forces)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:social_state
   (&optional __social_state)
   (if __social_state (setq _social_state __social_state)) _social_state)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:twist
   (&rest __twist)
   (if (keywordp (car __twist))
       (send* _twist __twist)
     (progn
       (if __twist (setq _twist (car __twist)))
       _twist)))
  (:forces
   (&rest __forces)
   (if (keywordp (car __forces))
       (send* _forces __forces)
     (progn
       (if __forces (setq _forces (car __forces)))
       _forces)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint64 _id
    8
    ;; uint16 _type
    2
    ;; string _social_state
    4 (length _social_state)
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/Twist _twist
    (send _twist :serialization-length)
    ;; pedsim_msgs/AgentForce _forces
    (send _forces :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint64 _id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _id) (= (length (_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_id . bv) 0) 0) s)
              (write-long (ash (elt (_id . bv) 1) -1) s))
             ((and (class _id) (= (length (_id . bv)) 1)) ;; big1
              (write-long (elt (_id . bv) 0) s)
              (write-long (if (>= _id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _id s)(write-long (if (>= _id 0) 0 #xffffffff) s)))
     ;; uint16 _type
       (write-word _type s)
     ;; string _social_state
       (write-long (length _social_state) s) (princ _social_state s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; geometry_msgs/Twist _twist
       (send _twist :serialize s)
     ;; pedsim_msgs/AgentForce _forces
       (send _forces :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint64 _id
     
#+(or :alpha :irix6 :x86_64)
      (setf _id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint16 _type
     (setq _type (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; string _social_state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _social_state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/Twist _twist
     (send _twist :deserialize buf ptr-) (incf ptr- (send _twist :serialization-length))
   ;; pedsim_msgs/AgentForce _forces
     (send _forces :deserialize buf ptr-) (incf ptr- (send _forces :serialization-length))
   ;;
   self)
  )

(setf (get pedsim_msgs::AgentState :md5sum-) "b47b93bd58ccba0e6f058fd6a71a4915")
(setf (get pedsim_msgs::AgentState :datatype-) "pedsim_msgs/AgentState")
(setf (get pedsim_msgs::AgentState :definition-)
      "Header header
uint64 id
uint16 type
string social_state
geometry_msgs/Pose pose
geometry_msgs/Twist twist
pedsim_msgs/AgentForce forces

# Use sensors package to control observability

# Social State string constants
string      TYPE_STANDING = \"standing\"
string      TYPE_INDIVIDUAL_MOVING = \"individual_moving\"
string      TYPE_WAITING_IN_QUEUE = \"waiting_in_queue\"
string      TYPE_GROUP_MOVING = \"group_moving\"


# Agent types
# 0, 1 -> ordinary agents
# 2 -> Robot
# 3 -> standing/elderly agents

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

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: pedsim_msgs/AgentForce
# Forces acting on an agent.

# Basic SFM forces.
geometry_msgs/Vector3 desired_force
geometry_msgs/Vector3 obstacle_force
geometry_msgs/Vector3 social_force

# Additional Group Forces
geometry_msgs/Vector3 group_coherence_force
geometry_msgs/Vector3 group_gaze_force
geometry_msgs/Vector3 group_repulsion_force

# Extra stabilization/custom forces.
geometry_msgs/Vector3 random_force

")



(provide :pedsim_msgs/AgentState "b47b93bd58ccba0e6f058fd6a71a4915")


