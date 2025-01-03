;; Auto-generated. Do not edit!


(when (boundp 'pedsim_msgs::AgentForce)
  (if (not (find-package "PEDSIM_MSGS"))
    (make-package "PEDSIM_MSGS"))
  (shadow 'AgentForce (find-package "PEDSIM_MSGS")))
(unless (find-package "PEDSIM_MSGS::AGENTFORCE")
  (make-package "PEDSIM_MSGS::AGENTFORCE"))

(in-package "ROS")
;;//! \htmlinclude AgentForce.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass pedsim_msgs::AgentForce
  :super ros::object
  :slots (_desired_force _obstacle_force _social_force _group_coherence_force _group_gaze_force _group_repulsion_force _random_force ))

(defmethod pedsim_msgs::AgentForce
  (:init
   (&key
    ((:desired_force __desired_force) (instance geometry_msgs::Vector3 :init))
    ((:obstacle_force __obstacle_force) (instance geometry_msgs::Vector3 :init))
    ((:social_force __social_force) (instance geometry_msgs::Vector3 :init))
    ((:group_coherence_force __group_coherence_force) (instance geometry_msgs::Vector3 :init))
    ((:group_gaze_force __group_gaze_force) (instance geometry_msgs::Vector3 :init))
    ((:group_repulsion_force __group_repulsion_force) (instance geometry_msgs::Vector3 :init))
    ((:random_force __random_force) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _desired_force __desired_force)
   (setq _obstacle_force __obstacle_force)
   (setq _social_force __social_force)
   (setq _group_coherence_force __group_coherence_force)
   (setq _group_gaze_force __group_gaze_force)
   (setq _group_repulsion_force __group_repulsion_force)
   (setq _random_force __random_force)
   self)
  (:desired_force
   (&rest __desired_force)
   (if (keywordp (car __desired_force))
       (send* _desired_force __desired_force)
     (progn
       (if __desired_force (setq _desired_force (car __desired_force)))
       _desired_force)))
  (:obstacle_force
   (&rest __obstacle_force)
   (if (keywordp (car __obstacle_force))
       (send* _obstacle_force __obstacle_force)
     (progn
       (if __obstacle_force (setq _obstacle_force (car __obstacle_force)))
       _obstacle_force)))
  (:social_force
   (&rest __social_force)
   (if (keywordp (car __social_force))
       (send* _social_force __social_force)
     (progn
       (if __social_force (setq _social_force (car __social_force)))
       _social_force)))
  (:group_coherence_force
   (&rest __group_coherence_force)
   (if (keywordp (car __group_coherence_force))
       (send* _group_coherence_force __group_coherence_force)
     (progn
       (if __group_coherence_force (setq _group_coherence_force (car __group_coherence_force)))
       _group_coherence_force)))
  (:group_gaze_force
   (&rest __group_gaze_force)
   (if (keywordp (car __group_gaze_force))
       (send* _group_gaze_force __group_gaze_force)
     (progn
       (if __group_gaze_force (setq _group_gaze_force (car __group_gaze_force)))
       _group_gaze_force)))
  (:group_repulsion_force
   (&rest __group_repulsion_force)
   (if (keywordp (car __group_repulsion_force))
       (send* _group_repulsion_force __group_repulsion_force)
     (progn
       (if __group_repulsion_force (setq _group_repulsion_force (car __group_repulsion_force)))
       _group_repulsion_force)))
  (:random_force
   (&rest __random_force)
   (if (keywordp (car __random_force))
       (send* _random_force __random_force)
     (progn
       (if __random_force (setq _random_force (car __random_force)))
       _random_force)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Vector3 _desired_force
    (send _desired_force :serialization-length)
    ;; geometry_msgs/Vector3 _obstacle_force
    (send _obstacle_force :serialization-length)
    ;; geometry_msgs/Vector3 _social_force
    (send _social_force :serialization-length)
    ;; geometry_msgs/Vector3 _group_coherence_force
    (send _group_coherence_force :serialization-length)
    ;; geometry_msgs/Vector3 _group_gaze_force
    (send _group_gaze_force :serialization-length)
    ;; geometry_msgs/Vector3 _group_repulsion_force
    (send _group_repulsion_force :serialization-length)
    ;; geometry_msgs/Vector3 _random_force
    (send _random_force :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Vector3 _desired_force
       (send _desired_force :serialize s)
     ;; geometry_msgs/Vector3 _obstacle_force
       (send _obstacle_force :serialize s)
     ;; geometry_msgs/Vector3 _social_force
       (send _social_force :serialize s)
     ;; geometry_msgs/Vector3 _group_coherence_force
       (send _group_coherence_force :serialize s)
     ;; geometry_msgs/Vector3 _group_gaze_force
       (send _group_gaze_force :serialize s)
     ;; geometry_msgs/Vector3 _group_repulsion_force
       (send _group_repulsion_force :serialize s)
     ;; geometry_msgs/Vector3 _random_force
       (send _random_force :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Vector3 _desired_force
     (send _desired_force :deserialize buf ptr-) (incf ptr- (send _desired_force :serialization-length))
   ;; geometry_msgs/Vector3 _obstacle_force
     (send _obstacle_force :deserialize buf ptr-) (incf ptr- (send _obstacle_force :serialization-length))
   ;; geometry_msgs/Vector3 _social_force
     (send _social_force :deserialize buf ptr-) (incf ptr- (send _social_force :serialization-length))
   ;; geometry_msgs/Vector3 _group_coherence_force
     (send _group_coherence_force :deserialize buf ptr-) (incf ptr- (send _group_coherence_force :serialization-length))
   ;; geometry_msgs/Vector3 _group_gaze_force
     (send _group_gaze_force :deserialize buf ptr-) (incf ptr- (send _group_gaze_force :serialization-length))
   ;; geometry_msgs/Vector3 _group_repulsion_force
     (send _group_repulsion_force :deserialize buf ptr-) (incf ptr- (send _group_repulsion_force :serialization-length))
   ;; geometry_msgs/Vector3 _random_force
     (send _random_force :deserialize buf ptr-) (incf ptr- (send _random_force :serialization-length))
   ;;
   self)
  )

(setf (get pedsim_msgs::AgentForce :md5sum-) "dcd8d82cea8453731000bbf59474a5b8")
(setf (get pedsim_msgs::AgentForce :datatype-) "pedsim_msgs/AgentForce")
(setf (get pedsim_msgs::AgentForce :definition-)
      "# Forces acting on an agent.

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
")



(provide :pedsim_msgs/AgentForce "dcd8d82cea8453731000bbf59474a5b8")


