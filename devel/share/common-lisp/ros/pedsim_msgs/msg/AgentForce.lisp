; Auto-generated. Do not edit!


(cl:in-package pedsim_msgs-msg)


;//! \htmlinclude AgentForce.msg.html

(cl:defclass <AgentForce> (roslisp-msg-protocol:ros-message)
  ((desired_force
    :reader desired_force
    :initarg :desired_force
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (obstacle_force
    :reader obstacle_force
    :initarg :obstacle_force
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (social_force
    :reader social_force
    :initarg :social_force
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (group_coherence_force
    :reader group_coherence_force
    :initarg :group_coherence_force
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (group_gaze_force
    :reader group_gaze_force
    :initarg :group_gaze_force
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (group_repulsion_force
    :reader group_repulsion_force
    :initarg :group_repulsion_force
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (random_force
    :reader random_force
    :initarg :random_force
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass AgentForce (<AgentForce>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AgentForce>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AgentForce)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pedsim_msgs-msg:<AgentForce> is deprecated: use pedsim_msgs-msg:AgentForce instead.")))

(cl:ensure-generic-function 'desired_force-val :lambda-list '(m))
(cl:defmethod desired_force-val ((m <AgentForce>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:desired_force-val is deprecated.  Use pedsim_msgs-msg:desired_force instead.")
  (desired_force m))

(cl:ensure-generic-function 'obstacle_force-val :lambda-list '(m))
(cl:defmethod obstacle_force-val ((m <AgentForce>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:obstacle_force-val is deprecated.  Use pedsim_msgs-msg:obstacle_force instead.")
  (obstacle_force m))

(cl:ensure-generic-function 'social_force-val :lambda-list '(m))
(cl:defmethod social_force-val ((m <AgentForce>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:social_force-val is deprecated.  Use pedsim_msgs-msg:social_force instead.")
  (social_force m))

(cl:ensure-generic-function 'group_coherence_force-val :lambda-list '(m))
(cl:defmethod group_coherence_force-val ((m <AgentForce>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:group_coherence_force-val is deprecated.  Use pedsim_msgs-msg:group_coherence_force instead.")
  (group_coherence_force m))

(cl:ensure-generic-function 'group_gaze_force-val :lambda-list '(m))
(cl:defmethod group_gaze_force-val ((m <AgentForce>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:group_gaze_force-val is deprecated.  Use pedsim_msgs-msg:group_gaze_force instead.")
  (group_gaze_force m))

(cl:ensure-generic-function 'group_repulsion_force-val :lambda-list '(m))
(cl:defmethod group_repulsion_force-val ((m <AgentForce>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:group_repulsion_force-val is deprecated.  Use pedsim_msgs-msg:group_repulsion_force instead.")
  (group_repulsion_force m))

(cl:ensure-generic-function 'random_force-val :lambda-list '(m))
(cl:defmethod random_force-val ((m <AgentForce>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:random_force-val is deprecated.  Use pedsim_msgs-msg:random_force instead.")
  (random_force m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AgentForce>) ostream)
  "Serializes a message object of type '<AgentForce>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'desired_force) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'obstacle_force) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'social_force) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'group_coherence_force) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'group_gaze_force) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'group_repulsion_force) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'random_force) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AgentForce>) istream)
  "Deserializes a message object of type '<AgentForce>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'desired_force) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'obstacle_force) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'social_force) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'group_coherence_force) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'group_gaze_force) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'group_repulsion_force) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'random_force) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AgentForce>)))
  "Returns string type for a message object of type '<AgentForce>"
  "pedsim_msgs/AgentForce")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AgentForce)))
  "Returns string type for a message object of type 'AgentForce"
  "pedsim_msgs/AgentForce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AgentForce>)))
  "Returns md5sum for a message object of type '<AgentForce>"
  "dcd8d82cea8453731000bbf59474a5b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AgentForce)))
  "Returns md5sum for a message object of type 'AgentForce"
  "dcd8d82cea8453731000bbf59474a5b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AgentForce>)))
  "Returns full string definition for message of type '<AgentForce>"
  (cl:format cl:nil "# Forces acting on an agent.~%~%# Basic SFM forces.~%geometry_msgs/Vector3 desired_force~%geometry_msgs/Vector3 obstacle_force~%geometry_msgs/Vector3 social_force~%~%# Additional Group Forces~%geometry_msgs/Vector3 group_coherence_force~%geometry_msgs/Vector3 group_gaze_force~%geometry_msgs/Vector3 group_repulsion_force~%~%# Extra stabilization/custom forces.~%geometry_msgs/Vector3 random_force~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AgentForce)))
  "Returns full string definition for message of type 'AgentForce"
  (cl:format cl:nil "# Forces acting on an agent.~%~%# Basic SFM forces.~%geometry_msgs/Vector3 desired_force~%geometry_msgs/Vector3 obstacle_force~%geometry_msgs/Vector3 social_force~%~%# Additional Group Forces~%geometry_msgs/Vector3 group_coherence_force~%geometry_msgs/Vector3 group_gaze_force~%geometry_msgs/Vector3 group_repulsion_force~%~%# Extra stabilization/custom forces.~%geometry_msgs/Vector3 random_force~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AgentForce>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'desired_force))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'obstacle_force))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'social_force))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'group_coherence_force))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'group_gaze_force))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'group_repulsion_force))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'random_force))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AgentForce>))
  "Converts a ROS message object to a list"
  (cl:list 'AgentForce
    (cl:cons ':desired_force (desired_force msg))
    (cl:cons ':obstacle_force (obstacle_force msg))
    (cl:cons ':social_force (social_force msg))
    (cl:cons ':group_coherence_force (group_coherence_force msg))
    (cl:cons ':group_gaze_force (group_gaze_force msg))
    (cl:cons ':group_repulsion_force (group_repulsion_force msg))
    (cl:cons ':random_force (random_force msg))
))
