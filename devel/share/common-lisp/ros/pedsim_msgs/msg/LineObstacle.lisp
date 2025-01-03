; Auto-generated. Do not edit!


(cl:in-package pedsim_msgs-msg)


;//! \htmlinclude LineObstacle.msg.html

(cl:defclass <LineObstacle> (roslisp-msg-protocol:ros-message)
  ((start
    :reader start
    :initarg :start
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (end
    :reader end
    :initarg :end
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass LineObstacle (<LineObstacle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LineObstacle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LineObstacle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pedsim_msgs-msg:<LineObstacle> is deprecated: use pedsim_msgs-msg:LineObstacle instead.")))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <LineObstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:start-val is deprecated.  Use pedsim_msgs-msg:start instead.")
  (start m))

(cl:ensure-generic-function 'end-val :lambda-list '(m))
(cl:defmethod end-val ((m <LineObstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:end-val is deprecated.  Use pedsim_msgs-msg:end instead.")
  (end m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LineObstacle>) ostream)
  "Serializes a message object of type '<LineObstacle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'end) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LineObstacle>) istream)
  "Deserializes a message object of type '<LineObstacle>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'end) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LineObstacle>)))
  "Returns string type for a message object of type '<LineObstacle>"
  "pedsim_msgs/LineObstacle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LineObstacle)))
  "Returns string type for a message object of type 'LineObstacle"
  "pedsim_msgs/LineObstacle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LineObstacle>)))
  "Returns md5sum for a message object of type '<LineObstacle>"
  "ad6f4eea34a193d38008f1d4053cce66")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LineObstacle)))
  "Returns md5sum for a message object of type 'LineObstacle"
  "ad6f4eea34a193d38008f1d4053cce66")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LineObstacle>)))
  "Returns full string definition for message of type '<LineObstacle>"
  (cl:format cl:nil "# A line obstacle in the simulator.~%~%geometry_msgs/Point start~%geometry_msgs/Point end~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LineObstacle)))
  "Returns full string definition for message of type 'LineObstacle"
  (cl:format cl:nil "# A line obstacle in the simulator.~%~%geometry_msgs/Point start~%geometry_msgs/Point end~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LineObstacle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'end))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LineObstacle>))
  "Converts a ROS message object to a list"
  (cl:list 'LineObstacle
    (cl:cons ':start (start msg))
    (cl:cons ':end (end msg))
))
