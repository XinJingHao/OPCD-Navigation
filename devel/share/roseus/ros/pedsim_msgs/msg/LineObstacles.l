;; Auto-generated. Do not edit!


(when (boundp 'pedsim_msgs::LineObstacles)
  (if (not (find-package "PEDSIM_MSGS"))
    (make-package "PEDSIM_MSGS"))
  (shadow 'LineObstacles (find-package "PEDSIM_MSGS")))
(unless (find-package "PEDSIM_MSGS::LINEOBSTACLES")
  (make-package "PEDSIM_MSGS::LINEOBSTACLES"))

(in-package "ROS")
;;//! \htmlinclude LineObstacles.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pedsim_msgs::LineObstacles
  :super ros::object
  :slots (_header _obstacles ))

(defmethod pedsim_msgs::LineObstacles
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:obstacles __obstacles) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _obstacles __obstacles)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:obstacles
   (&rest __obstacles)
   (if (keywordp (car __obstacles))
       (send* _obstacles __obstacles)
     (progn
       (if __obstacles (setq _obstacles (car __obstacles)))
       _obstacles)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; pedsim_msgs/LineObstacle[] _obstacles
    (apply #'+ (send-all _obstacles :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; pedsim_msgs/LineObstacle[] _obstacles
     (write-long (length _obstacles) s)
     (dolist (elem _obstacles)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; pedsim_msgs/LineObstacle[] _obstacles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacles (let (r) (dotimes (i n) (push (instance pedsim_msgs::LineObstacle :init) r)) r))
     (dolist (elem- _obstacles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get pedsim_msgs::LineObstacles :md5sum-) "4de3122fdaa1292012d39892365813ee")
(setf (get pedsim_msgs::LineObstacles :datatype-) "pedsim_msgs/LineObstacles")
(setf (get pedsim_msgs::LineObstacles :definition-)
      "# A collection of line obstacles.
# No need to header since these are detemined at sim initiation time.
Header header
pedsim_msgs/LineObstacle[] obstacles

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
MSG: pedsim_msgs/LineObstacle
# A line obstacle in the simulator.

geometry_msgs/Point start
geometry_msgs/Point end

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :pedsim_msgs/LineObstacles "4de3122fdaa1292012d39892365813ee")


