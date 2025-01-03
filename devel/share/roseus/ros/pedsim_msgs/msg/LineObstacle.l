;; Auto-generated. Do not edit!


(when (boundp 'pedsim_msgs::LineObstacle)
  (if (not (find-package "PEDSIM_MSGS"))
    (make-package "PEDSIM_MSGS"))
  (shadow 'LineObstacle (find-package "PEDSIM_MSGS")))
(unless (find-package "PEDSIM_MSGS::LINEOBSTACLE")
  (make-package "PEDSIM_MSGS::LINEOBSTACLE"))

(in-package "ROS")
;;//! \htmlinclude LineObstacle.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass pedsim_msgs::LineObstacle
  :super ros::object
  :slots (_start _end ))

(defmethod pedsim_msgs::LineObstacle
  (:init
   (&key
    ((:start __start) (instance geometry_msgs::Point :init))
    ((:end __end) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _start __start)
   (setq _end __end)
   self)
  (:start
   (&rest __start)
   (if (keywordp (car __start))
       (send* _start __start)
     (progn
       (if __start (setq _start (car __start)))
       _start)))
  (:end
   (&rest __end)
   (if (keywordp (car __end))
       (send* _end __end)
     (progn
       (if __end (setq _end (car __end)))
       _end)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _start
    (send _start :serialization-length)
    ;; geometry_msgs/Point _end
    (send _end :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _start
       (send _start :serialize s)
     ;; geometry_msgs/Point _end
       (send _end :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _start
     (send _start :deserialize buf ptr-) (incf ptr- (send _start :serialization-length))
   ;; geometry_msgs/Point _end
     (send _end :deserialize buf ptr-) (incf ptr- (send _end :serialization-length))
   ;;
   self)
  )

(setf (get pedsim_msgs::LineObstacle :md5sum-) "ad6f4eea34a193d38008f1d4053cce66")
(setf (get pedsim_msgs::LineObstacle :datatype-) "pedsim_msgs/LineObstacle")
(setf (get pedsim_msgs::LineObstacle :definition-)
      "# A line obstacle in the simulator.

geometry_msgs/Point start
geometry_msgs/Point end

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :pedsim_msgs/LineObstacle "ad6f4eea34a193d38008f1d4053cce66")


