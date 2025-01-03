;; Auto-generated. Do not edit!


(when (boundp 'pedsim_msgs::Waypoint)
  (if (not (find-package "PEDSIM_MSGS"))
    (make-package "PEDSIM_MSGS"))
  (shadow 'Waypoint (find-package "PEDSIM_MSGS")))
(unless (find-package "PEDSIM_MSGS::WAYPOINT")
  (make-package "PEDSIM_MSGS::WAYPOINT"))

(in-package "ROS")
;;//! \htmlinclude Waypoint.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(intern "*BHV_SIMPLE*" (find-package "PEDSIM_MSGS::WAYPOINT"))
(shadow '*BHV_SIMPLE* (find-package "PEDSIM_MSGS::WAYPOINT"))
(defconstant pedsim_msgs::Waypoint::*BHV_SIMPLE* 0)
(intern "*BHV_SOURCE*" (find-package "PEDSIM_MSGS::WAYPOINT"))
(shadow '*BHV_SOURCE* (find-package "PEDSIM_MSGS::WAYPOINT"))
(defconstant pedsim_msgs::Waypoint::*BHV_SOURCE* 1)
(intern "*BHV_SINK*" (find-package "PEDSIM_MSGS::WAYPOINT"))
(shadow '*BHV_SINK* (find-package "PEDSIM_MSGS::WAYPOINT"))
(defconstant pedsim_msgs::Waypoint::*BHV_SINK* 2)

(defun pedsim_msgs::Waypoint-to-symbol (const)
  (cond
        ((= const 0) 'pedsim_msgs::Waypoint::*BHV_SIMPLE*)
        ((= const 1) 'pedsim_msgs::Waypoint::*BHV_SOURCE*)
        ((= const 2) 'pedsim_msgs::Waypoint::*BHV_SINK*)
        (t nil)))

(defclass pedsim_msgs::Waypoint
  :super ros::object
  :slots (_name _behavior _position _radius ))

(defmethod pedsim_msgs::Waypoint
  (:init
   (&key
    ((:name __name) "")
    ((:behavior __behavior) 0)
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:radius __radius) 0.0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _behavior (round __behavior))
   (setq _position __position)
   (setq _radius (float __radius))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:behavior
   (&optional __behavior)
   (if __behavior (setq _behavior __behavior)) _behavior)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:radius
   (&optional __radius)
   (if __radius (setq _radius __radius)) _radius)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; int8 _behavior
    1
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; float32 _radius
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; int8 _behavior
       (write-byte _behavior s)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; float32 _radius
       (sys::poke _radius (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8 _behavior
     (setq _behavior (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _behavior 127) (setq _behavior (- _behavior 256)))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; float32 _radius
     (setq _radius (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get pedsim_msgs::Waypoint :md5sum-) "7fe6c11b241f6ddddc1c756dacf4a21f")
(setf (get pedsim_msgs::Waypoint :datatype-) "pedsim_msgs/Waypoint")
(setf (get pedsim_msgs::Waypoint :definition-)
      "int8 BHV_SIMPLE = 0
int8 BHV_SOURCE = 1
int8 BHV_SINK = 2

string name
int8 behavior
geometry_msgs/Point position
float32 radius

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :pedsim_msgs/Waypoint "7fe6c11b241f6ddddc1c756dacf4a21f")


