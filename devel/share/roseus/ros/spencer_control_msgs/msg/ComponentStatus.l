;; Auto-generated. Do not edit!


(when (boundp 'spencer_control_msgs::ComponentStatus)
  (if (not (find-package "SPENCER_CONTROL_MSGS"))
    (make-package "SPENCER_CONTROL_MSGS"))
  (shadow 'ComponentStatus (find-package "SPENCER_CONTROL_MSGS")))
(unless (find-package "SPENCER_CONTROL_MSGS::COMPONENTSTATUS")
  (make-package "SPENCER_CONTROL_MSGS::COMPONENTSTATUS"))

(in-package "ROS")
;;//! \htmlinclude ComponentStatus.msg.html


(defclass spencer_control_msgs::ComponentStatus
  :super ros::object
  :slots (_name _alive _detail ))

(defmethod spencer_control_msgs::ComponentStatus
  (:init
   (&key
    ((:name __name) "")
    ((:alive __alive) nil)
    ((:detail __detail) "")
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _alive __alive)
   (setq _detail (string __detail))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:alive
   (&optional (__alive :null))
   (if (not (eq __alive :null)) (setq _alive __alive)) _alive)
  (:detail
   (&optional __detail)
   (if __detail (setq _detail __detail)) _detail)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; bool _alive
    1
    ;; string _detail
    4 (length _detail)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; bool _alive
       (if _alive (write-byte -1 s) (write-byte 0 s))
     ;; string _detail
       (write-long (length _detail) s) (princ _detail s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _alive
     (setq _alive (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _detail
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _detail (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get spencer_control_msgs::ComponentStatus :md5sum-) "b570ce0ae5316c3ad53df1d245486902")
(setf (get spencer_control_msgs::ComponentStatus :datatype-) "spencer_control_msgs/ComponentStatus")
(setf (get spencer_control_msgs::ComponentStatus :definition-)
      "string name
bool alive
string detail
")



(provide :spencer_control_msgs/ComponentStatus "b570ce0ae5316c3ad53df1d245486902")


