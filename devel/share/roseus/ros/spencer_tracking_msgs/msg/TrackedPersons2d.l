;; Auto-generated. Do not edit!


(when (boundp 'spencer_tracking_msgs::TrackedPersons2d)
  (if (not (find-package "SPENCER_TRACKING_MSGS"))
    (make-package "SPENCER_TRACKING_MSGS"))
  (shadow 'TrackedPersons2d (find-package "SPENCER_TRACKING_MSGS")))
(unless (find-package "SPENCER_TRACKING_MSGS::TRACKEDPERSONS2D")
  (make-package "SPENCER_TRACKING_MSGS::TRACKEDPERSONS2D"))

(in-package "ROS")
;;//! \htmlinclude TrackedPersons2d.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass spencer_tracking_msgs::TrackedPersons2d
  :super ros::object
  :slots (_header _boxes ))

(defmethod spencer_tracking_msgs::TrackedPersons2d
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:boxes __boxes) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _boxes __boxes)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:boxes
   (&rest __boxes)
   (if (keywordp (car __boxes))
       (send* _boxes __boxes)
     (progn
       (if __boxes (setq _boxes (car __boxes)))
       _boxes)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; spencer_tracking_msgs/TrackedPerson2d[] _boxes
    (apply #'+ (send-all _boxes :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; spencer_tracking_msgs/TrackedPerson2d[] _boxes
     (write-long (length _boxes) s)
     (dolist (elem _boxes)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; spencer_tracking_msgs/TrackedPerson2d[] _boxes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _boxes (let (r) (dotimes (i n) (push (instance spencer_tracking_msgs::TrackedPerson2d :init) r)) r))
     (dolist (elem- _boxes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get spencer_tracking_msgs::TrackedPersons2d :md5sum-) "972b7d693ce31c3b18f092f43387621c")
(setf (get spencer_tracking_msgs::TrackedPersons2d :datatype-) "spencer_tracking_msgs/TrackedPersons2d")
(setf (get spencer_tracking_msgs::TrackedPersons2d :definition-)
      "# Message with all 2d bbox in image of currently tracked persons 
#

Header                header      # Header containing timestamp etc. of this message
TrackedPerson2d[]     boxes       # All persons that are currently being tracked (2d image bbox)

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
MSG: spencer_tracking_msgs/TrackedPerson2d
# Message defining a 2d image bbox of a tracked person
#

uint64      track_id        # unique identifier of the target, consistent over time
float32     person_height   # 3d height of person in m
int32       x               # top left corner x of 2d image bbox
int32       y               # top left corner y of 2d image bbox
uint32      w               # width of 2d image bbox
uint32      h               # height of 2d image bbox
float32     depth           # distance from the camera in m

")



(provide :spencer_tracking_msgs/TrackedPersons2d "972b7d693ce31c3b18f092f43387621c")


