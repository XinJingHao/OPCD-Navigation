;; Auto-generated. Do not edit!


(when (boundp 'spencer_tracking_msgs::DetectedPersons)
  (if (not (find-package "SPENCER_TRACKING_MSGS"))
    (make-package "SPENCER_TRACKING_MSGS"))
  (shadow 'DetectedPersons (find-package "SPENCER_TRACKING_MSGS")))
(unless (find-package "SPENCER_TRACKING_MSGS::DETECTEDPERSONS")
  (make-package "SPENCER_TRACKING_MSGS::DETECTEDPERSONS"))

(in-package "ROS")
;;//! \htmlinclude DetectedPersons.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass spencer_tracking_msgs::DetectedPersons
  :super ros::object
  :slots (_header _detections ))

(defmethod spencer_tracking_msgs::DetectedPersons
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:detections __detections) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _detections __detections)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:detections
   (&rest __detections)
   (if (keywordp (car __detections))
       (send* _detections __detections)
     (progn
       (if __detections (setq _detections (car __detections)))
       _detections)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; spencer_tracking_msgs/DetectedPerson[] _detections
    (apply #'+ (send-all _detections :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; spencer_tracking_msgs/DetectedPerson[] _detections
     (write-long (length _detections) s)
     (dolist (elem _detections)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; spencer_tracking_msgs/DetectedPerson[] _detections
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detections (let (r) (dotimes (i n) (push (instance spencer_tracking_msgs::DetectedPerson :init) r)) r))
     (dolist (elem- _detections)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get spencer_tracking_msgs::DetectedPersons :md5sum-) "38e416cebb5dfd7363bde91113e7f3bd")
(setf (get spencer_tracking_msgs::DetectedPersons :datatype-) "spencer_tracking_msgs/DetectedPersons")
(setf (get spencer_tracking_msgs::DetectedPersons :definition-)
      "# Message with all currently detected persons
#

Header                  header          # Header containing timestamp etc. of this message
DetectedPerson[]        detections      # All persons that are currently being detected
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
MSG: spencer_tracking_msgs/DetectedPerson
# Message describing a detection of a person
#

# Unique id of the detection, monotonically increasing over time
uint64          detection_id

# (Pseudo-)probabilistic value between 0.0 and 1.0 describing the detector's confidence in the detection
float64         confidence

# 3D pose (position + orientation) of the *center* of the detection
# check covariance to see which dimensions are actually set! unset dimensions shall have a covariance > 9999
geometry_msgs/PoseWithCovariance    pose    

# Sensor modality / detector type, see example constants below. 
# used e.g. later in tracking to check which tracks have been visually confirmed
string          modality            

                                    
string          MODALITY_GENERIC_LASER_2D = laser2d
string          MODALITY_GENERIC_LASER_3D = laser3d
string          MODALITY_GENERIC_MONOCULAR_VISION = mono
string          MODALITY_GENERIC_STEREO_VISION = stereo
string          MODALITY_GENERIC_RGBD = rgbd

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

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



(provide :spencer_tracking_msgs/DetectedPersons "38e416cebb5dfd7363bde91113e7f3bd")


