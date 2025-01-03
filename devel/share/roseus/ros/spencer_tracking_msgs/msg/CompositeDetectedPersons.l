;; Auto-generated. Do not edit!


(when (boundp 'spencer_tracking_msgs::CompositeDetectedPersons)
  (if (not (find-package "SPENCER_TRACKING_MSGS"))
    (make-package "SPENCER_TRACKING_MSGS"))
  (shadow 'CompositeDetectedPersons (find-package "SPENCER_TRACKING_MSGS")))
(unless (find-package "SPENCER_TRACKING_MSGS::COMPOSITEDETECTEDPERSONS")
  (make-package "SPENCER_TRACKING_MSGS::COMPOSITEDETECTEDPERSONS"))

(in-package "ROS")
;;//! \htmlinclude CompositeDetectedPersons.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass spencer_tracking_msgs::CompositeDetectedPersons
  :super ros::object
  :slots (_header _elements ))

(defmethod spencer_tracking_msgs::CompositeDetectedPersons
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:elements __elements) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _elements __elements)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:elements
   (&rest __elements)
   (if (keywordp (car __elements))
       (send* _elements __elements)
     (progn
       (if __elements (setq _elements (car __elements)))
       _elements)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; spencer_tracking_msgs/CompositeDetectedPerson[] _elements
    (apply #'+ (send-all _elements :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; spencer_tracking_msgs/CompositeDetectedPerson[] _elements
     (write-long (length _elements) s)
     (dolist (elem _elements)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; spencer_tracking_msgs/CompositeDetectedPerson[] _elements
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _elements (let (r) (dotimes (i n) (push (instance spencer_tracking_msgs::CompositeDetectedPerson :init) r)) r))
     (dolist (elem- _elements)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get spencer_tracking_msgs::CompositeDetectedPersons :md5sum-) "c4f9f34f5cb712f010de12fc027da3e6")
(setf (get spencer_tracking_msgs::CompositeDetectedPersons :datatype-) "spencer_tracking_msgs/CompositeDetectedPersons")
(setf (get spencer_tracking_msgs::CompositeDetectedPersons :definition-)
      "# Message specifying which original detected persons (from all kinds of sensors) have been merged into one fused detection before being processed by the person tracker, in the current time step.
#
# This information is processed by the spencer_detected_person_association module, such that other perception components can associate their results (e.g. person age, gender)
# with a particular spencer_tracking_msgs/TrackedPerson (which contains a reference to a composite person detection ID).

Header                      header          # Header timestamp is set to the *latest* timestamp of all fused DetectedPerson messages.
                                            # Before fusion, all detections are transformed into a common coordinate frame (usually base_footprint).
CompositeDetectedPerson[]   elements        # Fused (merged) detected persons
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
MSG: spencer_tracking_msgs/CompositeDetectedPerson
# Specifies which detected persons have been merged into a composite detection by the spencer_detected_person_association module.

# TODO: Do we need a composite person-specific timestamp (or even a full message header including frame ID)?
# Having a separate timestamp per person could be useful if the timestamps of the merged DetectedPersons messages vary a lot,
# and some persons are only seen by a single sensor (so averaging over all input timestamps would have a detrimental effect). 

uint64      composite_detection_id          # ID of the fused detection

float64     mean_confidence                 # mean of the confidences of the original detections
float64     max_confidence                  # maximum confidence of original detections
float64     min_confidence                  # minimum confidence of original detections


geometry_msgs/PoseWithCovariance    pose    # Merged 3D pose (position + orientation) of the detection center
                                            # check covariance to see which dimensions are actually set!
                                            # unset dimensions shall have a covariance > 9999

DetectedPerson[] original_detections        # The original detections from individual sensor-specific detectors that have been combined into a composite detection
                                            # We are copying the entire DetectedPersons messages, *with poses transformed into the target frame*, such that subscribers
                                            # do not have to subscribe to all the original DetectedPersons topics.
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

")



(provide :spencer_tracking_msgs/CompositeDetectedPersons "c4f9f34f5cb712f010de12fc027da3e6")


