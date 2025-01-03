;; Auto-generated. Do not edit!


(when (boundp 'spencer_tracking_msgs::CompositeDetectedPerson)
  (if (not (find-package "SPENCER_TRACKING_MSGS"))
    (make-package "SPENCER_TRACKING_MSGS"))
  (shadow 'CompositeDetectedPerson (find-package "SPENCER_TRACKING_MSGS")))
(unless (find-package "SPENCER_TRACKING_MSGS::COMPOSITEDETECTEDPERSON")
  (make-package "SPENCER_TRACKING_MSGS::COMPOSITEDETECTEDPERSON"))

(in-package "ROS")
;;//! \htmlinclude CompositeDetectedPerson.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass spencer_tracking_msgs::CompositeDetectedPerson
  :super ros::object
  :slots (_composite_detection_id _mean_confidence _max_confidence _min_confidence _pose _original_detections ))

(defmethod spencer_tracking_msgs::CompositeDetectedPerson
  (:init
   (&key
    ((:composite_detection_id __composite_detection_id) 0)
    ((:mean_confidence __mean_confidence) 0.0)
    ((:max_confidence __max_confidence) 0.0)
    ((:min_confidence __min_confidence) 0.0)
    ((:pose __pose) (instance geometry_msgs::PoseWithCovariance :init))
    ((:original_detections __original_detections) ())
    )
   (send-super :init)
   (setq _composite_detection_id (round __composite_detection_id))
   (setq _mean_confidence (float __mean_confidence))
   (setq _max_confidence (float __max_confidence))
   (setq _min_confidence (float __min_confidence))
   (setq _pose __pose)
   (setq _original_detections __original_detections)
   self)
  (:composite_detection_id
   (&optional __composite_detection_id)
   (if __composite_detection_id (setq _composite_detection_id __composite_detection_id)) _composite_detection_id)
  (:mean_confidence
   (&optional __mean_confidence)
   (if __mean_confidence (setq _mean_confidence __mean_confidence)) _mean_confidence)
  (:max_confidence
   (&optional __max_confidence)
   (if __max_confidence (setq _max_confidence __max_confidence)) _max_confidence)
  (:min_confidence
   (&optional __min_confidence)
   (if __min_confidence (setq _min_confidence __min_confidence)) _min_confidence)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:original_detections
   (&rest __original_detections)
   (if (keywordp (car __original_detections))
       (send* _original_detections __original_detections)
     (progn
       (if __original_detections (setq _original_detections (car __original_detections)))
       _original_detections)))
  (:serialization-length
   ()
   (+
    ;; uint64 _composite_detection_id
    8
    ;; float64 _mean_confidence
    8
    ;; float64 _max_confidence
    8
    ;; float64 _min_confidence
    8
    ;; geometry_msgs/PoseWithCovariance _pose
    (send _pose :serialization-length)
    ;; spencer_tracking_msgs/DetectedPerson[] _original_detections
    (apply #'+ (send-all _original_detections :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint64 _composite_detection_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _composite_detection_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _composite_detection_id) (= (length (_composite_detection_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_composite_detection_id . bv) 0) 0) s)
              (write-long (ash (elt (_composite_detection_id . bv) 1) -1) s))
             ((and (class _composite_detection_id) (= (length (_composite_detection_id . bv)) 1)) ;; big1
              (write-long (elt (_composite_detection_id . bv) 0) s)
              (write-long (if (>= _composite_detection_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _composite_detection_id s)(write-long (if (>= _composite_detection_id 0) 0 #xffffffff) s)))
     ;; float64 _mean_confidence
       (sys::poke _mean_confidence (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _max_confidence
       (sys::poke _max_confidence (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _min_confidence
       (sys::poke _min_confidence (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/PoseWithCovariance _pose
       (send _pose :serialize s)
     ;; spencer_tracking_msgs/DetectedPerson[] _original_detections
     (write-long (length _original_detections) s)
     (dolist (elem _original_detections)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint64 _composite_detection_id
     
#+(or :alpha :irix6 :x86_64)
      (setf _composite_detection_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _composite_detection_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _mean_confidence
     (setq _mean_confidence (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _max_confidence
     (setq _max_confidence (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _min_confidence
     (setq _min_confidence (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/PoseWithCovariance _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; spencer_tracking_msgs/DetectedPerson[] _original_detections
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _original_detections (let (r) (dotimes (i n) (push (instance spencer_tracking_msgs::DetectedPerson :init) r)) r))
     (dolist (elem- _original_detections)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get spencer_tracking_msgs::CompositeDetectedPerson :md5sum-) "10e83f06a9bfbf6da1ae6f0fcdbe2cc4")
(setf (get spencer_tracking_msgs::CompositeDetectedPerson :datatype-) "spencer_tracking_msgs/CompositeDetectedPerson")
(setf (get spencer_tracking_msgs::CompositeDetectedPerson :definition-)
      "# Specifies which detected persons have been merged into a composite detection by the spencer_detected_person_association module.

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



(provide :spencer_tracking_msgs/CompositeDetectedPerson "10e83f06a9bfbf6da1ae6f0fcdbe2cc4")


