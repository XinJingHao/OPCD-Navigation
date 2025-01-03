;; Auto-generated. Do not edit!


(when (boundp 'spencer_tracking_msgs::TrackedPerson)
  (if (not (find-package "SPENCER_TRACKING_MSGS"))
    (make-package "SPENCER_TRACKING_MSGS"))
  (shadow 'TrackedPerson (find-package "SPENCER_TRACKING_MSGS")))
(unless (find-package "SPENCER_TRACKING_MSGS::TRACKEDPERSON")
  (make-package "SPENCER_TRACKING_MSGS::TRACKEDPERSON"))

(in-package "ROS")
;;//! \htmlinclude TrackedPerson.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass spencer_tracking_msgs::TrackedPerson
  :super ros::object
  :slots (_track_id _is_occluded _is_matched _detection_id _age _pose _twist ))

(defmethod spencer_tracking_msgs::TrackedPerson
  (:init
   (&key
    ((:track_id __track_id) 0)
    ((:is_occluded __is_occluded) nil)
    ((:is_matched __is_matched) nil)
    ((:detection_id __detection_id) 0)
    ((:age __age) (instance ros::time :init))
    ((:pose __pose) (instance geometry_msgs::PoseWithCovariance :init))
    ((:twist __twist) (instance geometry_msgs::TwistWithCovariance :init))
    )
   (send-super :init)
   (setq _track_id (round __track_id))
   (setq _is_occluded __is_occluded)
   (setq _is_matched __is_matched)
   (setq _detection_id (round __detection_id))
   (setq _age __age)
   (setq _pose __pose)
   (setq _twist __twist)
   self)
  (:track_id
   (&optional __track_id)
   (if __track_id (setq _track_id __track_id)) _track_id)
  (:is_occluded
   (&optional (__is_occluded :null))
   (if (not (eq __is_occluded :null)) (setq _is_occluded __is_occluded)) _is_occluded)
  (:is_matched
   (&optional (__is_matched :null))
   (if (not (eq __is_matched :null)) (setq _is_matched __is_matched)) _is_matched)
  (:detection_id
   (&optional __detection_id)
   (if __detection_id (setq _detection_id __detection_id)) _detection_id)
  (:age
   (&optional __age)
   (if __age (setq _age __age)) _age)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:twist
   (&rest __twist)
   (if (keywordp (car __twist))
       (send* _twist __twist)
     (progn
       (if __twist (setq _twist (car __twist)))
       _twist)))
  (:serialization-length
   ()
   (+
    ;; uint64 _track_id
    8
    ;; bool _is_occluded
    1
    ;; bool _is_matched
    1
    ;; uint64 _detection_id
    8
    ;; duration _age
    8
    ;; geometry_msgs/PoseWithCovariance _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/TwistWithCovariance _twist
    (send _twist :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint64 _track_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _track_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _track_id) (= (length (_track_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_track_id . bv) 0) 0) s)
              (write-long (ash (elt (_track_id . bv) 1) -1) s))
             ((and (class _track_id) (= (length (_track_id . bv)) 1)) ;; big1
              (write-long (elt (_track_id . bv) 0) s)
              (write-long (if (>= _track_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _track_id s)(write-long (if (>= _track_id 0) 0 #xffffffff) s)))
     ;; bool _is_occluded
       (if _is_occluded (write-byte -1 s) (write-byte 0 s))
     ;; bool _is_matched
       (if _is_matched (write-byte -1 s) (write-byte 0 s))
     ;; uint64 _detection_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _detection_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _detection_id) (= (length (_detection_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_detection_id . bv) 0) 0) s)
              (write-long (ash (elt (_detection_id . bv) 1) -1) s))
             ((and (class _detection_id) (= (length (_detection_id . bv)) 1)) ;; big1
              (write-long (elt (_detection_id . bv) 0) s)
              (write-long (if (>= _detection_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _detection_id s)(write-long (if (>= _detection_id 0) 0 #xffffffff) s)))
     ;; duration _age
       (write-long (send _age :sec) s) (write-long (send _age :nsec) s)
     ;; geometry_msgs/PoseWithCovariance _pose
       (send _pose :serialize s)
     ;; geometry_msgs/TwistWithCovariance _twist
       (send _twist :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint64 _track_id
     
#+(or :alpha :irix6 :x86_64)
      (setf _track_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _track_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; bool _is_occluded
     (setq _is_occluded (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _is_matched
     (setq _is_matched (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint64 _detection_id
     
#+(or :alpha :irix6 :x86_64)
      (setf _detection_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _detection_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; duration _age
     (send _age :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _age :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/PoseWithCovariance _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/TwistWithCovariance _twist
     (send _twist :deserialize buf ptr-) (incf ptr- (send _twist :serialization-length))
   ;;
   self)
  )

(setf (get spencer_tracking_msgs::TrackedPerson :md5sum-) "28bdd0d6d6551c668e4fde8aecdf1885")
(setf (get spencer_tracking_msgs::TrackedPerson :datatype-) "spencer_tracking_msgs/TrackedPerson")
(setf (get spencer_tracking_msgs::TrackedPerson :definition-)
      "# Message defining a tracked person
#

uint64      track_id        # unique identifier of the target, consistent over time
bool        is_occluded     # if the track is currently not observable in a physical way
bool        is_matched      # if the track is currently matched by a detection
uint64      detection_id    # id of the corresponding detection in the current cycle (undefined if occluded)
duration    age             # age of the track

# The following fields are extracted from the Kalman state x and its covariance C

geometry_msgs/PoseWithCovariance       pose   # pose of the track (z value and orientation might not be set, check if corresponding variance on diagonal is > 99999)

geometry_msgs/TwistWithCovariance   twist     # velocity of the track (z value and rotational velocities might not be set, check if corresponding variance on diagonal is > 99999)

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
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :spencer_tracking_msgs/TrackedPerson "28bdd0d6d6551c668e4fde8aecdf1885")


