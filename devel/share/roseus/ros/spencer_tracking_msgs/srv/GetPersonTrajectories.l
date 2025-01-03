;; Auto-generated. Do not edit!


(when (boundp 'spencer_tracking_msgs::GetPersonTrajectories)
  (if (not (find-package "SPENCER_TRACKING_MSGS"))
    (make-package "SPENCER_TRACKING_MSGS"))
  (shadow 'GetPersonTrajectories (find-package "SPENCER_TRACKING_MSGS")))
(unless (find-package "SPENCER_TRACKING_MSGS::GETPERSONTRAJECTORIES")
  (make-package "SPENCER_TRACKING_MSGS::GETPERSONTRAJECTORIES"))
(unless (find-package "SPENCER_TRACKING_MSGS::GETPERSONTRAJECTORIESREQUEST")
  (make-package "SPENCER_TRACKING_MSGS::GETPERSONTRAJECTORIESREQUEST"))
(unless (find-package "SPENCER_TRACKING_MSGS::GETPERSONTRAJECTORIESRESPONSE")
  (make-package "SPENCER_TRACKING_MSGS::GETPERSONTRAJECTORIESRESPONSE"))

(in-package "ROS")





(defclass spencer_tracking_msgs::GetPersonTrajectoriesRequest
  :super ros::object
  :slots (_requested_ids _max_age ))

(defmethod spencer_tracking_msgs::GetPersonTrajectoriesRequest
  (:init
   (&key
    ((:requested_ids __requested_ids) (make-array 0 :initial-element 0 :element-type :integer))
    ((:max_age __max_age) (instance ros::time :init))
    )
   (send-super :init)
   (setq _requested_ids __requested_ids)
   (setq _max_age __max_age)
   self)
  (:requested_ids
   (&optional __requested_ids)
   (if __requested_ids (setq _requested_ids __requested_ids)) _requested_ids)
  (:max_age
   (&optional __max_age)
   (if __max_age (setq _max_age __max_age)) _max_age)
  (:serialization-length
   ()
   (+
    ;; uint64[] _requested_ids
    (* 8    (length _requested_ids)) 4
    ;; duration _max_age
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint64[] _requested_ids
     (write-long (length _requested_ids) s)
     (dotimes (i (length _requested_ids))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _requested_ids i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _requested_ids i)) (= (length ((elt _requested_ids i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _requested_ids i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _requested_ids i) . bv) 1) -1) s))
             ((and (class (elt _requested_ids i)) (= (length ((elt _requested_ids i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _requested_ids i) . bv) 0) s)
              (write-long (if (>= (elt _requested_ids i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _requested_ids i) s)(write-long (if (>= (elt _requested_ids i) 0) 0 #xffffffff) s)))
       )
     ;; duration _max_age
       (write-long (send _max_age :sec) s) (write-long (send _max_age :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint64[] _requested_ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _requested_ids (instantiate integer-vector n))
     (dotimes (i n)
     
#+(or :alpha :irix6 :x86_64)
      (setf (elt _requested_ids i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _requested_ids i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;; duration _max_age
     (send _max_age :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _max_age :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass spencer_tracking_msgs::GetPersonTrajectoriesResponse
  :super ros::object
  :slots (_trajectories ))

(defmethod spencer_tracking_msgs::GetPersonTrajectoriesResponse
  (:init
   (&key
    ((:trajectories __trajectories) ())
    )
   (send-super :init)
   (setq _trajectories __trajectories)
   self)
  (:trajectories
   (&rest __trajectories)
   (if (keywordp (car __trajectories))
       (send* _trajectories __trajectories)
     (progn
       (if __trajectories (setq _trajectories (car __trajectories)))
       _trajectories)))
  (:serialization-length
   ()
   (+
    ;; spencer_tracking_msgs/PersonTrajectory[] _trajectories
    (apply #'+ (send-all _trajectories :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; spencer_tracking_msgs/PersonTrajectory[] _trajectories
     (write-long (length _trajectories) s)
     (dolist (elem _trajectories)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; spencer_tracking_msgs/PersonTrajectory[] _trajectories
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _trajectories (let (r) (dotimes (i n) (push (instance spencer_tracking_msgs::PersonTrajectory :init) r)) r))
     (dolist (elem- _trajectories)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass spencer_tracking_msgs::GetPersonTrajectories
  :super ros::object
  :slots ())

(setf (get spencer_tracking_msgs::GetPersonTrajectories :md5sum-) "e0e3d63b99808526b112847474001abe")
(setf (get spencer_tracking_msgs::GetPersonTrajectories :datatype-) "spencer_tracking_msgs/GetPersonTrajectories")
(setf (get spencer_tracking_msgs::GetPersonTrajectories :request) spencer_tracking_msgs::GetPersonTrajectoriesRequest)
(setf (get spencer_tracking_msgs::GetPersonTrajectories :response) spencer_tracking_msgs::GetPersonTrajectoriesResponse)

(defmethod spencer_tracking_msgs::GetPersonTrajectoriesRequest
  (:response () (instance spencer_tracking_msgs::GetPersonTrajectoriesResponse :init)))

(setf (get spencer_tracking_msgs::GetPersonTrajectoriesRequest :md5sum-) "e0e3d63b99808526b112847474001abe")
(setf (get spencer_tracking_msgs::GetPersonTrajectoriesRequest :datatype-) "spencer_tracking_msgs/GetPersonTrajectoriesRequest")
(setf (get spencer_tracking_msgs::GetPersonTrajectoriesRequest :definition-)
      "uint64[] requested_ids           # The IDs of the tracks you are interested in getting the trajectories of. An empty array means all available tracks.
duration max_age                 # The maximum age of a trajectory you want to get. A duration of 0 means \"since the beginning of times.\"
---
PersonTrajectory[] trajectories  # The trajectories of the tracks that have been asked for in requested_ids, in the same order.


================================================================================
MSG: spencer_tracking_msgs/PersonTrajectory
# Message defining the trajectory of a tracked person.
#
# The distinction between track and trajectory is that, depending on the
# implementation of the tracker, a single track (i.e. tracked person) might
# change the trajectory if at some point a new trajectory \"fits\" that track (person)
# better.
#

uint64                   track_id   # Unique identifier of the tracked person.
PersonTrajectoryEntry[]  trajectory # All states of the last T frames of the most likely trajectory of that tracked person.

================================================================================
MSG: spencer_tracking_msgs/PersonTrajectoryEntry
# Message defining an entry of a person trajectory.
#

time     stamp           # age of the track
bool     is_occluded     # if the track is currently not matched by a detection
uint64   detection_id    # id of the corresponding detection in the current cycle (undefined if occluded)

# The following fields are extracted from the Kalman state x and its covariance C

geometry_msgs/PoseWithCovariance    pose   # pose of the track (z value and orientation might not be set, check if corresponding variance on diagonal is > 99999)
geometry_msgs/TwistWithCovariance   twist  # velocity of the track (z value and rotational velocities might not be set, check if corresponding variance on diagonal is > 99999)

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
")

(setf (get spencer_tracking_msgs::GetPersonTrajectoriesResponse :md5sum-) "e0e3d63b99808526b112847474001abe")
(setf (get spencer_tracking_msgs::GetPersonTrajectoriesResponse :datatype-) "spencer_tracking_msgs/GetPersonTrajectoriesResponse")
(setf (get spencer_tracking_msgs::GetPersonTrajectoriesResponse :definition-)
      "uint64[] requested_ids           # The IDs of the tracks you are interested in getting the trajectories of. An empty array means all available tracks.
duration max_age                 # The maximum age of a trajectory you want to get. A duration of 0 means \"since the beginning of times.\"
---
PersonTrajectory[] trajectories  # The trajectories of the tracks that have been asked for in requested_ids, in the same order.


================================================================================
MSG: spencer_tracking_msgs/PersonTrajectory
# Message defining the trajectory of a tracked person.
#
# The distinction between track and trajectory is that, depending on the
# implementation of the tracker, a single track (i.e. tracked person) might
# change the trajectory if at some point a new trajectory \"fits\" that track (person)
# better.
#

uint64                   track_id   # Unique identifier of the tracked person.
PersonTrajectoryEntry[]  trajectory # All states of the last T frames of the most likely trajectory of that tracked person.

================================================================================
MSG: spencer_tracking_msgs/PersonTrajectoryEntry
# Message defining an entry of a person trajectory.
#

time     stamp           # age of the track
bool     is_occluded     # if the track is currently not matched by a detection
uint64   detection_id    # id of the corresponding detection in the current cycle (undefined if occluded)

# The following fields are extracted from the Kalman state x and its covariance C

geometry_msgs/PoseWithCovariance    pose   # pose of the track (z value and orientation might not be set, check if corresponding variance on diagonal is > 99999)
geometry_msgs/TwistWithCovariance   twist  # velocity of the track (z value and rotational velocities might not be set, check if corresponding variance on diagonal is > 99999)

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
")



(provide :spencer_tracking_msgs/GetPersonTrajectories "e0e3d63b99808526b112847474001abe")


