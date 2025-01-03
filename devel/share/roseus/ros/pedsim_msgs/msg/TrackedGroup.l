;; Auto-generated. Do not edit!


(when (boundp 'pedsim_msgs::TrackedGroup)
  (if (not (find-package "PEDSIM_MSGS"))
    (make-package "PEDSIM_MSGS"))
  (shadow 'TrackedGroup (find-package "PEDSIM_MSGS")))
(unless (find-package "PEDSIM_MSGS::TRACKEDGROUP")
  (make-package "PEDSIM_MSGS::TRACKEDGROUP"))

(in-package "ROS")
;;//! \htmlinclude TrackedGroup.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass pedsim_msgs::TrackedGroup
  :super ros::object
  :slots (_group_id _age _centerOfGravity _track_ids ))

(defmethod pedsim_msgs::TrackedGroup
  (:init
   (&key
    ((:group_id __group_id) 0)
    ((:age __age) (instance ros::time :init))
    ((:centerOfGravity __centerOfGravity) (instance geometry_msgs::PoseWithCovariance :init))
    ((:track_ids __track_ids) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _group_id (round __group_id))
   (setq _age __age)
   (setq _centerOfGravity __centerOfGravity)
   (setq _track_ids __track_ids)
   self)
  (:group_id
   (&optional __group_id)
   (if __group_id (setq _group_id __group_id)) _group_id)
  (:age
   (&optional __age)
   (if __age (setq _age __age)) _age)
  (:centerOfGravity
   (&rest __centerOfGravity)
   (if (keywordp (car __centerOfGravity))
       (send* _centerOfGravity __centerOfGravity)
     (progn
       (if __centerOfGravity (setq _centerOfGravity (car __centerOfGravity)))
       _centerOfGravity)))
  (:track_ids
   (&optional __track_ids)
   (if __track_ids (setq _track_ids __track_ids)) _track_ids)
  (:serialization-length
   ()
   (+
    ;; uint64 _group_id
    8
    ;; duration _age
    8
    ;; geometry_msgs/PoseWithCovariance _centerOfGravity
    (send _centerOfGravity :serialization-length)
    ;; uint64[] _track_ids
    (* 8    (length _track_ids)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint64 _group_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _group_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _group_id) (= (length (_group_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_group_id . bv) 0) 0) s)
              (write-long (ash (elt (_group_id . bv) 1) -1) s))
             ((and (class _group_id) (= (length (_group_id . bv)) 1)) ;; big1
              (write-long (elt (_group_id . bv) 0) s)
              (write-long (if (>= _group_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _group_id s)(write-long (if (>= _group_id 0) 0 #xffffffff) s)))
     ;; duration _age
       (write-long (send _age :sec) s) (write-long (send _age :nsec) s)
     ;; geometry_msgs/PoseWithCovariance _centerOfGravity
       (send _centerOfGravity :serialize s)
     ;; uint64[] _track_ids
     (write-long (length _track_ids) s)
     (dotimes (i (length _track_ids))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _track_ids i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _track_ids i)) (= (length ((elt _track_ids i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _track_ids i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _track_ids i) . bv) 1) -1) s))
             ((and (class (elt _track_ids i)) (= (length ((elt _track_ids i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _track_ids i) . bv) 0) s)
              (write-long (if (>= (elt _track_ids i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _track_ids i) s)(write-long (if (>= (elt _track_ids i) 0) 0 #xffffffff) s)))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint64 _group_id
     
#+(or :alpha :irix6 :x86_64)
      (setf _group_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _group_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; duration _age
     (send _age :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _age :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/PoseWithCovariance _centerOfGravity
     (send _centerOfGravity :deserialize buf ptr-) (incf ptr- (send _centerOfGravity :serialization-length))
   ;; uint64[] _track_ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _track_ids (instantiate integer-vector n))
     (dotimes (i n)
     
#+(or :alpha :irix6 :x86_64)
      (setf (elt _track_ids i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _track_ids i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;;
   self)
  )

(setf (get pedsim_msgs::TrackedGroup :md5sum-) "6a5318bfb8e49948a4dc15c1267f7e54")
(setf (get pedsim_msgs::TrackedGroup :datatype-) "pedsim_msgs/TrackedGroup")
(setf (get pedsim_msgs::TrackedGroup :definition-)
      "# Message defining a tracked group
#

uint64      group_id        # unique identifier of the target, consistent over time

duration    age             # age of the group

geometry_msgs/PoseWithCovariance    centerOfGravity   # mean and covariance of the group (calculated from its person tracks)

uint64[]    track_ids       # IDs of the tracked persons in this group. See srl_tracking_msgs/TrackedPersons
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



(provide :pedsim_msgs/TrackedGroup "6a5318bfb8e49948a4dc15c1267f7e54")


