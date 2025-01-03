;; Auto-generated. Do not edit!


(when (boundp 'spencer_tracking_msgs::TrackedPerson2d)
  (if (not (find-package "SPENCER_TRACKING_MSGS"))
    (make-package "SPENCER_TRACKING_MSGS"))
  (shadow 'TrackedPerson2d (find-package "SPENCER_TRACKING_MSGS")))
(unless (find-package "SPENCER_TRACKING_MSGS::TRACKEDPERSON2D")
  (make-package "SPENCER_TRACKING_MSGS::TRACKEDPERSON2D"))

(in-package "ROS")
;;//! \htmlinclude TrackedPerson2d.msg.html


(defclass spencer_tracking_msgs::TrackedPerson2d
  :super ros::object
  :slots (_track_id _person_height _x _y _w _h _depth ))

(defmethod spencer_tracking_msgs::TrackedPerson2d
  (:init
   (&key
    ((:track_id __track_id) 0)
    ((:person_height __person_height) 0.0)
    ((:x __x) 0)
    ((:y __y) 0)
    ((:w __w) 0)
    ((:h __h) 0)
    ((:depth __depth) 0.0)
    )
   (send-super :init)
   (setq _track_id (round __track_id))
   (setq _person_height (float __person_height))
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _w (round __w))
   (setq _h (round __h))
   (setq _depth (float __depth))
   self)
  (:track_id
   (&optional __track_id)
   (if __track_id (setq _track_id __track_id)) _track_id)
  (:person_height
   (&optional __person_height)
   (if __person_height (setq _person_height __person_height)) _person_height)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:w
   (&optional __w)
   (if __w (setq _w __w)) _w)
  (:h
   (&optional __h)
   (if __h (setq _h __h)) _h)
  (:depth
   (&optional __depth)
   (if __depth (setq _depth __depth)) _depth)
  (:serialization-length
   ()
   (+
    ;; uint64 _track_id
    8
    ;; float32 _person_height
    4
    ;; int32 _x
    4
    ;; int32 _y
    4
    ;; uint32 _w
    4
    ;; uint32 _h
    4
    ;; float32 _depth
    4
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
     ;; float32 _person_height
       (sys::poke _person_height (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _x
       (write-long _x s)
     ;; int32 _y
       (write-long _y s)
     ;; uint32 _w
       (write-long _w s)
     ;; uint32 _h
       (write-long _h s)
     ;; float32 _depth
       (sys::poke _depth (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float32 _person_height
     (setq _person_height (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _x
     (setq _x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y
     (setq _y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _w
     (setq _w (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _h
     (setq _h (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _depth
     (setq _depth (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get spencer_tracking_msgs::TrackedPerson2d :md5sum-) "12df4823c658df0d660d2a5c68ae4aea")
(setf (get spencer_tracking_msgs::TrackedPerson2d :datatype-) "spencer_tracking_msgs/TrackedPerson2d")
(setf (get spencer_tracking_msgs::TrackedPerson2d :definition-)
      "# Message defining a 2d image bbox of a tracked person
#

uint64      track_id        # unique identifier of the target, consistent over time
float32     person_height   # 3d height of person in m
int32       x               # top left corner x of 2d image bbox
int32       y               # top left corner y of 2d image bbox
uint32      w               # width of 2d image bbox
uint32      h               # height of 2d image bbox
float32     depth           # distance from the camera in m

")



(provide :spencer_tracking_msgs/TrackedPerson2d "12df4823c658df0d660d2a5c68ae4aea")


