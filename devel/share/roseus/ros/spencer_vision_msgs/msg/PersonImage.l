;; Auto-generated. Do not edit!


(when (boundp 'spencer_vision_msgs::PersonImage)
  (if (not (find-package "SPENCER_VISION_MSGS"))
    (make-package "SPENCER_VISION_MSGS"))
  (shadow 'PersonImage (find-package "SPENCER_VISION_MSGS")))
(unless (find-package "SPENCER_VISION_MSGS::PERSONIMAGE")
  (make-package "SPENCER_VISION_MSGS::PERSONIMAGE"))

(in-package "ROS")
;;//! \htmlinclude PersonImage.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass spencer_vision_msgs::PersonImage
  :super ros::object
  :slots (_detection_id _image ))

(defmethod spencer_vision_msgs::PersonImage
  (:init
   (&key
    ((:detection_id __detection_id) 0)
    ((:image __image) (instance sensor_msgs::Image :init))
    )
   (send-super :init)
   (setq _detection_id (round __detection_id))
   (setq _image __image)
   self)
  (:detection_id
   (&optional __detection_id)
   (if __detection_id (setq _detection_id __detection_id)) _detection_id)
  (:image
   (&rest __image)
   (if (keywordp (car __image))
       (send* _image __image)
     (progn
       (if __image (setq _image (car __image)))
       _image)))
  (:serialization-length
   ()
   (+
    ;; uint64 _detection_id
    8
    ;; sensor_msgs/Image _image
    (send _image :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
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
     ;; sensor_msgs/Image _image
       (send _image :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
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
   ;; sensor_msgs/Image _image
     (send _image :deserialize buf ptr-) (incf ptr- (send _image :serialization-length))
   ;;
   self)
  )

(setf (get spencer_vision_msgs::PersonImage :md5sum-) "d8c9467666c46ebf784964cbf610d8c1")
(setf (get spencer_vision_msgs::PersonImage :datatype-) "spencer_vision_msgs/PersonImage")
(setf (get spencer_vision_msgs::PersonImage :definition-)
      "# Message describing a depth or RGB image containing a part of a person (e.g. head, face, full body...), which is usually encoded in the topic title
#

uint64              detection_id
sensor_msgs/Image   image


================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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

")



(provide :spencer_vision_msgs/PersonImage "d8c9467666c46ebf784964cbf610d8c1")


