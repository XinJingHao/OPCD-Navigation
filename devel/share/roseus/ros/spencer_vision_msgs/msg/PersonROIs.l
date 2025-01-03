;; Auto-generated. Do not edit!


(when (boundp 'spencer_vision_msgs::PersonROIs)
  (if (not (find-package "SPENCER_VISION_MSGS"))
    (make-package "SPENCER_VISION_MSGS"))
  (shadow 'PersonROIs (find-package "SPENCER_VISION_MSGS")))
(unless (find-package "SPENCER_VISION_MSGS::PERSONROIS")
  (make-package "SPENCER_VISION_MSGS::PERSONROIS"))

(in-package "ROS")
;;//! \htmlinclude PersonROIs.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass spencer_vision_msgs::PersonROIs
  :super ros::object
  :slots (_header _rgb_topic _depth_topic _elements ))

(defmethod spencer_vision_msgs::PersonROIs
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:rgb_topic __rgb_topic) "")
    ((:depth_topic __depth_topic) "")
    ((:elements __elements) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _rgb_topic (string __rgb_topic))
   (setq _depth_topic (string __depth_topic))
   (setq _elements __elements)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:rgb_topic
   (&optional __rgb_topic)
   (if __rgb_topic (setq _rgb_topic __rgb_topic)) _rgb_topic)
  (:depth_topic
   (&optional __depth_topic)
   (if __depth_topic (setq _depth_topic __depth_topic)) _depth_topic)
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
    ;; string _rgb_topic
    4 (length _rgb_topic)
    ;; string _depth_topic
    4 (length _depth_topic)
    ;; spencer_vision_msgs/PersonROI[] _elements
    (apply #'+ (send-all _elements :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _rgb_topic
       (write-long (length _rgb_topic) s) (princ _rgb_topic s)
     ;; string _depth_topic
       (write-long (length _depth_topic) s) (princ _depth_topic s)
     ;; spencer_vision_msgs/PersonROI[] _elements
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
   ;; string _rgb_topic
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _rgb_topic (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _depth_topic
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _depth_topic (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; spencer_vision_msgs/PersonROI[] _elements
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _elements (let (r) (dotimes (i n) (push (instance spencer_vision_msgs::PersonROI :init) r)) r))
     (dolist (elem- _elements)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get spencer_vision_msgs::PersonROIs :md5sum-) "39d733db5b2ece3bd129f8a360116d23")
(setf (get spencer_vision_msgs::PersonROIs :datatype-) "spencer_vision_msgs/PersonROIs")
(setf (get spencer_vision_msgs::PersonROIs :definition-)
      "# Message describing an array of rectangular regions of interest in a depth or RGB image containing a part of a person (e.g. head, face, full body...), which is usually encoded in the topic title
#

std_msgs/Header     header

string              rgb_topic       # not necessarily the raw camera output; might be preprocessed for rectification etc.
string              depth_topic     # might not be set if depth is not available, otherwise it is the registered depth image

PersonROI[]         elements

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
MSG: spencer_vision_msgs/PersonROI
# Message describing a rectangular region of interest in a depth or RGB image containing a part of a person (e.g. head, face, full body...), which is usually encoded in the topic title
#

uint64          detection_id

sensor_msgs/RegionOfInterest    roi


================================================================================
MSG: sensor_msgs/RegionOfInterest
# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the \"raw\"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify

")



(provide :spencer_vision_msgs/PersonROIs "39d733db5b2ece3bd129f8a360116d23")


