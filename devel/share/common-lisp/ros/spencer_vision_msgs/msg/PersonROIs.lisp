; Auto-generated. Do not edit!


(cl:in-package spencer_vision_msgs-msg)


;//! \htmlinclude PersonROIs.msg.html

(cl:defclass <PersonROIs> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (rgb_topic
    :reader rgb_topic
    :initarg :rgb_topic
    :type cl:string
    :initform "")
   (depth_topic
    :reader depth_topic
    :initarg :depth_topic
    :type cl:string
    :initform "")
   (elements
    :reader elements
    :initarg :elements
    :type (cl:vector spencer_vision_msgs-msg:PersonROI)
   :initform (cl:make-array 0 :element-type 'spencer_vision_msgs-msg:PersonROI :initial-element (cl:make-instance 'spencer_vision_msgs-msg:PersonROI))))
)

(cl:defclass PersonROIs (<PersonROIs>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PersonROIs>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PersonROIs)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_vision_msgs-msg:<PersonROIs> is deprecated: use spencer_vision_msgs-msg:PersonROIs instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PersonROIs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_vision_msgs-msg:header-val is deprecated.  Use spencer_vision_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'rgb_topic-val :lambda-list '(m))
(cl:defmethod rgb_topic-val ((m <PersonROIs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_vision_msgs-msg:rgb_topic-val is deprecated.  Use spencer_vision_msgs-msg:rgb_topic instead.")
  (rgb_topic m))

(cl:ensure-generic-function 'depth_topic-val :lambda-list '(m))
(cl:defmethod depth_topic-val ((m <PersonROIs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_vision_msgs-msg:depth_topic-val is deprecated.  Use spencer_vision_msgs-msg:depth_topic instead.")
  (depth_topic m))

(cl:ensure-generic-function 'elements-val :lambda-list '(m))
(cl:defmethod elements-val ((m <PersonROIs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_vision_msgs-msg:elements-val is deprecated.  Use spencer_vision_msgs-msg:elements instead.")
  (elements m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PersonROIs>) ostream)
  "Serializes a message object of type '<PersonROIs>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'rgb_topic))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'rgb_topic))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'depth_topic))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'depth_topic))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'elements))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'elements))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PersonROIs>) istream)
  "Deserializes a message object of type '<PersonROIs>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rgb_topic) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'rgb_topic) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'depth_topic) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'depth_topic) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'elements) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'elements)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'spencer_vision_msgs-msg:PersonROI))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PersonROIs>)))
  "Returns string type for a message object of type '<PersonROIs>"
  "spencer_vision_msgs/PersonROIs")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PersonROIs)))
  "Returns string type for a message object of type 'PersonROIs"
  "spencer_vision_msgs/PersonROIs")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PersonROIs>)))
  "Returns md5sum for a message object of type '<PersonROIs>"
  "39d733db5b2ece3bd129f8a360116d23")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PersonROIs)))
  "Returns md5sum for a message object of type 'PersonROIs"
  "39d733db5b2ece3bd129f8a360116d23")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PersonROIs>)))
  "Returns full string definition for message of type '<PersonROIs>"
  (cl:format cl:nil "# Message describing an array of rectangular regions of interest in a depth or RGB image containing a part of a person (e.g. head, face, full body...), which is usually encoded in the topic title~%#~%~%std_msgs/Header     header~%~%string              rgb_topic       # not necessarily the raw camera output; might be preprocessed for rectification etc.~%string              depth_topic     # might not be set if depth is not available, otherwise it is the registered depth image~%~%PersonROI[]         elements~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: spencer_vision_msgs/PersonROI~%# Message describing a rectangular region of interest in a depth or RGB image containing a part of a person (e.g. head, face, full body...), which is usually encoded in the topic title~%#~%~%uint64          detection_id~%~%sensor_msgs/RegionOfInterest    roi~%~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PersonROIs)))
  "Returns full string definition for message of type 'PersonROIs"
  (cl:format cl:nil "# Message describing an array of rectangular regions of interest in a depth or RGB image containing a part of a person (e.g. head, face, full body...), which is usually encoded in the topic title~%#~%~%std_msgs/Header     header~%~%string              rgb_topic       # not necessarily the raw camera output; might be preprocessed for rectification etc.~%string              depth_topic     # might not be set if depth is not available, otherwise it is the registered depth image~%~%PersonROI[]         elements~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: spencer_vision_msgs/PersonROI~%# Message describing a rectangular region of interest in a depth or RGB image containing a part of a person (e.g. head, face, full body...), which is usually encoded in the topic title~%#~%~%uint64          detection_id~%~%sensor_msgs/RegionOfInterest    roi~%~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PersonROIs>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'rgb_topic))
     4 (cl:length (cl:slot-value msg 'depth_topic))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'elements) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PersonROIs>))
  "Converts a ROS message object to a list"
  (cl:list 'PersonROIs
    (cl:cons ':header (header msg))
    (cl:cons ':rgb_topic (rgb_topic msg))
    (cl:cons ':depth_topic (depth_topic msg))
    (cl:cons ':elements (elements msg))
))
