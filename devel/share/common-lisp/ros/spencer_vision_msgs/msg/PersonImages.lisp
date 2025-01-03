; Auto-generated. Do not edit!


(cl:in-package spencer_vision_msgs-msg)


;//! \htmlinclude PersonImages.msg.html

(cl:defclass <PersonImages> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (elements
    :reader elements
    :initarg :elements
    :type (cl:vector spencer_vision_msgs-msg:PersonImage)
   :initform (cl:make-array 0 :element-type 'spencer_vision_msgs-msg:PersonImage :initial-element (cl:make-instance 'spencer_vision_msgs-msg:PersonImage))))
)

(cl:defclass PersonImages (<PersonImages>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PersonImages>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PersonImages)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_vision_msgs-msg:<PersonImages> is deprecated: use spencer_vision_msgs-msg:PersonImages instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PersonImages>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_vision_msgs-msg:header-val is deprecated.  Use spencer_vision_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'elements-val :lambda-list '(m))
(cl:defmethod elements-val ((m <PersonImages>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_vision_msgs-msg:elements-val is deprecated.  Use spencer_vision_msgs-msg:elements instead.")
  (elements m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PersonImages>) ostream)
  "Serializes a message object of type '<PersonImages>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'elements))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'elements))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PersonImages>) istream)
  "Deserializes a message object of type '<PersonImages>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'elements) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'elements)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'spencer_vision_msgs-msg:PersonImage))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PersonImages>)))
  "Returns string type for a message object of type '<PersonImages>"
  "spencer_vision_msgs/PersonImages")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PersonImages)))
  "Returns string type for a message object of type 'PersonImages"
  "spencer_vision_msgs/PersonImages")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PersonImages>)))
  "Returns md5sum for a message object of type '<PersonImages>"
  "6c5881059a7a7f9c813cdc2429f1b5cf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PersonImages)))
  "Returns md5sum for a message object of type 'PersonImages"
  "6c5881059a7a7f9c813cdc2429f1b5cf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PersonImages>)))
  "Returns full string definition for message of type '<PersonImages>"
  (cl:format cl:nil "# Message describing an array of depth or RGB images containing a part of a person (e.g. head, face, full body...), which is usually encoded in the topic title~%#~%~%std_msgs/Header     header~%PersonImage[]       elements~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: spencer_vision_msgs/PersonImage~%# Message describing a depth or RGB image containing a part of a person (e.g. head, face, full body...), which is usually encoded in the topic title~%#~%~%uint64              detection_id~%sensor_msgs/Image   image~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PersonImages)))
  "Returns full string definition for message of type 'PersonImages"
  (cl:format cl:nil "# Message describing an array of depth or RGB images containing a part of a person (e.g. head, face, full body...), which is usually encoded in the topic title~%#~%~%std_msgs/Header     header~%PersonImage[]       elements~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: spencer_vision_msgs/PersonImage~%# Message describing a depth or RGB image containing a part of a person (e.g. head, face, full body...), which is usually encoded in the topic title~%#~%~%uint64              detection_id~%sensor_msgs/Image   image~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PersonImages>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'elements) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PersonImages>))
  "Converts a ROS message object to a list"
  (cl:list 'PersonImages
    (cl:cons ':header (header msg))
    (cl:cons ':elements (elements msg))
))
