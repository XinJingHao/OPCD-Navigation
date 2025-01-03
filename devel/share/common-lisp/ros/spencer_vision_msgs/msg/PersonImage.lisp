; Auto-generated. Do not edit!


(cl:in-package spencer_vision_msgs-msg)


;//! \htmlinclude PersonImage.msg.html

(cl:defclass <PersonImage> (roslisp-msg-protocol:ros-message)
  ((detection_id
    :reader detection_id
    :initarg :detection_id
    :type cl:integer
    :initform 0)
   (image
    :reader image
    :initarg :image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass PersonImage (<PersonImage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PersonImage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PersonImage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_vision_msgs-msg:<PersonImage> is deprecated: use spencer_vision_msgs-msg:PersonImage instead.")))

(cl:ensure-generic-function 'detection_id-val :lambda-list '(m))
(cl:defmethod detection_id-val ((m <PersonImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_vision_msgs-msg:detection_id-val is deprecated.  Use spencer_vision_msgs-msg:detection_id instead.")
  (detection_id m))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <PersonImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_vision_msgs-msg:image-val is deprecated.  Use spencer_vision_msgs-msg:image instead.")
  (image m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PersonImage>) ostream)
  "Serializes a message object of type '<PersonImage>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'detection_id)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PersonImage>) istream)
  "Deserializes a message object of type '<PersonImage>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PersonImage>)))
  "Returns string type for a message object of type '<PersonImage>"
  "spencer_vision_msgs/PersonImage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PersonImage)))
  "Returns string type for a message object of type 'PersonImage"
  "spencer_vision_msgs/PersonImage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PersonImage>)))
  "Returns md5sum for a message object of type '<PersonImage>"
  "d8c9467666c46ebf784964cbf610d8c1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PersonImage)))
  "Returns md5sum for a message object of type 'PersonImage"
  "d8c9467666c46ebf784964cbf610d8c1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PersonImage>)))
  "Returns full string definition for message of type '<PersonImage>"
  (cl:format cl:nil "# Message describing a depth or RGB image containing a part of a person (e.g. head, face, full body...), which is usually encoded in the topic title~%#~%~%uint64              detection_id~%sensor_msgs/Image   image~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PersonImage)))
  "Returns full string definition for message of type 'PersonImage"
  (cl:format cl:nil "# Message describing a depth or RGB image containing a part of a person (e.g. head, face, full body...), which is usually encoded in the topic title~%#~%~%uint64              detection_id~%sensor_msgs/Image   image~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PersonImage>))
  (cl:+ 0
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PersonImage>))
  "Converts a ROS message object to a list"
  (cl:list 'PersonImage
    (cl:cons ':detection_id (detection_id msg))
    (cl:cons ':image (image msg))
))
