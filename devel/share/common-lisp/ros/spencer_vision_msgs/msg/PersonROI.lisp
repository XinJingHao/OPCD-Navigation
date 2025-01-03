; Auto-generated. Do not edit!


(cl:in-package spencer_vision_msgs-msg)


;//! \htmlinclude PersonROI.msg.html

(cl:defclass <PersonROI> (roslisp-msg-protocol:ros-message)
  ((detection_id
    :reader detection_id
    :initarg :detection_id
    :type cl:integer
    :initform 0)
   (roi
    :reader roi
    :initarg :roi
    :type sensor_msgs-msg:RegionOfInterest
    :initform (cl:make-instance 'sensor_msgs-msg:RegionOfInterest)))
)

(cl:defclass PersonROI (<PersonROI>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PersonROI>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PersonROI)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_vision_msgs-msg:<PersonROI> is deprecated: use spencer_vision_msgs-msg:PersonROI instead.")))

(cl:ensure-generic-function 'detection_id-val :lambda-list '(m))
(cl:defmethod detection_id-val ((m <PersonROI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_vision_msgs-msg:detection_id-val is deprecated.  Use spencer_vision_msgs-msg:detection_id instead.")
  (detection_id m))

(cl:ensure-generic-function 'roi-val :lambda-list '(m))
(cl:defmethod roi-val ((m <PersonROI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_vision_msgs-msg:roi-val is deprecated.  Use spencer_vision_msgs-msg:roi instead.")
  (roi m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PersonROI>) ostream)
  "Serializes a message object of type '<PersonROI>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'detection_id)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'roi) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PersonROI>) istream)
  "Deserializes a message object of type '<PersonROI>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'roi) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PersonROI>)))
  "Returns string type for a message object of type '<PersonROI>"
  "spencer_vision_msgs/PersonROI")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PersonROI)))
  "Returns string type for a message object of type 'PersonROI"
  "spencer_vision_msgs/PersonROI")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PersonROI>)))
  "Returns md5sum for a message object of type '<PersonROI>"
  "4484c510821bd11dbd7b6b3627d4e4ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PersonROI)))
  "Returns md5sum for a message object of type 'PersonROI"
  "4484c510821bd11dbd7b6b3627d4e4ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PersonROI>)))
  "Returns full string definition for message of type '<PersonROI>"
  (cl:format cl:nil "# Message describing a rectangular region of interest in a depth or RGB image containing a part of a person (e.g. head, face, full body...), which is usually encoded in the topic title~%#~%~%uint64          detection_id~%~%sensor_msgs/RegionOfInterest    roi~%~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PersonROI)))
  "Returns full string definition for message of type 'PersonROI"
  (cl:format cl:nil "# Message describing a rectangular region of interest in a depth or RGB image containing a part of a person (e.g. head, face, full body...), which is usually encoded in the topic title~%#~%~%uint64          detection_id~%~%sensor_msgs/RegionOfInterest    roi~%~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PersonROI>))
  (cl:+ 0
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'roi))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PersonROI>))
  "Converts a ROS message object to a list"
  (cl:list 'PersonROI
    (cl:cons ':detection_id (detection_id msg))
    (cl:cons ':roi (roi msg))
))
