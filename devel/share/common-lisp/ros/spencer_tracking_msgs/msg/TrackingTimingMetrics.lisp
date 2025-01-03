; Auto-generated. Do not edit!


(cl:in-package spencer_tracking_msgs-msg)


;//! \htmlinclude TrackingTimingMetrics.msg.html

(cl:defclass <TrackingTimingMetrics> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (track_count
    :reader track_count
    :initarg :track_count
    :type cl:integer
    :initform 0)
   (cycle_no
    :reader cycle_no
    :initarg :cycle_no
    :type cl:integer
    :initform 0)
   (average_cycle_time
    :reader average_cycle_time
    :initarg :average_cycle_time
    :type cl:float
    :initform 0.0)
   (cycle_time
    :reader cycle_time
    :initarg :cycle_time
    :type cl:float
    :initform 0.0)
   (average_processing_rate
    :reader average_processing_rate
    :initarg :average_processing_rate
    :type cl:float
    :initform 0.0)
   (cpu_load
    :reader cpu_load
    :initarg :cpu_load
    :type cl:float
    :initform 0.0)
   (average_cpu_load
    :reader average_cpu_load
    :initarg :average_cpu_load
    :type cl:float
    :initform 0.0)
   (elapsed_time
    :reader elapsed_time
    :initarg :elapsed_time
    :type cl:float
    :initform 0.0)
   (elapsed_cpu_time
    :reader elapsed_cpu_time
    :initarg :elapsed_cpu_time
    :type cl:float
    :initform 0.0))
)

(cl:defclass TrackingTimingMetrics (<TrackingTimingMetrics>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackingTimingMetrics>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackingTimingMetrics)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_tracking_msgs-msg:<TrackingTimingMetrics> is deprecated: use spencer_tracking_msgs-msg:TrackingTimingMetrics instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TrackingTimingMetrics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:header-val is deprecated.  Use spencer_tracking_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'track_count-val :lambda-list '(m))
(cl:defmethod track_count-val ((m <TrackingTimingMetrics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:track_count-val is deprecated.  Use spencer_tracking_msgs-msg:track_count instead.")
  (track_count m))

(cl:ensure-generic-function 'cycle_no-val :lambda-list '(m))
(cl:defmethod cycle_no-val ((m <TrackingTimingMetrics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:cycle_no-val is deprecated.  Use spencer_tracking_msgs-msg:cycle_no instead.")
  (cycle_no m))

(cl:ensure-generic-function 'average_cycle_time-val :lambda-list '(m))
(cl:defmethod average_cycle_time-val ((m <TrackingTimingMetrics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:average_cycle_time-val is deprecated.  Use spencer_tracking_msgs-msg:average_cycle_time instead.")
  (average_cycle_time m))

(cl:ensure-generic-function 'cycle_time-val :lambda-list '(m))
(cl:defmethod cycle_time-val ((m <TrackingTimingMetrics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:cycle_time-val is deprecated.  Use spencer_tracking_msgs-msg:cycle_time instead.")
  (cycle_time m))

(cl:ensure-generic-function 'average_processing_rate-val :lambda-list '(m))
(cl:defmethod average_processing_rate-val ((m <TrackingTimingMetrics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:average_processing_rate-val is deprecated.  Use spencer_tracking_msgs-msg:average_processing_rate instead.")
  (average_processing_rate m))

(cl:ensure-generic-function 'cpu_load-val :lambda-list '(m))
(cl:defmethod cpu_load-val ((m <TrackingTimingMetrics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:cpu_load-val is deprecated.  Use spencer_tracking_msgs-msg:cpu_load instead.")
  (cpu_load m))

(cl:ensure-generic-function 'average_cpu_load-val :lambda-list '(m))
(cl:defmethod average_cpu_load-val ((m <TrackingTimingMetrics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:average_cpu_load-val is deprecated.  Use spencer_tracking_msgs-msg:average_cpu_load instead.")
  (average_cpu_load m))

(cl:ensure-generic-function 'elapsed_time-val :lambda-list '(m))
(cl:defmethod elapsed_time-val ((m <TrackingTimingMetrics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:elapsed_time-val is deprecated.  Use spencer_tracking_msgs-msg:elapsed_time instead.")
  (elapsed_time m))

(cl:ensure-generic-function 'elapsed_cpu_time-val :lambda-list '(m))
(cl:defmethod elapsed_cpu_time-val ((m <TrackingTimingMetrics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_tracking_msgs-msg:elapsed_cpu_time-val is deprecated.  Use spencer_tracking_msgs-msg:elapsed_cpu_time instead.")
  (elapsed_cpu_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackingTimingMetrics>) ostream)
  "Serializes a message object of type '<TrackingTimingMetrics>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'track_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'track_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'track_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'track_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'track_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'track_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'track_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'track_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cycle_no)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cycle_no)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cycle_no)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cycle_no)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'cycle_no)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'cycle_no)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'cycle_no)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'cycle_no)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'average_cycle_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cycle_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'average_processing_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cpu_load))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'average_cpu_load))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'elapsed_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'elapsed_cpu_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackingTimingMetrics>) istream)
  "Deserializes a message object of type '<TrackingTimingMetrics>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'track_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'track_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'track_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'track_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'track_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'track_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'track_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'track_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cycle_no)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cycle_no)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cycle_no)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cycle_no)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'cycle_no)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'cycle_no)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'cycle_no)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'cycle_no)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'average_cycle_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cycle_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'average_processing_rate) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cpu_load) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'average_cpu_load) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'elapsed_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'elapsed_cpu_time) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackingTimingMetrics>)))
  "Returns string type for a message object of type '<TrackingTimingMetrics>"
  "spencer_tracking_msgs/TrackingTimingMetrics")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackingTimingMetrics)))
  "Returns string type for a message object of type 'TrackingTimingMetrics"
  "spencer_tracking_msgs/TrackingTimingMetrics")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackingTimingMetrics>)))
  "Returns md5sum for a message object of type '<TrackingTimingMetrics>"
  "e5e4499959c0302da5d8864744a1d65a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackingTimingMetrics)))
  "Returns md5sum for a message object of type 'TrackingTimingMetrics"
  "e5e4499959c0302da5d8864744a1d65a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackingTimingMetrics>)))
  "Returns full string definition for message of type '<TrackingTimingMetrics>"
  (cl:format cl:nil "# Message with timing values for analyzing the efficency ~%#~%~%Header              header                  # Header containing timestamp etc. of this message~%uint64              track_count             # number of person currentl tracked~%uint64              cycle_no                # incremented number of cycles        ~%float32             average_cycle_time      # average time for one cycle of tracker~%float32             cycle_time              # cycle time of current cycle~%float32             average_processing_rate # average frequency of processing data~%float32             cpu_load                # current cpu load~%float32             average_cpu_load        # average cpu load~%float32             elapsed_time            # elapsed seconds since start~%float32             elapsed_cpu_time        # elapsed cpu time since start~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackingTimingMetrics)))
  "Returns full string definition for message of type 'TrackingTimingMetrics"
  (cl:format cl:nil "# Message with timing values for analyzing the efficency ~%#~%~%Header              header                  # Header containing timestamp etc. of this message~%uint64              track_count             # number of person currentl tracked~%uint64              cycle_no                # incremented number of cycles        ~%float32             average_cycle_time      # average time for one cycle of tracker~%float32             cycle_time              # cycle time of current cycle~%float32             average_processing_rate # average frequency of processing data~%float32             cpu_load                # current cpu load~%float32             average_cpu_load        # average cpu load~%float32             elapsed_time            # elapsed seconds since start~%float32             elapsed_cpu_time        # elapsed cpu time since start~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackingTimingMetrics>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackingTimingMetrics>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackingTimingMetrics
    (cl:cons ':header (header msg))
    (cl:cons ':track_count (track_count msg))
    (cl:cons ':cycle_no (cycle_no msg))
    (cl:cons ':average_cycle_time (average_cycle_time msg))
    (cl:cons ':cycle_time (cycle_time msg))
    (cl:cons ':average_processing_rate (average_processing_rate msg))
    (cl:cons ':cpu_load (cpu_load msg))
    (cl:cons ':average_cpu_load (average_cpu_load msg))
    (cl:cons ':elapsed_time (elapsed_time msg))
    (cl:cons ':elapsed_cpu_time (elapsed_cpu_time msg))
))
