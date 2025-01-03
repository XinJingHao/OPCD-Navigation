// Auto-generated. Do not edit!

// (in-package spencer_tracking_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DetectedPerson = require('./DetectedPerson.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class DetectedPersons {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.detections = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('detections')) {
        this.detections = initObj.detections
      }
      else {
        this.detections = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DetectedPersons
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [detections]
    // Serialize the length for message field [detections]
    bufferOffset = _serializer.uint32(obj.detections.length, buffer, bufferOffset);
    obj.detections.forEach((val) => {
      bufferOffset = DetectedPerson.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DetectedPersons
    let len;
    let data = new DetectedPersons(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [detections]
    // Deserialize array length for message field [detections]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.detections = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.detections[i] = DetectedPerson.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.detections.forEach((val) => {
      length += DetectedPerson.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'spencer_tracking_msgs/DetectedPersons';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '38e416cebb5dfd7363bde91113e7f3bd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message with all currently detected persons
    #
    
    Header                  header          # Header containing timestamp etc. of this message
    DetectedPerson[]        detections      # All persons that are currently being detected
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
    MSG: spencer_tracking_msgs/DetectedPerson
    # Message describing a detection of a person
    #
    
    # Unique id of the detection, monotonically increasing over time
    uint64          detection_id
    
    # (Pseudo-)probabilistic value between 0.0 and 1.0 describing the detector's confidence in the detection
    float64         confidence
    
    # 3D pose (position + orientation) of the *center* of the detection
    # check covariance to see which dimensions are actually set! unset dimensions shall have a covariance > 9999
    geometry_msgs/PoseWithCovariance    pose    
    
    # Sensor modality / detector type, see example constants below. 
    # used e.g. later in tracking to check which tracks have been visually confirmed
    string          modality            
    
                                        
    string          MODALITY_GENERIC_LASER_2D = laser2d
    string          MODALITY_GENERIC_LASER_3D = laser3d
    string          MODALITY_GENERIC_MONOCULAR_VISION = mono
    string          MODALITY_GENERIC_STEREO_VISION = stereo
    string          MODALITY_GENERIC_RGBD = rgbd
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DetectedPersons(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.detections !== undefined) {
      resolved.detections = new Array(msg.detections.length);
      for (let i = 0; i < resolved.detections.length; ++i) {
        resolved.detections[i] = DetectedPerson.Resolve(msg.detections[i]);
      }
    }
    else {
      resolved.detections = []
    }

    return resolved;
    }
};

module.exports = DetectedPersons;
