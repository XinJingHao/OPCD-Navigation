// Auto-generated. Do not edit!

// (in-package spencer_tracking_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CompositeDetectedPerson = require('./CompositeDetectedPerson.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class CompositeDetectedPersons {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.elements = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('elements')) {
        this.elements = initObj.elements
      }
      else {
        this.elements = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CompositeDetectedPersons
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [elements]
    // Serialize the length for message field [elements]
    bufferOffset = _serializer.uint32(obj.elements.length, buffer, bufferOffset);
    obj.elements.forEach((val) => {
      bufferOffset = CompositeDetectedPerson.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CompositeDetectedPersons
    let len;
    let data = new CompositeDetectedPersons(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [elements]
    // Deserialize array length for message field [elements]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.elements = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.elements[i] = CompositeDetectedPerson.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.elements.forEach((val) => {
      length += CompositeDetectedPerson.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'spencer_tracking_msgs/CompositeDetectedPersons';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c4f9f34f5cb712f010de12fc027da3e6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message specifying which original detected persons (from all kinds of sensors) have been merged into one fused detection before being processed by the person tracker, in the current time step.
    #
    # This information is processed by the spencer_detected_person_association module, such that other perception components can associate their results (e.g. person age, gender)
    # with a particular spencer_tracking_msgs/TrackedPerson (which contains a reference to a composite person detection ID).
    
    Header                      header          # Header timestamp is set to the *latest* timestamp of all fused DetectedPerson messages.
                                                # Before fusion, all detections are transformed into a common coordinate frame (usually base_footprint).
    CompositeDetectedPerson[]   elements        # Fused (merged) detected persons
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
    MSG: spencer_tracking_msgs/CompositeDetectedPerson
    # Specifies which detected persons have been merged into a composite detection by the spencer_detected_person_association module.
    
    # TODO: Do we need a composite person-specific timestamp (or even a full message header including frame ID)?
    # Having a separate timestamp per person could be useful if the timestamps of the merged DetectedPersons messages vary a lot,
    # and some persons are only seen by a single sensor (so averaging over all input timestamps would have a detrimental effect). 
    
    uint64      composite_detection_id          # ID of the fused detection
    
    float64     mean_confidence                 # mean of the confidences of the original detections
    float64     max_confidence                  # maximum confidence of original detections
    float64     min_confidence                  # minimum confidence of original detections
    
    
    geometry_msgs/PoseWithCovariance    pose    # Merged 3D pose (position + orientation) of the detection center
                                                # check covariance to see which dimensions are actually set!
                                                # unset dimensions shall have a covariance > 9999
    
    DetectedPerson[] original_detections        # The original detections from individual sensor-specific detectors that have been combined into a composite detection
                                                # We are copying the entire DetectedPersons messages, *with poses transformed into the target frame*, such that subscribers
                                                # do not have to subscribe to all the original DetectedPersons topics.
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CompositeDetectedPersons(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.elements !== undefined) {
      resolved.elements = new Array(msg.elements.length);
      for (let i = 0; i < resolved.elements.length; ++i) {
        resolved.elements[i] = CompositeDetectedPerson.Resolve(msg.elements[i]);
      }
    }
    else {
      resolved.elements = []
    }

    return resolved;
    }
};

module.exports = CompositeDetectedPersons;
