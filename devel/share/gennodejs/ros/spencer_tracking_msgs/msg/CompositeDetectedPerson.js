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
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class CompositeDetectedPerson {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.composite_detection_id = null;
      this.mean_confidence = null;
      this.max_confidence = null;
      this.min_confidence = null;
      this.pose = null;
      this.original_detections = null;
    }
    else {
      if (initObj.hasOwnProperty('composite_detection_id')) {
        this.composite_detection_id = initObj.composite_detection_id
      }
      else {
        this.composite_detection_id = 0;
      }
      if (initObj.hasOwnProperty('mean_confidence')) {
        this.mean_confidence = initObj.mean_confidence
      }
      else {
        this.mean_confidence = 0.0;
      }
      if (initObj.hasOwnProperty('max_confidence')) {
        this.max_confidence = initObj.max_confidence
      }
      else {
        this.max_confidence = 0.0;
      }
      if (initObj.hasOwnProperty('min_confidence')) {
        this.min_confidence = initObj.min_confidence
      }
      else {
        this.min_confidence = 0.0;
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.PoseWithCovariance();
      }
      if (initObj.hasOwnProperty('original_detections')) {
        this.original_detections = initObj.original_detections
      }
      else {
        this.original_detections = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CompositeDetectedPerson
    // Serialize message field [composite_detection_id]
    bufferOffset = _serializer.uint64(obj.composite_detection_id, buffer, bufferOffset);
    // Serialize message field [mean_confidence]
    bufferOffset = _serializer.float64(obj.mean_confidence, buffer, bufferOffset);
    // Serialize message field [max_confidence]
    bufferOffset = _serializer.float64(obj.max_confidence, buffer, bufferOffset);
    // Serialize message field [min_confidence]
    bufferOffset = _serializer.float64(obj.min_confidence, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.PoseWithCovariance.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [original_detections]
    // Serialize the length for message field [original_detections]
    bufferOffset = _serializer.uint32(obj.original_detections.length, buffer, bufferOffset);
    obj.original_detections.forEach((val) => {
      bufferOffset = DetectedPerson.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CompositeDetectedPerson
    let len;
    let data = new CompositeDetectedPerson(null);
    // Deserialize message field [composite_detection_id]
    data.composite_detection_id = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [mean_confidence]
    data.mean_confidence = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [max_confidence]
    data.max_confidence = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [min_confidence]
    data.min_confidence = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.PoseWithCovariance.deserialize(buffer, bufferOffset);
    // Deserialize message field [original_detections]
    // Deserialize array length for message field [original_detections]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.original_detections = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.original_detections[i] = DetectedPerson.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.original_detections.forEach((val) => {
      length += DetectedPerson.getMessageSize(val);
    });
    return length + 380;
  }

  static datatype() {
    // Returns string type for a message object
    return 'spencer_tracking_msgs/CompositeDetectedPerson';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '10e83f06a9bfbf6da1ae6f0fcdbe2cc4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new CompositeDetectedPerson(null);
    if (msg.composite_detection_id !== undefined) {
      resolved.composite_detection_id = msg.composite_detection_id;
    }
    else {
      resolved.composite_detection_id = 0
    }

    if (msg.mean_confidence !== undefined) {
      resolved.mean_confidence = msg.mean_confidence;
    }
    else {
      resolved.mean_confidence = 0.0
    }

    if (msg.max_confidence !== undefined) {
      resolved.max_confidence = msg.max_confidence;
    }
    else {
      resolved.max_confidence = 0.0
    }

    if (msg.min_confidence !== undefined) {
      resolved.min_confidence = msg.min_confidence;
    }
    else {
      resolved.min_confidence = 0.0
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.PoseWithCovariance.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.PoseWithCovariance()
    }

    if (msg.original_detections !== undefined) {
      resolved.original_detections = new Array(msg.original_detections.length);
      for (let i = 0; i < resolved.original_detections.length; ++i) {
        resolved.original_detections[i] = DetectedPerson.Resolve(msg.original_detections[i]);
      }
    }
    else {
      resolved.original_detections = []
    }

    return resolved;
    }
};

module.exports = CompositeDetectedPerson;
