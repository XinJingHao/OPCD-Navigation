// Auto-generated. Do not edit!

// (in-package spencer_tracking_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class DetectedPerson {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.detection_id = null;
      this.confidence = null;
      this.pose = null;
      this.modality = null;
    }
    else {
      if (initObj.hasOwnProperty('detection_id')) {
        this.detection_id = initObj.detection_id
      }
      else {
        this.detection_id = 0;
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0.0;
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.PoseWithCovariance();
      }
      if (initObj.hasOwnProperty('modality')) {
        this.modality = initObj.modality
      }
      else {
        this.modality = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DetectedPerson
    // Serialize message field [detection_id]
    bufferOffset = _serializer.uint64(obj.detection_id, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float64(obj.confidence, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.PoseWithCovariance.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [modality]
    bufferOffset = _serializer.string(obj.modality, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DetectedPerson
    let len;
    let data = new DetectedPerson(null);
    // Deserialize message field [detection_id]
    data.detection_id = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.PoseWithCovariance.deserialize(buffer, bufferOffset);
    // Deserialize message field [modality]
    data.modality = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.modality);
    return length + 364;
  }

  static datatype() {
    // Returns string type for a message object
    return 'spencer_tracking_msgs/DetectedPerson';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '62855d424a3d5f142c0e8f5f63be52fe';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new DetectedPerson(null);
    if (msg.detection_id !== undefined) {
      resolved.detection_id = msg.detection_id;
    }
    else {
      resolved.detection_id = 0
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0.0
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.PoseWithCovariance.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.PoseWithCovariance()
    }

    if (msg.modality !== undefined) {
      resolved.modality = msg.modality;
    }
    else {
      resolved.modality = ''
    }

    return resolved;
    }
};

// Constants for message
DetectedPerson.Constants = {
  MODALITY_GENERIC_LASER_2D: 'laser2d',
  MODALITY_GENERIC_LASER_3D: 'laser3d',
  MODALITY_GENERIC_MONOCULAR_VISION: 'mono',
  MODALITY_GENERIC_STEREO_VISION: 'stereo',
  MODALITY_GENERIC_RGBD: 'rgbd',
}

module.exports = DetectedPerson;
