// Auto-generated. Do not edit!

// (in-package pedsim_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class TrackedPerson {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.track_id = null;
      this.is_occluded = null;
      this.is_matched = null;
      this.detection_id = null;
      this.age = null;
      this.pose = null;
      this.twist = null;
    }
    else {
      if (initObj.hasOwnProperty('track_id')) {
        this.track_id = initObj.track_id
      }
      else {
        this.track_id = 0;
      }
      if (initObj.hasOwnProperty('is_occluded')) {
        this.is_occluded = initObj.is_occluded
      }
      else {
        this.is_occluded = false;
      }
      if (initObj.hasOwnProperty('is_matched')) {
        this.is_matched = initObj.is_matched
      }
      else {
        this.is_matched = false;
      }
      if (initObj.hasOwnProperty('detection_id')) {
        this.detection_id = initObj.detection_id
      }
      else {
        this.detection_id = 0;
      }
      if (initObj.hasOwnProperty('age')) {
        this.age = initObj.age
      }
      else {
        this.age = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.PoseWithCovariance();
      }
      if (initObj.hasOwnProperty('twist')) {
        this.twist = initObj.twist
      }
      else {
        this.twist = new geometry_msgs.msg.TwistWithCovariance();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrackedPerson
    // Serialize message field [track_id]
    bufferOffset = _serializer.uint64(obj.track_id, buffer, bufferOffset);
    // Serialize message field [is_occluded]
    bufferOffset = _serializer.bool(obj.is_occluded, buffer, bufferOffset);
    // Serialize message field [is_matched]
    bufferOffset = _serializer.bool(obj.is_matched, buffer, bufferOffset);
    // Serialize message field [detection_id]
    bufferOffset = _serializer.uint64(obj.detection_id, buffer, bufferOffset);
    // Serialize message field [age]
    bufferOffset = _serializer.duration(obj.age, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.PoseWithCovariance.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [twist]
    bufferOffset = geometry_msgs.msg.TwistWithCovariance.serialize(obj.twist, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrackedPerson
    let len;
    let data = new TrackedPerson(null);
    // Deserialize message field [track_id]
    data.track_id = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [is_occluded]
    data.is_occluded = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [is_matched]
    data.is_matched = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [detection_id]
    data.detection_id = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [age]
    data.age = _deserializer.duration(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.PoseWithCovariance.deserialize(buffer, bufferOffset);
    // Deserialize message field [twist]
    data.twist = geometry_msgs.msg.TwistWithCovariance.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 706;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pedsim_msgs/TrackedPerson';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '28bdd0d6d6551c668e4fde8aecdf1885';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message defining a tracked person
    #
    
    uint64      track_id        # unique identifier of the target, consistent over time
    bool        is_occluded     # if the track is currently not observable in a physical way
    bool        is_matched      # if the track is currently matched by a detection
    uint64      detection_id    # id of the corresponding detection in the current cycle (undefined if occluded)
    duration    age             # age of the track
    
    # The following fields are extracted from the Kalman state x and its covariance C
    
    geometry_msgs/PoseWithCovariance       pose   # pose of the track (z value and orientation might not be set, check if corresponding variance on diagonal is > 99999)
    
    geometry_msgs/TwistWithCovariance   twist     # velocity of the track (z value and rotational velocities might not be set, check if corresponding variance on diagonal is > 99999)
    
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
    MSG: geometry_msgs/TwistWithCovariance
    # This expresses velocity in free space with uncertainty.
    
    Twist twist
    
    # Row-major representation of the 6x6 covariance matrix
    # The orientation parameters use a fixed-axis representation.
    # In order, the parameters are:
    # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    float64[36] covariance
    
    ================================================================================
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrackedPerson(null);
    if (msg.track_id !== undefined) {
      resolved.track_id = msg.track_id;
    }
    else {
      resolved.track_id = 0
    }

    if (msg.is_occluded !== undefined) {
      resolved.is_occluded = msg.is_occluded;
    }
    else {
      resolved.is_occluded = false
    }

    if (msg.is_matched !== undefined) {
      resolved.is_matched = msg.is_matched;
    }
    else {
      resolved.is_matched = false
    }

    if (msg.detection_id !== undefined) {
      resolved.detection_id = msg.detection_id;
    }
    else {
      resolved.detection_id = 0
    }

    if (msg.age !== undefined) {
      resolved.age = msg.age;
    }
    else {
      resolved.age = {secs: 0, nsecs: 0}
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.PoseWithCovariance.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.PoseWithCovariance()
    }

    if (msg.twist !== undefined) {
      resolved.twist = geometry_msgs.msg.TwistWithCovariance.Resolve(msg.twist)
    }
    else {
      resolved.twist = new geometry_msgs.msg.TwistWithCovariance()
    }

    return resolved;
    }
};

module.exports = TrackedPerson;
