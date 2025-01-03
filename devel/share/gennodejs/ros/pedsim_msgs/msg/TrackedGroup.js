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

class TrackedGroup {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.group_id = null;
      this.age = null;
      this.centerOfGravity = null;
      this.track_ids = null;
    }
    else {
      if (initObj.hasOwnProperty('group_id')) {
        this.group_id = initObj.group_id
      }
      else {
        this.group_id = 0;
      }
      if (initObj.hasOwnProperty('age')) {
        this.age = initObj.age
      }
      else {
        this.age = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('centerOfGravity')) {
        this.centerOfGravity = initObj.centerOfGravity
      }
      else {
        this.centerOfGravity = new geometry_msgs.msg.PoseWithCovariance();
      }
      if (initObj.hasOwnProperty('track_ids')) {
        this.track_ids = initObj.track_ids
      }
      else {
        this.track_ids = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrackedGroup
    // Serialize message field [group_id]
    bufferOffset = _serializer.uint64(obj.group_id, buffer, bufferOffset);
    // Serialize message field [age]
    bufferOffset = _serializer.duration(obj.age, buffer, bufferOffset);
    // Serialize message field [centerOfGravity]
    bufferOffset = geometry_msgs.msg.PoseWithCovariance.serialize(obj.centerOfGravity, buffer, bufferOffset);
    // Serialize message field [track_ids]
    bufferOffset = _arraySerializer.uint64(obj.track_ids, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrackedGroup
    let len;
    let data = new TrackedGroup(null);
    // Deserialize message field [group_id]
    data.group_id = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [age]
    data.age = _deserializer.duration(buffer, bufferOffset);
    // Deserialize message field [centerOfGravity]
    data.centerOfGravity = geometry_msgs.msg.PoseWithCovariance.deserialize(buffer, bufferOffset);
    // Deserialize message field [track_ids]
    data.track_ids = _arrayDeserializer.uint64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.track_ids.length;
    return length + 364;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pedsim_msgs/TrackedGroup';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6a5318bfb8e49948a4dc15c1267f7e54';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message defining a tracked group
    #
    
    uint64      group_id        # unique identifier of the target, consistent over time
    
    duration    age             # age of the group
    
    geometry_msgs/PoseWithCovariance    centerOfGravity   # mean and covariance of the group (calculated from its person tracks)
    
    uint64[]    track_ids       # IDs of the tracked persons in this group. See srl_tracking_msgs/TrackedPersons
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
    const resolved = new TrackedGroup(null);
    if (msg.group_id !== undefined) {
      resolved.group_id = msg.group_id;
    }
    else {
      resolved.group_id = 0
    }

    if (msg.age !== undefined) {
      resolved.age = msg.age;
    }
    else {
      resolved.age = {secs: 0, nsecs: 0}
    }

    if (msg.centerOfGravity !== undefined) {
      resolved.centerOfGravity = geometry_msgs.msg.PoseWithCovariance.Resolve(msg.centerOfGravity)
    }
    else {
      resolved.centerOfGravity = new geometry_msgs.msg.PoseWithCovariance()
    }

    if (msg.track_ids !== undefined) {
      resolved.track_ids = msg.track_ids;
    }
    else {
      resolved.track_ids = []
    }

    return resolved;
    }
};

module.exports = TrackedGroup;
