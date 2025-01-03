// Auto-generated. Do not edit!

// (in-package spencer_tracking_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TrackedGroup = require('./TrackedGroup.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class TrackedGroups {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.groups = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('groups')) {
        this.groups = initObj.groups
      }
      else {
        this.groups = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrackedGroups
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [groups]
    // Serialize the length for message field [groups]
    bufferOffset = _serializer.uint32(obj.groups.length, buffer, bufferOffset);
    obj.groups.forEach((val) => {
      bufferOffset = TrackedGroup.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrackedGroups
    let len;
    let data = new TrackedGroups(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [groups]
    // Deserialize array length for message field [groups]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.groups = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.groups[i] = TrackedGroup.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.groups.forEach((val) => {
      length += TrackedGroup.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'spencer_tracking_msgs/TrackedGroups';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '44229e5f365e63f958cbe69d06c05cc4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message with all currently tracked groups 
    #
    
    Header              header      # Header containing timestamp etc. of this message
    TrackedGroup[]      groups      # All groups that are currently being tracked
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
    MSG: spencer_tracking_msgs/TrackedGroup
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
    const resolved = new TrackedGroups(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.groups !== undefined) {
      resolved.groups = new Array(msg.groups.length);
      for (let i = 0; i < resolved.groups.length; ++i) {
        resolved.groups[i] = TrackedGroup.Resolve(msg.groups[i]);
      }
    }
    else {
      resolved.groups = []
    }

    return resolved;
    }
};

module.exports = TrackedGroups;
