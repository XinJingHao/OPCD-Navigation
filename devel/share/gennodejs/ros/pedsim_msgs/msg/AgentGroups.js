// Auto-generated. Do not edit!

// (in-package pedsim_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let AgentGroup = require('./AgentGroup.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class AgentGroups {
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
    // Serializes a message object of type AgentGroups
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [groups]
    // Serialize the length for message field [groups]
    bufferOffset = _serializer.uint32(obj.groups.length, buffer, bufferOffset);
    obj.groups.forEach((val) => {
      bufferOffset = AgentGroup.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AgentGroups
    let len;
    let data = new AgentGroups(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [groups]
    // Deserialize array length for message field [groups]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.groups = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.groups[i] = AgentGroup.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.groups.forEach((val) => {
      length += AgentGroup.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pedsim_msgs/AgentGroups';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6166f52c402612b904d0fda690f00b1c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    pedsim_msgs/AgentGroup[] groups
    
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
    MSG: pedsim_msgs/AgentGroup
    Header header
    uint16 group_id
    float64 age
    uint64[] members
    geometry_msgs/Pose center_of_mass
    
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
    const resolved = new AgentGroups(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.groups !== undefined) {
      resolved.groups = new Array(msg.groups.length);
      for (let i = 0; i < resolved.groups.length; ++i) {
        resolved.groups[i] = AgentGroup.Resolve(msg.groups[i]);
      }
    }
    else {
      resolved.groups = []
    }

    return resolved;
    }
};

module.exports = AgentGroups;
