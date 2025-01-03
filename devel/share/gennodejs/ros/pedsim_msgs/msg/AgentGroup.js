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
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class AgentGroup {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.group_id = null;
      this.age = null;
      this.members = null;
      this.center_of_mass = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
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
        this.age = 0.0;
      }
      if (initObj.hasOwnProperty('members')) {
        this.members = initObj.members
      }
      else {
        this.members = [];
      }
      if (initObj.hasOwnProperty('center_of_mass')) {
        this.center_of_mass = initObj.center_of_mass
      }
      else {
        this.center_of_mass = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AgentGroup
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [group_id]
    bufferOffset = _serializer.uint16(obj.group_id, buffer, bufferOffset);
    // Serialize message field [age]
    bufferOffset = _serializer.float64(obj.age, buffer, bufferOffset);
    // Serialize message field [members]
    bufferOffset = _arraySerializer.uint64(obj.members, buffer, bufferOffset, null);
    // Serialize message field [center_of_mass]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.center_of_mass, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AgentGroup
    let len;
    let data = new AgentGroup(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [group_id]
    data.group_id = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [age]
    data.age = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [members]
    data.members = _arrayDeserializer.uint64(buffer, bufferOffset, null)
    // Deserialize message field [center_of_mass]
    data.center_of_mass = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 8 * object.members.length;
    return length + 70;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pedsim_msgs/AgentGroup';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5fd22ef81e8e7ee63e1028ae9b798458';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    uint16 group_id
    float64 age
    uint64[] members
    geometry_msgs/Pose center_of_mass
    
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
    const resolved = new AgentGroup(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

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
      resolved.age = 0.0
    }

    if (msg.members !== undefined) {
      resolved.members = msg.members;
    }
    else {
      resolved.members = []
    }

    if (msg.center_of_mass !== undefined) {
      resolved.center_of_mass = geometry_msgs.msg.Pose.Resolve(msg.center_of_mass)
    }
    else {
      resolved.center_of_mass = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

module.exports = AgentGroup;
