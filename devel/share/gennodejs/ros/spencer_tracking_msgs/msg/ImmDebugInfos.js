// Auto-generated. Do not edit!

// (in-package spencer_tracking_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ImmDebugInfo = require('./ImmDebugInfo.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ImmDebugInfos {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.infos = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('infos')) {
        this.infos = initObj.infos
      }
      else {
        this.infos = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ImmDebugInfos
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [infos]
    // Serialize the length for message field [infos]
    bufferOffset = _serializer.uint32(obj.infos.length, buffer, bufferOffset);
    obj.infos.forEach((val) => {
      bufferOffset = ImmDebugInfo.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ImmDebugInfos
    let len;
    let data = new ImmDebugInfos(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [infos]
    // Deserialize array length for message field [infos]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.infos = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.infos[i] = ImmDebugInfo.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.infos.forEach((val) => {
      length += ImmDebugInfo.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'spencer_tracking_msgs/ImmDebugInfos';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ce7fa675b582455db7386ac3eaa36b5b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message with all debug infos per frame
    #
    
    Header              header      # Header containing timestamp etc. of this message
    ImmDebugInfo[]      infos      # All persons that are currently being tracked
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
    MSG: spencer_tracking_msgs/ImmDebugInfo
    # Message for passing debug information of filter performance
    #
    
    uint64      track_id        # unique identifier of the target, consistent over time
    float64      innovation      # innovation of prediction and associated observation
    float64      CpXX            # variance of prediction acc. to x
    float64      CpYY            # variance of prediction acc. to y
    float64      CXX             # variance of state acc. to x
    float64      CYY             # variance of state acc. to y
    float64[]    modeProbabilities# array containing mode probabilities
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ImmDebugInfos(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.infos !== undefined) {
      resolved.infos = new Array(msg.infos.length);
      for (let i = 0; i < resolved.infos.length; ++i) {
        resolved.infos[i] = ImmDebugInfo.Resolve(msg.infos[i]);
      }
    }
    else {
      resolved.infos = []
    }

    return resolved;
    }
};

module.exports = ImmDebugInfos;
