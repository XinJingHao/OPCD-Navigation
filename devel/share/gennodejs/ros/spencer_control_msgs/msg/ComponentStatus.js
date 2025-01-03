// Auto-generated. Do not edit!

// (in-package spencer_control_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ComponentStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.alive = null;
      this.detail = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('alive')) {
        this.alive = initObj.alive
      }
      else {
        this.alive = false;
      }
      if (initObj.hasOwnProperty('detail')) {
        this.detail = initObj.detail
      }
      else {
        this.detail = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ComponentStatus
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [alive]
    bufferOffset = _serializer.bool(obj.alive, buffer, bufferOffset);
    // Serialize message field [detail]
    bufferOffset = _serializer.string(obj.detail, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ComponentStatus
    let len;
    let data = new ComponentStatus(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [alive]
    data.alive = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [detail]
    data.detail = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.name);
    length += _getByteLength(object.detail);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'spencer_control_msgs/ComponentStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b570ce0ae5316c3ad53df1d245486902';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string name
    bool alive
    string detail
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ComponentStatus(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.alive !== undefined) {
      resolved.alive = msg.alive;
    }
    else {
      resolved.alive = false
    }

    if (msg.detail !== undefined) {
      resolved.detail = msg.detail;
    }
    else {
      resolved.detail = ''
    }

    return resolved;
    }
};

module.exports = ComponentStatus;
