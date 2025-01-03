// Auto-generated. Do not edit!

// (in-package pedsim_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SocialRelation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.type = null;
      this.strength = null;
      this.track1_id = null;
      this.track2_id = null;
    }
    else {
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = '';
      }
      if (initObj.hasOwnProperty('strength')) {
        this.strength = initObj.strength
      }
      else {
        this.strength = 0.0;
      }
      if (initObj.hasOwnProperty('track1_id')) {
        this.track1_id = initObj.track1_id
      }
      else {
        this.track1_id = 0;
      }
      if (initObj.hasOwnProperty('track2_id')) {
        this.track2_id = initObj.track2_id
      }
      else {
        this.track2_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SocialRelation
    // Serialize message field [type]
    bufferOffset = _serializer.string(obj.type, buffer, bufferOffset);
    // Serialize message field [strength]
    bufferOffset = _serializer.float32(obj.strength, buffer, bufferOffset);
    // Serialize message field [track1_id]
    bufferOffset = _serializer.uint32(obj.track1_id, buffer, bufferOffset);
    // Serialize message field [track2_id]
    bufferOffset = _serializer.uint32(obj.track2_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SocialRelation
    let len;
    let data = new SocialRelation(null);
    // Deserialize message field [type]
    data.type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [strength]
    data.strength = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [track1_id]
    data.track1_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [track2_id]
    data.track2_id = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.type);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pedsim_msgs/SocialRelation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '004e9c919342c749d66dbc051dba51f6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string      type        # e.g. mother-son relationship, romantic relationship, etc.
    float32     strength    # relationship strength between 0.0 and 1.0
    
    uint32      track1_id
    uint32      track2_id
    
    
    # Constants for type (just examples at the moment)
    string      TYPE_SPATIAL  = "spatial"
    string      TYPE_ROMANTIC = "romantic"
    string      TYPE_PARENT_CHILD = "parent_child"
    string      TYPE_FRIENDSHIP = "friendship"
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SocialRelation(null);
    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = ''
    }

    if (msg.strength !== undefined) {
      resolved.strength = msg.strength;
    }
    else {
      resolved.strength = 0.0
    }

    if (msg.track1_id !== undefined) {
      resolved.track1_id = msg.track1_id;
    }
    else {
      resolved.track1_id = 0
    }

    if (msg.track2_id !== undefined) {
      resolved.track2_id = msg.track2_id;
    }
    else {
      resolved.track2_id = 0
    }

    return resolved;
    }
};

// Constants for message
SocialRelation.Constants = {
  TYPE_SPATIAL: '"spatial"',
  TYPE_ROMANTIC: '"romantic"',
  TYPE_PARENT_CHILD: '"parent_child"',
  TYPE_FRIENDSHIP: '"friendship"',
}

module.exports = SocialRelation;
