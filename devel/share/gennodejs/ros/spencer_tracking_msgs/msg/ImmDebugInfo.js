// Auto-generated. Do not edit!

// (in-package spencer_tracking_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ImmDebugInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.track_id = null;
      this.innovation = null;
      this.CpXX = null;
      this.CpYY = null;
      this.CXX = null;
      this.CYY = null;
      this.modeProbabilities = null;
    }
    else {
      if (initObj.hasOwnProperty('track_id')) {
        this.track_id = initObj.track_id
      }
      else {
        this.track_id = 0;
      }
      if (initObj.hasOwnProperty('innovation')) {
        this.innovation = initObj.innovation
      }
      else {
        this.innovation = 0.0;
      }
      if (initObj.hasOwnProperty('CpXX')) {
        this.CpXX = initObj.CpXX
      }
      else {
        this.CpXX = 0.0;
      }
      if (initObj.hasOwnProperty('CpYY')) {
        this.CpYY = initObj.CpYY
      }
      else {
        this.CpYY = 0.0;
      }
      if (initObj.hasOwnProperty('CXX')) {
        this.CXX = initObj.CXX
      }
      else {
        this.CXX = 0.0;
      }
      if (initObj.hasOwnProperty('CYY')) {
        this.CYY = initObj.CYY
      }
      else {
        this.CYY = 0.0;
      }
      if (initObj.hasOwnProperty('modeProbabilities')) {
        this.modeProbabilities = initObj.modeProbabilities
      }
      else {
        this.modeProbabilities = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ImmDebugInfo
    // Serialize message field [track_id]
    bufferOffset = _serializer.uint64(obj.track_id, buffer, bufferOffset);
    // Serialize message field [innovation]
    bufferOffset = _serializer.float64(obj.innovation, buffer, bufferOffset);
    // Serialize message field [CpXX]
    bufferOffset = _serializer.float64(obj.CpXX, buffer, bufferOffset);
    // Serialize message field [CpYY]
    bufferOffset = _serializer.float64(obj.CpYY, buffer, bufferOffset);
    // Serialize message field [CXX]
    bufferOffset = _serializer.float64(obj.CXX, buffer, bufferOffset);
    // Serialize message field [CYY]
    bufferOffset = _serializer.float64(obj.CYY, buffer, bufferOffset);
    // Serialize message field [modeProbabilities]
    bufferOffset = _arraySerializer.float64(obj.modeProbabilities, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ImmDebugInfo
    let len;
    let data = new ImmDebugInfo(null);
    // Deserialize message field [track_id]
    data.track_id = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [innovation]
    data.innovation = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [CpXX]
    data.CpXX = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [CpYY]
    data.CpYY = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [CXX]
    data.CXX = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [CYY]
    data.CYY = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [modeProbabilities]
    data.modeProbabilities = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.modeProbabilities.length;
    return length + 52;
  }

  static datatype() {
    // Returns string type for a message object
    return 'spencer_tracking_msgs/ImmDebugInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '824cd837fd158664a6f185fb8316da53';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new ImmDebugInfo(null);
    if (msg.track_id !== undefined) {
      resolved.track_id = msg.track_id;
    }
    else {
      resolved.track_id = 0
    }

    if (msg.innovation !== undefined) {
      resolved.innovation = msg.innovation;
    }
    else {
      resolved.innovation = 0.0
    }

    if (msg.CpXX !== undefined) {
      resolved.CpXX = msg.CpXX;
    }
    else {
      resolved.CpXX = 0.0
    }

    if (msg.CpYY !== undefined) {
      resolved.CpYY = msg.CpYY;
    }
    else {
      resolved.CpYY = 0.0
    }

    if (msg.CXX !== undefined) {
      resolved.CXX = msg.CXX;
    }
    else {
      resolved.CXX = 0.0
    }

    if (msg.CYY !== undefined) {
      resolved.CYY = msg.CYY;
    }
    else {
      resolved.CYY = 0.0
    }

    if (msg.modeProbabilities !== undefined) {
      resolved.modeProbabilities = msg.modeProbabilities;
    }
    else {
      resolved.modeProbabilities = []
    }

    return resolved;
    }
};

module.exports = ImmDebugInfo;
