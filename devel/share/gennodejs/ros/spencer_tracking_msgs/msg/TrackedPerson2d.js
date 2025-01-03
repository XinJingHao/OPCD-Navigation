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

class TrackedPerson2d {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.track_id = null;
      this.person_height = null;
      this.x = null;
      this.y = null;
      this.w = null;
      this.h = null;
      this.depth = null;
    }
    else {
      if (initObj.hasOwnProperty('track_id')) {
        this.track_id = initObj.track_id
      }
      else {
        this.track_id = 0;
      }
      if (initObj.hasOwnProperty('person_height')) {
        this.person_height = initObj.person_height
      }
      else {
        this.person_height = 0.0;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0;
      }
      if (initObj.hasOwnProperty('w')) {
        this.w = initObj.w
      }
      else {
        this.w = 0;
      }
      if (initObj.hasOwnProperty('h')) {
        this.h = initObj.h
      }
      else {
        this.h = 0;
      }
      if (initObj.hasOwnProperty('depth')) {
        this.depth = initObj.depth
      }
      else {
        this.depth = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrackedPerson2d
    // Serialize message field [track_id]
    bufferOffset = _serializer.uint64(obj.track_id, buffer, bufferOffset);
    // Serialize message field [person_height]
    bufferOffset = _serializer.float32(obj.person_height, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.int32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.int32(obj.y, buffer, bufferOffset);
    // Serialize message field [w]
    bufferOffset = _serializer.uint32(obj.w, buffer, bufferOffset);
    // Serialize message field [h]
    bufferOffset = _serializer.uint32(obj.h, buffer, bufferOffset);
    // Serialize message field [depth]
    bufferOffset = _serializer.float32(obj.depth, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrackedPerson2d
    let len;
    let data = new TrackedPerson2d(null);
    // Deserialize message field [track_id]
    data.track_id = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [person_height]
    data.person_height = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [w]
    data.w = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [h]
    data.h = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [depth]
    data.depth = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'spencer_tracking_msgs/TrackedPerson2d';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '12df4823c658df0d660d2a5c68ae4aea';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message defining a 2d image bbox of a tracked person
    #
    
    uint64      track_id        # unique identifier of the target, consistent over time
    float32     person_height   # 3d height of person in m
    int32       x               # top left corner x of 2d image bbox
    int32       y               # top left corner y of 2d image bbox
    uint32      w               # width of 2d image bbox
    uint32      h               # height of 2d image bbox
    float32     depth           # distance from the camera in m
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrackedPerson2d(null);
    if (msg.track_id !== undefined) {
      resolved.track_id = msg.track_id;
    }
    else {
      resolved.track_id = 0
    }

    if (msg.person_height !== undefined) {
      resolved.person_height = msg.person_height;
    }
    else {
      resolved.person_height = 0.0
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0
    }

    if (msg.w !== undefined) {
      resolved.w = msg.w;
    }
    else {
      resolved.w = 0
    }

    if (msg.h !== undefined) {
      resolved.h = msg.h;
    }
    else {
      resolved.h = 0
    }

    if (msg.depth !== undefined) {
      resolved.depth = msg.depth;
    }
    else {
      resolved.depth = 0.0
    }

    return resolved;
    }
};

module.exports = TrackedPerson2d;
