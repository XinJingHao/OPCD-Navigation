// Auto-generated. Do not edit!

// (in-package spencer_tracking_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TrackedPerson2d = require('./TrackedPerson2d.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class TrackedPersons2d {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.boxes = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('boxes')) {
        this.boxes = initObj.boxes
      }
      else {
        this.boxes = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrackedPersons2d
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [boxes]
    // Serialize the length for message field [boxes]
    bufferOffset = _serializer.uint32(obj.boxes.length, buffer, bufferOffset);
    obj.boxes.forEach((val) => {
      bufferOffset = TrackedPerson2d.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrackedPersons2d
    let len;
    let data = new TrackedPersons2d(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [boxes]
    // Deserialize array length for message field [boxes]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.boxes = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.boxes[i] = TrackedPerson2d.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 32 * object.boxes.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'spencer_tracking_msgs/TrackedPersons2d';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '972b7d693ce31c3b18f092f43387621c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message with all 2d bbox in image of currently tracked persons 
    #
    
    Header                header      # Header containing timestamp etc. of this message
    TrackedPerson2d[]     boxes       # All persons that are currently being tracked (2d image bbox)
    
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
    MSG: spencer_tracking_msgs/TrackedPerson2d
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
    const resolved = new TrackedPersons2d(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.boxes !== undefined) {
      resolved.boxes = new Array(msg.boxes.length);
      for (let i = 0; i < resolved.boxes.length; ++i) {
        resolved.boxes[i] = TrackedPerson2d.Resolve(msg.boxes[i]);
      }
    }
    else {
      resolved.boxes = []
    }

    return resolved;
    }
};

module.exports = TrackedPersons2d;
