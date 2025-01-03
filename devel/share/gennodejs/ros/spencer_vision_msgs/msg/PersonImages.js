// Auto-generated. Do not edit!

// (in-package spencer_vision_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PersonImage = require('./PersonImage.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PersonImages {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.elements = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('elements')) {
        this.elements = initObj.elements
      }
      else {
        this.elements = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PersonImages
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [elements]
    // Serialize the length for message field [elements]
    bufferOffset = _serializer.uint32(obj.elements.length, buffer, bufferOffset);
    obj.elements.forEach((val) => {
      bufferOffset = PersonImage.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PersonImages
    let len;
    let data = new PersonImages(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [elements]
    // Deserialize array length for message field [elements]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.elements = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.elements[i] = PersonImage.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.elements.forEach((val) => {
      length += PersonImage.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'spencer_vision_msgs/PersonImages';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6c5881059a7a7f9c813cdc2429f1b5cf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message describing an array of depth or RGB images containing a part of a person (e.g. head, face, full body...), which is usually encoded in the topic title
    #
    
    std_msgs/Header     header
    PersonImage[]       elements
    
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
    MSG: spencer_vision_msgs/PersonImage
    # Message describing a depth or RGB image containing a part of a person (e.g. head, face, full body...), which is usually encoded in the topic title
    #
    
    uint64              detection_id
    sensor_msgs/Image   image
    
    
    ================================================================================
    MSG: sensor_msgs/Image
    # This message contains an uncompressed image
    # (0, 0) is at top-left corner of image
    #
    
    Header header        # Header timestamp should be acquisition time of image
                         # Header frame_id should be optical frame of camera
                         # origin of frame should be optical center of camera
                         # +x should point to the right in the image
                         # +y should point down in the image
                         # +z should point into to plane of the image
                         # If the frame_id here and the frame_id of the CameraInfo
                         # message associated with the image conflict
                         # the behavior is undefined
    
    uint32 height         # image height, that is, number of rows
    uint32 width          # image width, that is, number of columns
    
    # The legal values for encoding are in file src/image_encodings.cpp
    # If you want to standardize a new string format, join
    # ros-users@lists.sourceforge.net and send an email proposing a new encoding.
    
    string encoding       # Encoding of pixels -- channel meaning, ordering, size
                          # taken from the list of strings in include/sensor_msgs/image_encodings.h
    
    uint8 is_bigendian    # is this data bigendian?
    uint32 step           # Full row length in bytes
    uint8[] data          # actual matrix data, size is (step * rows)
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PersonImages(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.elements !== undefined) {
      resolved.elements = new Array(msg.elements.length);
      for (let i = 0; i < resolved.elements.length; ++i) {
        resolved.elements[i] = PersonImage.Resolve(msg.elements[i]);
      }
    }
    else {
      resolved.elements = []
    }

    return resolved;
    }
};

module.exports = PersonImages;
