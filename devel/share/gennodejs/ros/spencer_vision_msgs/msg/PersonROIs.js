// Auto-generated. Do not edit!

// (in-package spencer_vision_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PersonROI = require('./PersonROI.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PersonROIs {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.rgb_topic = null;
      this.depth_topic = null;
      this.elements = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('rgb_topic')) {
        this.rgb_topic = initObj.rgb_topic
      }
      else {
        this.rgb_topic = '';
      }
      if (initObj.hasOwnProperty('depth_topic')) {
        this.depth_topic = initObj.depth_topic
      }
      else {
        this.depth_topic = '';
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
    // Serializes a message object of type PersonROIs
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [rgb_topic]
    bufferOffset = _serializer.string(obj.rgb_topic, buffer, bufferOffset);
    // Serialize message field [depth_topic]
    bufferOffset = _serializer.string(obj.depth_topic, buffer, bufferOffset);
    // Serialize message field [elements]
    // Serialize the length for message field [elements]
    bufferOffset = _serializer.uint32(obj.elements.length, buffer, bufferOffset);
    obj.elements.forEach((val) => {
      bufferOffset = PersonROI.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PersonROIs
    let len;
    let data = new PersonROIs(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [rgb_topic]
    data.rgb_topic = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [depth_topic]
    data.depth_topic = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [elements]
    // Deserialize array length for message field [elements]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.elements = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.elements[i] = PersonROI.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.rgb_topic);
    length += _getByteLength(object.depth_topic);
    length += 25 * object.elements.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'spencer_vision_msgs/PersonROIs';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '39d733db5b2ece3bd129f8a360116d23';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message describing an array of rectangular regions of interest in a depth or RGB image containing a part of a person (e.g. head, face, full body...), which is usually encoded in the topic title
    #
    
    std_msgs/Header     header
    
    string              rgb_topic       # not necessarily the raw camera output; might be preprocessed for rectification etc.
    string              depth_topic     # might not be set if depth is not available, otherwise it is the registered depth image
    
    PersonROI[]         elements
    
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
    MSG: spencer_vision_msgs/PersonROI
    # Message describing a rectangular region of interest in a depth or RGB image containing a part of a person (e.g. head, face, full body...), which is usually encoded in the topic title
    #
    
    uint64          detection_id
    
    sensor_msgs/RegionOfInterest    roi
    
    
    ================================================================================
    MSG: sensor_msgs/RegionOfInterest
    # This message is used to specify a region of interest within an image.
    #
    # When used to specify the ROI setting of the camera when the image was
    # taken, the height and width fields should either match the height and
    # width fields for the associated image; or height = width = 0
    # indicates that the full resolution image was captured.
    
    uint32 x_offset  # Leftmost pixel of the ROI
                     # (0 if the ROI includes the left edge of the image)
    uint32 y_offset  # Topmost pixel of the ROI
                     # (0 if the ROI includes the top edge of the image)
    uint32 height    # Height of ROI
    uint32 width     # Width of ROI
    
    # True if a distinct rectified ROI should be calculated from the "raw"
    # ROI in this message. Typically this should be False if the full image
    # is captured (ROI not used), and True if a subwindow is captured (ROI
    # used).
    bool do_rectify
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PersonROIs(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.rgb_topic !== undefined) {
      resolved.rgb_topic = msg.rgb_topic;
    }
    else {
      resolved.rgb_topic = ''
    }

    if (msg.depth_topic !== undefined) {
      resolved.depth_topic = msg.depth_topic;
    }
    else {
      resolved.depth_topic = ''
    }

    if (msg.elements !== undefined) {
      resolved.elements = new Array(msg.elements.length);
      for (let i = 0; i < resolved.elements.length; ++i) {
        resolved.elements[i] = PersonROI.Resolve(msg.elements[i]);
      }
    }
    else {
      resolved.elements = []
    }

    return resolved;
    }
};

module.exports = PersonROIs;
