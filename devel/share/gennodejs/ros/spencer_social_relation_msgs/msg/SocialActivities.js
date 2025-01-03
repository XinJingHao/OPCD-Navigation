// Auto-generated. Do not edit!

// (in-package spencer_social_relation_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SocialActivity = require('./SocialActivity.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SocialActivities {
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
    // Serializes a message object of type SocialActivities
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [elements]
    // Serialize the length for message field [elements]
    bufferOffset = _serializer.uint32(obj.elements.length, buffer, bufferOffset);
    obj.elements.forEach((val) => {
      bufferOffset = SocialActivity.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SocialActivities
    let len;
    let data = new SocialActivities(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [elements]
    // Deserialize array length for message field [elements]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.elements = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.elements[i] = SocialActivity.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.elements.forEach((val) => {
      length += SocialActivity.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'spencer_social_relation_msgs/SocialActivities';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '59f7b8ff037d2bbbd144704b9ef2b5be';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header     header
    
    # All social activities that have been detected in the current time step,
    # within sensor range of the robot.
    SocialActivity[]    elements
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
    MSG: spencer_social_relation_msgs/SocialActivity
    string      type        # see constants below
    float32     confidence  # detection confidence
    
    uint64[]      track_ids   # IDs of all person tracks involved in the activity, might be one or multiple
    
    
    # Constants for social activity type (just examples at the moment)
    string      TYPE_SHOPPING = shopping
    string      TYPE_STANDING = standing
    string      TYPE_INDIVIDUAL_MOVING = individual_moving
    string      TYPE_WAITING_IN_QUEUE = waiting_in_queue
    string      TYPE_LOOKING_AT_INFORMATION_SCREEN = looking_at_information_screen
    string      TYPE_LOOKING_AT_KIOSK = looking_at_kiosk
    string      TYPE_GROUP_ASSEMBLING = group_assembling
    string      TYPE_GROUP_MOVING = group_moving
    string      TYPE_FLOW_WITH_ROBOT = flow
    string      TYPE_ANTIFLOW_AGAINST_ROBOT = antiflow
    string      TYPE_WAITING_FOR_OTHERS = waiting_for_others
    string      TYPE_LOOKING_FOR_HELP = looking_for_help
    
    
    #string      TYPE_COMMUNICATING = communicating
    #string      TYPE_TAKING_PHOTOGRAPH = taking_photograph
    #string      TYPE_TALKING_ON_PHONE = talking_on_phone
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SocialActivities(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.elements !== undefined) {
      resolved.elements = new Array(msg.elements.length);
      for (let i = 0; i < resolved.elements.length; ++i) {
        resolved.elements[i] = SocialActivity.Resolve(msg.elements[i]);
      }
    }
    else {
      resolved.elements = []
    }

    return resolved;
    }
};

module.exports = SocialActivities;
