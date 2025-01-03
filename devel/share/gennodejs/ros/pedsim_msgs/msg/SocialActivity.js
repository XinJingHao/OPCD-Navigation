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

class SocialActivity {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.type = null;
      this.confidence = null;
      this.track_ids = null;
    }
    else {
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = '';
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0.0;
      }
      if (initObj.hasOwnProperty('track_ids')) {
        this.track_ids = initObj.track_ids
      }
      else {
        this.track_ids = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SocialActivity
    // Serialize message field [type]
    bufferOffset = _serializer.string(obj.type, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float32(obj.confidence, buffer, bufferOffset);
    // Serialize message field [track_ids]
    bufferOffset = _arraySerializer.uint64(obj.track_ids, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SocialActivity
    let len;
    let data = new SocialActivity(null);
    // Deserialize message field [type]
    data.type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [track_ids]
    data.track_ids = _arrayDeserializer.uint64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.type);
    length += 8 * object.track_ids.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pedsim_msgs/SocialActivity';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '064aac12909022edb4df5d568c180144';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new SocialActivity(null);
    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = ''
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0.0
    }

    if (msg.track_ids !== undefined) {
      resolved.track_ids = msg.track_ids;
    }
    else {
      resolved.track_ids = []
    }

    return resolved;
    }
};

// Constants for message
SocialActivity.Constants = {
  TYPE_SHOPPING: 'shopping',
  TYPE_STANDING: 'standing',
  TYPE_INDIVIDUAL_MOVING: 'individual_moving',
  TYPE_WAITING_IN_QUEUE: 'waiting_in_queue',
  TYPE_LOOKING_AT_INFORMATION_SCREEN: 'looking_at_information_screen',
  TYPE_LOOKING_AT_KIOSK: 'looking_at_kiosk',
  TYPE_GROUP_ASSEMBLING: 'group_assembling',
  TYPE_GROUP_MOVING: 'group_moving',
  TYPE_FLOW_WITH_ROBOT: 'flow',
  TYPE_ANTIFLOW_AGAINST_ROBOT: 'antiflow',
  TYPE_WAITING_FOR_OTHERS: 'waiting_for_others',
}

module.exports = SocialActivity;
