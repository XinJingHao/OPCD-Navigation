// Auto-generated. Do not edit!

// (in-package spencer_human_attribute_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ScalarAttribute {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.subject_id = null;
      this.type = null;
      this.values = null;
      this.confidences = null;
    }
    else {
      if (initObj.hasOwnProperty('subject_id')) {
        this.subject_id = initObj.subject_id
      }
      else {
        this.subject_id = 0;
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = '';
      }
      if (initObj.hasOwnProperty('values')) {
        this.values = initObj.values
      }
      else {
        this.values = [];
      }
      if (initObj.hasOwnProperty('confidences')) {
        this.confidences = initObj.confidences
      }
      else {
        this.confidences = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ScalarAttribute
    // Serialize message field [subject_id]
    bufferOffset = _serializer.uint64(obj.subject_id, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.string(obj.type, buffer, bufferOffset);
    // Serialize message field [values]
    bufferOffset = _arraySerializer.float32(obj.values, buffer, bufferOffset, null);
    // Serialize message field [confidences]
    bufferOffset = _arraySerializer.float32(obj.confidences, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ScalarAttribute
    let len;
    let data = new ScalarAttribute(null);
    // Deserialize message field [subject_id]
    data.subject_id = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [values]
    data.values = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [confidences]
    data.confidences = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.type);
    length += 4 * object.values.length;
    length += 4 * object.confidences.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'spencer_human_attribute_msgs/ScalarAttribute';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e10ac15d3bace9d3787523d1dd728fe0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint64                  subject_id              # either an observation ID or a track ID (if information has been integrated over time); should be encoded in topic name
    string                  type                    # type of the attribute, see constants below
    
    float32[]               values                  # values, each value also should have a confidence; highest-confidence value comes first!
    float32[]               confidences             # corresponding confidences should sum up to 1.0
    
    
    ###########################################
    ### Constants for scalar attribute types. #
    ###########################################
    
    string      PERSON_HEIGHT        = person_height
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ScalarAttribute(null);
    if (msg.subject_id !== undefined) {
      resolved.subject_id = msg.subject_id;
    }
    else {
      resolved.subject_id = 0
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = ''
    }

    if (msg.values !== undefined) {
      resolved.values = msg.values;
    }
    else {
      resolved.values = []
    }

    if (msg.confidences !== undefined) {
      resolved.confidences = msg.confidences;
    }
    else {
      resolved.confidences = []
    }

    return resolved;
    }
};

// Constants for message
ScalarAttribute.Constants = {
  PERSON_HEIGHT: 'person_height',
}

module.exports = ScalarAttribute;
