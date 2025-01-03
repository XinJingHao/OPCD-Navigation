// Auto-generated. Do not edit!

// (in-package spencer_human_attribute_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CategoricalAttribute = require('./CategoricalAttribute.js');
let ScalarAttribute = require('./ScalarAttribute.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class HumanAttributes {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.categoricalAttributes = null;
      this.scalarAttributes = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('categoricalAttributes')) {
        this.categoricalAttributes = initObj.categoricalAttributes
      }
      else {
        this.categoricalAttributes = [];
      }
      if (initObj.hasOwnProperty('scalarAttributes')) {
        this.scalarAttributes = initObj.scalarAttributes
      }
      else {
        this.scalarAttributes = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HumanAttributes
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [categoricalAttributes]
    // Serialize the length for message field [categoricalAttributes]
    bufferOffset = _serializer.uint32(obj.categoricalAttributes.length, buffer, bufferOffset);
    obj.categoricalAttributes.forEach((val) => {
      bufferOffset = CategoricalAttribute.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [scalarAttributes]
    // Serialize the length for message field [scalarAttributes]
    bufferOffset = _serializer.uint32(obj.scalarAttributes.length, buffer, bufferOffset);
    obj.scalarAttributes.forEach((val) => {
      bufferOffset = ScalarAttribute.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HumanAttributes
    let len;
    let data = new HumanAttributes(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [categoricalAttributes]
    // Deserialize array length for message field [categoricalAttributes]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.categoricalAttributes = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.categoricalAttributes[i] = CategoricalAttribute.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [scalarAttributes]
    // Deserialize array length for message field [scalarAttributes]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.scalarAttributes = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.scalarAttributes[i] = ScalarAttribute.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.categoricalAttributes.forEach((val) => {
      length += CategoricalAttribute.getMessageSize(val);
    });
    object.scalarAttributes.forEach((val) => {
      length += ScalarAttribute.getMessageSize(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'spencer_human_attribute_msgs/HumanAttributes';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0ce67039f788378180e089c471174c58';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    # One entry per attribute type per person
    CategoricalAttribute[]   categoricalAttributes
    ScalarAttribute[]       scalarAttributes
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
    MSG: spencer_human_attribute_msgs/CategoricalAttribute
    uint64                  subject_id              # either an observation ID or a track ID (if information has been integrated over time); should be encoded in topic name
    string                  type                    # type of the attribute, see constants below
    
    string[]                values                  # values, each value also should have a confidence, highest-confidence attribute should come first
    float32[]               confidences             # corresponding confidences should sum up to 1.0, highest confidence comes first
    
    
    ##################################################
    ### Constants for categorical attribute types. ###
    ##################################################
    
    string      GENDER        = gender
    string      AGE_GROUP     = age_group
    
    ###################################################
    ### Constants for categorical attribute values. ###
    ###################################################
    
    string      GENDER_MALE             = male
    string      GENDER_FEMALE           = female
    
    # Age groups are based upon the categories from the "Images Of Groups" RGB database
    string      AGE_GROUP_0_TO_2        = 0-2
    string      AGE_GROUP_3_TO_7        = 3-7
    string      AGE_GROUP_8_TO_12       = 8-12
    string      AGE_GROUP_13_TO_19      = 13-19
    string      AGE_GROUP_20_TO_36      = 20-36
    string      AGE_GROUP_37_TO_65      = 37-65
    string      AGE_GROUP_66_TO_99      = 66-99
    
    
    
    
    ================================================================================
    MSG: spencer_human_attribute_msgs/ScalarAttribute
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
    const resolved = new HumanAttributes(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.categoricalAttributes !== undefined) {
      resolved.categoricalAttributes = new Array(msg.categoricalAttributes.length);
      for (let i = 0; i < resolved.categoricalAttributes.length; ++i) {
        resolved.categoricalAttributes[i] = CategoricalAttribute.Resolve(msg.categoricalAttributes[i]);
      }
    }
    else {
      resolved.categoricalAttributes = []
    }

    if (msg.scalarAttributes !== undefined) {
      resolved.scalarAttributes = new Array(msg.scalarAttributes.length);
      for (let i = 0; i < resolved.scalarAttributes.length; ++i) {
        resolved.scalarAttributes[i] = ScalarAttribute.Resolve(msg.scalarAttributes[i]);
      }
    }
    else {
      resolved.scalarAttributes = []
    }

    return resolved;
    }
};

module.exports = HumanAttributes;
