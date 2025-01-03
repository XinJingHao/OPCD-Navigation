// Auto-generated. Do not edit!

// (in-package spencer_vision_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

class PersonROI {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.detection_id = null;
      this.roi = null;
    }
    else {
      if (initObj.hasOwnProperty('detection_id')) {
        this.detection_id = initObj.detection_id
      }
      else {
        this.detection_id = 0;
      }
      if (initObj.hasOwnProperty('roi')) {
        this.roi = initObj.roi
      }
      else {
        this.roi = new sensor_msgs.msg.RegionOfInterest();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PersonROI
    // Serialize message field [detection_id]
    bufferOffset = _serializer.uint64(obj.detection_id, buffer, bufferOffset);
    // Serialize message field [roi]
    bufferOffset = sensor_msgs.msg.RegionOfInterest.serialize(obj.roi, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PersonROI
    let len;
    let data = new PersonROI(null);
    // Deserialize message field [detection_id]
    data.detection_id = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [roi]
    data.roi = sensor_msgs.msg.RegionOfInterest.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 25;
  }

  static datatype() {
    // Returns string type for a message object
    return 'spencer_vision_msgs/PersonROI';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4484c510821bd11dbd7b6b3627d4e4ad';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new PersonROI(null);
    if (msg.detection_id !== undefined) {
      resolved.detection_id = msg.detection_id;
    }
    else {
      resolved.detection_id = 0
    }

    if (msg.roi !== undefined) {
      resolved.roi = sensor_msgs.msg.RegionOfInterest.Resolve(msg.roi)
    }
    else {
      resolved.roi = new sensor_msgs.msg.RegionOfInterest()
    }

    return resolved;
    }
};

module.exports = PersonROI;
