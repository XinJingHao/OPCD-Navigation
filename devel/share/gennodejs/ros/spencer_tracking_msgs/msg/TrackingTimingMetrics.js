// Auto-generated. Do not edit!

// (in-package spencer_tracking_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class TrackingTimingMetrics {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.track_count = null;
      this.cycle_no = null;
      this.average_cycle_time = null;
      this.cycle_time = null;
      this.average_processing_rate = null;
      this.cpu_load = null;
      this.average_cpu_load = null;
      this.elapsed_time = null;
      this.elapsed_cpu_time = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('track_count')) {
        this.track_count = initObj.track_count
      }
      else {
        this.track_count = 0;
      }
      if (initObj.hasOwnProperty('cycle_no')) {
        this.cycle_no = initObj.cycle_no
      }
      else {
        this.cycle_no = 0;
      }
      if (initObj.hasOwnProperty('average_cycle_time')) {
        this.average_cycle_time = initObj.average_cycle_time
      }
      else {
        this.average_cycle_time = 0.0;
      }
      if (initObj.hasOwnProperty('cycle_time')) {
        this.cycle_time = initObj.cycle_time
      }
      else {
        this.cycle_time = 0.0;
      }
      if (initObj.hasOwnProperty('average_processing_rate')) {
        this.average_processing_rate = initObj.average_processing_rate
      }
      else {
        this.average_processing_rate = 0.0;
      }
      if (initObj.hasOwnProperty('cpu_load')) {
        this.cpu_load = initObj.cpu_load
      }
      else {
        this.cpu_load = 0.0;
      }
      if (initObj.hasOwnProperty('average_cpu_load')) {
        this.average_cpu_load = initObj.average_cpu_load
      }
      else {
        this.average_cpu_load = 0.0;
      }
      if (initObj.hasOwnProperty('elapsed_time')) {
        this.elapsed_time = initObj.elapsed_time
      }
      else {
        this.elapsed_time = 0.0;
      }
      if (initObj.hasOwnProperty('elapsed_cpu_time')) {
        this.elapsed_cpu_time = initObj.elapsed_cpu_time
      }
      else {
        this.elapsed_cpu_time = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrackingTimingMetrics
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [track_count]
    bufferOffset = _serializer.uint64(obj.track_count, buffer, bufferOffset);
    // Serialize message field [cycle_no]
    bufferOffset = _serializer.uint64(obj.cycle_no, buffer, bufferOffset);
    // Serialize message field [average_cycle_time]
    bufferOffset = _serializer.float32(obj.average_cycle_time, buffer, bufferOffset);
    // Serialize message field [cycle_time]
    bufferOffset = _serializer.float32(obj.cycle_time, buffer, bufferOffset);
    // Serialize message field [average_processing_rate]
    bufferOffset = _serializer.float32(obj.average_processing_rate, buffer, bufferOffset);
    // Serialize message field [cpu_load]
    bufferOffset = _serializer.float32(obj.cpu_load, buffer, bufferOffset);
    // Serialize message field [average_cpu_load]
    bufferOffset = _serializer.float32(obj.average_cpu_load, buffer, bufferOffset);
    // Serialize message field [elapsed_time]
    bufferOffset = _serializer.float32(obj.elapsed_time, buffer, bufferOffset);
    // Serialize message field [elapsed_cpu_time]
    bufferOffset = _serializer.float32(obj.elapsed_cpu_time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrackingTimingMetrics
    let len;
    let data = new TrackingTimingMetrics(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [track_count]
    data.track_count = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [cycle_no]
    data.cycle_no = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [average_cycle_time]
    data.average_cycle_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cycle_time]
    data.cycle_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [average_processing_rate]
    data.average_processing_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cpu_load]
    data.cpu_load = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [average_cpu_load]
    data.average_cpu_load = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [elapsed_time]
    data.elapsed_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [elapsed_cpu_time]
    data.elapsed_cpu_time = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'spencer_tracking_msgs/TrackingTimingMetrics';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e5e4499959c0302da5d8864744a1d65a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message with timing values for analyzing the efficency 
    #
    
    Header              header                  # Header containing timestamp etc. of this message
    uint64              track_count             # number of person currentl tracked
    uint64              cycle_no                # incremented number of cycles        
    float32             average_cycle_time      # average time for one cycle of tracker
    float32             cycle_time              # cycle time of current cycle
    float32             average_processing_rate # average frequency of processing data
    float32             cpu_load                # current cpu load
    float32             average_cpu_load        # average cpu load
    float32             elapsed_time            # elapsed seconds since start
    float32             elapsed_cpu_time        # elapsed cpu time since start
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrackingTimingMetrics(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.track_count !== undefined) {
      resolved.track_count = msg.track_count;
    }
    else {
      resolved.track_count = 0
    }

    if (msg.cycle_no !== undefined) {
      resolved.cycle_no = msg.cycle_no;
    }
    else {
      resolved.cycle_no = 0
    }

    if (msg.average_cycle_time !== undefined) {
      resolved.average_cycle_time = msg.average_cycle_time;
    }
    else {
      resolved.average_cycle_time = 0.0
    }

    if (msg.cycle_time !== undefined) {
      resolved.cycle_time = msg.cycle_time;
    }
    else {
      resolved.cycle_time = 0.0
    }

    if (msg.average_processing_rate !== undefined) {
      resolved.average_processing_rate = msg.average_processing_rate;
    }
    else {
      resolved.average_processing_rate = 0.0
    }

    if (msg.cpu_load !== undefined) {
      resolved.cpu_load = msg.cpu_load;
    }
    else {
      resolved.cpu_load = 0.0
    }

    if (msg.average_cpu_load !== undefined) {
      resolved.average_cpu_load = msg.average_cpu_load;
    }
    else {
      resolved.average_cpu_load = 0.0
    }

    if (msg.elapsed_time !== undefined) {
      resolved.elapsed_time = msg.elapsed_time;
    }
    else {
      resolved.elapsed_time = 0.0
    }

    if (msg.elapsed_cpu_time !== undefined) {
      resolved.elapsed_cpu_time = msg.elapsed_cpu_time;
    }
    else {
      resolved.elapsed_cpu_time = 0.0
    }

    return resolved;
    }
};

module.exports = TrackingTimingMetrics;
