// Auto-generated. Do not edit!

// (in-package spencer_tracking_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TrackedPerson = require('./TrackedPerson.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class TrackedPersons {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.tracks = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('tracks')) {
        this.tracks = initObj.tracks
      }
      else {
        this.tracks = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrackedPersons
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [tracks]
    // Serialize the length for message field [tracks]
    bufferOffset = _serializer.uint32(obj.tracks.length, buffer, bufferOffset);
    obj.tracks.forEach((val) => {
      bufferOffset = TrackedPerson.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrackedPersons
    let len;
    let data = new TrackedPersons(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [tracks]
    // Deserialize array length for message field [tracks]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.tracks = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.tracks[i] = TrackedPerson.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 706 * object.tracks.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'spencer_tracking_msgs/TrackedPersons';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '21c0b1a57c4933e68f39aa3802861828';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message with all currently tracked persons 
    #
    
    Header              header      # Header containing timestamp etc. of this message
    TrackedPerson[]     tracks      # All persons that are currently being tracked
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
    MSG: spencer_tracking_msgs/TrackedPerson
    # Message defining a tracked person
    #
    
    uint64      track_id        # unique identifier of the target, consistent over time
    bool        is_occluded     # if the track is currently not observable in a physical way
    bool        is_matched      # if the track is currently matched by a detection
    uint64      detection_id    # id of the corresponding detection in the current cycle (undefined if occluded)
    duration    age             # age of the track
    
    # The following fields are extracted from the Kalman state x and its covariance C
    
    geometry_msgs/PoseWithCovariance       pose   # pose of the track (z value and orientation might not be set, check if corresponding variance on diagonal is > 99999)
    
    geometry_msgs/TwistWithCovariance   twist     # velocity of the track (z value and rotational velocities might not be set, check if corresponding variance on diagonal is > 99999)
    
    ================================================================================
    MSG: geometry_msgs/PoseWithCovariance
    # This represents a pose in free space with uncertainty.
    
    Pose pose
    
    # Row-major representation of the 6x6 covariance matrix
    # The orientation parameters use a fixed-axis representation.
    # In order, the parameters are:
    # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    float64[36] covariance
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: geometry_msgs/TwistWithCovariance
    # This expresses velocity in free space with uncertainty.
    
    Twist twist
    
    # Row-major representation of the 6x6 covariance matrix
    # The orientation parameters use a fixed-axis representation.
    # In order, the parameters are:
    # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    float64[36] covariance
    
    ================================================================================
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrackedPersons(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.tracks !== undefined) {
      resolved.tracks = new Array(msg.tracks.length);
      for (let i = 0; i < resolved.tracks.length; ++i) {
        resolved.tracks[i] = TrackedPerson.Resolve(msg.tracks[i]);
      }
    }
    else {
      resolved.tracks = []
    }

    return resolved;
    }
};

module.exports = TrackedPersons;
