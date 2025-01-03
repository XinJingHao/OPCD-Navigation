// Auto-generated. Do not edit!

// (in-package spencer_tracking_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let PersonTrajectory = require('../msg/PersonTrajectory.js');

//-----------------------------------------------------------

class GetPersonTrajectoriesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.requested_ids = null;
      this.max_age = null;
    }
    else {
      if (initObj.hasOwnProperty('requested_ids')) {
        this.requested_ids = initObj.requested_ids
      }
      else {
        this.requested_ids = [];
      }
      if (initObj.hasOwnProperty('max_age')) {
        this.max_age = initObj.max_age
      }
      else {
        this.max_age = {secs: 0, nsecs: 0};
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPersonTrajectoriesRequest
    // Serialize message field [requested_ids]
    bufferOffset = _arraySerializer.uint64(obj.requested_ids, buffer, bufferOffset, null);
    // Serialize message field [max_age]
    bufferOffset = _serializer.duration(obj.max_age, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPersonTrajectoriesRequest
    let len;
    let data = new GetPersonTrajectoriesRequest(null);
    // Deserialize message field [requested_ids]
    data.requested_ids = _arrayDeserializer.uint64(buffer, bufferOffset, null)
    // Deserialize message field [max_age]
    data.max_age = _deserializer.duration(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.requested_ids.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'spencer_tracking_msgs/GetPersonTrajectoriesRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '771ecc55f598ab2830cd2cba4bd8c15e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint64[] requested_ids           # The IDs of the tracks you are interested in getting the trajectories of. An empty array means all available tracks.
    duration max_age                 # The maximum age of a trajectory you want to get. A duration of 0 means "since the beginning of times."
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetPersonTrajectoriesRequest(null);
    if (msg.requested_ids !== undefined) {
      resolved.requested_ids = msg.requested_ids;
    }
    else {
      resolved.requested_ids = []
    }

    if (msg.max_age !== undefined) {
      resolved.max_age = msg.max_age;
    }
    else {
      resolved.max_age = {secs: 0, nsecs: 0}
    }

    return resolved;
    }
};

class GetPersonTrajectoriesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.trajectories = null;
    }
    else {
      if (initObj.hasOwnProperty('trajectories')) {
        this.trajectories = initObj.trajectories
      }
      else {
        this.trajectories = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPersonTrajectoriesResponse
    // Serialize message field [trajectories]
    // Serialize the length for message field [trajectories]
    bufferOffset = _serializer.uint32(obj.trajectories.length, buffer, bufferOffset);
    obj.trajectories.forEach((val) => {
      bufferOffset = PersonTrajectory.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPersonTrajectoriesResponse
    let len;
    let data = new GetPersonTrajectoriesResponse(null);
    // Deserialize message field [trajectories]
    // Deserialize array length for message field [trajectories]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.trajectories = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.trajectories[i] = PersonTrajectory.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.trajectories.forEach((val) => {
      length += PersonTrajectory.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'spencer_tracking_msgs/GetPersonTrajectoriesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cac69139f499658fd82ffbcabd799a3d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    PersonTrajectory[] trajectories  # The trajectories of the tracks that have been asked for in requested_ids, in the same order.
    
    
    ================================================================================
    MSG: spencer_tracking_msgs/PersonTrajectory
    # Message defining the trajectory of a tracked person.
    #
    # The distinction between track and trajectory is that, depending on the
    # implementation of the tracker, a single track (i.e. tracked person) might
    # change the trajectory if at some point a new trajectory "fits" that track (person)
    # better.
    #
    
    uint64                   track_id   # Unique identifier of the tracked person.
    PersonTrajectoryEntry[]  trajectory # All states of the last T frames of the most likely trajectory of that tracked person.
    
    ================================================================================
    MSG: spencer_tracking_msgs/PersonTrajectoryEntry
    # Message defining an entry of a person trajectory.
    #
    
    time     stamp           # age of the track
    bool     is_occluded     # if the track is currently not matched by a detection
    uint64   detection_id    # id of the corresponding detection in the current cycle (undefined if occluded)
    
    # The following fields are extracted from the Kalman state x and its covariance C
    
    geometry_msgs/PoseWithCovariance    pose   # pose of the track (z value and orientation might not be set, check if corresponding variance on diagonal is > 99999)
    geometry_msgs/TwistWithCovariance   twist  # velocity of the track (z value and rotational velocities might not be set, check if corresponding variance on diagonal is > 99999)
    
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
    const resolved = new GetPersonTrajectoriesResponse(null);
    if (msg.trajectories !== undefined) {
      resolved.trajectories = new Array(msg.trajectories.length);
      for (let i = 0; i < resolved.trajectories.length; ++i) {
        resolved.trajectories[i] = PersonTrajectory.Resolve(msg.trajectories[i]);
      }
    }
    else {
      resolved.trajectories = []
    }

    return resolved;
    }
};

module.exports = {
  Request: GetPersonTrajectoriesRequest,
  Response: GetPersonTrajectoriesResponse,
  md5sum() { return 'e0e3d63b99808526b112847474001abe'; },
  datatype() { return 'spencer_tracking_msgs/GetPersonTrajectories'; }
};
