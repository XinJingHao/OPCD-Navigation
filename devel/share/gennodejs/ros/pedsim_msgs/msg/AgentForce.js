// Auto-generated. Do not edit!

// (in-package pedsim_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class AgentForce {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.desired_force = null;
      this.obstacle_force = null;
      this.social_force = null;
      this.group_coherence_force = null;
      this.group_gaze_force = null;
      this.group_repulsion_force = null;
      this.random_force = null;
    }
    else {
      if (initObj.hasOwnProperty('desired_force')) {
        this.desired_force = initObj.desired_force
      }
      else {
        this.desired_force = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('obstacle_force')) {
        this.obstacle_force = initObj.obstacle_force
      }
      else {
        this.obstacle_force = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('social_force')) {
        this.social_force = initObj.social_force
      }
      else {
        this.social_force = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('group_coherence_force')) {
        this.group_coherence_force = initObj.group_coherence_force
      }
      else {
        this.group_coherence_force = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('group_gaze_force')) {
        this.group_gaze_force = initObj.group_gaze_force
      }
      else {
        this.group_gaze_force = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('group_repulsion_force')) {
        this.group_repulsion_force = initObj.group_repulsion_force
      }
      else {
        this.group_repulsion_force = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('random_force')) {
        this.random_force = initObj.random_force
      }
      else {
        this.random_force = new geometry_msgs.msg.Vector3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AgentForce
    // Serialize message field [desired_force]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.desired_force, buffer, bufferOffset);
    // Serialize message field [obstacle_force]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.obstacle_force, buffer, bufferOffset);
    // Serialize message field [social_force]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.social_force, buffer, bufferOffset);
    // Serialize message field [group_coherence_force]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.group_coherence_force, buffer, bufferOffset);
    // Serialize message field [group_gaze_force]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.group_gaze_force, buffer, bufferOffset);
    // Serialize message field [group_repulsion_force]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.group_repulsion_force, buffer, bufferOffset);
    // Serialize message field [random_force]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.random_force, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AgentForce
    let len;
    let data = new AgentForce(null);
    // Deserialize message field [desired_force]
    data.desired_force = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [obstacle_force]
    data.obstacle_force = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [social_force]
    data.social_force = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [group_coherence_force]
    data.group_coherence_force = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [group_gaze_force]
    data.group_gaze_force = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [group_repulsion_force]
    data.group_repulsion_force = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [random_force]
    data.random_force = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 168;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pedsim_msgs/AgentForce';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dcd8d82cea8453731000bbf59474a5b8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Forces acting on an agent.
    
    # Basic SFM forces.
    geometry_msgs/Vector3 desired_force
    geometry_msgs/Vector3 obstacle_force
    geometry_msgs/Vector3 social_force
    
    # Additional Group Forces
    geometry_msgs/Vector3 group_coherence_force
    geometry_msgs/Vector3 group_gaze_force
    geometry_msgs/Vector3 group_repulsion_force
    
    # Extra stabilization/custom forces.
    geometry_msgs/Vector3 random_force
    
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
    const resolved = new AgentForce(null);
    if (msg.desired_force !== undefined) {
      resolved.desired_force = geometry_msgs.msg.Vector3.Resolve(msg.desired_force)
    }
    else {
      resolved.desired_force = new geometry_msgs.msg.Vector3()
    }

    if (msg.obstacle_force !== undefined) {
      resolved.obstacle_force = geometry_msgs.msg.Vector3.Resolve(msg.obstacle_force)
    }
    else {
      resolved.obstacle_force = new geometry_msgs.msg.Vector3()
    }

    if (msg.social_force !== undefined) {
      resolved.social_force = geometry_msgs.msg.Vector3.Resolve(msg.social_force)
    }
    else {
      resolved.social_force = new geometry_msgs.msg.Vector3()
    }

    if (msg.group_coherence_force !== undefined) {
      resolved.group_coherence_force = geometry_msgs.msg.Vector3.Resolve(msg.group_coherence_force)
    }
    else {
      resolved.group_coherence_force = new geometry_msgs.msg.Vector3()
    }

    if (msg.group_gaze_force !== undefined) {
      resolved.group_gaze_force = geometry_msgs.msg.Vector3.Resolve(msg.group_gaze_force)
    }
    else {
      resolved.group_gaze_force = new geometry_msgs.msg.Vector3()
    }

    if (msg.group_repulsion_force !== undefined) {
      resolved.group_repulsion_force = geometry_msgs.msg.Vector3.Resolve(msg.group_repulsion_force)
    }
    else {
      resolved.group_repulsion_force = new geometry_msgs.msg.Vector3()
    }

    if (msg.random_force !== undefined) {
      resolved.random_force = geometry_msgs.msg.Vector3.Resolve(msg.random_force)
    }
    else {
      resolved.random_force = new geometry_msgs.msg.Vector3()
    }

    return resolved;
    }
};

module.exports = AgentForce;
