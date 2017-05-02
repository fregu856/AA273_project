// Auto-generated. Do not edit!

// (in-package nav2d_navigator.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class ExploreResult {
  constructor() {
    this.final_pose = new geometry_msgs.msg.Pose2D();
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type ExploreResult
    // Serialize message field [final_pose]
    bufferInfo = geometry_msgs.msg.Pose2D.serialize(obj.final_pose, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type ExploreResult
    let tmp;
    let len;
    let data = new ExploreResult();
    // Deserialize message field [final_pose]
    tmp = geometry_msgs.msg.Pose2D.deserialize(buffer);
    data.final_pose = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'nav2d_navigator/ExploreResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9b03b05e2f5c62e96e4cec4715bf432f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    geometry_msgs/Pose2D final_pose
    
    ================================================================================
    MSG: geometry_msgs/Pose2D
    # This expresses a position and orientation on a 2D manifold.
    
    float64 x
    float64 y
    float64 theta
    `;
  }

};

module.exports = ExploreResult;