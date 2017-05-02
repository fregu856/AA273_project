// Auto-generated. Do not edit!

// (in-package nav2d_navigator.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class ExploreFeedback {
  constructor() {
    this.robot_pose = new geometry_msgs.msg.Pose2D();
    this.target_pose = new geometry_msgs.msg.Pose2D();
    this.distance = 0.0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type ExploreFeedback
    // Serialize message field [robot_pose]
    bufferInfo = geometry_msgs.msg.Pose2D.serialize(obj.robot_pose, bufferInfo);
    // Serialize message field [target_pose]
    bufferInfo = geometry_msgs.msg.Pose2D.serialize(obj.target_pose, bufferInfo);
    // Serialize message field [distance]
    bufferInfo = _serializer.float32(obj.distance, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type ExploreFeedback
    let tmp;
    let len;
    let data = new ExploreFeedback();
    // Deserialize message field [robot_pose]
    tmp = geometry_msgs.msg.Pose2D.deserialize(buffer);
    data.robot_pose = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [target_pose]
    tmp = geometry_msgs.msg.Pose2D.deserialize(buffer);
    data.target_pose = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [distance]
    tmp = _deserializer.float32(buffer);
    data.distance = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'nav2d_navigator/ExploreFeedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e64a606b3357bbb098996ab9c2799a9f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    geometry_msgs/Pose2D robot_pose
    geometry_msgs/Pose2D target_pose
    float32 distance
    
    
    ================================================================================
    MSG: geometry_msgs/Pose2D
    # This expresses a position and orientation on a 2D manifold.
    
    float64 x
    float64 y
    float64 theta
    `;
  }

};

module.exports = ExploreFeedback;
