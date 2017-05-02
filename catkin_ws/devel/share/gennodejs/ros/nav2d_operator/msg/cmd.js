// Auto-generated. Do not edit!

// (in-package nav2d_operator.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class cmd {
  constructor() {
    this.Velocity = 0.0;
    this.Turn = 0.0;
    this.Mode = 0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type cmd
    // Serialize message field [Velocity]
    bufferInfo = _serializer.float64(obj.Velocity, bufferInfo);
    // Serialize message field [Turn]
    bufferInfo = _serializer.float64(obj.Turn, bufferInfo);
    // Serialize message field [Mode]
    bufferInfo = _serializer.int8(obj.Mode, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type cmd
    let tmp;
    let len;
    let data = new cmd();
    // Deserialize message field [Velocity]
    tmp = _deserializer.float64(buffer);
    data.Velocity = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [Turn]
    tmp = _deserializer.float64(buffer);
    data.Turn = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [Mode]
    tmp = _deserializer.int8(buffer);
    data.Mode = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'nav2d_operator/cmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '90c9a043660646e2102f124332ecb8b7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 Velocity
    float64 Turn
    int8    Mode
    
    `;
  }

};

module.exports = cmd;
