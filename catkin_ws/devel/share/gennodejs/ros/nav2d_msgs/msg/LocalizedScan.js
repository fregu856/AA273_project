// Auto-generated. Do not edit!

// (in-package nav2d_msgs.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

class LocalizedScan {
  constructor() {
    this.robot_id = 0;
    this.laser_type = 0;
    this.x = 0.0;
    this.y = 0.0;
    this.yaw = 0.0;
    this.scan = new sensor_msgs.msg.LaserScan();
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type LocalizedScan
    // Serialize message field [robot_id]
    bufferInfo = _serializer.int8(obj.robot_id, bufferInfo);
    // Serialize message field [laser_type]
    bufferInfo = _serializer.int8(obj.laser_type, bufferInfo);
    // Serialize message field [x]
    bufferInfo = _serializer.float32(obj.x, bufferInfo);
    // Serialize message field [y]
    bufferInfo = _serializer.float32(obj.y, bufferInfo);
    // Serialize message field [yaw]
    bufferInfo = _serializer.float32(obj.yaw, bufferInfo);
    // Serialize message field [scan]
    bufferInfo = sensor_msgs.msg.LaserScan.serialize(obj.scan, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type LocalizedScan
    let tmp;
    let len;
    let data = new LocalizedScan();
    // Deserialize message field [robot_id]
    tmp = _deserializer.int8(buffer);
    data.robot_id = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [laser_type]
    tmp = _deserializer.int8(buffer);
    data.laser_type = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [x]
    tmp = _deserializer.float32(buffer);
    data.x = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [y]
    tmp = _deserializer.float32(buffer);
    data.y = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [yaw]
    tmp = _deserializer.float32(buffer);
    data.yaw = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [scan]
    tmp = sensor_msgs.msg.LaserScan.deserialize(buffer);
    data.scan = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'nav2d_msgs/LocalizedScan';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bab53504723a56692b3864ccf3dfe635';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8      robot_id
    int8      laser_type
    float32   x
    float32   y
    float32   yaw
    sensor_msgs/LaserScan scan
    ================================================================================
    MSG: sensor_msgs/LaserScan
    # Single scan from a planar laser range-finder
    #
    # If you have another ranging device with different behavior (e.g. a sonar
    # array), please find or create a different message, since applications
    # will make fairly laser-specific assumptions about this data
    
    Header header            # timestamp in the header is the acquisition time of 
                             # the first ray in the scan.
                             #
                             # in frame frame_id, angles are measured around 
                             # the positive Z axis (counterclockwise, if Z is up)
                             # with zero angle being forward along the x axis
                             
    float32 angle_min        # start angle of the scan [rad]
    float32 angle_max        # end angle of the scan [rad]
    float32 angle_increment  # angular distance between measurements [rad]
    
    float32 time_increment   # time between measurements [seconds] - if your scanner
                             # is moving, this will be used in interpolating position
                             # of 3d points
    float32 scan_time        # time between scans [seconds]
    
    float32 range_min        # minimum range value [m]
    float32 range_max        # maximum range value [m]
    
    float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
    float32[] intensities    # intensity data [device-specific units].  If your
                             # device does not provide intensities, please leave
                             # the array empty.
    
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

};

module.exports = LocalizedScan;
