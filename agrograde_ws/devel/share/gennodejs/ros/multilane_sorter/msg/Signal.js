// Auto-generated. Do not edit!

// (in-package multilane_sorter.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Signal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.ac_a = null;
      this.ac_b = null;
      this.ac_c = null;
      this.ac_d = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('ac_a')) {
        this.ac_a = initObj.ac_a
      }
      else {
        this.ac_a = 0;
      }
      if (initObj.hasOwnProperty('ac_b')) {
        this.ac_b = initObj.ac_b
      }
      else {
        this.ac_b = 0;
      }
      if (initObj.hasOwnProperty('ac_c')) {
        this.ac_c = initObj.ac_c
      }
      else {
        this.ac_c = 0;
      }
      if (initObj.hasOwnProperty('ac_d')) {
        this.ac_d = initObj.ac_d
      }
      else {
        this.ac_d = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Signal
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [ac_a]
    bufferOffset = _serializer.uint16(obj.ac_a, buffer, bufferOffset);
    // Serialize message field [ac_b]
    bufferOffset = _serializer.uint16(obj.ac_b, buffer, bufferOffset);
    // Serialize message field [ac_c]
    bufferOffset = _serializer.uint16(obj.ac_c, buffer, bufferOffset);
    // Serialize message field [ac_d]
    bufferOffset = _serializer.uint16(obj.ac_d, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Signal
    let len;
    let data = new Signal(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [ac_a]
    data.ac_a = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [ac_b]
    data.ac_b = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [ac_c]
    data.ac_c = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [ac_d]
    data.ac_d = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'multilane_sorter/Signal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bf6d5cfdfe1165ba84740a209c2b5cb1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    uint16 ac_a
    uint16 ac_b
    uint16 ac_c
    uint16 ac_d
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
    const resolved = new Signal(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.ac_a !== undefined) {
      resolved.ac_a = msg.ac_a;
    }
    else {
      resolved.ac_a = 0
    }

    if (msg.ac_b !== undefined) {
      resolved.ac_b = msg.ac_b;
    }
    else {
      resolved.ac_b = 0
    }

    if (msg.ac_c !== undefined) {
      resolved.ac_c = msg.ac_c;
    }
    else {
      resolved.ac_c = 0
    }

    if (msg.ac_d !== undefined) {
      resolved.ac_d = msg.ac_d;
    }
    else {
      resolved.ac_d = 0
    }

    return resolved;
    }
};

module.exports = Signal;
