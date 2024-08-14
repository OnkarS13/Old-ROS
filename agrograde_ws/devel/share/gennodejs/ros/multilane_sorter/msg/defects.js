// Auto-generated. Do not edit!

// (in-package multilane_sorter.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class defects {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.normal = null;
      this.bs = null;
      this.peeled = null;
      this.sprouts = null;
      this.rotten = null;
      this.doubles = null;
    }
    else {
      if (initObj.hasOwnProperty('normal')) {
        this.normal = initObj.normal
      }
      else {
        this.normal = 0.0;
      }
      if (initObj.hasOwnProperty('bs')) {
        this.bs = initObj.bs
      }
      else {
        this.bs = 0.0;
      }
      if (initObj.hasOwnProperty('peeled')) {
        this.peeled = initObj.peeled
      }
      else {
        this.peeled = 0.0;
      }
      if (initObj.hasOwnProperty('sprouts')) {
        this.sprouts = initObj.sprouts
      }
      else {
        this.sprouts = 0.0;
      }
      if (initObj.hasOwnProperty('rotten')) {
        this.rotten = initObj.rotten
      }
      else {
        this.rotten = 0.0;
      }
      if (initObj.hasOwnProperty('doubles')) {
        this.doubles = initObj.doubles
      }
      else {
        this.doubles = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type defects
    // Serialize message field [normal]
    bufferOffset = _serializer.float32(obj.normal, buffer, bufferOffset);
    // Serialize message field [bs]
    bufferOffset = _serializer.float32(obj.bs, buffer, bufferOffset);
    // Serialize message field [peeled]
    bufferOffset = _serializer.float32(obj.peeled, buffer, bufferOffset);
    // Serialize message field [sprouts]
    bufferOffset = _serializer.float32(obj.sprouts, buffer, bufferOffset);
    // Serialize message field [rotten]
    bufferOffset = _serializer.float32(obj.rotten, buffer, bufferOffset);
    // Serialize message field [doubles]
    bufferOffset = _serializer.float32(obj.doubles, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type defects
    let len;
    let data = new defects(null);
    // Deserialize message field [normal]
    data.normal = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bs]
    data.bs = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [peeled]
    data.peeled = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [sprouts]
    data.sprouts = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rotten]
    data.rotten = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [doubles]
    data.doubles = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'multilane_sorter/defects';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cedfcaa9a7bdfcb45d2788094a026d07';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 normal
    float32 bs
    float32 peeled
    float32 sprouts
    float32 rotten
    float32 doubles
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new defects(null);
    if (msg.normal !== undefined) {
      resolved.normal = msg.normal;
    }
    else {
      resolved.normal = 0.0
    }

    if (msg.bs !== undefined) {
      resolved.bs = msg.bs;
    }
    else {
      resolved.bs = 0.0
    }

    if (msg.peeled !== undefined) {
      resolved.peeled = msg.peeled;
    }
    else {
      resolved.peeled = 0.0
    }

    if (msg.sprouts !== undefined) {
      resolved.sprouts = msg.sprouts;
    }
    else {
      resolved.sprouts = 0.0
    }

    if (msg.rotten !== undefined) {
      resolved.rotten = msg.rotten;
    }
    else {
      resolved.rotten = 0.0
    }

    if (msg.doubles !== undefined) {
      resolved.doubles = msg.doubles;
    }
    else {
      resolved.doubles = 0.0
    }

    return resolved;
    }
};

module.exports = defects;
