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

class inference {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.sprout = null;
      this.peeled = null;
      this.rotten = null;
      this.blacksmut = null;
      this.double = null;
      this.size = null;
      this.user1 = null;
      this.user2 = null;
      this.user3 = null;
      this.user4 = null;
      this.user5 = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('sprout')) {
        this.sprout = initObj.sprout
      }
      else {
        this.sprout = 0.0;
      }
      if (initObj.hasOwnProperty('peeled')) {
        this.peeled = initObj.peeled
      }
      else {
        this.peeled = 0.0;
      }
      if (initObj.hasOwnProperty('rotten')) {
        this.rotten = initObj.rotten
      }
      else {
        this.rotten = 0.0;
      }
      if (initObj.hasOwnProperty('blacksmut')) {
        this.blacksmut = initObj.blacksmut
      }
      else {
        this.blacksmut = 0.0;
      }
      if (initObj.hasOwnProperty('double')) {
        this.double = initObj.double
      }
      else {
        this.double = 0.0;
      }
      if (initObj.hasOwnProperty('size')) {
        this.size = initObj.size
      }
      else {
        this.size = 0.0;
      }
      if (initObj.hasOwnProperty('user1')) {
        this.user1 = initObj.user1
      }
      else {
        this.user1 = 0.0;
      }
      if (initObj.hasOwnProperty('user2')) {
        this.user2 = initObj.user2
      }
      else {
        this.user2 = 0.0;
      }
      if (initObj.hasOwnProperty('user3')) {
        this.user3 = initObj.user3
      }
      else {
        this.user3 = 0.0;
      }
      if (initObj.hasOwnProperty('user4')) {
        this.user4 = initObj.user4
      }
      else {
        this.user4 = 0.0;
      }
      if (initObj.hasOwnProperty('user5')) {
        this.user5 = initObj.user5
      }
      else {
        this.user5 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type inference
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [sprout]
    bufferOffset = _serializer.float32(obj.sprout, buffer, bufferOffset);
    // Serialize message field [peeled]
    bufferOffset = _serializer.float32(obj.peeled, buffer, bufferOffset);
    // Serialize message field [rotten]
    bufferOffset = _serializer.float32(obj.rotten, buffer, bufferOffset);
    // Serialize message field [blacksmut]
    bufferOffset = _serializer.float32(obj.blacksmut, buffer, bufferOffset);
    // Serialize message field [double]
    bufferOffset = _serializer.float32(obj.double, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.float32(obj.size, buffer, bufferOffset);
    // Serialize message field [user1]
    bufferOffset = _serializer.float32(obj.user1, buffer, bufferOffset);
    // Serialize message field [user2]
    bufferOffset = _serializer.float32(obj.user2, buffer, bufferOffset);
    // Serialize message field [user3]
    bufferOffset = _serializer.float32(obj.user3, buffer, bufferOffset);
    // Serialize message field [user4]
    bufferOffset = _serializer.float32(obj.user4, buffer, bufferOffset);
    // Serialize message field [user5]
    bufferOffset = _serializer.float32(obj.user5, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type inference
    let len;
    let data = new inference(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [sprout]
    data.sprout = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [peeled]
    data.peeled = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rotten]
    data.rotten = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [blacksmut]
    data.blacksmut = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [double]
    data.double = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [user1]
    data.user1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [user2]
    data.user2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [user3]
    data.user3 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [user4]
    data.user4 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [user5]
    data.user5 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'multilane_sorter/inference';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aa06a43907df191d69024618ed6dd509';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    Header header
    float32 sprout
    float32 peeled
    float32 rotten
    float32 blacksmut
    float32 double
    
    float32 size
    float32 user1
    float32 user2
    float32 user3
    float32 user4
    float32 user5
    
    # Header header
    # float32 prediction
    # float32 onion
    # float32 blacksmut
    # float32 peel
    # float32 size
    
    
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
    const resolved = new inference(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.sprout !== undefined) {
      resolved.sprout = msg.sprout;
    }
    else {
      resolved.sprout = 0.0
    }

    if (msg.peeled !== undefined) {
      resolved.peeled = msg.peeled;
    }
    else {
      resolved.peeled = 0.0
    }

    if (msg.rotten !== undefined) {
      resolved.rotten = msg.rotten;
    }
    else {
      resolved.rotten = 0.0
    }

    if (msg.blacksmut !== undefined) {
      resolved.blacksmut = msg.blacksmut;
    }
    else {
      resolved.blacksmut = 0.0
    }

    if (msg.double !== undefined) {
      resolved.double = msg.double;
    }
    else {
      resolved.double = 0.0
    }

    if (msg.size !== undefined) {
      resolved.size = msg.size;
    }
    else {
      resolved.size = 0.0
    }

    if (msg.user1 !== undefined) {
      resolved.user1 = msg.user1;
    }
    else {
      resolved.user1 = 0.0
    }

    if (msg.user2 !== undefined) {
      resolved.user2 = msg.user2;
    }
    else {
      resolved.user2 = 0.0
    }

    if (msg.user3 !== undefined) {
      resolved.user3 = msg.user3;
    }
    else {
      resolved.user3 = 0.0
    }

    if (msg.user4 !== undefined) {
      resolved.user4 = msg.user4;
    }
    else {
      resolved.user4 = 0.0
    }

    if (msg.user5 !== undefined) {
      resolved.user5 = msg.user5;
    }
    else {
      resolved.user5 = 0.0
    }

    return resolved;
    }
};

module.exports = inference;
