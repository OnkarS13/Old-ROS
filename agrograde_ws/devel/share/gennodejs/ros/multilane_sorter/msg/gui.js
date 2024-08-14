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
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

class gui {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.sensorStatus = null;
      this.internetStatus = null;
      this.dataUploadStatus = null;
      this.systemWarning = null;
      this.guiWarning = null;
      this.throughputOverAll = null;
      this.throughputAccepted = null;
      this.throughputRejected = null;
      this.sortingUptime = null;
      this.systemUptime = null;
      this.actuatorStats = null;
      this.lane_1 = null;
      this.lane_2 = null;
      this.lane_3 = null;
      this.lane_4 = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('sensorStatus')) {
        this.sensorStatus = initObj.sensorStatus
      }
      else {
        this.sensorStatus = '';
      }
      if (initObj.hasOwnProperty('internetStatus')) {
        this.internetStatus = initObj.internetStatus
      }
      else {
        this.internetStatus = '';
      }
      if (initObj.hasOwnProperty('dataUploadStatus')) {
        this.dataUploadStatus = initObj.dataUploadStatus
      }
      else {
        this.dataUploadStatus = '';
      }
      if (initObj.hasOwnProperty('systemWarning')) {
        this.systemWarning = initObj.systemWarning
      }
      else {
        this.systemWarning = '';
      }
      if (initObj.hasOwnProperty('guiWarning')) {
        this.guiWarning = initObj.guiWarning
      }
      else {
        this.guiWarning = '';
      }
      if (initObj.hasOwnProperty('throughputOverAll')) {
        this.throughputOverAll = initObj.throughputOverAll
      }
      else {
        this.throughputOverAll = '';
      }
      if (initObj.hasOwnProperty('throughputAccepted')) {
        this.throughputAccepted = initObj.throughputAccepted
      }
      else {
        this.throughputAccepted = '';
      }
      if (initObj.hasOwnProperty('throughputRejected')) {
        this.throughputRejected = initObj.throughputRejected
      }
      else {
        this.throughputRejected = '';
      }
      if (initObj.hasOwnProperty('sortingUptime')) {
        this.sortingUptime = initObj.sortingUptime
      }
      else {
        this.sortingUptime = '';
      }
      if (initObj.hasOwnProperty('systemUptime')) {
        this.systemUptime = initObj.systemUptime
      }
      else {
        this.systemUptime = '';
      }
      if (initObj.hasOwnProperty('actuatorStats')) {
        this.actuatorStats = initObj.actuatorStats
      }
      else {
        this.actuatorStats = '';
      }
      if (initObj.hasOwnProperty('lane_1')) {
        this.lane_1 = initObj.lane_1
      }
      else {
        this.lane_1 = new sensor_msgs.msg.CompressedImage();
      }
      if (initObj.hasOwnProperty('lane_2')) {
        this.lane_2 = initObj.lane_2
      }
      else {
        this.lane_2 = new sensor_msgs.msg.CompressedImage();
      }
      if (initObj.hasOwnProperty('lane_3')) {
        this.lane_3 = initObj.lane_3
      }
      else {
        this.lane_3 = new sensor_msgs.msg.CompressedImage();
      }
      if (initObj.hasOwnProperty('lane_4')) {
        this.lane_4 = initObj.lane_4
      }
      else {
        this.lane_4 = new sensor_msgs.msg.CompressedImage();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gui
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [sensorStatus]
    bufferOffset = _serializer.string(obj.sensorStatus, buffer, bufferOffset);
    // Serialize message field [internetStatus]
    bufferOffset = _serializer.string(obj.internetStatus, buffer, bufferOffset);
    // Serialize message field [dataUploadStatus]
    bufferOffset = _serializer.string(obj.dataUploadStatus, buffer, bufferOffset);
    // Serialize message field [systemWarning]
    bufferOffset = _serializer.string(obj.systemWarning, buffer, bufferOffset);
    // Serialize message field [guiWarning]
    bufferOffset = _serializer.string(obj.guiWarning, buffer, bufferOffset);
    // Serialize message field [throughputOverAll]
    bufferOffset = _serializer.string(obj.throughputOverAll, buffer, bufferOffset);
    // Serialize message field [throughputAccepted]
    bufferOffset = _serializer.string(obj.throughputAccepted, buffer, bufferOffset);
    // Serialize message field [throughputRejected]
    bufferOffset = _serializer.string(obj.throughputRejected, buffer, bufferOffset);
    // Serialize message field [sortingUptime]
    bufferOffset = _serializer.string(obj.sortingUptime, buffer, bufferOffset);
    // Serialize message field [systemUptime]
    bufferOffset = _serializer.string(obj.systemUptime, buffer, bufferOffset);
    // Serialize message field [actuatorStats]
    bufferOffset = _serializer.string(obj.actuatorStats, buffer, bufferOffset);
    // Serialize message field [lane_1]
    bufferOffset = sensor_msgs.msg.CompressedImage.serialize(obj.lane_1, buffer, bufferOffset);
    // Serialize message field [lane_2]
    bufferOffset = sensor_msgs.msg.CompressedImage.serialize(obj.lane_2, buffer, bufferOffset);
    // Serialize message field [lane_3]
    bufferOffset = sensor_msgs.msg.CompressedImage.serialize(obj.lane_3, buffer, bufferOffset);
    // Serialize message field [lane_4]
    bufferOffset = sensor_msgs.msg.CompressedImage.serialize(obj.lane_4, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gui
    let len;
    let data = new gui(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [sensorStatus]
    data.sensorStatus = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [internetStatus]
    data.internetStatus = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [dataUploadStatus]
    data.dataUploadStatus = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [systemWarning]
    data.systemWarning = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [guiWarning]
    data.guiWarning = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [throughputOverAll]
    data.throughputOverAll = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [throughputAccepted]
    data.throughputAccepted = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [throughputRejected]
    data.throughputRejected = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [sortingUptime]
    data.sortingUptime = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [systemUptime]
    data.systemUptime = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [actuatorStats]
    data.actuatorStats = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [lane_1]
    data.lane_1 = sensor_msgs.msg.CompressedImage.deserialize(buffer, bufferOffset);
    // Deserialize message field [lane_2]
    data.lane_2 = sensor_msgs.msg.CompressedImage.deserialize(buffer, bufferOffset);
    // Deserialize message field [lane_3]
    data.lane_3 = sensor_msgs.msg.CompressedImage.deserialize(buffer, bufferOffset);
    // Deserialize message field [lane_4]
    data.lane_4 = sensor_msgs.msg.CompressedImage.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.sensorStatus);
    length += _getByteLength(object.internetStatus);
    length += _getByteLength(object.dataUploadStatus);
    length += _getByteLength(object.systemWarning);
    length += _getByteLength(object.guiWarning);
    length += _getByteLength(object.throughputOverAll);
    length += _getByteLength(object.throughputAccepted);
    length += _getByteLength(object.throughputRejected);
    length += _getByteLength(object.sortingUptime);
    length += _getByteLength(object.systemUptime);
    length += _getByteLength(object.actuatorStats);
    length += sensor_msgs.msg.CompressedImage.getMessageSize(object.lane_1);
    length += sensor_msgs.msg.CompressedImage.getMessageSize(object.lane_2);
    length += sensor_msgs.msg.CompressedImage.getMessageSize(object.lane_3);
    length += sensor_msgs.msg.CompressedImage.getMessageSize(object.lane_4);
    return length + 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'multilane_sorter/gui';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '65a059bbb76f43602ed8d5fcfa654321';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    string sensorStatus
    string internetStatus
    string dataUploadStatus
    string systemWarning
    string guiWarning
    string throughputOverAll
    string throughputAccepted
    string throughputRejected
    string sortingUptime
    string systemUptime
    string actuatorStats
    sensor_msgs/CompressedImage lane_1
    sensor_msgs/CompressedImage lane_2
    sensor_msgs/CompressedImage lane_3
    sensor_msgs/CompressedImage lane_4
    
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
    MSG: sensor_msgs/CompressedImage
    # This message contains a compressed image
    
    Header header        # Header timestamp should be acquisition time of image
                         # Header frame_id should be optical frame of camera
                         # origin of frame should be optical center of camera
                         # +x should point to the right in the image
                         # +y should point down in the image
                         # +z should point into to plane of the image
    
    string format        # Specifies the format of the data
                         #   Acceptable values:
                         #     jpeg, png
    uint8[] data         # Compressed image buffer
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gui(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.sensorStatus !== undefined) {
      resolved.sensorStatus = msg.sensorStatus;
    }
    else {
      resolved.sensorStatus = ''
    }

    if (msg.internetStatus !== undefined) {
      resolved.internetStatus = msg.internetStatus;
    }
    else {
      resolved.internetStatus = ''
    }

    if (msg.dataUploadStatus !== undefined) {
      resolved.dataUploadStatus = msg.dataUploadStatus;
    }
    else {
      resolved.dataUploadStatus = ''
    }

    if (msg.systemWarning !== undefined) {
      resolved.systemWarning = msg.systemWarning;
    }
    else {
      resolved.systemWarning = ''
    }

    if (msg.guiWarning !== undefined) {
      resolved.guiWarning = msg.guiWarning;
    }
    else {
      resolved.guiWarning = ''
    }

    if (msg.throughputOverAll !== undefined) {
      resolved.throughputOverAll = msg.throughputOverAll;
    }
    else {
      resolved.throughputOverAll = ''
    }

    if (msg.throughputAccepted !== undefined) {
      resolved.throughputAccepted = msg.throughputAccepted;
    }
    else {
      resolved.throughputAccepted = ''
    }

    if (msg.throughputRejected !== undefined) {
      resolved.throughputRejected = msg.throughputRejected;
    }
    else {
      resolved.throughputRejected = ''
    }

    if (msg.sortingUptime !== undefined) {
      resolved.sortingUptime = msg.sortingUptime;
    }
    else {
      resolved.sortingUptime = ''
    }

    if (msg.systemUptime !== undefined) {
      resolved.systemUptime = msg.systemUptime;
    }
    else {
      resolved.systemUptime = ''
    }

    if (msg.actuatorStats !== undefined) {
      resolved.actuatorStats = msg.actuatorStats;
    }
    else {
      resolved.actuatorStats = ''
    }

    if (msg.lane_1 !== undefined) {
      resolved.lane_1 = sensor_msgs.msg.CompressedImage.Resolve(msg.lane_1)
    }
    else {
      resolved.lane_1 = new sensor_msgs.msg.CompressedImage()
    }

    if (msg.lane_2 !== undefined) {
      resolved.lane_2 = sensor_msgs.msg.CompressedImage.Resolve(msg.lane_2)
    }
    else {
      resolved.lane_2 = new sensor_msgs.msg.CompressedImage()
    }

    if (msg.lane_3 !== undefined) {
      resolved.lane_3 = sensor_msgs.msg.CompressedImage.Resolve(msg.lane_3)
    }
    else {
      resolved.lane_3 = new sensor_msgs.msg.CompressedImage()
    }

    if (msg.lane_4 !== undefined) {
      resolved.lane_4 = sensor_msgs.msg.CompressedImage.Resolve(msg.lane_4)
    }
    else {
      resolved.lane_4 = new sensor_msgs.msg.CompressedImage()
    }

    return resolved;
    }
};

module.exports = gui;
