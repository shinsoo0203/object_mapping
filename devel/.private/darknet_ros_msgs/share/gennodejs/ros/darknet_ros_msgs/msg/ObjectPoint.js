// Auto-generated. Do not edit!

// (in-package darknet_ros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ObjectPoint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Class = null;
      this.probability = null;
      this.width = null;
      this.height = null;
    }
    else {
      if (initObj.hasOwnProperty('Class')) {
        this.Class = initObj.Class
      }
      else {
        this.Class = '';
      }
      if (initObj.hasOwnProperty('probability')) {
        this.probability = initObj.probability
      }
      else {
        this.probability = 0.0;
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = 0;
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObjectPoint
    // Serialize message field [Class]
    bufferOffset = _serializer.string(obj.Class, buffer, bufferOffset);
    // Serialize message field [probability]
    bufferOffset = _serializer.float64(obj.probability, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = _serializer.int8(obj.width, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = _serializer.int8(obj.height, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObjectPoint
    let len;
    let data = new ObjectPoint(null);
    // Deserialize message field [Class]
    data.Class = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [probability]
    data.probability = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.Class.length;
    return length + 14;
  }

  static datatype() {
    // Returns string type for a message object
    return 'darknet_ros_msgs/ObjectPoint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b1c5f9b8d2aa5f8a567fa1f72efd61ed';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string Class
    float64 probability
    int8 width
    int8 height
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ObjectPoint(null);
    if (msg.Class !== undefined) {
      resolved.Class = msg.Class;
    }
    else {
      resolved.Class = ''
    }

    if (msg.probability !== undefined) {
      resolved.probability = msg.probability;
    }
    else {
      resolved.probability = 0.0
    }

    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = 0
    }

    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0
    }

    return resolved;
    }
};

module.exports = ObjectPoint;
