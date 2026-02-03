// Auto-generated. Do not edit!

// (in-package tof_preprocessing.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Blob {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.cx = null;
      this.cy = null;
      this.x = null;
      this.y = null;
      this.w = null;
      this.h = null;
      this.area = null;
      this.is_static = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('cx')) {
        this.cx = initObj.cx
      }
      else {
        this.cx = 0.0;
      }
      if (initObj.hasOwnProperty('cy')) {
        this.cy = initObj.cy
      }
      else {
        this.cy = 0.0;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0;
      }
      if (initObj.hasOwnProperty('w')) {
        this.w = initObj.w
      }
      else {
        this.w = 0;
      }
      if (initObj.hasOwnProperty('h')) {
        this.h = initObj.h
      }
      else {
        this.h = 0;
      }
      if (initObj.hasOwnProperty('area')) {
        this.area = initObj.area
      }
      else {
        this.area = 0.0;
      }
      if (initObj.hasOwnProperty('is_static')) {
        this.is_static = initObj.is_static
      }
      else {
        this.is_static = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Blob
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [cx]
    bufferOffset = _serializer.float32(obj.cx, buffer, bufferOffset);
    // Serialize message field [cy]
    bufferOffset = _serializer.float32(obj.cy, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.int32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.int32(obj.y, buffer, bufferOffset);
    // Serialize message field [w]
    bufferOffset = _serializer.int32(obj.w, buffer, bufferOffset);
    // Serialize message field [h]
    bufferOffset = _serializer.int32(obj.h, buffer, bufferOffset);
    // Serialize message field [area]
    bufferOffset = _serializer.float32(obj.area, buffer, bufferOffset);
    // Serialize message field [is_static]
    bufferOffset = _serializer.bool(obj.is_static, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Blob
    let len;
    let data = new Blob(null);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [cx]
    data.cx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cy]
    data.cy = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [w]
    data.w = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [h]
    data.h = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [area]
    data.area = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [is_static]
    data.is_static = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 33;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tof_preprocessing/Blob';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ae73ff5c5745f08a16d613edb0951120';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 id
    float32 cx
    float32 cy
    int32 x
    int32 y
    int32 w
    int32 h
    float32 area
    bool is_static
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Blob(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.cx !== undefined) {
      resolved.cx = msg.cx;
    }
    else {
      resolved.cx = 0.0
    }

    if (msg.cy !== undefined) {
      resolved.cy = msg.cy;
    }
    else {
      resolved.cy = 0.0
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0
    }

    if (msg.w !== undefined) {
      resolved.w = msg.w;
    }
    else {
      resolved.w = 0
    }

    if (msg.h !== undefined) {
      resolved.h = msg.h;
    }
    else {
      resolved.h = 0
    }

    if (msg.area !== undefined) {
      resolved.area = msg.area;
    }
    else {
      resolved.area = 0.0
    }

    if (msg.is_static !== undefined) {
      resolved.is_static = msg.is_static;
    }
    else {
      resolved.is_static = false
    }

    return resolved;
    }
};

module.exports = Blob;
