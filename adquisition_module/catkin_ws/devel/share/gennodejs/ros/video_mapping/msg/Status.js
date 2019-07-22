// Auto-generated. Do not edit!

// (in-package video_mapping.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Status {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.resolution = null;
      this.quality = null;
    }
    else {
      if (initObj.hasOwnProperty('resolution')) {
        this.resolution = initObj.resolution
      }
      else {
        this.resolution = '';
      }
      if (initObj.hasOwnProperty('quality')) {
        this.quality = initObj.quality
      }
      else {
        this.quality = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Status
    // Serialize message field [resolution]
    bufferOffset = _serializer.string(obj.resolution, buffer, bufferOffset);
    // Serialize message field [quality]
    bufferOffset = _serializer.float32(obj.quality, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Status
    let len;
    let data = new Status(null);
    // Deserialize message field [resolution]
    data.resolution = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [quality]
    data.quality = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.resolution.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'video_mapping/Status';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '68de8bf5b6d9757586af8a4bdbc8a1a5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string resolution
    float32 quality
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Status(null);
    if (msg.resolution !== undefined) {
      resolved.resolution = msg.resolution;
    }
    else {
      resolved.resolution = ''
    }

    if (msg.quality !== undefined) {
      resolved.quality = msg.quality;
    }
    else {
      resolved.quality = 0.0
    }

    return resolved;
    }
};

module.exports = Status;
