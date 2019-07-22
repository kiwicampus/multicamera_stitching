// Auto-generated. Do not edit!

// (in-package data_capture.msg)


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
      this.recording = null;
      this.fps = null;
      this.space_left = null;
      this.number_images = null;
      this.resolution = null;
      this.quality = null;
    }
    else {
      if (initObj.hasOwnProperty('recording')) {
        this.recording = initObj.recording
      }
      else {
        this.recording = false;
      }
      if (initObj.hasOwnProperty('fps')) {
        this.fps = initObj.fps
      }
      else {
        this.fps = 0.0;
      }
      if (initObj.hasOwnProperty('space_left')) {
        this.space_left = initObj.space_left
      }
      else {
        this.space_left = 0.0;
      }
      if (initObj.hasOwnProperty('number_images')) {
        this.number_images = initObj.number_images
      }
      else {
        this.number_images = 0;
      }
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
    // Serialize message field [recording]
    bufferOffset = _serializer.bool(obj.recording, buffer, bufferOffset);
    // Serialize message field [fps]
    bufferOffset = _serializer.float32(obj.fps, buffer, bufferOffset);
    // Serialize message field [space_left]
    bufferOffset = _serializer.float32(obj.space_left, buffer, bufferOffset);
    // Serialize message field [number_images]
    bufferOffset = _serializer.int32(obj.number_images, buffer, bufferOffset);
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
    // Deserialize message field [recording]
    data.recording = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [fps]
    data.fps = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [space_left]
    data.space_left = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [number_images]
    data.number_images = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [resolution]
    data.resolution = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [quality]
    data.quality = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.resolution.length;
    return length + 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'data_capture/Status';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a2b0fb0d87014489c2b3bdd33a6bb65d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool recording
    float32 fps
    float32 space_left
    int32 number_images
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
    if (msg.recording !== undefined) {
      resolved.recording = msg.recording;
    }
    else {
      resolved.recording = false
    }

    if (msg.fps !== undefined) {
      resolved.fps = msg.fps;
    }
    else {
      resolved.fps = 0.0
    }

    if (msg.space_left !== undefined) {
      resolved.space_left = msg.space_left;
    }
    else {
      resolved.space_left = 0.0
    }

    if (msg.number_images !== undefined) {
      resolved.number_images = msg.number_images;
    }
    else {
      resolved.number_images = 0
    }

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
