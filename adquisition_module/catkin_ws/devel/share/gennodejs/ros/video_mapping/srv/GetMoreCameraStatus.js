// Auto-generated. Do not edit!

// (in-package video_mapping.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetMoreCameraStatusRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetMoreCameraStatusRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetMoreCameraStatusRequest
    let len;
    let data = new GetMoreCameraStatusRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'video_mapping/GetMoreCameraStatusRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetMoreCameraStatusRequest(null);
    return resolved;
    }
};

class GetMoreCameraStatusResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.camera_status = null;
    }
    else {
      if (initObj.hasOwnProperty('camera_status')) {
        this.camera_status = initObj.camera_status
      }
      else {
        this.camera_status = new Array(3).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetMoreCameraStatusResponse
    // Check that the constant length array field [camera_status] has the right length
    if (obj.camera_status.length !== 3) {
      throw new Error('Unable to serialize array field camera_status - length must be 3')
    }
    // Serialize message field [camera_status]
    bufferOffset = _arraySerializer.string(obj.camera_status, buffer, bufferOffset, 3);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetMoreCameraStatusResponse
    let len;
    let data = new GetMoreCameraStatusResponse(null);
    // Deserialize message field [camera_status]
    data.camera_status = _arrayDeserializer.string(buffer, bufferOffset, 3)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.camera_status.forEach((val) => {
      length += 4 + val.length;
    });
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'video_mapping/GetMoreCameraStatusResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '086d9f0441c56e91ac64021d69c7ba0c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[3] camera_status
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetMoreCameraStatusResponse(null);
    if (msg.camera_status !== undefined) {
      resolved.camera_status = msg.camera_status;
    }
    else {
      resolved.camera_status = new Array(3).fill(0)
    }

    return resolved;
    }
};

module.exports = {
  Request: GetMoreCameraStatusRequest,
  Response: GetMoreCameraStatusResponse,
  md5sum() { return '086d9f0441c56e91ac64021d69c7ba0c'; },
  datatype() { return 'video_mapping/GetMoreCameraStatus'; }
};
