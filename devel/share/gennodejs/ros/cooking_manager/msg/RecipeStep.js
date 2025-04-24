// Auto-generated. Do not edit!

// (in-package cooking_manager.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class RecipeStep {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.step = null;
    }
    else {
      if (initObj.hasOwnProperty('step')) {
        this.step = initObj.step
      }
      else {
        this.step = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RecipeStep
    // Serialize message field [step]
    bufferOffset = _serializer.string(obj.step, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RecipeStep
    let len;
    let data = new RecipeStep(null);
    // Deserialize message field [step]
    data.step = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.step.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cooking_manager/RecipeStep';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd2ee9423a5e4755eeacf5800eb738c64';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string step
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RecipeStep(null);
    if (msg.step !== undefined) {
      resolved.step = msg.step;
    }
    else {
      resolved.step = ''
    }

    return resolved;
    }
};

module.exports = RecipeStep;
