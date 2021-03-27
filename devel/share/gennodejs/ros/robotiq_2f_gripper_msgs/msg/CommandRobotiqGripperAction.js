// Auto-generated. Do not edit!

// (in-package robotiq_2f_gripper_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CommandRobotiqGripperActionGoal = require('./CommandRobotiqGripperActionGoal.js');
let CommandRobotiqGripperActionResult = require('./CommandRobotiqGripperActionResult.js');
let CommandRobotiqGripperActionFeedback = require('./CommandRobotiqGripperActionFeedback.js');

//-----------------------------------------------------------

class CommandRobotiqGripperAction {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.action_goal = null;
      this.action_result = null;
      this.action_feedback = null;
    }
    else {
      if (initObj.hasOwnProperty('action_goal')) {
        this.action_goal = initObj.action_goal
      }
      else {
        this.action_goal = new CommandRobotiqGripperActionGoal();
      }
      if (initObj.hasOwnProperty('action_result')) {
        this.action_result = initObj.action_result
      }
      else {
        this.action_result = new CommandRobotiqGripperActionResult();
      }
      if (initObj.hasOwnProperty('action_feedback')) {
        this.action_feedback = initObj.action_feedback
      }
      else {
        this.action_feedback = new CommandRobotiqGripperActionFeedback();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CommandRobotiqGripperAction
    // Serialize message field [action_goal]
    bufferOffset = CommandRobotiqGripperActionGoal.serialize(obj.action_goal, buffer, bufferOffset);
    // Serialize message field [action_result]
    bufferOffset = CommandRobotiqGripperActionResult.serialize(obj.action_result, buffer, bufferOffset);
    // Serialize message field [action_feedback]
    bufferOffset = CommandRobotiqGripperActionFeedback.serialize(obj.action_feedback, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CommandRobotiqGripperAction
    let len;
    let data = new CommandRobotiqGripperAction(null);
    // Deserialize message field [action_goal]
    data.action_goal = CommandRobotiqGripperActionGoal.deserialize(buffer, bufferOffset);
    // Deserialize message field [action_result]
    data.action_result = CommandRobotiqGripperActionResult.deserialize(buffer, bufferOffset);
    // Deserialize message field [action_feedback]
    data.action_feedback = CommandRobotiqGripperActionFeedback.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += CommandRobotiqGripperActionGoal.getMessageSize(object.action_goal);
    length += CommandRobotiqGripperActionResult.getMessageSize(object.action_result);
    length += CommandRobotiqGripperActionFeedback.getMessageSize(object.action_feedback);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robotiq_2f_gripper_msgs/CommandRobotiqGripperAction';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '032bc3d9e57d9371c7d34e6cda2e7867';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    CommandRobotiqGripperActionGoal action_goal
    CommandRobotiqGripperActionResult action_result
    CommandRobotiqGripperActionFeedback action_feedback
    
    ================================================================================
    MSG: robotiq_2f_gripper_msgs/CommandRobotiqGripperActionGoal
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    Header header
    actionlib_msgs/GoalID goal_id
    CommandRobotiqGripperGoal goal
    
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: actionlib_msgs/GoalID
    # The stamp should store the time at which this goal was requested.
    # It is used by an action server when it tries to preempt all
    # goals that were requested before a certain time
    time stamp
    
    # The id provides a way to associate feedback and
    # result message with specific goal requests. The id
    # specified must be unique.
    string id
    
    
    ================================================================================
    MSG: robotiq_2f_gripper_msgs/CommandRobotiqGripperGoal
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    # See Robotiq documentation for fields documentation.
    # See Robotiq documentation for fields documentation.
    
    #goal definition
    bool emergency_release
    uint32 emergency_release_dir
    bool stop
    float32 position #[m] Distance in bewteen fingers 
    float32 speed    #[m/s] Speed (Min: 0.013m/s - Max: 0.1m/s)
    float32 force    #[%] Percentage of gripper force to apply (Min: 0.0% - Max: 100.0%)
    
    ================================================================================
    MSG: robotiq_2f_gripper_msgs/CommandRobotiqGripperActionResult
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    Header header
    actionlib_msgs/GoalStatus status
    CommandRobotiqGripperResult result
    
    ================================================================================
    MSG: actionlib_msgs/GoalStatus
    GoalID goal_id
    uint8 status
    uint8 PENDING         = 0   # The goal has yet to be processed by the action server
    uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
    uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                                #   and has since completed its execution (Terminal State)
    uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
    uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                                #    to some failure (Terminal State)
    uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                                #    because the goal was unattainable or invalid (Terminal State)
    uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                                #    and has not yet completed execution
    uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                                #    but the action server has not yet confirmed that the goal is canceled
    uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                                #    and was successfully cancelled (Terminal State)
    uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                                #    sent over the wire by an action server
    
    #Allow for the user to associate a string with GoalStatus for debugging
    string text
    
    
    ================================================================================
    MSG: robotiq_2f_gripper_msgs/CommandRobotiqGripperResult
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    #result definition
    Header header
    bool is_ready
    bool is_reset
    bool is_moving
    bool obj_detected
    uint8 fault_status
    float32 position
    float32 requested_position
    float32 current
    
    ================================================================================
    MSG: robotiq_2f_gripper_msgs/CommandRobotiqGripperActionFeedback
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    Header header
    actionlib_msgs/GoalStatus status
    CommandRobotiqGripperFeedback feedback
    
    ================================================================================
    MSG: robotiq_2f_gripper_msgs/CommandRobotiqGripperFeedback
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    #feedback
    Header header
    bool is_ready
    bool is_reset
    bool is_moving
    bool obj_detected
    uint8 fault_status
    float32 position
    float32 requested_position
    float32 current
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CommandRobotiqGripperAction(null);
    if (msg.action_goal !== undefined) {
      resolved.action_goal = CommandRobotiqGripperActionGoal.Resolve(msg.action_goal)
    }
    else {
      resolved.action_goal = new CommandRobotiqGripperActionGoal()
    }

    if (msg.action_result !== undefined) {
      resolved.action_result = CommandRobotiqGripperActionResult.Resolve(msg.action_result)
    }
    else {
      resolved.action_result = new CommandRobotiqGripperActionResult()
    }

    if (msg.action_feedback !== undefined) {
      resolved.action_feedback = CommandRobotiqGripperActionFeedback.Resolve(msg.action_feedback)
    }
    else {
      resolved.action_feedback = new CommandRobotiqGripperActionFeedback()
    }

    return resolved;
    }
};

module.exports = CommandRobotiqGripperAction;
