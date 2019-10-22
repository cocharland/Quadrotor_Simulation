// Auto-generated. Do not edit!

// (in-package quadrotor_sim.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let nav_msgs = _finder('nav_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class mc_planRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.currentPose = null;
      this.ProjectedGrid = null;
    }
    else {
      if (initObj.hasOwnProperty('currentPose')) {
        this.currentPose = initObj.currentPose
      }
      else {
        this.currentPose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('ProjectedGrid')) {
        this.ProjectedGrid = initObj.ProjectedGrid
      }
      else {
        this.ProjectedGrid = new nav_msgs.msg.OccupancyGrid();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type mc_planRequest
    // Serialize message field [currentPose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.currentPose, buffer, bufferOffset);
    // Serialize message field [ProjectedGrid]
    bufferOffset = nav_msgs.msg.OccupancyGrid.serialize(obj.ProjectedGrid, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type mc_planRequest
    let len;
    let data = new mc_planRequest(null);
    // Deserialize message field [currentPose]
    data.currentPose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [ProjectedGrid]
    data.ProjectedGrid = nav_msgs.msg.OccupancyGrid.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += nav_msgs.msg.OccupancyGrid.getMessageSize(object.ProjectedGrid);
    return length + 56;
  }

  static datatype() {
    // Returns string type for a service object
    return 'quadrotor_sim/mc_planRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bb975d1ef6b37dd9003c880e25194ca5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose currentPose
    nav_msgs/OccupancyGrid ProjectedGrid
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: nav_msgs/OccupancyGrid
    # This represents a 2-D grid map, in which each cell represents the probability of
    # occupancy.
    
    Header header 
    
    #MetaData for the map
    MapMetaData info
    
    # The map data, in row-major order, starting with (0,0).  Occupancy
    # probabilities are in the range [0,100].  Unknown is -1.
    int8[] data
    
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
    MSG: nav_msgs/MapMetaData
    # This hold basic information about the characterists of the OccupancyGrid
    
    # The time at which the map was loaded
    time map_load_time
    # The map resolution [m/cell]
    float32 resolution
    # Map width [cells]
    uint32 width
    # Map height [cells]
    uint32 height
    # The origin of the map [m, m, rad].  This is the real-world pose of the
    # cell (0,0) in the map.
    geometry_msgs/Pose origin
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new mc_planRequest(null);
    if (msg.currentPose !== undefined) {
      resolved.currentPose = geometry_msgs.msg.Pose.Resolve(msg.currentPose)
    }
    else {
      resolved.currentPose = new geometry_msgs.msg.Pose()
    }

    if (msg.ProjectedGrid !== undefined) {
      resolved.ProjectedGrid = nav_msgs.msg.OccupancyGrid.Resolve(msg.ProjectedGrid)
    }
    else {
      resolved.ProjectedGrid = new nav_msgs.msg.OccupancyGrid()
    }

    return resolved;
    }
};

class mc_planResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.nextPose = null;
    }
    else {
      if (initObj.hasOwnProperty('nextPose')) {
        this.nextPose = initObj.nextPose
      }
      else {
        this.nextPose = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type mc_planResponse
    // Serialize message field [nextPose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.nextPose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type mc_planResponse
    let len;
    let data = new mc_planResponse(null);
    // Deserialize message field [nextPose]
    data.nextPose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a service object
    return 'quadrotor_sim/mc_planResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '43874bb3f8ebc78572ab5e24194fe094';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose nextPose
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new mc_planResponse(null);
    if (msg.nextPose !== undefined) {
      resolved.nextPose = geometry_msgs.msg.Pose.Resolve(msg.nextPose)
    }
    else {
      resolved.nextPose = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

module.exports = {
  Request: mc_planRequest,
  Response: mc_planResponse,
  md5sum() { return '7bebb743b8230fc9c43aee4ab5473ce7'; },
  datatype() { return 'quadrotor_sim/mc_plan'; }
};
