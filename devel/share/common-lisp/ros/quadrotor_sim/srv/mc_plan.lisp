; Auto-generated. Do not edit!


(cl:in-package quadrotor_sim-srv)


;//! \htmlinclude mc_plan-request.msg.html

(cl:defclass <mc_plan-request> (roslisp-msg-protocol:ros-message)
  ((currentPose
    :reader currentPose
    :initarg :currentPose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (ProjectedGrid
    :reader ProjectedGrid
    :initarg :ProjectedGrid
    :type nav_msgs-msg:OccupancyGrid
    :initform (cl:make-instance 'nav_msgs-msg:OccupancyGrid)))
)

(cl:defclass mc_plan-request (<mc_plan-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mc_plan-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mc_plan-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_sim-srv:<mc_plan-request> is deprecated: use quadrotor_sim-srv:mc_plan-request instead.")))

(cl:ensure-generic-function 'currentPose-val :lambda-list '(m))
(cl:defmethod currentPose-val ((m <mc_plan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_sim-srv:currentPose-val is deprecated.  Use quadrotor_sim-srv:currentPose instead.")
  (currentPose m))

(cl:ensure-generic-function 'ProjectedGrid-val :lambda-list '(m))
(cl:defmethod ProjectedGrid-val ((m <mc_plan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_sim-srv:ProjectedGrid-val is deprecated.  Use quadrotor_sim-srv:ProjectedGrid instead.")
  (ProjectedGrid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mc_plan-request>) ostream)
  "Serializes a message object of type '<mc_plan-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'currentPose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ProjectedGrid) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mc_plan-request>) istream)
  "Deserializes a message object of type '<mc_plan-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'currentPose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ProjectedGrid) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mc_plan-request>)))
  "Returns string type for a service object of type '<mc_plan-request>"
  "quadrotor_sim/mc_planRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mc_plan-request)))
  "Returns string type for a service object of type 'mc_plan-request"
  "quadrotor_sim/mc_planRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mc_plan-request>)))
  "Returns md5sum for a message object of type '<mc_plan-request>"
  "7bebb743b8230fc9c43aee4ab5473ce7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mc_plan-request)))
  "Returns md5sum for a message object of type 'mc_plan-request"
  "7bebb743b8230fc9c43aee4ab5473ce7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mc_plan-request>)))
  "Returns full string definition for message of type '<mc_plan-request>"
  (cl:format cl:nil "geometry_msgs/Pose currentPose~%nav_msgs/OccupancyGrid ProjectedGrid~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: nav_msgs/OccupancyGrid~%# This represents a 2-D grid map, in which each cell represents the probability of~%# occupancy.~%~%Header header ~%~%#MetaData for the map~%MapMetaData info~%~%# The map data, in row-major order, starting with (0,0).  Occupancy~%# probabilities are in the range [0,100].  Unknown is -1.~%int8[] data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: nav_msgs/MapMetaData~%# This hold basic information about the characterists of the OccupancyGrid~%~%# The time at which the map was loaded~%time map_load_time~%# The map resolution [m/cell]~%float32 resolution~%# Map width [cells]~%uint32 width~%# Map height [cells]~%uint32 height~%# The origin of the map [m, m, rad].  This is the real-world pose of the~%# cell (0,0) in the map.~%geometry_msgs/Pose origin~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mc_plan-request)))
  "Returns full string definition for message of type 'mc_plan-request"
  (cl:format cl:nil "geometry_msgs/Pose currentPose~%nav_msgs/OccupancyGrid ProjectedGrid~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: nav_msgs/OccupancyGrid~%# This represents a 2-D grid map, in which each cell represents the probability of~%# occupancy.~%~%Header header ~%~%#MetaData for the map~%MapMetaData info~%~%# The map data, in row-major order, starting with (0,0).  Occupancy~%# probabilities are in the range [0,100].  Unknown is -1.~%int8[] data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: nav_msgs/MapMetaData~%# This hold basic information about the characterists of the OccupancyGrid~%~%# The time at which the map was loaded~%time map_load_time~%# The map resolution [m/cell]~%float32 resolution~%# Map width [cells]~%uint32 width~%# Map height [cells]~%uint32 height~%# The origin of the map [m, m, rad].  This is the real-world pose of the~%# cell (0,0) in the map.~%geometry_msgs/Pose origin~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mc_plan-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'currentPose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ProjectedGrid))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mc_plan-request>))
  "Converts a ROS message object to a list"
  (cl:list 'mc_plan-request
    (cl:cons ':currentPose (currentPose msg))
    (cl:cons ':ProjectedGrid (ProjectedGrid msg))
))
;//! \htmlinclude mc_plan-response.msg.html

(cl:defclass <mc_plan-response> (roslisp-msg-protocol:ros-message)
  ((nextPose
    :reader nextPose
    :initarg :nextPose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass mc_plan-response (<mc_plan-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mc_plan-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mc_plan-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_sim-srv:<mc_plan-response> is deprecated: use quadrotor_sim-srv:mc_plan-response instead.")))

(cl:ensure-generic-function 'nextPose-val :lambda-list '(m))
(cl:defmethod nextPose-val ((m <mc_plan-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_sim-srv:nextPose-val is deprecated.  Use quadrotor_sim-srv:nextPose instead.")
  (nextPose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mc_plan-response>) ostream)
  "Serializes a message object of type '<mc_plan-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'nextPose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mc_plan-response>) istream)
  "Deserializes a message object of type '<mc_plan-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'nextPose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mc_plan-response>)))
  "Returns string type for a service object of type '<mc_plan-response>"
  "quadrotor_sim/mc_planResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mc_plan-response)))
  "Returns string type for a service object of type 'mc_plan-response"
  "quadrotor_sim/mc_planResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mc_plan-response>)))
  "Returns md5sum for a message object of type '<mc_plan-response>"
  "7bebb743b8230fc9c43aee4ab5473ce7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mc_plan-response)))
  "Returns md5sum for a message object of type 'mc_plan-response"
  "7bebb743b8230fc9c43aee4ab5473ce7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mc_plan-response>)))
  "Returns full string definition for message of type '<mc_plan-response>"
  (cl:format cl:nil "geometry_msgs/Pose nextPose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mc_plan-response)))
  "Returns full string definition for message of type 'mc_plan-response"
  (cl:format cl:nil "geometry_msgs/Pose nextPose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mc_plan-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'nextPose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mc_plan-response>))
  "Converts a ROS message object to a list"
  (cl:list 'mc_plan-response
    (cl:cons ':nextPose (nextPose msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'mc_plan)))
  'mc_plan-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'mc_plan)))
  'mc_plan-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mc_plan)))
  "Returns string type for a service object of type '<mc_plan>"
  "quadrotor_sim/mc_plan")