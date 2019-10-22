; Auto-generated. Do not edit!


(cl:in-package quadrotor_sim-srv)


;//! \htmlinclude move_quad-request.msg.html

(cl:defclass <move_quad-request> (roslisp-msg-protocol:ros-message)
  ((commandedPose
    :reader commandedPose
    :initarg :commandedPose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass move_quad-request (<move_quad-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <move_quad-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'move_quad-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_sim-srv:<move_quad-request> is deprecated: use quadrotor_sim-srv:move_quad-request instead.")))

(cl:ensure-generic-function 'commandedPose-val :lambda-list '(m))
(cl:defmethod commandedPose-val ((m <move_quad-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_sim-srv:commandedPose-val is deprecated.  Use quadrotor_sim-srv:commandedPose instead.")
  (commandedPose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <move_quad-request>) ostream)
  "Serializes a message object of type '<move_quad-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'commandedPose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <move_quad-request>) istream)
  "Deserializes a message object of type '<move_quad-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'commandedPose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<move_quad-request>)))
  "Returns string type for a service object of type '<move_quad-request>"
  "quadrotor_sim/move_quadRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'move_quad-request)))
  "Returns string type for a service object of type 'move_quad-request"
  "quadrotor_sim/move_quadRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<move_quad-request>)))
  "Returns md5sum for a message object of type '<move_quad-request>"
  "30f567500cb1fdfeac7274b9d9e7eaf0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'move_quad-request)))
  "Returns md5sum for a message object of type 'move_quad-request"
  "30f567500cb1fdfeac7274b9d9e7eaf0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<move_quad-request>)))
  "Returns full string definition for message of type '<move_quad-request>"
  (cl:format cl:nil "geometry_msgs/Pose commandedPose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'move_quad-request)))
  "Returns full string definition for message of type 'move_quad-request"
  (cl:format cl:nil "geometry_msgs/Pose commandedPose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <move_quad-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'commandedPose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <move_quad-request>))
  "Converts a ROS message object to a list"
  (cl:list 'move_quad-request
    (cl:cons ':commandedPose (commandedPose msg))
))
;//! \htmlinclude move_quad-response.msg.html

(cl:defclass <move_quad-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass move_quad-response (<move_quad-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <move_quad-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'move_quad-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_sim-srv:<move_quad-response> is deprecated: use quadrotor_sim-srv:move_quad-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <move_quad-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_sim-srv:success-val is deprecated.  Use quadrotor_sim-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <move_quad-response>) ostream)
  "Serializes a message object of type '<move_quad-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <move_quad-response>) istream)
  "Deserializes a message object of type '<move_quad-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<move_quad-response>)))
  "Returns string type for a service object of type '<move_quad-response>"
  "quadrotor_sim/move_quadResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'move_quad-response)))
  "Returns string type for a service object of type 'move_quad-response"
  "quadrotor_sim/move_quadResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<move_quad-response>)))
  "Returns md5sum for a message object of type '<move_quad-response>"
  "30f567500cb1fdfeac7274b9d9e7eaf0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'move_quad-response)))
  "Returns md5sum for a message object of type 'move_quad-response"
  "30f567500cb1fdfeac7274b9d9e7eaf0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<move_quad-response>)))
  "Returns full string definition for message of type '<move_quad-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'move_quad-response)))
  "Returns full string definition for message of type 'move_quad-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <move_quad-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <move_quad-response>))
  "Converts a ROS message object to a list"
  (cl:list 'move_quad-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'move_quad)))
  'move_quad-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'move_quad)))
  'move_quad-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'move_quad)))
  "Returns string type for a service object of type '<move_quad>"
  "quadrotor_sim/move_quad")