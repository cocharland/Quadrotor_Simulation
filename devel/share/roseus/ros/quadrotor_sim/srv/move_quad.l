;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_sim::move_quad)
  (if (not (find-package "QUADROTOR_SIM"))
    (make-package "QUADROTOR_SIM"))
  (shadow 'move_quad (find-package "QUADROTOR_SIM")))
(unless (find-package "QUADROTOR_SIM::MOVE_QUAD")
  (make-package "QUADROTOR_SIM::MOVE_QUAD"))
(unless (find-package "QUADROTOR_SIM::MOVE_QUADREQUEST")
  (make-package "QUADROTOR_SIM::MOVE_QUADREQUEST"))
(unless (find-package "QUADROTOR_SIM::MOVE_QUADRESPONSE")
  (make-package "QUADROTOR_SIM::MOVE_QUADRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass quadrotor_sim::move_quadRequest
  :super ros::object
  :slots (_commandedPose ))

(defmethod quadrotor_sim::move_quadRequest
  (:init
   (&key
    ((:commandedPose __commandedPose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _commandedPose __commandedPose)
   self)
  (:commandedPose
   (&rest __commandedPose)
   (if (keywordp (car __commandedPose))
       (send* _commandedPose __commandedPose)
     (progn
       (if __commandedPose (setq _commandedPose (car __commandedPose)))
       _commandedPose)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _commandedPose
    (send _commandedPose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _commandedPose
       (send _commandedPose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _commandedPose
     (send _commandedPose :deserialize buf ptr-) (incf ptr- (send _commandedPose :serialization-length))
   ;;
   self)
  )

(defclass quadrotor_sim::move_quadResponse
  :super ros::object
  :slots (_success ))

(defmethod quadrotor_sim::move_quadResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass quadrotor_sim::move_quad
  :super ros::object
  :slots ())

(setf (get quadrotor_sim::move_quad :md5sum-) "30f567500cb1fdfeac7274b9d9e7eaf0")
(setf (get quadrotor_sim::move_quad :datatype-) "quadrotor_sim/move_quad")
(setf (get quadrotor_sim::move_quad :request) quadrotor_sim::move_quadRequest)
(setf (get quadrotor_sim::move_quad :response) quadrotor_sim::move_quadResponse)

(defmethod quadrotor_sim::move_quadRequest
  (:response () (instance quadrotor_sim::move_quadResponse :init)))

(setf (get quadrotor_sim::move_quadRequest :md5sum-) "30f567500cb1fdfeac7274b9d9e7eaf0")
(setf (get quadrotor_sim::move_quadRequest :datatype-) "quadrotor_sim/move_quadRequest")
(setf (get quadrotor_sim::move_quadRequest :definition-)
      "geometry_msgs/Pose commandedPose

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
---
bool success

")

(setf (get quadrotor_sim::move_quadResponse :md5sum-) "30f567500cb1fdfeac7274b9d9e7eaf0")
(setf (get quadrotor_sim::move_quadResponse :datatype-) "quadrotor_sim/move_quadResponse")
(setf (get quadrotor_sim::move_quadResponse :definition-)
      "geometry_msgs/Pose commandedPose

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
---
bool success

")



(provide :quadrotor_sim/move_quad "30f567500cb1fdfeac7274b9d9e7eaf0")


