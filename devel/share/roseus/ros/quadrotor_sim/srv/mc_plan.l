;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_sim::mc_plan)
  (if (not (find-package "QUADROTOR_SIM"))
    (make-package "QUADROTOR_SIM"))
  (shadow 'mc_plan (find-package "QUADROTOR_SIM")))
(unless (find-package "QUADROTOR_SIM::MC_PLAN")
  (make-package "QUADROTOR_SIM::MC_PLAN"))
(unless (find-package "QUADROTOR_SIM::MC_PLANREQUEST")
  (make-package "QUADROTOR_SIM::MC_PLANREQUEST"))
(unless (find-package "QUADROTOR_SIM::MC_PLANRESPONSE")
  (make-package "QUADROTOR_SIM::MC_PLANRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass quadrotor_sim::mc_planRequest
  :super ros::object
  :slots (_currentPose _ProjectedGrid ))

(defmethod quadrotor_sim::mc_planRequest
  (:init
   (&key
    ((:currentPose __currentPose) (instance geometry_msgs::Pose :init))
    ((:ProjectedGrid __ProjectedGrid) (instance nav_msgs::OccupancyGrid :init))
    )
   (send-super :init)
   (setq _currentPose __currentPose)
   (setq _ProjectedGrid __ProjectedGrid)
   self)
  (:currentPose
   (&rest __currentPose)
   (if (keywordp (car __currentPose))
       (send* _currentPose __currentPose)
     (progn
       (if __currentPose (setq _currentPose (car __currentPose)))
       _currentPose)))
  (:ProjectedGrid
   (&rest __ProjectedGrid)
   (if (keywordp (car __ProjectedGrid))
       (send* _ProjectedGrid __ProjectedGrid)
     (progn
       (if __ProjectedGrid (setq _ProjectedGrid (car __ProjectedGrid)))
       _ProjectedGrid)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _currentPose
    (send _currentPose :serialization-length)
    ;; nav_msgs/OccupancyGrid _ProjectedGrid
    (send _ProjectedGrid :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _currentPose
       (send _currentPose :serialize s)
     ;; nav_msgs/OccupancyGrid _ProjectedGrid
       (send _ProjectedGrid :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _currentPose
     (send _currentPose :deserialize buf ptr-) (incf ptr- (send _currentPose :serialization-length))
   ;; nav_msgs/OccupancyGrid _ProjectedGrid
     (send _ProjectedGrid :deserialize buf ptr-) (incf ptr- (send _ProjectedGrid :serialization-length))
   ;;
   self)
  )

(defclass quadrotor_sim::mc_planResponse
  :super ros::object
  :slots (_nextPose ))

(defmethod quadrotor_sim::mc_planResponse
  (:init
   (&key
    ((:nextPose __nextPose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _nextPose __nextPose)
   self)
  (:nextPose
   (&rest __nextPose)
   (if (keywordp (car __nextPose))
       (send* _nextPose __nextPose)
     (progn
       (if __nextPose (setq _nextPose (car __nextPose)))
       _nextPose)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _nextPose
    (send _nextPose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _nextPose
       (send _nextPose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _nextPose
     (send _nextPose :deserialize buf ptr-) (incf ptr- (send _nextPose :serialization-length))
   ;;
   self)
  )

(defclass quadrotor_sim::mc_plan
  :super ros::object
  :slots ())

(setf (get quadrotor_sim::mc_plan :md5sum-) "7bebb743b8230fc9c43aee4ab5473ce7")
(setf (get quadrotor_sim::mc_plan :datatype-) "quadrotor_sim/mc_plan")
(setf (get quadrotor_sim::mc_plan :request) quadrotor_sim::mc_planRequest)
(setf (get quadrotor_sim::mc_plan :response) quadrotor_sim::mc_planResponse)

(defmethod quadrotor_sim::mc_planRequest
  (:response () (instance quadrotor_sim::mc_planResponse :init)))

(setf (get quadrotor_sim::mc_planRequest :md5sum-) "7bebb743b8230fc9c43aee4ab5473ce7")
(setf (get quadrotor_sim::mc_planRequest :datatype-) "quadrotor_sim/mc_planRequest")
(setf (get quadrotor_sim::mc_planRequest :definition-)
      "geometry_msgs/Pose currentPose
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
---
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
")

(setf (get quadrotor_sim::mc_planResponse :md5sum-) "7bebb743b8230fc9c43aee4ab5473ce7")
(setf (get quadrotor_sim::mc_planResponse :datatype-) "quadrotor_sim/mc_planResponse")
(setf (get quadrotor_sim::mc_planResponse :definition-)
      "geometry_msgs/Pose currentPose
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
---
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
")



(provide :quadrotor_sim/mc_plan "7bebb743b8230fc9c43aee4ab5473ce7")


