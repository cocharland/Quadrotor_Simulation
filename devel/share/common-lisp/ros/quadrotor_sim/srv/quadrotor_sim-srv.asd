
(cl:in-package :asdf)

(defsystem "quadrotor_sim-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :nav_msgs-msg
)
  :components ((:file "_package")
    (:file "AddTwoInts" :depends-on ("_package_AddTwoInts"))
    (:file "_package_AddTwoInts" :depends-on ("_package"))
    (:file "mc_plan" :depends-on ("_package_mc_plan"))
    (:file "_package_mc_plan" :depends-on ("_package"))
    (:file "move_quad" :depends-on ("_package_move_quad"))
    (:file "_package_move_quad" :depends-on ("_package"))
  ))