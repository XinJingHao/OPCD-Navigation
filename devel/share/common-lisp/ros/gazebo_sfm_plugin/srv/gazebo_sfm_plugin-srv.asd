
(cl:in-package :asdf)

(defsystem "gazebo_sfm_plugin-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ped_state" :depends-on ("_package_ped_state"))
    (:file "_package_ped_state" :depends-on ("_package"))
  ))