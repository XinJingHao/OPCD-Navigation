
(cl:in-package :asdf)

(defsystem "spencer_control_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ComponentStatus" :depends-on ("_package_ComponentStatus"))
    (:file "_package_ComponentStatus" :depends-on ("_package"))
  ))