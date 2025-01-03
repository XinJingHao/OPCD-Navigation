
(cl:in-package :asdf)

(defsystem "spencer_tracking_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :spencer_tracking_msgs-msg
)
  :components ((:file "_package")
    (:file "GetPersonTrajectories" :depends-on ("_package_GetPersonTrajectories"))
    (:file "_package_GetPersonTrajectories" :depends-on ("_package"))
  ))