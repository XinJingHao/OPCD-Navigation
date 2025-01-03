
(cl:in-package :asdf)

(defsystem "spencer_vision_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PersonImage" :depends-on ("_package_PersonImage"))
    (:file "_package_PersonImage" :depends-on ("_package"))
    (:file "PersonImages" :depends-on ("_package_PersonImages"))
    (:file "_package_PersonImages" :depends-on ("_package"))
    (:file "PersonROI" :depends-on ("_package_PersonROI"))
    (:file "_package_PersonROI" :depends-on ("_package"))
    (:file "PersonROIs" :depends-on ("_package_PersonROIs"))
    (:file "_package_PersonROIs" :depends-on ("_package"))
  ))