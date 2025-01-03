
(cl:in-package :asdf)

(defsystem "spencer_human_attribute_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CategoricalAttribute" :depends-on ("_package_CategoricalAttribute"))
    (:file "_package_CategoricalAttribute" :depends-on ("_package"))
    (:file "HumanAttributes" :depends-on ("_package_HumanAttributes"))
    (:file "_package_HumanAttributes" :depends-on ("_package"))
    (:file "ScalarAttribute" :depends-on ("_package_ScalarAttribute"))
    (:file "_package_ScalarAttribute" :depends-on ("_package"))
  ))