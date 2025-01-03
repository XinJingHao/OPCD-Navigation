
(cl:in-package :asdf)

(defsystem "spencer_social_relation_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "SocialActivities" :depends-on ("_package_SocialActivities"))
    (:file "_package_SocialActivities" :depends-on ("_package"))
    (:file "SocialActivity" :depends-on ("_package_SocialActivity"))
    (:file "_package_SocialActivity" :depends-on ("_package"))
    (:file "SocialRelation" :depends-on ("_package_SocialRelation"))
    (:file "_package_SocialRelation" :depends-on ("_package"))
    (:file "SocialRelations" :depends-on ("_package_SocialRelations"))
    (:file "_package_SocialRelations" :depends-on ("_package"))
  ))