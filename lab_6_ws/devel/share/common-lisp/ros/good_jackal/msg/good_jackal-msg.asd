
(cl:in-package :asdf)

(defsystem "good_jackal-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Tracked_Object" :depends-on ("_package_Tracked_Object"))
    (:file "_package_Tracked_Object" :depends-on ("_package"))
  ))