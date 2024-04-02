
(cl:in-package :asdf)

(defsystem "vn_driver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Vectornav" :depends-on ("_package_Vectornav"))
    (:file "_package_Vectornav" :depends-on ("_package"))
  ))