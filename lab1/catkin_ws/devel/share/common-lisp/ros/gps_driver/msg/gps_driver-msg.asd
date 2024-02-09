
(cl:in-package :asdf)

(defsystem "gps_driver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Customgps" :depends-on ("_package_Customgps"))
    (:file "_package_Customgps" :depends-on ("_package"))
    (:file "Customrtk" :depends-on ("_package_Customrtk"))
    (:file "_package_Customrtk" :depends-on ("_package"))
  ))