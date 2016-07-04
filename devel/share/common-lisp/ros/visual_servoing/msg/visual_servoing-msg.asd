
(cl:in-package :asdf)

(defsystem "visual_servoing-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CameraObject" :depends-on ("_package_CameraObject"))
    (:file "_package_CameraObject" :depends-on ("_package"))
    (:file "CameraObjectsStamped" :depends-on ("_package_CameraObjectsStamped"))
    (:file "_package_CameraObjectsStamped" :depends-on ("_package"))
  ))