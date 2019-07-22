
(cl:in-package :asdf)

(defsystem "video_mapping-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetMoreCameraStatus" :depends-on ("_package_GetMoreCameraStatus"))
    (:file "_package_GetMoreCameraStatus" :depends-on ("_package"))
  ))