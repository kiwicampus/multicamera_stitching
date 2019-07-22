;; Auto-generated. Do not edit!


(when (boundp 'video_mapping::GetMoreCameraStatus)
  (if (not (find-package "VIDEO_MAPPING"))
    (make-package "VIDEO_MAPPING"))
  (shadow 'GetMoreCameraStatus (find-package "VIDEO_MAPPING")))
(unless (find-package "VIDEO_MAPPING::GETMORECAMERASTATUS")
  (make-package "VIDEO_MAPPING::GETMORECAMERASTATUS"))
(unless (find-package "VIDEO_MAPPING::GETMORECAMERASTATUSREQUEST")
  (make-package "VIDEO_MAPPING::GETMORECAMERASTATUSREQUEST"))
(unless (find-package "VIDEO_MAPPING::GETMORECAMERASTATUSRESPONSE")
  (make-package "VIDEO_MAPPING::GETMORECAMERASTATUSRESPONSE"))

(in-package "ROS")





(defclass video_mapping::GetMoreCameraStatusRequest
  :super ros::object
  :slots ())

(defmethod video_mapping::GetMoreCameraStatusRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass video_mapping::GetMoreCameraStatusResponse
  :super ros::object
  :slots (_camera_status ))

(defmethod video_mapping::GetMoreCameraStatusResponse
  (:init
   (&key
    ((:camera_status __camera_status) (let (r) (dotimes (i 3) (push "" r)) r))
    )
   (send-super :init)
   (setq _camera_status __camera_status)
   self)
  (:camera_status
   (&optional __camera_status)
   (if __camera_status (setq _camera_status __camera_status)) _camera_status)
  (:serialization-length
   ()
   (+
    ;; string[3] _camera_status
    (apply #'+ 4 (length _camera_status)    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[3] _camera_status
     (write-long (length _camera_status) s)
     (dolist (elem _camera_status)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[3] _camera_status
   (dotimes (i (length _camera_status))
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _camera_status i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     )
   ;;
   self)
  )

(defclass video_mapping::GetMoreCameraStatus
  :super ros::object
  :slots ())

(setf (get video_mapping::GetMoreCameraStatus :md5sum-) "086d9f0441c56e91ac64021d69c7ba0c")
(setf (get video_mapping::GetMoreCameraStatus :datatype-) "video_mapping/GetMoreCameraStatus")
(setf (get video_mapping::GetMoreCameraStatus :request) video_mapping::GetMoreCameraStatusRequest)
(setf (get video_mapping::GetMoreCameraStatus :response) video_mapping::GetMoreCameraStatusResponse)

(defmethod video_mapping::GetMoreCameraStatusRequest
  (:response () (instance video_mapping::GetMoreCameraStatusResponse :init)))

(setf (get video_mapping::GetMoreCameraStatusRequest :md5sum-) "086d9f0441c56e91ac64021d69c7ba0c")
(setf (get video_mapping::GetMoreCameraStatusRequest :datatype-) "video_mapping/GetMoreCameraStatusRequest")
(setf (get video_mapping::GetMoreCameraStatusRequest :definition-)
      "---
string[3] camera_status

")

(setf (get video_mapping::GetMoreCameraStatusResponse :md5sum-) "086d9f0441c56e91ac64021d69c7ba0c")
(setf (get video_mapping::GetMoreCameraStatusResponse :datatype-) "video_mapping/GetMoreCameraStatusResponse")
(setf (get video_mapping::GetMoreCameraStatusResponse :definition-)
      "---
string[3] camera_status

")



(provide :video_mapping/GetMoreCameraStatus "086d9f0441c56e91ac64021d69c7ba0c")


