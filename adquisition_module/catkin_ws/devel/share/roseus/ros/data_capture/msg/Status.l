;; Auto-generated. Do not edit!


(when (boundp 'data_capture::Status)
  (if (not (find-package "DATA_CAPTURE"))
    (make-package "DATA_CAPTURE"))
  (shadow 'Status (find-package "DATA_CAPTURE")))
(unless (find-package "DATA_CAPTURE::STATUS")
  (make-package "DATA_CAPTURE::STATUS"))

(in-package "ROS")
;;//! \htmlinclude Status.msg.html


(defclass data_capture::Status
  :super ros::object
  :slots (_recording _fps _space_left _number_images _resolution _quality ))

(defmethod data_capture::Status
  (:init
   (&key
    ((:recording __recording) nil)
    ((:fps __fps) 0.0)
    ((:space_left __space_left) 0.0)
    ((:number_images __number_images) 0)
    ((:resolution __resolution) "")
    ((:quality __quality) 0.0)
    )
   (send-super :init)
   (setq _recording __recording)
   (setq _fps (float __fps))
   (setq _space_left (float __space_left))
   (setq _number_images (round __number_images))
   (setq _resolution (string __resolution))
   (setq _quality (float __quality))
   self)
  (:recording
   (&optional __recording)
   (if __recording (setq _recording __recording)) _recording)
  (:fps
   (&optional __fps)
   (if __fps (setq _fps __fps)) _fps)
  (:space_left
   (&optional __space_left)
   (if __space_left (setq _space_left __space_left)) _space_left)
  (:number_images
   (&optional __number_images)
   (if __number_images (setq _number_images __number_images)) _number_images)
  (:resolution
   (&optional __resolution)
   (if __resolution (setq _resolution __resolution)) _resolution)
  (:quality
   (&optional __quality)
   (if __quality (setq _quality __quality)) _quality)
  (:serialization-length
   ()
   (+
    ;; bool _recording
    1
    ;; float32 _fps
    4
    ;; float32 _space_left
    4
    ;; int32 _number_images
    4
    ;; string _resolution
    4 (length _resolution)
    ;; float32 _quality
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _recording
       (if _recording (write-byte -1 s) (write-byte 0 s))
     ;; float32 _fps
       (sys::poke _fps (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _space_left
       (sys::poke _space_left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _number_images
       (write-long _number_images s)
     ;; string _resolution
       (write-long (length _resolution) s) (princ _resolution s)
     ;; float32 _quality
       (sys::poke _quality (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _recording
     (setq _recording (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _fps
     (setq _fps (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _space_left
     (setq _space_left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _number_images
     (setq _number_images (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _resolution
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _resolution (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _quality
     (setq _quality (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get data_capture::Status :md5sum-) "a2b0fb0d87014489c2b3bdd33a6bb65d")
(setf (get data_capture::Status :datatype-) "data_capture/Status")
(setf (get data_capture::Status :definition-)
      "bool recording
float32 fps
float32 space_left
int32 number_images
string resolution
float32 quality

")



(provide :data_capture/Status "a2b0fb0d87014489c2b3bdd33a6bb65d")


