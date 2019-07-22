;; Auto-generated. Do not edit!


(when (boundp 'video_mapping::Status)
  (if (not (find-package "VIDEO_MAPPING"))
    (make-package "VIDEO_MAPPING"))
  (shadow 'Status (find-package "VIDEO_MAPPING")))
(unless (find-package "VIDEO_MAPPING::STATUS")
  (make-package "VIDEO_MAPPING::STATUS"))

(in-package "ROS")
;;//! \htmlinclude Status.msg.html


(defclass video_mapping::Status
  :super ros::object
  :slots (_resolution _quality ))

(defmethod video_mapping::Status
  (:init
   (&key
    ((:resolution __resolution) "")
    ((:quality __quality) 0.0)
    )
   (send-super :init)
   (setq _resolution (string __resolution))
   (setq _quality (float __quality))
   self)
  (:resolution
   (&optional __resolution)
   (if __resolution (setq _resolution __resolution)) _resolution)
  (:quality
   (&optional __quality)
   (if __quality (setq _quality __quality)) _quality)
  (:serialization-length
   ()
   (+
    ;; string _resolution
    4 (length _resolution)
    ;; float32 _quality
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _resolution
       (write-long (length _resolution) s) (princ _resolution s)
     ;; float32 _quality
       (sys::poke _quality (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _resolution
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _resolution (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _quality
     (setq _quality (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get video_mapping::Status :md5sum-) "68de8bf5b6d9757586af8a4bdbc8a1a5")
(setf (get video_mapping::Status :datatype-) "video_mapping/Status")
(setf (get video_mapping::Status :definition-)
      "string resolution
float32 quality

")



(provide :video_mapping/Status "68de8bf5b6d9757586af8a4bdbc8a1a5")


