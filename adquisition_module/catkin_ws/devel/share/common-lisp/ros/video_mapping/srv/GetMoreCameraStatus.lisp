; Auto-generated. Do not edit!


(cl:in-package video_mapping-srv)


;//! \htmlinclude GetMoreCameraStatus-request.msg.html

(cl:defclass <GetMoreCameraStatus-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetMoreCameraStatus-request (<GetMoreCameraStatus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMoreCameraStatus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMoreCameraStatus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name video_mapping-srv:<GetMoreCameraStatus-request> is deprecated: use video_mapping-srv:GetMoreCameraStatus-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMoreCameraStatus-request>) ostream)
  "Serializes a message object of type '<GetMoreCameraStatus-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMoreCameraStatus-request>) istream)
  "Deserializes a message object of type '<GetMoreCameraStatus-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMoreCameraStatus-request>)))
  "Returns string type for a service object of type '<GetMoreCameraStatus-request>"
  "video_mapping/GetMoreCameraStatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMoreCameraStatus-request)))
  "Returns string type for a service object of type 'GetMoreCameraStatus-request"
  "video_mapping/GetMoreCameraStatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMoreCameraStatus-request>)))
  "Returns md5sum for a message object of type '<GetMoreCameraStatus-request>"
  "086d9f0441c56e91ac64021d69c7ba0c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMoreCameraStatus-request)))
  "Returns md5sum for a message object of type 'GetMoreCameraStatus-request"
  "086d9f0441c56e91ac64021d69c7ba0c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMoreCameraStatus-request>)))
  "Returns full string definition for message of type '<GetMoreCameraStatus-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMoreCameraStatus-request)))
  "Returns full string definition for message of type 'GetMoreCameraStatus-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMoreCameraStatus-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMoreCameraStatus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMoreCameraStatus-request
))
;//! \htmlinclude GetMoreCameraStatus-response.msg.html

(cl:defclass <GetMoreCameraStatus-response> (roslisp-msg-protocol:ros-message)
  ((camera_status
    :reader camera_status
    :initarg :camera_status
    :type (cl:vector cl:string)
   :initform (cl:make-array 3 :element-type 'cl:string :initial-element "")))
)

(cl:defclass GetMoreCameraStatus-response (<GetMoreCameraStatus-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMoreCameraStatus-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMoreCameraStatus-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name video_mapping-srv:<GetMoreCameraStatus-response> is deprecated: use video_mapping-srv:GetMoreCameraStatus-response instead.")))

(cl:ensure-generic-function 'camera_status-val :lambda-list '(m))
(cl:defmethod camera_status-val ((m <GetMoreCameraStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader video_mapping-srv:camera_status-val is deprecated.  Use video_mapping-srv:camera_status instead.")
  (camera_status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMoreCameraStatus-response>) ostream)
  "Serializes a message object of type '<GetMoreCameraStatus-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'camera_status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMoreCameraStatus-response>) istream)
  "Deserializes a message object of type '<GetMoreCameraStatus-response>"
  (cl:setf (cl:slot-value msg 'camera_status) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'camera_status)))
    (cl:dotimes (i 3)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMoreCameraStatus-response>)))
  "Returns string type for a service object of type '<GetMoreCameraStatus-response>"
  "video_mapping/GetMoreCameraStatusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMoreCameraStatus-response)))
  "Returns string type for a service object of type 'GetMoreCameraStatus-response"
  "video_mapping/GetMoreCameraStatusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMoreCameraStatus-response>)))
  "Returns md5sum for a message object of type '<GetMoreCameraStatus-response>"
  "086d9f0441c56e91ac64021d69c7ba0c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMoreCameraStatus-response)))
  "Returns md5sum for a message object of type 'GetMoreCameraStatus-response"
  "086d9f0441c56e91ac64021d69c7ba0c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMoreCameraStatus-response>)))
  "Returns full string definition for message of type '<GetMoreCameraStatus-response>"
  (cl:format cl:nil "string[3] camera_status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMoreCameraStatus-response)))
  "Returns full string definition for message of type 'GetMoreCameraStatus-response"
  (cl:format cl:nil "string[3] camera_status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMoreCameraStatus-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'camera_status) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMoreCameraStatus-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMoreCameraStatus-response
    (cl:cons ':camera_status (camera_status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetMoreCameraStatus)))
  'GetMoreCameraStatus-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetMoreCameraStatus)))
  'GetMoreCameraStatus-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMoreCameraStatus)))
  "Returns string type for a service object of type '<GetMoreCameraStatus>"
  "video_mapping/GetMoreCameraStatus")