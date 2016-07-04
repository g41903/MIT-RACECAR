; Auto-generated. Do not edit!


(cl:in-package visual_servoing-msg)


;//! \htmlinclude CameraObject.msg.html

(cl:defclass <CameraObject> (roslisp-msg-protocol:ros-message)
  ((label
    :reader label
    :initarg :label
    :type cl:string
    :initform "")
   (center
    :reader center
    :initarg :center
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (size
    :reader size
    :initarg :size
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass CameraObject (<CameraObject>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CameraObject>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CameraObject)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name visual_servoing-msg:<CameraObject> is deprecated: use visual_servoing-msg:CameraObject instead.")))

(cl:ensure-generic-function 'label-val :lambda-list '(m))
(cl:defmethod label-val ((m <CameraObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visual_servoing-msg:label-val is deprecated.  Use visual_servoing-msg:label instead.")
  (label m))

(cl:ensure-generic-function 'center-val :lambda-list '(m))
(cl:defmethod center-val ((m <CameraObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visual_servoing-msg:center-val is deprecated.  Use visual_servoing-msg:center instead.")
  (center m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <CameraObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visual_servoing-msg:size-val is deprecated.  Use visual_servoing-msg:size instead.")
  (size m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CameraObject>) ostream)
  "Serializes a message object of type '<CameraObject>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'label))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'label))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'center) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'size) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CameraObject>) istream)
  "Deserializes a message object of type '<CameraObject>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'label) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'label) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'center) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'size) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CameraObject>)))
  "Returns string type for a message object of type '<CameraObject>"
  "visual_servoing/CameraObject")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CameraObject)))
  "Returns string type for a message object of type 'CameraObject"
  "visual_servoing/CameraObject")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CameraObject>)))
  "Returns md5sum for a message object of type '<CameraObject>"
  "d478c17254577f2d7871f7b466c8761a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CameraObject)))
  "Returns md5sum for a message object of type 'CameraObject"
  "d478c17254577f2d7871f7b466c8761a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CameraObject>)))
  "Returns full string definition for message of type '<CameraObject>"
  (cl:format cl:nil "string label~%geometry_msgs/Point center  # in pixels, from center (haha, maybe not really)~%geometry_msgs/Vector3 size  # in pixels~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CameraObject)))
  "Returns full string definition for message of type 'CameraObject"
  (cl:format cl:nil "string label~%geometry_msgs/Point center  # in pixels, from center (haha, maybe not really)~%geometry_msgs/Vector3 size  # in pixels~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CameraObject>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'label))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'center))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'size))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CameraObject>))
  "Converts a ROS message object to a list"
  (cl:list 'CameraObject
    (cl:cons ':label (label msg))
    (cl:cons ':center (center msg))
    (cl:cons ':size (size msg))
))
