; Auto-generated. Do not edit!


(cl:in-package visual_servoing-msg)


;//! \htmlinclude CameraObjectsStamped.msg.html

(cl:defclass <CameraObjectsStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (objects
    :reader objects
    :initarg :objects
    :type (cl:vector visual_servoing-msg:CameraObject)
   :initform (cl:make-array 0 :element-type 'visual_servoing-msg:CameraObject :initial-element (cl:make-instance 'visual_servoing-msg:CameraObject))))
)

(cl:defclass CameraObjectsStamped (<CameraObjectsStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CameraObjectsStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CameraObjectsStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name visual_servoing-msg:<CameraObjectsStamped> is deprecated: use visual_servoing-msg:CameraObjectsStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CameraObjectsStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visual_servoing-msg:header-val is deprecated.  Use visual_servoing-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'objects-val :lambda-list '(m))
(cl:defmethod objects-val ((m <CameraObjectsStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visual_servoing-msg:objects-val is deprecated.  Use visual_servoing-msg:objects instead.")
  (objects m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CameraObjectsStamped>) ostream)
  "Serializes a message object of type '<CameraObjectsStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'objects))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'objects))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CameraObjectsStamped>) istream)
  "Deserializes a message object of type '<CameraObjectsStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'objects) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'objects)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'visual_servoing-msg:CameraObject))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CameraObjectsStamped>)))
  "Returns string type for a message object of type '<CameraObjectsStamped>"
  "visual_servoing/CameraObjectsStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CameraObjectsStamped)))
  "Returns string type for a message object of type 'CameraObjectsStamped"
  "visual_servoing/CameraObjectsStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CameraObjectsStamped>)))
  "Returns md5sum for a message object of type '<CameraObjectsStamped>"
  "8b0bdfe879d8fa2c0145283a6281f997")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CameraObjectsStamped)))
  "Returns md5sum for a message object of type 'CameraObjectsStamped"
  "8b0bdfe879d8fa2c0145283a6281f997")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CameraObjectsStamped>)))
  "Returns full string definition for message of type '<CameraObjectsStamped>"
  (cl:format cl:nil "Header header~%CameraObject[] objects~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: visual_servoing/CameraObject~%string label~%geometry_msgs/Point center  # in pixels, from center (haha, maybe not really)~%geometry_msgs/Vector3 size  # in pixels~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CameraObjectsStamped)))
  "Returns full string definition for message of type 'CameraObjectsStamped"
  (cl:format cl:nil "Header header~%CameraObject[] objects~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: visual_servoing/CameraObject~%string label~%geometry_msgs/Point center  # in pixels, from center (haha, maybe not really)~%geometry_msgs/Vector3 size  # in pixels~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CameraObjectsStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'objects) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CameraObjectsStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'CameraObjectsStamped
    (cl:cons ':header (header msg))
    (cl:cons ':objects (objects msg))
))
