; Auto-generated. Do not edit!


(cl:in-package darknet_ros_msgs-msg)


;//! \htmlinclude ObjectPoint.msg.html

(cl:defclass <ObjectPoint> (roslisp-msg-protocol:ros-message)
  ((Class
    :reader Class
    :initarg :Class
    :type cl:string
    :initform "")
   (probability
    :reader probability
    :initarg :probability
    :type cl:float
    :initform 0.0)
   (width
    :reader width
    :initarg :width
    :type cl:fixnum
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:fixnum
    :initform 0)
   (point
    :reader point
    :initarg :point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass ObjectPoint (<ObjectPoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjectPoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjectPoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name darknet_ros_msgs-msg:<ObjectPoint> is deprecated: use darknet_ros_msgs-msg:ObjectPoint instead.")))

(cl:ensure-generic-function 'Class-val :lambda-list '(m))
(cl:defmethod Class-val ((m <ObjectPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader darknet_ros_msgs-msg:Class-val is deprecated.  Use darknet_ros_msgs-msg:Class instead.")
  (Class m))

(cl:ensure-generic-function 'probability-val :lambda-list '(m))
(cl:defmethod probability-val ((m <ObjectPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader darknet_ros_msgs-msg:probability-val is deprecated.  Use darknet_ros_msgs-msg:probability instead.")
  (probability m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <ObjectPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader darknet_ros_msgs-msg:width-val is deprecated.  Use darknet_ros_msgs-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <ObjectPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader darknet_ros_msgs-msg:height-val is deprecated.  Use darknet_ros_msgs-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <ObjectPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader darknet_ros_msgs-msg:point-val is deprecated.  Use darknet_ros_msgs-msg:point instead.")
  (point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjectPoint>) ostream)
  "Serializes a message object of type '<ObjectPoint>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Class))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Class))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'probability))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'width)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'height)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjectPoint>) istream)
  "Deserializes a message object of type '<ObjectPoint>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Class) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Class) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'probability) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'width) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'height) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjectPoint>)))
  "Returns string type for a message object of type '<ObjectPoint>"
  "darknet_ros_msgs/ObjectPoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjectPoint)))
  "Returns string type for a message object of type 'ObjectPoint"
  "darknet_ros_msgs/ObjectPoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjectPoint>)))
  "Returns md5sum for a message object of type '<ObjectPoint>"
  "67ee1f8adf25d2c6b869bd19b2901b2c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjectPoint)))
  "Returns md5sum for a message object of type 'ObjectPoint"
  "67ee1f8adf25d2c6b869bd19b2901b2c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjectPoint>)))
  "Returns full string definition for message of type '<ObjectPoint>"
  (cl:format cl:nil "string Class~%float64 probability~%int8 width~%int8 height~%geometry_msgs/Point point~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjectPoint)))
  "Returns full string definition for message of type 'ObjectPoint"
  (cl:format cl:nil "string Class~%float64 probability~%int8 width~%int8 height~%geometry_msgs/Point point~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjectPoint>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'Class))
     8
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjectPoint>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjectPoint
    (cl:cons ':Class (Class msg))
    (cl:cons ':probability (probability msg))
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
    (cl:cons ':point (point msg))
))
