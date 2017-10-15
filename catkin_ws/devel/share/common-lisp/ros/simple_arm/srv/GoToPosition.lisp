; Auto-generated. Do not edit!


(cl:in-package simple_arm-srv)


;//! \htmlinclude GoToPosition-request.msg.html

(cl:defclass <GoToPosition-request> (roslisp-msg-protocol:ros-message)
  ((joint1
    :reader joint1
    :initarg :joint1
    :type cl:float
    :initform 0.0)
   (joint2
    :reader joint2
    :initarg :joint2
    :type cl:float
    :initform 0.0))
)

(cl:defclass GoToPosition-request (<GoToPosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoToPosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoToPosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simple_arm-srv:<GoToPosition-request> is deprecated: use simple_arm-srv:GoToPosition-request instead.")))

(cl:ensure-generic-function 'joint1-val :lambda-list '(m))
(cl:defmethod joint1-val ((m <GoToPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simple_arm-srv:joint1-val is deprecated.  Use simple_arm-srv:joint1 instead.")
  (joint1 m))

(cl:ensure-generic-function 'joint2-val :lambda-list '(m))
(cl:defmethod joint2-val ((m <GoToPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simple_arm-srv:joint2-val is deprecated.  Use simple_arm-srv:joint2 instead.")
  (joint2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoToPosition-request>) ostream)
  "Serializes a message object of type '<GoToPosition-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'joint1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'joint2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoToPosition-request>) istream)
  "Deserializes a message object of type '<GoToPosition-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'joint1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'joint2) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoToPosition-request>)))
  "Returns string type for a service object of type '<GoToPosition-request>"
  "simple_arm/GoToPositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoToPosition-request)))
  "Returns string type for a service object of type 'GoToPosition-request"
  "simple_arm/GoToPositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoToPosition-request>)))
  "Returns md5sum for a message object of type '<GoToPosition-request>"
  "ae7a4cd99d8425ae0392ffa28e862a3c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoToPosition-request)))
  "Returns md5sum for a message object of type 'GoToPosition-request"
  "ae7a4cd99d8425ae0392ffa28e862a3c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoToPosition-request>)))
  "Returns full string definition for message of type '<GoToPosition-request>"
  (cl:format cl:nil "float64 joint1~%float64 joint2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoToPosition-request)))
  "Returns full string definition for message of type 'GoToPosition-request"
  (cl:format cl:nil "float64 joint1~%float64 joint2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoToPosition-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoToPosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GoToPosition-request
    (cl:cons ':joint1 (joint1 msg))
    (cl:cons ':joint2 (joint2 msg))
))
;//! \htmlinclude GoToPosition-response.msg.html

(cl:defclass <GoToPosition-response> (roslisp-msg-protocol:ros-message)
  ((elapsed_time
    :reader elapsed_time
    :initarg :elapsed_time
    :type cl:real
    :initform 0))
)

(cl:defclass GoToPosition-response (<GoToPosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoToPosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoToPosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simple_arm-srv:<GoToPosition-response> is deprecated: use simple_arm-srv:GoToPosition-response instead.")))

(cl:ensure-generic-function 'elapsed_time-val :lambda-list '(m))
(cl:defmethod elapsed_time-val ((m <GoToPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simple_arm-srv:elapsed_time-val is deprecated.  Use simple_arm-srv:elapsed_time instead.")
  (elapsed_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoToPosition-response>) ostream)
  "Serializes a message object of type '<GoToPosition-response>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'elapsed_time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'elapsed_time) (cl:floor (cl:slot-value msg 'elapsed_time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoToPosition-response>) istream)
  "Deserializes a message object of type '<GoToPosition-response>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'elapsed_time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoToPosition-response>)))
  "Returns string type for a service object of type '<GoToPosition-response>"
  "simple_arm/GoToPositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoToPosition-response)))
  "Returns string type for a service object of type 'GoToPosition-response"
  "simple_arm/GoToPositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoToPosition-response>)))
  "Returns md5sum for a message object of type '<GoToPosition-response>"
  "ae7a4cd99d8425ae0392ffa28e862a3c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoToPosition-response)))
  "Returns md5sum for a message object of type 'GoToPosition-response"
  "ae7a4cd99d8425ae0392ffa28e862a3c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoToPosition-response>)))
  "Returns full string definition for message of type '<GoToPosition-response>"
  (cl:format cl:nil "duration elapsed_time~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoToPosition-response)))
  "Returns full string definition for message of type 'GoToPosition-response"
  (cl:format cl:nil "duration elapsed_time~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoToPosition-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoToPosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GoToPosition-response
    (cl:cons ':elapsed_time (elapsed_time msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GoToPosition)))
  'GoToPosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GoToPosition)))
  'GoToPosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoToPosition)))
  "Returns string type for a service object of type '<GoToPosition>"
  "simple_arm/GoToPosition")