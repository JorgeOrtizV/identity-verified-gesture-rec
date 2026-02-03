; Auto-generated. Do not edit!


(cl:in-package tof_preprocessing-msg)


;//! \htmlinclude Blob.msg.html

(cl:defclass <Blob> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (cx
    :reader cx
    :initarg :cx
    :type cl:float
    :initform 0.0)
   (cy
    :reader cy
    :initarg :cy
    :type cl:float
    :initform 0.0)
   (x
    :reader x
    :initarg :x
    :type cl:integer
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:integer
    :initform 0)
   (w
    :reader w
    :initarg :w
    :type cl:integer
    :initform 0)
   (h
    :reader h
    :initarg :h
    :type cl:integer
    :initform 0)
   (area
    :reader area
    :initarg :area
    :type cl:float
    :initform 0.0)
   (is_static
    :reader is_static
    :initarg :is_static
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Blob (<Blob>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Blob>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Blob)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tof_preprocessing-msg:<Blob> is deprecated: use tof_preprocessing-msg:Blob instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <Blob>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tof_preprocessing-msg:id-val is deprecated.  Use tof_preprocessing-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'cx-val :lambda-list '(m))
(cl:defmethod cx-val ((m <Blob>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tof_preprocessing-msg:cx-val is deprecated.  Use tof_preprocessing-msg:cx instead.")
  (cx m))

(cl:ensure-generic-function 'cy-val :lambda-list '(m))
(cl:defmethod cy-val ((m <Blob>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tof_preprocessing-msg:cy-val is deprecated.  Use tof_preprocessing-msg:cy instead.")
  (cy m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Blob>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tof_preprocessing-msg:x-val is deprecated.  Use tof_preprocessing-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Blob>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tof_preprocessing-msg:y-val is deprecated.  Use tof_preprocessing-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'w-val :lambda-list '(m))
(cl:defmethod w-val ((m <Blob>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tof_preprocessing-msg:w-val is deprecated.  Use tof_preprocessing-msg:w instead.")
  (w m))

(cl:ensure-generic-function 'h-val :lambda-list '(m))
(cl:defmethod h-val ((m <Blob>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tof_preprocessing-msg:h-val is deprecated.  Use tof_preprocessing-msg:h instead.")
  (h m))

(cl:ensure-generic-function 'area-val :lambda-list '(m))
(cl:defmethod area-val ((m <Blob>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tof_preprocessing-msg:area-val is deprecated.  Use tof_preprocessing-msg:area instead.")
  (area m))

(cl:ensure-generic-function 'is_static-val :lambda-list '(m))
(cl:defmethod is_static-val ((m <Blob>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tof_preprocessing-msg:is_static-val is deprecated.  Use tof_preprocessing-msg:is_static instead.")
  (is_static m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Blob>) ostream)
  "Serializes a message object of type '<Blob>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'w)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'h)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'area))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_static) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Blob>) istream)
  "Deserializes a message object of type '<Blob>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'w) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'h) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'area) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'is_static) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Blob>)))
  "Returns string type for a message object of type '<Blob>"
  "tof_preprocessing/Blob")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Blob)))
  "Returns string type for a message object of type 'Blob"
  "tof_preprocessing/Blob")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Blob>)))
  "Returns md5sum for a message object of type '<Blob>"
  "ae73ff5c5745f08a16d613edb0951120")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Blob)))
  "Returns md5sum for a message object of type 'Blob"
  "ae73ff5c5745f08a16d613edb0951120")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Blob>)))
  "Returns full string definition for message of type '<Blob>"
  (cl:format cl:nil "int32 id~%float32 cx~%float32 cy~%int32 x~%int32 y~%int32 w~%int32 h~%float32 area~%bool is_static~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Blob)))
  "Returns full string definition for message of type 'Blob"
  (cl:format cl:nil "int32 id~%float32 cx~%float32 cy~%int32 x~%int32 y~%int32 w~%int32 h~%float32 area~%bool is_static~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Blob>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Blob>))
  "Converts a ROS message object to a list"
  (cl:list 'Blob
    (cl:cons ':id (id msg))
    (cl:cons ':cx (cx msg))
    (cl:cons ':cy (cy msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':w (w msg))
    (cl:cons ':h (h msg))
    (cl:cons ':area (area msg))
    (cl:cons ':is_static (is_static msg))
))
