; Auto-generated. Do not edit!


(cl:in-package tof_preprocessing-msg)


;//! \htmlinclude BlobArray.msg.html

(cl:defclass <BlobArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (blobs
    :reader blobs
    :initarg :blobs
    :type (cl:vector tof_preprocessing-msg:Blob)
   :initform (cl:make-array 0 :element-type 'tof_preprocessing-msg:Blob :initial-element (cl:make-instance 'tof_preprocessing-msg:Blob))))
)

(cl:defclass BlobArray (<BlobArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BlobArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BlobArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tof_preprocessing-msg:<BlobArray> is deprecated: use tof_preprocessing-msg:BlobArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <BlobArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tof_preprocessing-msg:header-val is deprecated.  Use tof_preprocessing-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'blobs-val :lambda-list '(m))
(cl:defmethod blobs-val ((m <BlobArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tof_preprocessing-msg:blobs-val is deprecated.  Use tof_preprocessing-msg:blobs instead.")
  (blobs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BlobArray>) ostream)
  "Serializes a message object of type '<BlobArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'blobs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'blobs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BlobArray>) istream)
  "Deserializes a message object of type '<BlobArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'blobs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'blobs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'tof_preprocessing-msg:Blob))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BlobArray>)))
  "Returns string type for a message object of type '<BlobArray>"
  "tof_preprocessing/BlobArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BlobArray)))
  "Returns string type for a message object of type 'BlobArray"
  "tof_preprocessing/BlobArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BlobArray>)))
  "Returns md5sum for a message object of type '<BlobArray>"
  "d70469d6a9ebe44f0592672765104aff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BlobArray)))
  "Returns md5sum for a message object of type 'BlobArray"
  "d70469d6a9ebe44f0592672765104aff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BlobArray>)))
  "Returns full string definition for message of type '<BlobArray>"
  (cl:format cl:nil "std_msgs/Header header~%Blob[] blobs~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: tof_preprocessing/Blob~%int32 id~%float32 cx~%float32 cy~%int32 x~%int32 y~%int32 w~%int32 h~%float32 area~%bool is_static~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BlobArray)))
  "Returns full string definition for message of type 'BlobArray"
  (cl:format cl:nil "std_msgs/Header header~%Blob[] blobs~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: tof_preprocessing/Blob~%int32 id~%float32 cx~%float32 cy~%int32 x~%int32 y~%int32 w~%int32 h~%float32 area~%bool is_static~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BlobArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'blobs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BlobArray>))
  "Converts a ROS message object to a list"
  (cl:list 'BlobArray
    (cl:cons ':header (header msg))
    (cl:cons ':blobs (blobs msg))
))
