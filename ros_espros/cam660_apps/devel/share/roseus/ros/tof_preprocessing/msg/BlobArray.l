;; Auto-generated. Do not edit!


(when (boundp 'tof_preprocessing::BlobArray)
  (if (not (find-package "TOF_PREPROCESSING"))
    (make-package "TOF_PREPROCESSING"))
  (shadow 'BlobArray (find-package "TOF_PREPROCESSING")))
(unless (find-package "TOF_PREPROCESSING::BLOBARRAY")
  (make-package "TOF_PREPROCESSING::BLOBARRAY"))

(in-package "ROS")
;;//! \htmlinclude BlobArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tof_preprocessing::BlobArray
  :super ros::object
  :slots (_header _blobs ))

(defmethod tof_preprocessing::BlobArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:blobs __blobs) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _blobs __blobs)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:blobs
   (&rest __blobs)
   (if (keywordp (car __blobs))
       (send* _blobs __blobs)
     (progn
       (if __blobs (setq _blobs (car __blobs)))
       _blobs)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; tof_preprocessing/Blob[] _blobs
    (apply #'+ (send-all _blobs :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; tof_preprocessing/Blob[] _blobs
     (write-long (length _blobs) s)
     (dolist (elem _blobs)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; tof_preprocessing/Blob[] _blobs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _blobs (let (r) (dotimes (i n) (push (instance tof_preprocessing::Blob :init) r)) r))
     (dolist (elem- _blobs)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get tof_preprocessing::BlobArray :md5sum-) "d70469d6a9ebe44f0592672765104aff")
(setf (get tof_preprocessing::BlobArray :datatype-) "tof_preprocessing/BlobArray")
(setf (get tof_preprocessing::BlobArray :definition-)
      "std_msgs/Header header
Blob[] blobs

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: tof_preprocessing/Blob
int32 id
float32 cx
float32 cy
int32 x
int32 y
int32 w
int32 h
float32 area
bool is_static

")



(provide :tof_preprocessing/BlobArray "d70469d6a9ebe44f0592672765104aff")


