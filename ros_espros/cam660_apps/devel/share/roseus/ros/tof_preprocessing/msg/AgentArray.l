;; Auto-generated. Do not edit!


(when (boundp 'tof_preprocessing::AgentArray)
  (if (not (find-package "TOF_PREPROCESSING"))
    (make-package "TOF_PREPROCESSING"))
  (shadow 'AgentArray (find-package "TOF_PREPROCESSING")))
(unless (find-package "TOF_PREPROCESSING::AGENTARRAY")
  (make-package "TOF_PREPROCESSING::AGENTARRAY"))

(in-package "ROS")
;;//! \htmlinclude AgentArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tof_preprocessing::AgentArray
  :super ros::object
  :slots (_header _agents ))

(defmethod tof_preprocessing::AgentArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:agents __agents) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _agents __agents)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:agents
   (&rest __agents)
   (if (keywordp (car __agents))
       (send* _agents __agents)
     (progn
       (if __agents (setq _agents (car __agents)))
       _agents)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; tof_preprocessing/AgentMsg[] _agents
    (apply #'+ (send-all _agents :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; tof_preprocessing/AgentMsg[] _agents
     (write-long (length _agents) s)
     (dolist (elem _agents)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; tof_preprocessing/AgentMsg[] _agents
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _agents (let (r) (dotimes (i n) (push (instance tof_preprocessing::AgentMsg :init) r)) r))
     (dolist (elem- _agents)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get tof_preprocessing::AgentArray :md5sum-) "78053eb215ec82a8b9dde5f1fe7feddf")
(setf (get tof_preprocessing::AgentArray :datatype-) "tof_preprocessing/AgentArray")
(setf (get tof_preprocessing::AgentArray :definition-)
      "std_msgs/Header header
AgentMsg[] agents

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
MSG: tof_preprocessing/AgentMsg
int32 id
float32 cx
float32 cy
float32 vx
float32 vy
int32 x
int32 y
int32 w
int32 h
int32 age
string state

")



(provide :tof_preprocessing/AgentArray "78053eb215ec82a8b9dde5f1fe7feddf")


