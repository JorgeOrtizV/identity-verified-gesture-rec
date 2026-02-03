;; Auto-generated. Do not edit!


(when (boundp 'tof_preprocessing::AgentMsg)
  (if (not (find-package "TOF_PREPROCESSING"))
    (make-package "TOF_PREPROCESSING"))
  (shadow 'AgentMsg (find-package "TOF_PREPROCESSING")))
(unless (find-package "TOF_PREPROCESSING::AGENTMSG")
  (make-package "TOF_PREPROCESSING::AGENTMSG"))

(in-package "ROS")
;;//! \htmlinclude AgentMsg.msg.html


(defclass tof_preprocessing::AgentMsg
  :super ros::object
  :slots (_id _cx _cy _vx _vy _x _y _w _h _age _state ))

(defmethod tof_preprocessing::AgentMsg
  (:init
   (&key
    ((:id __id) 0)
    ((:cx __cx) 0.0)
    ((:cy __cy) 0.0)
    ((:vx __vx) 0.0)
    ((:vy __vy) 0.0)
    ((:x __x) 0)
    ((:y __y) 0)
    ((:w __w) 0)
    ((:h __h) 0)
    ((:age __age) 0)
    ((:state __state) "")
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _cx (float __cx))
   (setq _cy (float __cy))
   (setq _vx (float __vx))
   (setq _vy (float __vy))
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _w (round __w))
   (setq _h (round __h))
   (setq _age (round __age))
   (setq _state (string __state))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:cx
   (&optional __cx)
   (if __cx (setq _cx __cx)) _cx)
  (:cy
   (&optional __cy)
   (if __cy (setq _cy __cy)) _cy)
  (:vx
   (&optional __vx)
   (if __vx (setq _vx __vx)) _vx)
  (:vy
   (&optional __vy)
   (if __vy (setq _vy __vy)) _vy)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:w
   (&optional __w)
   (if __w (setq _w __w)) _w)
  (:h
   (&optional __h)
   (if __h (setq _h __h)) _h)
  (:age
   (&optional __age)
   (if __age (setq _age __age)) _age)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; float32 _cx
    4
    ;; float32 _cy
    4
    ;; float32 _vx
    4
    ;; float32 _vy
    4
    ;; int32 _x
    4
    ;; int32 _y
    4
    ;; int32 _w
    4
    ;; int32 _h
    4
    ;; int32 _age
    4
    ;; string _state
    4 (length _state)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; float32 _cx
       (sys::poke _cx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cy
       (sys::poke _cy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vx
       (sys::poke _vx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vy
       (sys::poke _vy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _x
       (write-long _x s)
     ;; int32 _y
       (write-long _y s)
     ;; int32 _w
       (write-long _w s)
     ;; int32 _h
       (write-long _h s)
     ;; int32 _age
       (write-long _age s)
     ;; string _state
       (write-long (length _state) s) (princ _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _cx
     (setq _cx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cy
     (setq _cy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vx
     (setq _vx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vy
     (setq _vy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _x
     (setq _x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y
     (setq _y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _w
     (setq _w (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _h
     (setq _h (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _age
     (setq _age (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get tof_preprocessing::AgentMsg :md5sum-) "b8130ef0752937d05f659f991421dc66")
(setf (get tof_preprocessing::AgentMsg :datatype-) "tof_preprocessing/AgentMsg")
(setf (get tof_preprocessing::AgentMsg :definition-)
      "int32 id
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



(provide :tof_preprocessing/AgentMsg "b8130ef0752937d05f659f991421dc66")


