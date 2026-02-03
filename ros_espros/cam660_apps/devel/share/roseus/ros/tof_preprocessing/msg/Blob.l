;; Auto-generated. Do not edit!


(when (boundp 'tof_preprocessing::Blob)
  (if (not (find-package "TOF_PREPROCESSING"))
    (make-package "TOF_PREPROCESSING"))
  (shadow 'Blob (find-package "TOF_PREPROCESSING")))
(unless (find-package "TOF_PREPROCESSING::BLOB")
  (make-package "TOF_PREPROCESSING::BLOB"))

(in-package "ROS")
;;//! \htmlinclude Blob.msg.html


(defclass tof_preprocessing::Blob
  :super ros::object
  :slots (_id _cx _cy _x _y _w _h _area _is_static ))

(defmethod tof_preprocessing::Blob
  (:init
   (&key
    ((:id __id) 0)
    ((:cx __cx) 0.0)
    ((:cy __cy) 0.0)
    ((:x __x) 0)
    ((:y __y) 0)
    ((:w __w) 0)
    ((:h __h) 0)
    ((:area __area) 0.0)
    ((:is_static __is_static) nil)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _cx (float __cx))
   (setq _cy (float __cy))
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _w (round __w))
   (setq _h (round __h))
   (setq _area (float __area))
   (setq _is_static __is_static)
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
  (:area
   (&optional __area)
   (if __area (setq _area __area)) _area)
  (:is_static
   (&optional (__is_static :null))
   (if (not (eq __is_static :null)) (setq _is_static __is_static)) _is_static)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; float32 _cx
    4
    ;; float32 _cy
    4
    ;; int32 _x
    4
    ;; int32 _y
    4
    ;; int32 _w
    4
    ;; int32 _h
    4
    ;; float32 _area
    4
    ;; bool _is_static
    1
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
     ;; int32 _x
       (write-long _x s)
     ;; int32 _y
       (write-long _y s)
     ;; int32 _w
       (write-long _w s)
     ;; int32 _h
       (write-long _h s)
     ;; float32 _area
       (sys::poke _area (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _is_static
       (if _is_static (write-byte -1 s) (write-byte 0 s))
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
   ;; int32 _x
     (setq _x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y
     (setq _y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _w
     (setq _w (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _h
     (setq _h (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _area
     (setq _area (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _is_static
     (setq _is_static (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get tof_preprocessing::Blob :md5sum-) "ae73ff5c5745f08a16d613edb0951120")
(setf (get tof_preprocessing::Blob :datatype-) "tof_preprocessing/Blob")
(setf (get tof_preprocessing::Blob :definition-)
      "int32 id
float32 cx
float32 cy
int32 x
int32 y
int32 w
int32 h
float32 area
bool is_static

")



(provide :tof_preprocessing/Blob "ae73ff5c5745f08a16d613edb0951120")


