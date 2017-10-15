;; Auto-generated. Do not edit!


(when (boundp 'simple_arm::GoToPosition)
  (if (not (find-package "SIMPLE_ARM"))
    (make-package "SIMPLE_ARM"))
  (shadow 'GoToPosition (find-package "SIMPLE_ARM")))
(unless (find-package "SIMPLE_ARM::GOTOPOSITION")
  (make-package "SIMPLE_ARM::GOTOPOSITION"))
(unless (find-package "SIMPLE_ARM::GOTOPOSITIONREQUEST")
  (make-package "SIMPLE_ARM::GOTOPOSITIONREQUEST"))
(unless (find-package "SIMPLE_ARM::GOTOPOSITIONRESPONSE")
  (make-package "SIMPLE_ARM::GOTOPOSITIONRESPONSE"))

(in-package "ROS")





(defclass simple_arm::GoToPositionRequest
  :super ros::object
  :slots (_joint1 _joint2 ))

(defmethod simple_arm::GoToPositionRequest
  (:init
   (&key
    ((:joint1 __joint1) 0.0)
    ((:joint2 __joint2) 0.0)
    )
   (send-super :init)
   (setq _joint1 (float __joint1))
   (setq _joint2 (float __joint2))
   self)
  (:joint1
   (&optional __joint1)
   (if __joint1 (setq _joint1 __joint1)) _joint1)
  (:joint2
   (&optional __joint2)
   (if __joint2 (setq _joint2 __joint2)) _joint2)
  (:serialization-length
   ()
   (+
    ;; float64 _joint1
    8
    ;; float64 _joint2
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _joint1
       (sys::poke _joint1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _joint2
       (sys::poke _joint2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _joint1
     (setq _joint1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _joint2
     (setq _joint2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass simple_arm::GoToPositionResponse
  :super ros::object
  :slots (_elapsed_time ))

(defmethod simple_arm::GoToPositionResponse
  (:init
   (&key
    ((:elapsed_time __elapsed_time) (instance ros::time :init))
    )
   (send-super :init)
   (setq _elapsed_time __elapsed_time)
   self)
  (:elapsed_time
   (&optional __elapsed_time)
   (if __elapsed_time (setq _elapsed_time __elapsed_time)) _elapsed_time)
  (:serialization-length
   ()
   (+
    ;; duration _elapsed_time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; duration _elapsed_time
       (write-long (send _elapsed_time :sec) s) (write-long (send _elapsed_time :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; duration _elapsed_time
     (send _elapsed_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _elapsed_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass simple_arm::GoToPosition
  :super ros::object
  :slots ())

(setf (get simple_arm::GoToPosition :md5sum-) "ae7a4cd99d8425ae0392ffa28e862a3c")
(setf (get simple_arm::GoToPosition :datatype-) "simple_arm/GoToPosition")
(setf (get simple_arm::GoToPosition :request) simple_arm::GoToPositionRequest)
(setf (get simple_arm::GoToPosition :response) simple_arm::GoToPositionResponse)

(defmethod simple_arm::GoToPositionRequest
  (:response () (instance simple_arm::GoToPositionResponse :init)))

(setf (get simple_arm::GoToPositionRequest :md5sum-) "ae7a4cd99d8425ae0392ffa28e862a3c")
(setf (get simple_arm::GoToPositionRequest :datatype-) "simple_arm/GoToPositionRequest")
(setf (get simple_arm::GoToPositionRequest :definition-)
      "float64 joint1
float64 joint2
---
duration elapsed_time

")

(setf (get simple_arm::GoToPositionResponse :md5sum-) "ae7a4cd99d8425ae0392ffa28e862a3c")
(setf (get simple_arm::GoToPositionResponse :datatype-) "simple_arm/GoToPositionResponse")
(setf (get simple_arm::GoToPositionResponse :definition-)
      "float64 joint1
float64 joint2
---
duration elapsed_time

")



(provide :simple_arm/GoToPosition "ae7a4cd99d8425ae0392ffa28e862a3c")


