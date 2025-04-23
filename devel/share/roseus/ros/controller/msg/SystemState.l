;; Auto-generated. Do not edit!


(when (boundp 'controller::SystemState)
  (if (not (find-package "CONTROLLER"))
    (make-package "CONTROLLER"))
  (shadow 'SystemState (find-package "CONTROLLER")))
(unless (find-package "CONTROLLER::SYSTEMSTATE")
  (make-package "CONTROLLER::SYSTEMSTATE"))

(in-package "ROS")
;;//! \htmlinclude SystemState.msg.html


(defclass controller::SystemState
  :super ros::object
  :slots (_state ))

(defmethod controller::SystemState
  (:init
   (&key
    ((:state __state) "")
    )
   (send-super :init)
   (setq _state (string __state))
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; string _state
    4 (length _state)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _state
       (write-long (length _state) s) (princ _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get controller::SystemState :md5sum-) "af6d3a99f0fbeb66d3248fa4b3e675fb")
(setf (get controller::SystemState :datatype-) "controller/SystemState")
(setf (get controller::SystemState :definition-)
      "string state
")



(provide :controller/SystemState "af6d3a99f0fbeb66d3248fa4b3e675fb")


