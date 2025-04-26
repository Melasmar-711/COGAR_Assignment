;; Auto-generated. Do not edit!


(when (boundp 'assignments::CheckJointState)
  (if (not (find-package "ASSIGNMENTS"))
    (make-package "ASSIGNMENTS"))
  (shadow 'CheckJointState (find-package "ASSIGNMENTS")))
(unless (find-package "ASSIGNMENTS::CHECKJOINTSTATE")
  (make-package "ASSIGNMENTS::CHECKJOINTSTATE"))
(unless (find-package "ASSIGNMENTS::CHECKJOINTSTATEREQUEST")
  (make-package "ASSIGNMENTS::CHECKJOINTSTATEREQUEST"))
(unless (find-package "ASSIGNMENTS::CHECKJOINTSTATERESPONSE")
  (make-package "ASSIGNMENTS::CHECKJOINTSTATERESPONSE"))

(in-package "ROS")





(defclass assignments::CheckJointStateRequest
  :super ros::object
  :slots (_positions _velocities _efforts ))

(defmethod assignments::CheckJointStateRequest
  (:init
   (&key
    ((:positions __positions) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:velocities __velocities) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:efforts __efforts) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _positions __positions)
   (setq _velocities __velocities)
   (setq _efforts __efforts)
   self)
  (:positions
   (&optional __positions)
   (if __positions (setq _positions __positions)) _positions)
  (:velocities
   (&optional __velocities)
   (if __velocities (setq _velocities __velocities)) _velocities)
  (:efforts
   (&optional __efforts)
   (if __efforts (setq _efforts __efforts)) _efforts)
  (:serialization-length
   ()
   (+
    ;; float64[] _positions
    (* 8    (length _positions)) 4
    ;; float64[] _velocities
    (* 8    (length _velocities)) 4
    ;; float64[] _efforts
    (* 8    (length _efforts)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _positions
     (write-long (length _positions) s)
     (dotimes (i (length _positions))
       (sys::poke (elt _positions i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _velocities
     (write-long (length _velocities) s)
     (dotimes (i (length _velocities))
       (sys::poke (elt _velocities i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _efforts
     (write-long (length _efforts) s)
     (dotimes (i (length _efforts))
       (sys::poke (elt _efforts i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _positions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _positions (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _positions i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _velocities
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocities (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _velocities i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _efforts
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _efforts (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _efforts i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass assignments::CheckJointStateResponse
  :super ros::object
  :slots (_success ))

(defmethod assignments::CheckJointStateResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass assignments::CheckJointState
  :super ros::object
  :slots ())

(setf (get assignments::CheckJointState :md5sum-) "92da1e14e4cd448e7306767001ef56ff")
(setf (get assignments::CheckJointState :datatype-) "assignments/CheckJointState")
(setf (get assignments::CheckJointState :request) assignments::CheckJointStateRequest)
(setf (get assignments::CheckJointState :response) assignments::CheckJointStateResponse)

(defmethod assignments::CheckJointStateRequest
  (:response () (instance assignments::CheckJointStateResponse :init)))

(setf (get assignments::CheckJointStateRequest :md5sum-) "92da1e14e4cd448e7306767001ef56ff")
(setf (get assignments::CheckJointStateRequest :datatype-) "assignments/CheckJointStateRequest")
(setf (get assignments::CheckJointStateRequest :definition-)
      "# Request: a JointState-like structure
float64[] positions
float64[] velocities
float64[] efforts

---
# Response: boolean indicating if the dimensions match
bool success
")

(setf (get assignments::CheckJointStateResponse :md5sum-) "92da1e14e4cd448e7306767001ef56ff")
(setf (get assignments::CheckJointStateResponse :datatype-) "assignments/CheckJointStateResponse")
(setf (get assignments::CheckJointStateResponse :definition-)
      "# Request: a JointState-like structure
float64[] positions
float64[] velocities
float64[] efforts

---
# Response: boolean indicating if the dimensions match
bool success
")



(provide :assignments/CheckJointState "92da1e14e4cd448e7306767001ef56ff")


