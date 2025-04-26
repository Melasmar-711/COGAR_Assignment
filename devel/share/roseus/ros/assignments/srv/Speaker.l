;; Auto-generated. Do not edit!


(when (boundp 'assignments::Speaker)
  (if (not (find-package "ASSIGNMENTS"))
    (make-package "ASSIGNMENTS"))
  (shadow 'Speaker (find-package "ASSIGNMENTS")))
(unless (find-package "ASSIGNMENTS::SPEAKER")
  (make-package "ASSIGNMENTS::SPEAKER"))
(unless (find-package "ASSIGNMENTS::SPEAKERREQUEST")
  (make-package "ASSIGNMENTS::SPEAKERREQUEST"))
(unless (find-package "ASSIGNMENTS::SPEAKERRESPONSE")
  (make-package "ASSIGNMENTS::SPEAKERRESPONSE"))

(in-package "ROS")





(defclass assignments::SpeakerRequest
  :super ros::object
  :slots (_message ))

(defmethod assignments::SpeakerRequest
  (:init
   (&key
    ((:message __message) "")
    )
   (send-super :init)
   (setq _message (string __message))
   self)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass assignments::SpeakerResponse
  :super ros::object
  :slots (_success ))

(defmethod assignments::SpeakerResponse
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

(defclass assignments::Speaker
  :super ros::object
  :slots ())

(setf (get assignments::Speaker :md5sum-) "8b7095eb8dcd517ba7c37a0a06dcc50b")
(setf (get assignments::Speaker :datatype-) "assignments/Speaker")
(setf (get assignments::Speaker :request) assignments::SpeakerRequest)
(setf (get assignments::Speaker :response) assignments::SpeakerResponse)

(defmethod assignments::SpeakerRequest
  (:response () (instance assignments::SpeakerResponse :init)))

(setf (get assignments::SpeakerRequest :md5sum-) "8b7095eb8dcd517ba7c37a0a06dcc50b")
(setf (get assignments::SpeakerRequest :datatype-) "assignments/SpeakerRequest")
(setf (get assignments::SpeakerRequest :definition-)
      "# String.srv
string message

---
bool success
")

(setf (get assignments::SpeakerResponse :md5sum-) "8b7095eb8dcd517ba7c37a0a06dcc50b")
(setf (get assignments::SpeakerResponse :datatype-) "assignments/SpeakerResponse")
(setf (get assignments::SpeakerResponse :definition-)
      "# String.srv
string message

---
bool success
")



(provide :assignments/Speaker "8b7095eb8dcd517ba7c37a0a06dcc50b")


