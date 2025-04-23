;; Auto-generated. Do not edit!


(when (boundp 'cooking_manager::RecipeStep)
  (if (not (find-package "COOKING_MANAGER"))
    (make-package "COOKING_MANAGER"))
  (shadow 'RecipeStep (find-package "COOKING_MANAGER")))
(unless (find-package "COOKING_MANAGER::RECIPESTEP")
  (make-package "COOKING_MANAGER::RECIPESTEP"))

(in-package "ROS")
;;//! \htmlinclude RecipeStep.msg.html


(defclass cooking_manager::RecipeStep
  :super ros::object
  :slots (_step ))

(defmethod cooking_manager::RecipeStep
  (:init
   (&key
    ((:step __step) "")
    )
   (send-super :init)
   (setq _step (string __step))
   self)
  (:step
   (&optional __step)
   (if __step (setq _step __step)) _step)
  (:serialization-length
   ()
   (+
    ;; string _step
    4 (length _step)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _step
       (write-long (length _step) s) (princ _step s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _step
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _step (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get cooking_manager::RecipeStep :md5sum-) "d2ee9423a5e4755eeacf5800eb738c64")
(setf (get cooking_manager::RecipeStep :datatype-) "cooking_manager/RecipeStep")
(setf (get cooking_manager::RecipeStep :definition-)
      "string step
")



(provide :cooking_manager/RecipeStep "d2ee9423a5e4755eeacf5800eb738c64")


