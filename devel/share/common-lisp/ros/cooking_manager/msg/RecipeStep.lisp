; Auto-generated. Do not edit!


(cl:in-package cooking_manager-msg)


;//! \htmlinclude RecipeStep.msg.html

(cl:defclass <RecipeStep> (roslisp-msg-protocol:ros-message)
  ((step
    :reader step
    :initarg :step
    :type cl:string
    :initform ""))
)

(cl:defclass RecipeStep (<RecipeStep>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RecipeStep>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RecipeStep)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cooking_manager-msg:<RecipeStep> is deprecated: use cooking_manager-msg:RecipeStep instead.")))

(cl:ensure-generic-function 'step-val :lambda-list '(m))
(cl:defmethod step-val ((m <RecipeStep>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooking_manager-msg:step-val is deprecated.  Use cooking_manager-msg:step instead.")
  (step m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RecipeStep>) ostream)
  "Serializes a message object of type '<RecipeStep>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'step))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'step))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RecipeStep>) istream)
  "Deserializes a message object of type '<RecipeStep>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'step) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'step) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RecipeStep>)))
  "Returns string type for a message object of type '<RecipeStep>"
  "cooking_manager/RecipeStep")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RecipeStep)))
  "Returns string type for a message object of type 'RecipeStep"
  "cooking_manager/RecipeStep")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RecipeStep>)))
  "Returns md5sum for a message object of type '<RecipeStep>"
  "d2ee9423a5e4755eeacf5800eb738c64")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RecipeStep)))
  "Returns md5sum for a message object of type 'RecipeStep"
  "d2ee9423a5e4755eeacf5800eb738c64")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RecipeStep>)))
  "Returns full string definition for message of type '<RecipeStep>"
  (cl:format cl:nil "string step~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RecipeStep)))
  "Returns full string definition for message of type 'RecipeStep"
  (cl:format cl:nil "string step~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RecipeStep>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'step))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RecipeStep>))
  "Converts a ROS message object to a list"
  (cl:list 'RecipeStep
    (cl:cons ':step (step msg))
))
