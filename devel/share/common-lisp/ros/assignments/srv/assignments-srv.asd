
(cl:in-package :asdf)

(defsystem "assignments-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CheckJointState" :depends-on ("_package_CheckJointState"))
    (:file "_package_CheckJointState" :depends-on ("_package"))
    (:file "Speaker" :depends-on ("_package_Speaker"))
    (:file "_package_Speaker" :depends-on ("_package"))
  ))