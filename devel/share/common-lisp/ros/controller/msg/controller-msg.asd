
(cl:in-package :asdf)

(defsystem "controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SystemState" :depends-on ("_package_SystemState"))
    (:file "_package_SystemState" :depends-on ("_package"))
  ))