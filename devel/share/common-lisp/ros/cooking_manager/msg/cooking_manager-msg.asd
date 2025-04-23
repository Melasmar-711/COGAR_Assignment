
(cl:in-package :asdf)

(defsystem "cooking_manager-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RecipeStep" :depends-on ("_package_RecipeStep"))
    (:file "_package_RecipeStep" :depends-on ("_package"))
  ))