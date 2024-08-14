
(cl:in-package :asdf)

(defsystem "multilane_sorter-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Signal" :depends-on ("_package_Signal"))
    (:file "_package_Signal" :depends-on ("_package"))
    (:file "defects" :depends-on ("_package_defects"))
    (:file "_package_defects" :depends-on ("_package"))
    (:file "gui" :depends-on ("_package_gui"))
    (:file "_package_gui" :depends-on ("_package"))
    (:file "inference" :depends-on ("_package_inference"))
    (:file "_package_inference" :depends-on ("_package"))
  ))