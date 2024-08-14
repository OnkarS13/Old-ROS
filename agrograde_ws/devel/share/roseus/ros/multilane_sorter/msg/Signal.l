;; Auto-generated. Do not edit!


(when (boundp 'multilane_sorter::Signal)
  (if (not (find-package "MULTILANE_SORTER"))
    (make-package "MULTILANE_SORTER"))
  (shadow 'Signal (find-package "MULTILANE_SORTER")))
(unless (find-package "MULTILANE_SORTER::SIGNAL")
  (make-package "MULTILANE_SORTER::SIGNAL"))

(in-package "ROS")
;;//! \htmlinclude Signal.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass multilane_sorter::Signal
  :super ros::object
  :slots (_header _ac_a _ac_b _ac_c _ac_d ))

(defmethod multilane_sorter::Signal
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:ac_a __ac_a) 0)
    ((:ac_b __ac_b) 0)
    ((:ac_c __ac_c) 0)
    ((:ac_d __ac_d) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _ac_a (round __ac_a))
   (setq _ac_b (round __ac_b))
   (setq _ac_c (round __ac_c))
   (setq _ac_d (round __ac_d))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:ac_a
   (&optional __ac_a)
   (if __ac_a (setq _ac_a __ac_a)) _ac_a)
  (:ac_b
   (&optional __ac_b)
   (if __ac_b (setq _ac_b __ac_b)) _ac_b)
  (:ac_c
   (&optional __ac_c)
   (if __ac_c (setq _ac_c __ac_c)) _ac_c)
  (:ac_d
   (&optional __ac_d)
   (if __ac_d (setq _ac_d __ac_d)) _ac_d)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint16 _ac_a
    2
    ;; uint16 _ac_b
    2
    ;; uint16 _ac_c
    2
    ;; uint16 _ac_d
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint16 _ac_a
       (write-word _ac_a s)
     ;; uint16 _ac_b
       (write-word _ac_b s)
     ;; uint16 _ac_c
       (write-word _ac_c s)
     ;; uint16 _ac_d
       (write-word _ac_d s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint16 _ac_a
     (setq _ac_a (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _ac_b
     (setq _ac_b (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _ac_c
     (setq _ac_c (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _ac_d
     (setq _ac_d (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get multilane_sorter::Signal :md5sum-) "bf6d5cfdfe1165ba84740a209c2b5cb1")
(setf (get multilane_sorter::Signal :datatype-) "multilane_sorter/Signal")
(setf (get multilane_sorter::Signal :definition-)
      "Header header
uint16 ac_a
uint16 ac_b
uint16 ac_c
uint16 ac_d
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :multilane_sorter/Signal "bf6d5cfdfe1165ba84740a209c2b5cb1")


