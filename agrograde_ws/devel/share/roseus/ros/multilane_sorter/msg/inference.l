;; Auto-generated. Do not edit!


(when (boundp 'multilane_sorter::inference)
  (if (not (find-package "MULTILANE_SORTER"))
    (make-package "MULTILANE_SORTER"))
  (shadow 'inference (find-package "MULTILANE_SORTER")))
(unless (find-package "MULTILANE_SORTER::INFERENCE")
  (make-package "MULTILANE_SORTER::INFERENCE"))

(in-package "ROS")
;;//! \htmlinclude inference.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass multilane_sorter::inference
  :super ros::object
  :slots (_header _sprout _peeled _rotten _blacksmut _double _size _user1 _user2 _user3 _user4 _user5 ))

(defmethod multilane_sorter::inference
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:sprout __sprout) 0.0)
    ((:peeled __peeled) 0.0)
    ((:rotten __rotten) 0.0)
    ((:blacksmut __blacksmut) 0.0)
    ((:double __double) 0.0)
    ((:size __size) 0.0)
    ((:user1 __user1) 0.0)
    ((:user2 __user2) 0.0)
    ((:user3 __user3) 0.0)
    ((:user4 __user4) 0.0)
    ((:user5 __user5) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _sprout (float __sprout))
   (setq _peeled (float __peeled))
   (setq _rotten (float __rotten))
   (setq _blacksmut (float __blacksmut))
   (setq _double (float __double))
   (setq _size (float __size))
   (setq _user1 (float __user1))
   (setq _user2 (float __user2))
   (setq _user3 (float __user3))
   (setq _user4 (float __user4))
   (setq _user5 (float __user5))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:sprout
   (&optional __sprout)
   (if __sprout (setq _sprout __sprout)) _sprout)
  (:peeled
   (&optional __peeled)
   (if __peeled (setq _peeled __peeled)) _peeled)
  (:rotten
   (&optional __rotten)
   (if __rotten (setq _rotten __rotten)) _rotten)
  (:blacksmut
   (&optional __blacksmut)
   (if __blacksmut (setq _blacksmut __blacksmut)) _blacksmut)
  (:double
   (&optional __double)
   (if __double (setq _double __double)) _double)
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:user1
   (&optional __user1)
   (if __user1 (setq _user1 __user1)) _user1)
  (:user2
   (&optional __user2)
   (if __user2 (setq _user2 __user2)) _user2)
  (:user3
   (&optional __user3)
   (if __user3 (setq _user3 __user3)) _user3)
  (:user4
   (&optional __user4)
   (if __user4 (setq _user4 __user4)) _user4)
  (:user5
   (&optional __user5)
   (if __user5 (setq _user5 __user5)) _user5)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _sprout
    4
    ;; float32 _peeled
    4
    ;; float32 _rotten
    4
    ;; float32 _blacksmut
    4
    ;; float32 _double
    4
    ;; float32 _size
    4
    ;; float32 _user1
    4
    ;; float32 _user2
    4
    ;; float32 _user3
    4
    ;; float32 _user4
    4
    ;; float32 _user5
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _sprout
       (sys::poke _sprout (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _peeled
       (sys::poke _peeled (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rotten
       (sys::poke _rotten (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _blacksmut
       (sys::poke _blacksmut (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _double
       (sys::poke _double (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _size
       (sys::poke _size (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _user1
       (sys::poke _user1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _user2
       (sys::poke _user2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _user3
       (sys::poke _user3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _user4
       (sys::poke _user4 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _user5
       (sys::poke _user5 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _sprout
     (setq _sprout (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _peeled
     (setq _peeled (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rotten
     (setq _rotten (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _blacksmut
     (setq _blacksmut (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _double
     (setq _double (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _size
     (setq _size (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _user1
     (setq _user1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _user2
     (setq _user2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _user3
     (setq _user3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _user4
     (setq _user4 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _user5
     (setq _user5 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get multilane_sorter::inference :md5sum-) "aa06a43907df191d69024618ed6dd509")
(setf (get multilane_sorter::inference :datatype-) "multilane_sorter/inference")
(setf (get multilane_sorter::inference :definition-)
      "
Header header
float32 sprout
float32 peeled
float32 rotten
float32 blacksmut
float32 double

float32 size
float32 user1
float32 user2
float32 user3
float32 user4
float32 user5

# Header header
# float32 prediction
# float32 onion
# float32 blacksmut
# float32 peel
# float32 size


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



(provide :multilane_sorter/inference "aa06a43907df191d69024618ed6dd509")


