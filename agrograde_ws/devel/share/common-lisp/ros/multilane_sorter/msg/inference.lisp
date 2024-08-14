; Auto-generated. Do not edit!


(cl:in-package multilane_sorter-msg)


;//! \htmlinclude inference.msg.html

(cl:defclass <inference> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (sprout
    :reader sprout
    :initarg :sprout
    :type cl:float
    :initform 0.0)
   (peeled
    :reader peeled
    :initarg :peeled
    :type cl:float
    :initform 0.0)
   (rotten
    :reader rotten
    :initarg :rotten
    :type cl:float
    :initform 0.0)
   (blacksmut
    :reader blacksmut
    :initarg :blacksmut
    :type cl:float
    :initform 0.0)
   (double
    :reader double
    :initarg :double
    :type cl:float
    :initform 0.0)
   (size
    :reader size
    :initarg :size
    :type cl:float
    :initform 0.0)
   (user1
    :reader user1
    :initarg :user1
    :type cl:float
    :initform 0.0)
   (user2
    :reader user2
    :initarg :user2
    :type cl:float
    :initform 0.0)
   (user3
    :reader user3
    :initarg :user3
    :type cl:float
    :initform 0.0)
   (user4
    :reader user4
    :initarg :user4
    :type cl:float
    :initform 0.0)
   (user5
    :reader user5
    :initarg :user5
    :type cl:float
    :initform 0.0))
)

(cl:defclass inference (<inference>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <inference>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'inference)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multilane_sorter-msg:<inference> is deprecated: use multilane_sorter-msg:inference instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <inference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:header-val is deprecated.  Use multilane_sorter-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'sprout-val :lambda-list '(m))
(cl:defmethod sprout-val ((m <inference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:sprout-val is deprecated.  Use multilane_sorter-msg:sprout instead.")
  (sprout m))

(cl:ensure-generic-function 'peeled-val :lambda-list '(m))
(cl:defmethod peeled-val ((m <inference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:peeled-val is deprecated.  Use multilane_sorter-msg:peeled instead.")
  (peeled m))

(cl:ensure-generic-function 'rotten-val :lambda-list '(m))
(cl:defmethod rotten-val ((m <inference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:rotten-val is deprecated.  Use multilane_sorter-msg:rotten instead.")
  (rotten m))

(cl:ensure-generic-function 'blacksmut-val :lambda-list '(m))
(cl:defmethod blacksmut-val ((m <inference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:blacksmut-val is deprecated.  Use multilane_sorter-msg:blacksmut instead.")
  (blacksmut m))

(cl:ensure-generic-function 'double-val :lambda-list '(m))
(cl:defmethod double-val ((m <inference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:double-val is deprecated.  Use multilane_sorter-msg:double instead.")
  (double m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <inference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:size-val is deprecated.  Use multilane_sorter-msg:size instead.")
  (size m))

(cl:ensure-generic-function 'user1-val :lambda-list '(m))
(cl:defmethod user1-val ((m <inference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:user1-val is deprecated.  Use multilane_sorter-msg:user1 instead.")
  (user1 m))

(cl:ensure-generic-function 'user2-val :lambda-list '(m))
(cl:defmethod user2-val ((m <inference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:user2-val is deprecated.  Use multilane_sorter-msg:user2 instead.")
  (user2 m))

(cl:ensure-generic-function 'user3-val :lambda-list '(m))
(cl:defmethod user3-val ((m <inference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:user3-val is deprecated.  Use multilane_sorter-msg:user3 instead.")
  (user3 m))

(cl:ensure-generic-function 'user4-val :lambda-list '(m))
(cl:defmethod user4-val ((m <inference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:user4-val is deprecated.  Use multilane_sorter-msg:user4 instead.")
  (user4 m))

(cl:ensure-generic-function 'user5-val :lambda-list '(m))
(cl:defmethod user5-val ((m <inference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:user5-val is deprecated.  Use multilane_sorter-msg:user5 instead.")
  (user5 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <inference>) ostream)
  "Serializes a message object of type '<inference>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'sprout))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'peeled))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rotten))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'blacksmut))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'double))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'size))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'user1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'user2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'user3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'user4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'user5))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <inference>) istream)
  "Deserializes a message object of type '<inference>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sprout) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'peeled) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rotten) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'blacksmut) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'double) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'size) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'user1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'user2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'user3) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'user4) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'user5) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<inference>)))
  "Returns string type for a message object of type '<inference>"
  "multilane_sorter/inference")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'inference)))
  "Returns string type for a message object of type 'inference"
  "multilane_sorter/inference")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<inference>)))
  "Returns md5sum for a message object of type '<inference>"
  "aa06a43907df191d69024618ed6dd509")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'inference)))
  "Returns md5sum for a message object of type 'inference"
  "aa06a43907df191d69024618ed6dd509")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<inference>)))
  "Returns full string definition for message of type '<inference>"
  (cl:format cl:nil "~%Header header~%float32 sprout~%float32 peeled~%float32 rotten~%float32 blacksmut~%float32 double~%~%float32 size~%float32 user1~%float32 user2~%float32 user3~%float32 user4~%float32 user5~%~%# Header header~%# float32 prediction~%# float32 onion~%# float32 blacksmut~%# float32 peel~%# float32 size~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'inference)))
  "Returns full string definition for message of type 'inference"
  (cl:format cl:nil "~%Header header~%float32 sprout~%float32 peeled~%float32 rotten~%float32 blacksmut~%float32 double~%~%float32 size~%float32 user1~%float32 user2~%float32 user3~%float32 user4~%float32 user5~%~%# Header header~%# float32 prediction~%# float32 onion~%# float32 blacksmut~%# float32 peel~%# float32 size~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <inference>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <inference>))
  "Converts a ROS message object to a list"
  (cl:list 'inference
    (cl:cons ':header (header msg))
    (cl:cons ':sprout (sprout msg))
    (cl:cons ':peeled (peeled msg))
    (cl:cons ':rotten (rotten msg))
    (cl:cons ':blacksmut (blacksmut msg))
    (cl:cons ':double (double msg))
    (cl:cons ':size (size msg))
    (cl:cons ':user1 (user1 msg))
    (cl:cons ':user2 (user2 msg))
    (cl:cons ':user3 (user3 msg))
    (cl:cons ':user4 (user4 msg))
    (cl:cons ':user5 (user5 msg))
))
