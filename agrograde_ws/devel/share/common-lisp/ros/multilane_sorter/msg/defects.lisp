; Auto-generated. Do not edit!


(cl:in-package multilane_sorter-msg)


;//! \htmlinclude defects.msg.html

(cl:defclass <defects> (roslisp-msg-protocol:ros-message)
  ((normal
    :reader normal
    :initarg :normal
    :type cl:float
    :initform 0.0)
   (bs
    :reader bs
    :initarg :bs
    :type cl:float
    :initform 0.0)
   (peeled
    :reader peeled
    :initarg :peeled
    :type cl:float
    :initform 0.0)
   (sprouts
    :reader sprouts
    :initarg :sprouts
    :type cl:float
    :initform 0.0)
   (rotten
    :reader rotten
    :initarg :rotten
    :type cl:float
    :initform 0.0)
   (doubles
    :reader doubles
    :initarg :doubles
    :type cl:float
    :initform 0.0))
)

(cl:defclass defects (<defects>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <defects>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'defects)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multilane_sorter-msg:<defects> is deprecated: use multilane_sorter-msg:defects instead.")))

(cl:ensure-generic-function 'normal-val :lambda-list '(m))
(cl:defmethod normal-val ((m <defects>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:normal-val is deprecated.  Use multilane_sorter-msg:normal instead.")
  (normal m))

(cl:ensure-generic-function 'bs-val :lambda-list '(m))
(cl:defmethod bs-val ((m <defects>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:bs-val is deprecated.  Use multilane_sorter-msg:bs instead.")
  (bs m))

(cl:ensure-generic-function 'peeled-val :lambda-list '(m))
(cl:defmethod peeled-val ((m <defects>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:peeled-val is deprecated.  Use multilane_sorter-msg:peeled instead.")
  (peeled m))

(cl:ensure-generic-function 'sprouts-val :lambda-list '(m))
(cl:defmethod sprouts-val ((m <defects>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:sprouts-val is deprecated.  Use multilane_sorter-msg:sprouts instead.")
  (sprouts m))

(cl:ensure-generic-function 'rotten-val :lambda-list '(m))
(cl:defmethod rotten-val ((m <defects>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:rotten-val is deprecated.  Use multilane_sorter-msg:rotten instead.")
  (rotten m))

(cl:ensure-generic-function 'doubles-val :lambda-list '(m))
(cl:defmethod doubles-val ((m <defects>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:doubles-val is deprecated.  Use multilane_sorter-msg:doubles instead.")
  (doubles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <defects>) ostream)
  "Serializes a message object of type '<defects>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'normal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'peeled))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'sprouts))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rotten))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'doubles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <defects>) istream)
  "Deserializes a message object of type '<defects>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'normal) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bs) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'sprouts) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'doubles) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<defects>)))
  "Returns string type for a message object of type '<defects>"
  "multilane_sorter/defects")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'defects)))
  "Returns string type for a message object of type 'defects"
  "multilane_sorter/defects")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<defects>)))
  "Returns md5sum for a message object of type '<defects>"
  "cedfcaa9a7bdfcb45d2788094a026d07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'defects)))
  "Returns md5sum for a message object of type 'defects"
  "cedfcaa9a7bdfcb45d2788094a026d07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<defects>)))
  "Returns full string definition for message of type '<defects>"
  (cl:format cl:nil "float32 normal~%float32 bs~%float32 peeled~%float32 sprouts~%float32 rotten~%float32 doubles~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'defects)))
  "Returns full string definition for message of type 'defects"
  (cl:format cl:nil "float32 normal~%float32 bs~%float32 peeled~%float32 sprouts~%float32 rotten~%float32 doubles~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <defects>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <defects>))
  "Converts a ROS message object to a list"
  (cl:list 'defects
    (cl:cons ':normal (normal msg))
    (cl:cons ':bs (bs msg))
    (cl:cons ':peeled (peeled msg))
    (cl:cons ':sprouts (sprouts msg))
    (cl:cons ':rotten (rotten msg))
    (cl:cons ':doubles (doubles msg))
))
