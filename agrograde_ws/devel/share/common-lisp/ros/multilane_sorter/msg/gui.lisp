; Auto-generated. Do not edit!


(cl:in-package multilane_sorter-msg)


;//! \htmlinclude gui.msg.html

(cl:defclass <gui> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (sensorStatus
    :reader sensorStatus
    :initarg :sensorStatus
    :type cl:string
    :initform "")
   (internetStatus
    :reader internetStatus
    :initarg :internetStatus
    :type cl:string
    :initform "")
   (dataUploadStatus
    :reader dataUploadStatus
    :initarg :dataUploadStatus
    :type cl:string
    :initform "")
   (systemWarning
    :reader systemWarning
    :initarg :systemWarning
    :type cl:string
    :initform "")
   (guiWarning
    :reader guiWarning
    :initarg :guiWarning
    :type cl:string
    :initform "")
   (throughputOverAll
    :reader throughputOverAll
    :initarg :throughputOverAll
    :type cl:string
    :initform "")
   (throughputAccepted
    :reader throughputAccepted
    :initarg :throughputAccepted
    :type cl:string
    :initform "")
   (throughputRejected
    :reader throughputRejected
    :initarg :throughputRejected
    :type cl:string
    :initform "")
   (sortingUptime
    :reader sortingUptime
    :initarg :sortingUptime
    :type cl:string
    :initform "")
   (systemUptime
    :reader systemUptime
    :initarg :systemUptime
    :type cl:string
    :initform "")
   (actuatorStats
    :reader actuatorStats
    :initarg :actuatorStats
    :type cl:string
    :initform "")
   (lane_1
    :reader lane_1
    :initarg :lane_1
    :type sensor_msgs-msg:CompressedImage
    :initform (cl:make-instance 'sensor_msgs-msg:CompressedImage))
   (lane_2
    :reader lane_2
    :initarg :lane_2
    :type sensor_msgs-msg:CompressedImage
    :initform (cl:make-instance 'sensor_msgs-msg:CompressedImage))
   (lane_3
    :reader lane_3
    :initarg :lane_3
    :type sensor_msgs-msg:CompressedImage
    :initform (cl:make-instance 'sensor_msgs-msg:CompressedImage))
   (lane_4
    :reader lane_4
    :initarg :lane_4
    :type sensor_msgs-msg:CompressedImage
    :initform (cl:make-instance 'sensor_msgs-msg:CompressedImage)))
)

(cl:defclass gui (<gui>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gui>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gui)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multilane_sorter-msg:<gui> is deprecated: use multilane_sorter-msg:gui instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <gui>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:header-val is deprecated.  Use multilane_sorter-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'sensorStatus-val :lambda-list '(m))
(cl:defmethod sensorStatus-val ((m <gui>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:sensorStatus-val is deprecated.  Use multilane_sorter-msg:sensorStatus instead.")
  (sensorStatus m))

(cl:ensure-generic-function 'internetStatus-val :lambda-list '(m))
(cl:defmethod internetStatus-val ((m <gui>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:internetStatus-val is deprecated.  Use multilane_sorter-msg:internetStatus instead.")
  (internetStatus m))

(cl:ensure-generic-function 'dataUploadStatus-val :lambda-list '(m))
(cl:defmethod dataUploadStatus-val ((m <gui>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:dataUploadStatus-val is deprecated.  Use multilane_sorter-msg:dataUploadStatus instead.")
  (dataUploadStatus m))

(cl:ensure-generic-function 'systemWarning-val :lambda-list '(m))
(cl:defmethod systemWarning-val ((m <gui>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:systemWarning-val is deprecated.  Use multilane_sorter-msg:systemWarning instead.")
  (systemWarning m))

(cl:ensure-generic-function 'guiWarning-val :lambda-list '(m))
(cl:defmethod guiWarning-val ((m <gui>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:guiWarning-val is deprecated.  Use multilane_sorter-msg:guiWarning instead.")
  (guiWarning m))

(cl:ensure-generic-function 'throughputOverAll-val :lambda-list '(m))
(cl:defmethod throughputOverAll-val ((m <gui>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:throughputOverAll-val is deprecated.  Use multilane_sorter-msg:throughputOverAll instead.")
  (throughputOverAll m))

(cl:ensure-generic-function 'throughputAccepted-val :lambda-list '(m))
(cl:defmethod throughputAccepted-val ((m <gui>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:throughputAccepted-val is deprecated.  Use multilane_sorter-msg:throughputAccepted instead.")
  (throughputAccepted m))

(cl:ensure-generic-function 'throughputRejected-val :lambda-list '(m))
(cl:defmethod throughputRejected-val ((m <gui>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:throughputRejected-val is deprecated.  Use multilane_sorter-msg:throughputRejected instead.")
  (throughputRejected m))

(cl:ensure-generic-function 'sortingUptime-val :lambda-list '(m))
(cl:defmethod sortingUptime-val ((m <gui>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:sortingUptime-val is deprecated.  Use multilane_sorter-msg:sortingUptime instead.")
  (sortingUptime m))

(cl:ensure-generic-function 'systemUptime-val :lambda-list '(m))
(cl:defmethod systemUptime-val ((m <gui>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:systemUptime-val is deprecated.  Use multilane_sorter-msg:systemUptime instead.")
  (systemUptime m))

(cl:ensure-generic-function 'actuatorStats-val :lambda-list '(m))
(cl:defmethod actuatorStats-val ((m <gui>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:actuatorStats-val is deprecated.  Use multilane_sorter-msg:actuatorStats instead.")
  (actuatorStats m))

(cl:ensure-generic-function 'lane_1-val :lambda-list '(m))
(cl:defmethod lane_1-val ((m <gui>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:lane_1-val is deprecated.  Use multilane_sorter-msg:lane_1 instead.")
  (lane_1 m))

(cl:ensure-generic-function 'lane_2-val :lambda-list '(m))
(cl:defmethod lane_2-val ((m <gui>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:lane_2-val is deprecated.  Use multilane_sorter-msg:lane_2 instead.")
  (lane_2 m))

(cl:ensure-generic-function 'lane_3-val :lambda-list '(m))
(cl:defmethod lane_3-val ((m <gui>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:lane_3-val is deprecated.  Use multilane_sorter-msg:lane_3 instead.")
  (lane_3 m))

(cl:ensure-generic-function 'lane_4-val :lambda-list '(m))
(cl:defmethod lane_4-val ((m <gui>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multilane_sorter-msg:lane_4-val is deprecated.  Use multilane_sorter-msg:lane_4 instead.")
  (lane_4 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gui>) ostream)
  "Serializes a message object of type '<gui>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'sensorStatus))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'sensorStatus))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'internetStatus))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'internetStatus))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'dataUploadStatus))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'dataUploadStatus))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'systemWarning))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'systemWarning))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'guiWarning))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'guiWarning))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'throughputOverAll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'throughputOverAll))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'throughputAccepted))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'throughputAccepted))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'throughputRejected))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'throughputRejected))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'sortingUptime))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'sortingUptime))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'systemUptime))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'systemUptime))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'actuatorStats))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'actuatorStats))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'lane_1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'lane_2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'lane_3) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'lane_4) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gui>) istream)
  "Deserializes a message object of type '<gui>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sensorStatus) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'sensorStatus) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'internetStatus) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'internetStatus) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dataUploadStatus) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'dataUploadStatus) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'systemWarning) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'systemWarning) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'guiWarning) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'guiWarning) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'throughputOverAll) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'throughputOverAll) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'throughputAccepted) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'throughputAccepted) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'throughputRejected) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'throughputRejected) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sortingUptime) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'sortingUptime) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'systemUptime) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'systemUptime) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'actuatorStats) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'actuatorStats) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'lane_1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'lane_2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'lane_3) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'lane_4) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gui>)))
  "Returns string type for a message object of type '<gui>"
  "multilane_sorter/gui")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gui)))
  "Returns string type for a message object of type 'gui"
  "multilane_sorter/gui")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gui>)))
  "Returns md5sum for a message object of type '<gui>"
  "65a059bbb76f43602ed8d5fcfa654321")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gui)))
  "Returns md5sum for a message object of type 'gui"
  "65a059bbb76f43602ed8d5fcfa654321")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gui>)))
  "Returns full string definition for message of type '<gui>"
  (cl:format cl:nil "Header header~%string sensorStatus~%string internetStatus~%string dataUploadStatus~%string systemWarning~%string guiWarning~%string throughputOverAll~%string throughputAccepted~%string throughputRejected~%string sortingUptime~%string systemUptime~%string actuatorStats~%sensor_msgs/CompressedImage lane_1~%sensor_msgs/CompressedImage lane_2~%sensor_msgs/CompressedImage lane_3~%sensor_msgs/CompressedImage lane_4~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/CompressedImage~%# This message contains a compressed image~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%~%string format        # Specifies the format of the data~%                     #   Acceptable values:~%                     #     jpeg, png~%uint8[] data         # Compressed image buffer~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gui)))
  "Returns full string definition for message of type 'gui"
  (cl:format cl:nil "Header header~%string sensorStatus~%string internetStatus~%string dataUploadStatus~%string systemWarning~%string guiWarning~%string throughputOverAll~%string throughputAccepted~%string throughputRejected~%string sortingUptime~%string systemUptime~%string actuatorStats~%sensor_msgs/CompressedImage lane_1~%sensor_msgs/CompressedImage lane_2~%sensor_msgs/CompressedImage lane_3~%sensor_msgs/CompressedImage lane_4~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/CompressedImage~%# This message contains a compressed image~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%~%string format        # Specifies the format of the data~%                     #   Acceptable values:~%                     #     jpeg, png~%uint8[] data         # Compressed image buffer~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gui>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'sensorStatus))
     4 (cl:length (cl:slot-value msg 'internetStatus))
     4 (cl:length (cl:slot-value msg 'dataUploadStatus))
     4 (cl:length (cl:slot-value msg 'systemWarning))
     4 (cl:length (cl:slot-value msg 'guiWarning))
     4 (cl:length (cl:slot-value msg 'throughputOverAll))
     4 (cl:length (cl:slot-value msg 'throughputAccepted))
     4 (cl:length (cl:slot-value msg 'throughputRejected))
     4 (cl:length (cl:slot-value msg 'sortingUptime))
     4 (cl:length (cl:slot-value msg 'systemUptime))
     4 (cl:length (cl:slot-value msg 'actuatorStats))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'lane_1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'lane_2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'lane_3))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'lane_4))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gui>))
  "Converts a ROS message object to a list"
  (cl:list 'gui
    (cl:cons ':header (header msg))
    (cl:cons ':sensorStatus (sensorStatus msg))
    (cl:cons ':internetStatus (internetStatus msg))
    (cl:cons ':dataUploadStatus (dataUploadStatus msg))
    (cl:cons ':systemWarning (systemWarning msg))
    (cl:cons ':guiWarning (guiWarning msg))
    (cl:cons ':throughputOverAll (throughputOverAll msg))
    (cl:cons ':throughputAccepted (throughputAccepted msg))
    (cl:cons ':throughputRejected (throughputRejected msg))
    (cl:cons ':sortingUptime (sortingUptime msg))
    (cl:cons ':systemUptime (systemUptime msg))
    (cl:cons ':actuatorStats (actuatorStats msg))
    (cl:cons ':lane_1 (lane_1 msg))
    (cl:cons ':lane_2 (lane_2 msg))
    (cl:cons ':lane_3 (lane_3 msg))
    (cl:cons ':lane_4 (lane_4 msg))
))
