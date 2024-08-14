;; Auto-generated. Do not edit!


(when (boundp 'multilane_sorter::gui)
  (if (not (find-package "MULTILANE_SORTER"))
    (make-package "MULTILANE_SORTER"))
  (shadow 'gui (find-package "MULTILANE_SORTER")))
(unless (find-package "MULTILANE_SORTER::GUI")
  (make-package "MULTILANE_SORTER::GUI"))

(in-package "ROS")
;;//! \htmlinclude gui.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass multilane_sorter::gui
  :super ros::object
  :slots (_header _sensorStatus _internetStatus _dataUploadStatus _systemWarning _guiWarning _throughputOverAll _throughputAccepted _throughputRejected _sortingUptime _systemUptime _actuatorStats _lane_1 _lane_2 _lane_3 _lane_4 ))

(defmethod multilane_sorter::gui
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:sensorStatus __sensorStatus) "")
    ((:internetStatus __internetStatus) "")
    ((:dataUploadStatus __dataUploadStatus) "")
    ((:systemWarning __systemWarning) "")
    ((:guiWarning __guiWarning) "")
    ((:throughputOverAll __throughputOverAll) "")
    ((:throughputAccepted __throughputAccepted) "")
    ((:throughputRejected __throughputRejected) "")
    ((:sortingUptime __sortingUptime) "")
    ((:systemUptime __systemUptime) "")
    ((:actuatorStats __actuatorStats) "")
    ((:lane_1 __lane_1) (instance sensor_msgs::CompressedImage :init))
    ((:lane_2 __lane_2) (instance sensor_msgs::CompressedImage :init))
    ((:lane_3 __lane_3) (instance sensor_msgs::CompressedImage :init))
    ((:lane_4 __lane_4) (instance sensor_msgs::CompressedImage :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _sensorStatus (string __sensorStatus))
   (setq _internetStatus (string __internetStatus))
   (setq _dataUploadStatus (string __dataUploadStatus))
   (setq _systemWarning (string __systemWarning))
   (setq _guiWarning (string __guiWarning))
   (setq _throughputOverAll (string __throughputOverAll))
   (setq _throughputAccepted (string __throughputAccepted))
   (setq _throughputRejected (string __throughputRejected))
   (setq _sortingUptime (string __sortingUptime))
   (setq _systemUptime (string __systemUptime))
   (setq _actuatorStats (string __actuatorStats))
   (setq _lane_1 __lane_1)
   (setq _lane_2 __lane_2)
   (setq _lane_3 __lane_3)
   (setq _lane_4 __lane_4)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:sensorStatus
   (&optional __sensorStatus)
   (if __sensorStatus (setq _sensorStatus __sensorStatus)) _sensorStatus)
  (:internetStatus
   (&optional __internetStatus)
   (if __internetStatus (setq _internetStatus __internetStatus)) _internetStatus)
  (:dataUploadStatus
   (&optional __dataUploadStatus)
   (if __dataUploadStatus (setq _dataUploadStatus __dataUploadStatus)) _dataUploadStatus)
  (:systemWarning
   (&optional __systemWarning)
   (if __systemWarning (setq _systemWarning __systemWarning)) _systemWarning)
  (:guiWarning
   (&optional __guiWarning)
   (if __guiWarning (setq _guiWarning __guiWarning)) _guiWarning)
  (:throughputOverAll
   (&optional __throughputOverAll)
   (if __throughputOverAll (setq _throughputOverAll __throughputOverAll)) _throughputOverAll)
  (:throughputAccepted
   (&optional __throughputAccepted)
   (if __throughputAccepted (setq _throughputAccepted __throughputAccepted)) _throughputAccepted)
  (:throughputRejected
   (&optional __throughputRejected)
   (if __throughputRejected (setq _throughputRejected __throughputRejected)) _throughputRejected)
  (:sortingUptime
   (&optional __sortingUptime)
   (if __sortingUptime (setq _sortingUptime __sortingUptime)) _sortingUptime)
  (:systemUptime
   (&optional __systemUptime)
   (if __systemUptime (setq _systemUptime __systemUptime)) _systemUptime)
  (:actuatorStats
   (&optional __actuatorStats)
   (if __actuatorStats (setq _actuatorStats __actuatorStats)) _actuatorStats)
  (:lane_1
   (&rest __lane_1)
   (if (keywordp (car __lane_1))
       (send* _lane_1 __lane_1)
     (progn
       (if __lane_1 (setq _lane_1 (car __lane_1)))
       _lane_1)))
  (:lane_2
   (&rest __lane_2)
   (if (keywordp (car __lane_2))
       (send* _lane_2 __lane_2)
     (progn
       (if __lane_2 (setq _lane_2 (car __lane_2)))
       _lane_2)))
  (:lane_3
   (&rest __lane_3)
   (if (keywordp (car __lane_3))
       (send* _lane_3 __lane_3)
     (progn
       (if __lane_3 (setq _lane_3 (car __lane_3)))
       _lane_3)))
  (:lane_4
   (&rest __lane_4)
   (if (keywordp (car __lane_4))
       (send* _lane_4 __lane_4)
     (progn
       (if __lane_4 (setq _lane_4 (car __lane_4)))
       _lane_4)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _sensorStatus
    4 (length _sensorStatus)
    ;; string _internetStatus
    4 (length _internetStatus)
    ;; string _dataUploadStatus
    4 (length _dataUploadStatus)
    ;; string _systemWarning
    4 (length _systemWarning)
    ;; string _guiWarning
    4 (length _guiWarning)
    ;; string _throughputOverAll
    4 (length _throughputOverAll)
    ;; string _throughputAccepted
    4 (length _throughputAccepted)
    ;; string _throughputRejected
    4 (length _throughputRejected)
    ;; string _sortingUptime
    4 (length _sortingUptime)
    ;; string _systemUptime
    4 (length _systemUptime)
    ;; string _actuatorStats
    4 (length _actuatorStats)
    ;; sensor_msgs/CompressedImage _lane_1
    (send _lane_1 :serialization-length)
    ;; sensor_msgs/CompressedImage _lane_2
    (send _lane_2 :serialization-length)
    ;; sensor_msgs/CompressedImage _lane_3
    (send _lane_3 :serialization-length)
    ;; sensor_msgs/CompressedImage _lane_4
    (send _lane_4 :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _sensorStatus
       (write-long (length _sensorStatus) s) (princ _sensorStatus s)
     ;; string _internetStatus
       (write-long (length _internetStatus) s) (princ _internetStatus s)
     ;; string _dataUploadStatus
       (write-long (length _dataUploadStatus) s) (princ _dataUploadStatus s)
     ;; string _systemWarning
       (write-long (length _systemWarning) s) (princ _systemWarning s)
     ;; string _guiWarning
       (write-long (length _guiWarning) s) (princ _guiWarning s)
     ;; string _throughputOverAll
       (write-long (length _throughputOverAll) s) (princ _throughputOverAll s)
     ;; string _throughputAccepted
       (write-long (length _throughputAccepted) s) (princ _throughputAccepted s)
     ;; string _throughputRejected
       (write-long (length _throughputRejected) s) (princ _throughputRejected s)
     ;; string _sortingUptime
       (write-long (length _sortingUptime) s) (princ _sortingUptime s)
     ;; string _systemUptime
       (write-long (length _systemUptime) s) (princ _systemUptime s)
     ;; string _actuatorStats
       (write-long (length _actuatorStats) s) (princ _actuatorStats s)
     ;; sensor_msgs/CompressedImage _lane_1
       (send _lane_1 :serialize s)
     ;; sensor_msgs/CompressedImage _lane_2
       (send _lane_2 :serialize s)
     ;; sensor_msgs/CompressedImage _lane_3
       (send _lane_3 :serialize s)
     ;; sensor_msgs/CompressedImage _lane_4
       (send _lane_4 :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _sensorStatus
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _sensorStatus (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _internetStatus
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _internetStatus (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _dataUploadStatus
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _dataUploadStatus (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _systemWarning
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _systemWarning (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _guiWarning
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _guiWarning (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _throughputOverAll
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _throughputOverAll (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _throughputAccepted
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _throughputAccepted (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _throughputRejected
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _throughputRejected (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _sortingUptime
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _sortingUptime (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _systemUptime
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _systemUptime (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _actuatorStats
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _actuatorStats (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; sensor_msgs/CompressedImage _lane_1
     (send _lane_1 :deserialize buf ptr-) (incf ptr- (send _lane_1 :serialization-length))
   ;; sensor_msgs/CompressedImage _lane_2
     (send _lane_2 :deserialize buf ptr-) (incf ptr- (send _lane_2 :serialization-length))
   ;; sensor_msgs/CompressedImage _lane_3
     (send _lane_3 :deserialize buf ptr-) (incf ptr- (send _lane_3 :serialization-length))
   ;; sensor_msgs/CompressedImage _lane_4
     (send _lane_4 :deserialize buf ptr-) (incf ptr- (send _lane_4 :serialization-length))
   ;;
   self)
  )

(setf (get multilane_sorter::gui :md5sum-) "65a059bbb76f43602ed8d5fcfa654321")
(setf (get multilane_sorter::gui :datatype-) "multilane_sorter/gui")
(setf (get multilane_sorter::gui :definition-)
      "Header header
string sensorStatus
string internetStatus
string dataUploadStatus
string systemWarning
string guiWarning
string throughputOverAll
string throughputAccepted
string throughputRejected
string sortingUptime
string systemUptime
string actuatorStats
sensor_msgs/CompressedImage lane_1
sensor_msgs/CompressedImage lane_2
sensor_msgs/CompressedImage lane_3
sensor_msgs/CompressedImage lane_4

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

================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image

string format        # Specifies the format of the data
                     #   Acceptable values:
                     #     jpeg, png
uint8[] data         # Compressed image buffer

")



(provide :multilane_sorter/gui "65a059bbb76f43602ed8d5fcfa654321")


