;; Auto-generated. Do not edit!


(when (boundp 'multilane_sorter::defects)
  (if (not (find-package "MULTILANE_SORTER"))
    (make-package "MULTILANE_SORTER"))
  (shadow 'defects (find-package "MULTILANE_SORTER")))
(unless (find-package "MULTILANE_SORTER::DEFECTS")
  (make-package "MULTILANE_SORTER::DEFECTS"))

(in-package "ROS")
;;//! \htmlinclude defects.msg.html


(defclass multilane_sorter::defects
  :super ros::object
  :slots (_normal _bs _peeled _sprouts _rotten _doubles ))

(defmethod multilane_sorter::defects
  (:init
   (&key
    ((:normal __normal) 0.0)
    ((:bs __bs) 0.0)
    ((:peeled __peeled) 0.0)
    ((:sprouts __sprouts) 0.0)
    ((:rotten __rotten) 0.0)
    ((:doubles __doubles) 0.0)
    )
   (send-super :init)
   (setq _normal (float __normal))
   (setq _bs (float __bs))
   (setq _peeled (float __peeled))
   (setq _sprouts (float __sprouts))
   (setq _rotten (float __rotten))
   (setq _doubles (float __doubles))
   self)
  (:normal
   (&optional __normal)
   (if __normal (setq _normal __normal)) _normal)
  (:bs
   (&optional __bs)
   (if __bs (setq _bs __bs)) _bs)
  (:peeled
   (&optional __peeled)
   (if __peeled (setq _peeled __peeled)) _peeled)
  (:sprouts
   (&optional __sprouts)
   (if __sprouts (setq _sprouts __sprouts)) _sprouts)
  (:rotten
   (&optional __rotten)
   (if __rotten (setq _rotten __rotten)) _rotten)
  (:doubles
   (&optional __doubles)
   (if __doubles (setq _doubles __doubles)) _doubles)
  (:serialization-length
   ()
   (+
    ;; float32 _normal
    4
    ;; float32 _bs
    4
    ;; float32 _peeled
    4
    ;; float32 _sprouts
    4
    ;; float32 _rotten
    4
    ;; float32 _doubles
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _normal
       (sys::poke _normal (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bs
       (sys::poke _bs (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _peeled
       (sys::poke _peeled (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _sprouts
       (sys::poke _sprouts (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rotten
       (sys::poke _rotten (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _doubles
       (sys::poke _doubles (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _normal
     (setq _normal (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bs
     (setq _bs (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _peeled
     (setq _peeled (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _sprouts
     (setq _sprouts (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rotten
     (setq _rotten (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _doubles
     (setq _doubles (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get multilane_sorter::defects :md5sum-) "cedfcaa9a7bdfcb45d2788094a026d07")
(setf (get multilane_sorter::defects :datatype-) "multilane_sorter/defects")
(setf (get multilane_sorter::defects :definition-)
      "float32 normal
float32 bs
float32 peeled
float32 sprouts
float32 rotten
float32 doubles
")



(provide :multilane_sorter/defects "cedfcaa9a7bdfcb45d2788094a026d07")


