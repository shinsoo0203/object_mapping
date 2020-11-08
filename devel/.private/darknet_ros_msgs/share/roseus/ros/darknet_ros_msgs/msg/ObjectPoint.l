;; Auto-generated. Do not edit!


(when (boundp 'darknet_ros_msgs::ObjectPoint)
  (if (not (find-package "DARKNET_ROS_MSGS"))
    (make-package "DARKNET_ROS_MSGS"))
  (shadow 'ObjectPoint (find-package "DARKNET_ROS_MSGS")))
(unless (find-package "DARKNET_ROS_MSGS::OBJECTPOINT")
  (make-package "DARKNET_ROS_MSGS::OBJECTPOINT"))

(in-package "ROS")
;;//! \htmlinclude ObjectPoint.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass darknet_ros_msgs::ObjectPoint
  :super ros::object
  :slots (_Class _probability _point _width _height ))

(defmethod darknet_ros_msgs::ObjectPoint
  (:init
   (&key
    ((:Class __Class) "")
    ((:probability __probability) 0.0)
    ((:point __point) (instance geometry_msgs::Point :init))
    ((:width __width) 0)
    ((:height __height) 0)
    )
   (send-super :init)
   (setq _Class (string __Class))
   (setq _probability (float __probability))
   (setq _point __point)
   (setq _width (round __width))
   (setq _height (round __height))
   self)
  (:Class
   (&optional __Class)
   (if __Class (setq _Class __Class)) _Class)
  (:probability
   (&optional __probability)
   (if __probability (setq _probability __probability)) _probability)
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:serialization-length
   ()
   (+
    ;; string _Class
    4 (length _Class)
    ;; float64 _probability
    8
    ;; geometry_msgs/Point _point
    (send _point :serialization-length)
    ;; int8 _width
    1
    ;; int8 _height
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _Class
       (write-long (length _Class) s) (princ _Class s)
     ;; float64 _probability
       (sys::poke _probability (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Point _point
       (send _point :serialize s)
     ;; int8 _width
       (write-byte _width s)
     ;; int8 _height
       (write-byte _height s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _Class
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Class (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _probability
     (setq _probability (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Point _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;; int8 _width
     (setq _width (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _width 127) (setq _width (- _width 256)))
   ;; int8 _height
     (setq _height (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _height 127) (setq _height (- _height 256)))
   ;;
   self)
  )

(setf (get darknet_ros_msgs::ObjectPoint :md5sum-) "9bf661606dbc57bb3d728bb020605f6c")
(setf (get darknet_ros_msgs::ObjectPoint :datatype-) "darknet_ros_msgs/ObjectPoint")
(setf (get darknet_ros_msgs::ObjectPoint :definition-)
      "string Class
float64 probability
geometry_msgs/Point point
int8 width
int8 height


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :darknet_ros_msgs/ObjectPoint "9bf661606dbc57bb3d728bb020605f6c")


