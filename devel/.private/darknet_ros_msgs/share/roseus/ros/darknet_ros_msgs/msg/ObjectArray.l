;; Auto-generated. Do not edit!


(when (boundp 'darknet_ros_msgs::ObjectArray)
  (if (not (find-package "DARKNET_ROS_MSGS"))
    (make-package "DARKNET_ROS_MSGS"))
  (shadow 'ObjectArray (find-package "DARKNET_ROS_MSGS")))
(unless (find-package "DARKNET_ROS_MSGS::OBJECTARRAY")
  (make-package "DARKNET_ROS_MSGS::OBJECTARRAY"))

(in-package "ROS")
;;//! \htmlinclude ObjectArray.msg.html


(defclass darknet_ros_msgs::ObjectArray
  :super ros::object
  :slots (_objects ))

(defmethod darknet_ros_msgs::ObjectArray
  (:init
   (&key
    ((:objects __objects) (let (r) (dotimes (i 0) (push (instance darknet_ros_msgs::ObjectPoint :init) r)) r))
    )
   (send-super :init)
   (setq _objects __objects)
   self)
  (:objects
   (&rest __objects)
   (if (keywordp (car __objects))
       (send* _objects __objects)
     (progn
       (if __objects (setq _objects (car __objects)))
       _objects)))
  (:serialization-length
   ()
   (+
    ;; darknet_ros_msgs/ObjectPoint[] _objects
    (apply #'+ (send-all _objects :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; darknet_ros_msgs/ObjectPoint[] _objects
     (write-long (length _objects) s)
     (dolist (elem _objects)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; darknet_ros_msgs/ObjectPoint[] _objects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _objects (let (r) (dotimes (i n) (push (instance darknet_ros_msgs::ObjectPoint :init) r)) r))
     (dolist (elem- _objects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get darknet_ros_msgs::ObjectArray :md5sum-) "d6f2aabe531b975e415559a57a1142fd")
(setf (get darknet_ros_msgs::ObjectArray :datatype-) "darknet_ros_msgs/ObjectArray")
(setf (get darknet_ros_msgs::ObjectArray :definition-)
      "darknet_ros_msgs/ObjectPoint[] objects

================================================================================
MSG: darknet_ros_msgs/ObjectPoint
string Class
float64 probability
geometry_msgs/Point point
int8 width
int8 height
float64 distance

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :darknet_ros_msgs/ObjectArray "d6f2aabe531b975e415559a57a1142fd")


