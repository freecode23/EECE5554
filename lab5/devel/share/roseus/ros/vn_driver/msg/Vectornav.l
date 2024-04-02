;; Auto-generated. Do not edit!


(when (boundp 'vn_driver::Vectornav)
  (if (not (find-package "VN_DRIVER"))
    (make-package "VN_DRIVER"))
  (shadow 'Vectornav (find-package "VN_DRIVER")))
(unless (find-package "VN_DRIVER::VECTORNAV")
  (make-package "VN_DRIVER::VECTORNAV"))

(in-package "ROS")
;;//! \htmlinclude Vectornav.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass vn_driver::Vectornav
  :super ros::object
  :slots (_header _imu _mag_field _vnymr_read ))

(defmethod vn_driver::Vectornav
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:imu __imu) (instance sensor_msgs::Imu :init))
    ((:mag_field __mag_field) (instance sensor_msgs::MagneticField :init))
    ((:vnymr_read __vnymr_read) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _imu __imu)
   (setq _mag_field __mag_field)
   (setq _vnymr_read (string __vnymr_read))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:imu
   (&rest __imu)
   (if (keywordp (car __imu))
       (send* _imu __imu)
     (progn
       (if __imu (setq _imu (car __imu)))
       _imu)))
  (:mag_field
   (&rest __mag_field)
   (if (keywordp (car __mag_field))
       (send* _mag_field __mag_field)
     (progn
       (if __mag_field (setq _mag_field (car __mag_field)))
       _mag_field)))
  (:vnymr_read
   (&optional __vnymr_read)
   (if __vnymr_read (setq _vnymr_read __vnymr_read)) _vnymr_read)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; sensor_msgs/Imu _imu
    (send _imu :serialization-length)
    ;; sensor_msgs/MagneticField _mag_field
    (send _mag_field :serialization-length)
    ;; string _vnymr_read
    4 (length _vnymr_read)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; sensor_msgs/Imu _imu
       (send _imu :serialize s)
     ;; sensor_msgs/MagneticField _mag_field
       (send _mag_field :serialize s)
     ;; string _vnymr_read
       (write-long (length _vnymr_read) s) (princ _vnymr_read s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; sensor_msgs/Imu _imu
     (send _imu :deserialize buf ptr-) (incf ptr- (send _imu :serialization-length))
   ;; sensor_msgs/MagneticField _mag_field
     (send _mag_field :deserialize buf ptr-) (incf ptr- (send _mag_field :serialization-length))
   ;; string _vnymr_read
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _vnymr_read (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get vn_driver::Vectornav :md5sum-) "edb416495ceedc59ebf9738c869f9207")
(setf (get vn_driver::Vectornav :datatype-) "vn_driver/Vectornav")
(setf (get vn_driver::Vectornav :definition-)
      "std_msgs/Header header
sensor_msgs/Imu imu
sensor_msgs/MagneticField mag_field
string vnymr_read

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
MSG: sensor_msgs/Imu
# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the 
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
# estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each 
# covariance matrix, and disregard the associated estimate.

Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z 

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/MagneticField
 # Measurement of the Magnetic Field vector at a specific location.

 # If the covariance of the measurement is known, it should be filled in
 # (if all you know is the variance of each measurement, e.g. from the datasheet,
 #just put those along the diagonal)
 # A covariance matrix of all zeros will be interpreted as \"covariance unknown\",
 # and to use the data a covariance will have to be assumed or gotten from some
 # other source


 Header header                        # timestamp is the time the
                                      # field was measured
                                      # frame_id is the location and orientation
                                      # of the field measurement

 geometry_msgs/Vector3 magnetic_field # x, y, and z components of the
                                      # field vector in Tesla
                                      # If your sensor does not output 3 axes,
                                      # put NaNs in the components not reported.

 float64[9] magnetic_field_covariance # Row major about x, y, z axes
                                      # 0 is interpreted as variance unknown
")



(provide :vn_driver/Vectornav "edb416495ceedc59ebf9738c869f9207")


