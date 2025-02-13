# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from vn_driver/Vectornav.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg

class Vectornav(genpy.Message):
  _md5sum = "edb416495ceedc59ebf9738c869f9207"
  _type = "vn_driver/Vectornav"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """std_msgs/Header header
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
# A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
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
 # A covariance matrix of all zeros will be interpreted as "covariance unknown",
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
                                      # 0 is interpreted as variance unknown"""
  __slots__ = ['header','imu','mag_field','vnymr_read']
  _slot_types = ['std_msgs/Header','sensor_msgs/Imu','sensor_msgs/MagneticField','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,imu,mag_field,vnymr_read

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Vectornav, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.imu is None:
        self.imu = sensor_msgs.msg.Imu()
      if self.mag_field is None:
        self.mag_field = sensor_msgs.msg.MagneticField()
      if self.vnymr_read is None:
        self.vnymr_read = ''
    else:
      self.header = std_msgs.msg.Header()
      self.imu = sensor_msgs.msg.Imu()
      self.mag_field = sensor_msgs.msg.MagneticField()
      self.vnymr_read = ''

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_3I().pack(_x.imu.header.seq, _x.imu.header.stamp.secs, _x.imu.header.stamp.nsecs))
      _x = self.imu.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_4d().pack(_x.imu.orientation.x, _x.imu.orientation.y, _x.imu.orientation.z, _x.imu.orientation.w))
      buff.write(_get_struct_9d().pack(*self.imu.orientation_covariance))
      _x = self
      buff.write(_get_struct_3d().pack(_x.imu.angular_velocity.x, _x.imu.angular_velocity.y, _x.imu.angular_velocity.z))
      buff.write(_get_struct_9d().pack(*self.imu.angular_velocity_covariance))
      _x = self
      buff.write(_get_struct_3d().pack(_x.imu.linear_acceleration.x, _x.imu.linear_acceleration.y, _x.imu.linear_acceleration.z))
      buff.write(_get_struct_9d().pack(*self.imu.linear_acceleration_covariance))
      _x = self
      buff.write(_get_struct_3I().pack(_x.mag_field.header.seq, _x.mag_field.header.stamp.secs, _x.mag_field.header.stamp.nsecs))
      _x = self.mag_field.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_3d().pack(_x.mag_field.magnetic_field.x, _x.mag_field.magnetic_field.y, _x.mag_field.magnetic_field.z))
      buff.write(_get_struct_9d().pack(*self.mag_field.magnetic_field_covariance))
      _x = self.vnymr_read
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.imu is None:
        self.imu = sensor_msgs.msg.Imu()
      if self.mag_field is None:
        self.mag_field = sensor_msgs.msg.MagneticField()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.imu.header.seq, _x.imu.header.stamp.secs, _x.imu.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.imu.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.imu.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 32
      (_x.imu.orientation.x, _x.imu.orientation.y, _x.imu.orientation.z, _x.imu.orientation.w,) = _get_struct_4d().unpack(str[start:end])
      start = end
      end += 72
      self.imu.orientation_covariance = _get_struct_9d().unpack(str[start:end])
      _x = self
      start = end
      end += 24
      (_x.imu.angular_velocity.x, _x.imu.angular_velocity.y, _x.imu.angular_velocity.z,) = _get_struct_3d().unpack(str[start:end])
      start = end
      end += 72
      self.imu.angular_velocity_covariance = _get_struct_9d().unpack(str[start:end])
      _x = self
      start = end
      end += 24
      (_x.imu.linear_acceleration.x, _x.imu.linear_acceleration.y, _x.imu.linear_acceleration.z,) = _get_struct_3d().unpack(str[start:end])
      start = end
      end += 72
      self.imu.linear_acceleration_covariance = _get_struct_9d().unpack(str[start:end])
      _x = self
      start = end
      end += 12
      (_x.mag_field.header.seq, _x.mag_field.header.stamp.secs, _x.mag_field.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.mag_field.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.mag_field.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 24
      (_x.mag_field.magnetic_field.x, _x.mag_field.magnetic_field.y, _x.mag_field.magnetic_field.z,) = _get_struct_3d().unpack(str[start:end])
      start = end
      end += 72
      self.mag_field.magnetic_field_covariance = _get_struct_9d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.vnymr_read = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.vnymr_read = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_3I().pack(_x.imu.header.seq, _x.imu.header.stamp.secs, _x.imu.header.stamp.nsecs))
      _x = self.imu.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_4d().pack(_x.imu.orientation.x, _x.imu.orientation.y, _x.imu.orientation.z, _x.imu.orientation.w))
      buff.write(self.imu.orientation_covariance.tostring())
      _x = self
      buff.write(_get_struct_3d().pack(_x.imu.angular_velocity.x, _x.imu.angular_velocity.y, _x.imu.angular_velocity.z))
      buff.write(self.imu.angular_velocity_covariance.tostring())
      _x = self
      buff.write(_get_struct_3d().pack(_x.imu.linear_acceleration.x, _x.imu.linear_acceleration.y, _x.imu.linear_acceleration.z))
      buff.write(self.imu.linear_acceleration_covariance.tostring())
      _x = self
      buff.write(_get_struct_3I().pack(_x.mag_field.header.seq, _x.mag_field.header.stamp.secs, _x.mag_field.header.stamp.nsecs))
      _x = self.mag_field.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_3d().pack(_x.mag_field.magnetic_field.x, _x.mag_field.magnetic_field.y, _x.mag_field.magnetic_field.z))
      buff.write(self.mag_field.magnetic_field_covariance.tostring())
      _x = self.vnymr_read
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.imu is None:
        self.imu = sensor_msgs.msg.Imu()
      if self.mag_field is None:
        self.mag_field = sensor_msgs.msg.MagneticField()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.imu.header.seq, _x.imu.header.stamp.secs, _x.imu.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.imu.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.imu.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 32
      (_x.imu.orientation.x, _x.imu.orientation.y, _x.imu.orientation.z, _x.imu.orientation.w,) = _get_struct_4d().unpack(str[start:end])
      start = end
      end += 72
      self.imu.orientation_covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=9)
      _x = self
      start = end
      end += 24
      (_x.imu.angular_velocity.x, _x.imu.angular_velocity.y, _x.imu.angular_velocity.z,) = _get_struct_3d().unpack(str[start:end])
      start = end
      end += 72
      self.imu.angular_velocity_covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=9)
      _x = self
      start = end
      end += 24
      (_x.imu.linear_acceleration.x, _x.imu.linear_acceleration.y, _x.imu.linear_acceleration.z,) = _get_struct_3d().unpack(str[start:end])
      start = end
      end += 72
      self.imu.linear_acceleration_covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=9)
      _x = self
      start = end
      end += 12
      (_x.mag_field.header.seq, _x.mag_field.header.stamp.secs, _x.mag_field.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.mag_field.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.mag_field.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 24
      (_x.mag_field.magnetic_field.x, _x.mag_field.magnetic_field.y, _x.mag_field.magnetic_field.z,) = _get_struct_3d().unpack(str[start:end])
      start = end
      end += 72
      self.mag_field.magnetic_field_covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=9)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.vnymr_read = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.vnymr_read = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
_struct_4d = None
def _get_struct_4d():
    global _struct_4d
    if _struct_4d is None:
        _struct_4d = struct.Struct("<4d")
    return _struct_4d
_struct_9d = None
def _get_struct_9d():
    global _struct_9d
    if _struct_9d is None:
        _struct_9d = struct.Struct("<9d")
    return _struct_9d
