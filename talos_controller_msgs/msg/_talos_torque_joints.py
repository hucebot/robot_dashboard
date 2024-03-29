# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from talos_controller_msgs/talos_torque_joints.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class talos_torque_joints(genpy.Message):
  _md5sum = "2d80b2db8d9c653ea66130da86677968"
  _type = "talos_controller_msgs/talos_torque_joints"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """float64[2] torso
float64[4] arm_left
float64[4] arm_right
float64[6] leg_left
float64[6] leg_right
"""
  __slots__ = ['torso','arm_left','arm_right','leg_left','leg_right']
  _slot_types = ['float64[2]','float64[4]','float64[4]','float64[6]','float64[6]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       torso,arm_left,arm_right,leg_left,leg_right

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(talos_torque_joints, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.torso is None:
        self.torso = [0.] * 2
      if self.arm_left is None:
        self.arm_left = [0.] * 4
      if self.arm_right is None:
        self.arm_right = [0.] * 4
      if self.leg_left is None:
        self.leg_left = [0.] * 6
      if self.leg_right is None:
        self.leg_right = [0.] * 6
    else:
      self.torso = [0.] * 2
      self.arm_left = [0.] * 4
      self.arm_right = [0.] * 4
      self.leg_left = [0.] * 6
      self.leg_right = [0.] * 6

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
      buff.write(_get_struct_2d().pack(*self.torso))
      buff.write(_get_struct_4d().pack(*self.arm_left))
      buff.write(_get_struct_4d().pack(*self.arm_right))
      buff.write(_get_struct_6d().pack(*self.leg_left))
      buff.write(_get_struct_6d().pack(*self.leg_right))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 16
      self.torso = _get_struct_2d().unpack(str[start:end])
      start = end
      end += 32
      self.arm_left = _get_struct_4d().unpack(str[start:end])
      start = end
      end += 32
      self.arm_right = _get_struct_4d().unpack(str[start:end])
      start = end
      end += 48
      self.leg_left = _get_struct_6d().unpack(str[start:end])
      start = end
      end += 48
      self.leg_right = _get_struct_6d().unpack(str[start:end])
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
      buff.write(self.torso.tostring())
      buff.write(self.arm_left.tostring())
      buff.write(self.arm_right.tostring())
      buff.write(self.leg_left.tostring())
      buff.write(self.leg_right.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 16
      self.torso = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=2)
      start = end
      end += 32
      self.arm_left = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=4)
      start = end
      end += 32
      self.arm_right = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=4)
      start = end
      end += 48
      self.leg_left = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=6)
      start = end
      end += 48
      self.leg_right = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=6)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2d = None
def _get_struct_2d():
    global _struct_2d
    if _struct_2d is None:
        _struct_2d = struct.Struct("<2d")
    return _struct_2d
_struct_4d = None
def _get_struct_4d():
    global _struct_4d
    if _struct_4d is None:
        _struct_4d = struct.Struct("<4d")
    return _struct_4d
_struct_6d = None
def _get_struct_6d():
    global _struct_6d
    if _struct_6d is None:
        _struct_6d = struct.Struct("<6d")
    return _struct_6d
