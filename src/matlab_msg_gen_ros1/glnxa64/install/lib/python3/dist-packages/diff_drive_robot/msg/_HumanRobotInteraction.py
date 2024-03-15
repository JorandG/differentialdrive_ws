# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from diff_drive_robot/HumanRobotInteraction.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class HumanRobotInteraction(genpy.Message):
  _md5sum = "274af9f6b9a200f779772ba4dac2aa42"
  _type = "diff_drive_robot/HumanRobotInteraction"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int32 HumanID
float64 RobotVelocity
float64 WaitingTime
float64 StartFilling
float64 FinishFilling
float64 StartServing
float64 FinishServing
"""
  __slots__ = ['HumanID','RobotVelocity','WaitingTime','StartFilling','FinishFilling','StartServing','FinishServing']
  _slot_types = ['int32','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       HumanID,RobotVelocity,WaitingTime,StartFilling,FinishFilling,StartServing,FinishServing

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(HumanRobotInteraction, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.HumanID is None:
        self.HumanID = 0
      if self.RobotVelocity is None:
        self.RobotVelocity = 0.
      if self.WaitingTime is None:
        self.WaitingTime = 0.
      if self.StartFilling is None:
        self.StartFilling = 0.
      if self.FinishFilling is None:
        self.FinishFilling = 0.
      if self.StartServing is None:
        self.StartServing = 0.
      if self.FinishServing is None:
        self.FinishServing = 0.
    else:
      self.HumanID = 0
      self.RobotVelocity = 0.
      self.WaitingTime = 0.
      self.StartFilling = 0.
      self.FinishFilling = 0.
      self.StartServing = 0.
      self.FinishServing = 0.

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
      buff.write(_get_struct_i6d().pack(_x.HumanID, _x.RobotVelocity, _x.WaitingTime, _x.StartFilling, _x.FinishFilling, _x.StartServing, _x.FinishServing))
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
      end = 0
      _x = self
      start = end
      end += 52
      (_x.HumanID, _x.RobotVelocity, _x.WaitingTime, _x.StartFilling, _x.FinishFilling, _x.StartServing, _x.FinishServing,) = _get_struct_i6d().unpack(str[start:end])
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
      buff.write(_get_struct_i6d().pack(_x.HumanID, _x.RobotVelocity, _x.WaitingTime, _x.StartFilling, _x.FinishFilling, _x.StartServing, _x.FinishServing))
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
      end = 0
      _x = self
      start = end
      end += 52
      (_x.HumanID, _x.RobotVelocity, _x.WaitingTime, _x.StartFilling, _x.FinishFilling, _x.StartServing, _x.FinishServing,) = _get_struct_i6d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_i6d = None
def _get_struct_i6d():
    global _struct_i6d
    if _struct_i6d is None:
        _struct_i6d = struct.Struct("<i6d")
    return _struct_i6d
