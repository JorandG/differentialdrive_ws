# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from diff_drive_robot/HumanRobotInteraction.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class HumanRobotInteraction(genpy.Message):
  _md5sum = "57cfebb0e2a7d533881351d9686be0c2"
  _type = "diff_drive_robot/HumanRobotInteraction"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int32 HumanID
float64[] RobotWaitingDistance
float64[] RobotVelocityProximity
float64[] RobotMinVelocityProximity
float64[] RobotMaxVelocityProximity
float64[] RobotVelocityProximityWeight
float64[] WaitingTime
float64[] WaitingTimeWeight
float64[] StartFilling
float64[] FinishFilling
float64[] StartServing
float64[] FinishServing
float64[] TimeFilling
float64[] TimeServing
int32[] ConfirmServing
int32[] ConfirmFilling
int32 Task
int32 TaskFilling
"""
  __slots__ = ['HumanID','RobotWaitingDistance','RobotVelocityProximity','RobotMinVelocityProximity','RobotMaxVelocityProximity','RobotVelocityProximityWeight','WaitingTime','WaitingTimeWeight','StartFilling','FinishFilling','StartServing','FinishServing','TimeFilling','TimeServing','ConfirmServing','ConfirmFilling','Task','TaskFilling']
  _slot_types = ['int32','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','int32[]','int32[]','int32','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       HumanID,RobotWaitingDistance,RobotVelocityProximity,RobotMinVelocityProximity,RobotMaxVelocityProximity,RobotVelocityProximityWeight,WaitingTime,WaitingTimeWeight,StartFilling,FinishFilling,StartServing,FinishServing,TimeFilling,TimeServing,ConfirmServing,ConfirmFilling,Task,TaskFilling

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(HumanRobotInteraction, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.HumanID is None:
        self.HumanID = 0
      if self.RobotWaitingDistance is None:
        self.RobotWaitingDistance = []
      if self.RobotVelocityProximity is None:
        self.RobotVelocityProximity = []
      if self.RobotMinVelocityProximity is None:
        self.RobotMinVelocityProximity = []
      if self.RobotMaxVelocityProximity is None:
        self.RobotMaxVelocityProximity = []
      if self.RobotVelocityProximityWeight is None:
        self.RobotVelocityProximityWeight = []
      if self.WaitingTime is None:
        self.WaitingTime = []
      if self.WaitingTimeWeight is None:
        self.WaitingTimeWeight = []
      if self.StartFilling is None:
        self.StartFilling = []
      if self.FinishFilling is None:
        self.FinishFilling = []
      if self.StartServing is None:
        self.StartServing = []
      if self.FinishServing is None:
        self.FinishServing = []
      if self.TimeFilling is None:
        self.TimeFilling = []
      if self.TimeServing is None:
        self.TimeServing = []
      if self.ConfirmServing is None:
        self.ConfirmServing = []
      if self.ConfirmFilling is None:
        self.ConfirmFilling = []
      if self.Task is None:
        self.Task = 0
      if self.TaskFilling is None:
        self.TaskFilling = 0
    else:
      self.HumanID = 0
      self.RobotWaitingDistance = []
      self.RobotVelocityProximity = []
      self.RobotMinVelocityProximity = []
      self.RobotMaxVelocityProximity = []
      self.RobotVelocityProximityWeight = []
      self.WaitingTime = []
      self.WaitingTimeWeight = []
      self.StartFilling = []
      self.FinishFilling = []
      self.StartServing = []
      self.FinishServing = []
      self.TimeFilling = []
      self.TimeServing = []
      self.ConfirmServing = []
      self.ConfirmFilling = []
      self.Task = 0
      self.TaskFilling = 0

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
      _x = self.HumanID
      buff.write(_get_struct_i().pack(_x))
      length = len(self.RobotWaitingDistance)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.RobotWaitingDistance))
      length = len(self.RobotVelocityProximity)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.RobotVelocityProximity))
      length = len(self.RobotMinVelocityProximity)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.RobotMinVelocityProximity))
      length = len(self.RobotMaxVelocityProximity)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.RobotMaxVelocityProximity))
      length = len(self.RobotVelocityProximityWeight)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.RobotVelocityProximityWeight))
      length = len(self.WaitingTime)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.WaitingTime))
      length = len(self.WaitingTimeWeight)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.WaitingTimeWeight))
      length = len(self.StartFilling)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.StartFilling))
      length = len(self.FinishFilling)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.FinishFilling))
      length = len(self.StartServing)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.StartServing))
      length = len(self.FinishServing)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.FinishServing))
      length = len(self.TimeFilling)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.TimeFilling))
      length = len(self.TimeServing)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.TimeServing))
      length = len(self.ConfirmServing)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.Struct(pattern).pack(*self.ConfirmServing))
      length = len(self.ConfirmFilling)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.Struct(pattern).pack(*self.ConfirmFilling))
      _x = self
      buff.write(_get_struct_2i().pack(_x.Task, _x.TaskFilling))
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
      start = end
      end += 4
      (self.HumanID,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.RobotWaitingDistance = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.RobotVelocityProximity = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.RobotMinVelocityProximity = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.RobotMaxVelocityProximity = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.RobotVelocityProximityWeight = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.WaitingTime = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.WaitingTimeWeight = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.StartFilling = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.FinishFilling = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.StartServing = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.FinishServing = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.TimeFilling = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.TimeServing = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.ConfirmServing = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.ConfirmFilling = s.unpack(str[start:end])
      _x = self
      start = end
      end += 8
      (_x.Task, _x.TaskFilling,) = _get_struct_2i().unpack(str[start:end])
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
      _x = self.HumanID
      buff.write(_get_struct_i().pack(_x))
      length = len(self.RobotWaitingDistance)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.RobotWaitingDistance.tostring())
      length = len(self.RobotVelocityProximity)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.RobotVelocityProximity.tostring())
      length = len(self.RobotMinVelocityProximity)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.RobotMinVelocityProximity.tostring())
      length = len(self.RobotMaxVelocityProximity)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.RobotMaxVelocityProximity.tostring())
      length = len(self.RobotVelocityProximityWeight)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.RobotVelocityProximityWeight.tostring())
      length = len(self.WaitingTime)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.WaitingTime.tostring())
      length = len(self.WaitingTimeWeight)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.WaitingTimeWeight.tostring())
      length = len(self.StartFilling)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.StartFilling.tostring())
      length = len(self.FinishFilling)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.FinishFilling.tostring())
      length = len(self.StartServing)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.StartServing.tostring())
      length = len(self.FinishServing)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.FinishServing.tostring())
      length = len(self.TimeFilling)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.TimeFilling.tostring())
      length = len(self.TimeServing)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.TimeServing.tostring())
      length = len(self.ConfirmServing)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.ConfirmServing.tostring())
      length = len(self.ConfirmFilling)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.ConfirmFilling.tostring())
      _x = self
      buff.write(_get_struct_2i().pack(_x.Task, _x.TaskFilling))
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
      start = end
      end += 4
      (self.HumanID,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.RobotWaitingDistance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.RobotVelocityProximity = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.RobotMinVelocityProximity = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.RobotMaxVelocityProximity = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.RobotVelocityProximityWeight = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.WaitingTime = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.WaitingTimeWeight = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.StartFilling = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.FinishFilling = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.StartServing = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.FinishServing = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.TimeFilling = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.TimeServing = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.ConfirmServing = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.ConfirmFilling = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      _x = self
      start = end
      end += 8
      (_x.Task, _x.TaskFilling,) = _get_struct_2i().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2i = None
def _get_struct_2i():
    global _struct_2i
    if _struct_2i is None:
        _struct_2i = struct.Struct("<2i")
    return _struct_2i
_struct_i = None
def _get_struct_i():
    global _struct_i
    if _struct_i is None:
        _struct_i = struct.Struct("<i")
    return _struct_i
