"""autogenerated by genpy from p1Turtle/obst_msg.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class obst_msg(genpy.Message):
  _md5sum = "3a39c7c3847cdf7a8f5c01f8be66dcdb"
  _type = "p1Turtle/obst_msg"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """#This message is publishing possible states about obstacles
#Different integer values describe different states
#
#0 No Obstacle anywhere
#1 Obstacle Right in Front
#2 Obstacle To the left
#3 Obstacle To the right

int32 front_obstacle
int32 left_obstacle
int32 right_obstacle



"""
  __slots__ = ['front_obstacle','left_obstacle','right_obstacle']
  _slot_types = ['int32','int32','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       front_obstacle,left_obstacle,right_obstacle

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(obst_msg, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.front_obstacle is None:
        self.front_obstacle = 0
      if self.left_obstacle is None:
        self.left_obstacle = 0
      if self.right_obstacle is None:
        self.right_obstacle = 0
    else:
      self.front_obstacle = 0
      self.left_obstacle = 0
      self.right_obstacle = 0

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
      buff.write(_struct_3i.pack(_x.front_obstacle, _x.left_obstacle, _x.right_obstacle))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 12
      (_x.front_obstacle, _x.left_obstacle, _x.right_obstacle,) = _struct_3i.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3i.pack(_x.front_obstacle, _x.left_obstacle, _x.right_obstacle))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 12
      (_x.front_obstacle, _x.left_obstacle, _x.right_obstacle,) = _struct_3i.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3i = struct.Struct("<3i")
