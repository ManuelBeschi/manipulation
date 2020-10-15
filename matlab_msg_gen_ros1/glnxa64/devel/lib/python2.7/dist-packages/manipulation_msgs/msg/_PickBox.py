# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from manipulation_msgs/PickBox.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import manipulation_msgs.msg
import geometry_msgs.msg

class PickBox(genpy.Message):
  _md5sum = "0cae877e70eed9951fbd5fae9fd2ae3d"
  _type = "manipulation_msgs/PickBox"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """manipulation_msgs/Object[] objects
string name
geometry_msgs/Pose approach_pose

================================================================================
MSG: manipulation_msgs/Object
manipulation_msgs/Grasp[] grasping_poses
string type
string id
geometry_msgs/Pose pose

================================================================================
MSG: manipulation_msgs/Grasp
geometry_msgs/Pose pose
string tool_name

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
"""
  __slots__ = ['objects','name','approach_pose']
  _slot_types = ['manipulation_msgs/Object[]','string','geometry_msgs/Pose']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       objects,name,approach_pose

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PickBox, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.objects is None:
        self.objects = []
      if self.name is None:
        self.name = ''
      if self.approach_pose is None:
        self.approach_pose = geometry_msgs.msg.Pose()
    else:
      self.objects = []
      self.name = ''
      self.approach_pose = geometry_msgs.msg.Pose()

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
      length = len(self.objects)
      buff.write(_struct_I.pack(length))
      for val1 in self.objects:
        length = len(val1.grasping_poses)
        buff.write(_struct_I.pack(length))
        for val2 in val1.grasping_poses:
          _v1 = val2.pose
          _v2 = _v1.position
          _x = _v2
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
          _v3 = _v1.orientation
          _x = _v3
          buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
          _x = val2.tool_name
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.type
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v4 = val1.pose
        _v5 = _v4.position
        _x = _v5
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v6 = _v4.orientation
        _x = _v6
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
      _x = self.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_7d().pack(_x.approach_pose.position.x, _x.approach_pose.position.y, _x.approach_pose.position.z, _x.approach_pose.orientation.x, _x.approach_pose.orientation.y, _x.approach_pose.orientation.z, _x.approach_pose.orientation.w))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.objects is None:
        self.objects = None
      if self.approach_pose is None:
        self.approach_pose = geometry_msgs.msg.Pose()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.objects = []
      for i in range(0, length):
        val1 = manipulation_msgs.msg.Object()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.grasping_poses = []
        for i in range(0, length):
          val2 = manipulation_msgs.msg.Grasp()
          _v7 = val2.pose
          _v8 = _v7.position
          _x = _v8
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          _v9 = _v7.orientation
          _x = _v9
          start = end
          end += 32
          (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val2.tool_name = str[start:end].decode('utf-8')
          else:
            val2.tool_name = str[start:end]
          val1.grasping_poses.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.type = str[start:end].decode('utf-8')
        else:
          val1.type = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.id = str[start:end].decode('utf-8')
        else:
          val1.id = str[start:end]
        _v10 = val1.pose
        _v11 = _v10.position
        _x = _v11
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v12 = _v10.orientation
        _x = _v12
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        self.objects.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.name = str[start:end].decode('utf-8')
      else:
        self.name = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.approach_pose.position.x, _x.approach_pose.position.y, _x.approach_pose.position.z, _x.approach_pose.orientation.x, _x.approach_pose.orientation.y, _x.approach_pose.orientation.z, _x.approach_pose.orientation.w,) = _get_struct_7d().unpack(str[start:end])
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
      length = len(self.objects)
      buff.write(_struct_I.pack(length))
      for val1 in self.objects:
        length = len(val1.grasping_poses)
        buff.write(_struct_I.pack(length))
        for val2 in val1.grasping_poses:
          _v13 = val2.pose
          _v14 = _v13.position
          _x = _v14
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
          _v15 = _v13.orientation
          _x = _v15
          buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
          _x = val2.tool_name
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.type
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v16 = val1.pose
        _v17 = _v16.position
        _x = _v17
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v18 = _v16.orientation
        _x = _v18
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
      _x = self.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_7d().pack(_x.approach_pose.position.x, _x.approach_pose.position.y, _x.approach_pose.position.z, _x.approach_pose.orientation.x, _x.approach_pose.orientation.y, _x.approach_pose.orientation.z, _x.approach_pose.orientation.w))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.objects is None:
        self.objects = None
      if self.approach_pose is None:
        self.approach_pose = geometry_msgs.msg.Pose()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.objects = []
      for i in range(0, length):
        val1 = manipulation_msgs.msg.Object()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.grasping_poses = []
        for i in range(0, length):
          val2 = manipulation_msgs.msg.Grasp()
          _v19 = val2.pose
          _v20 = _v19.position
          _x = _v20
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          _v21 = _v19.orientation
          _x = _v21
          start = end
          end += 32
          (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val2.tool_name = str[start:end].decode('utf-8')
          else:
            val2.tool_name = str[start:end]
          val1.grasping_poses.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.type = str[start:end].decode('utf-8')
        else:
          val1.type = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.id = str[start:end].decode('utf-8')
        else:
          val1.id = str[start:end]
        _v22 = val1.pose
        _v23 = _v22.position
        _x = _v23
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v24 = _v22.orientation
        _x = _v24
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        self.objects.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.name = str[start:end].decode('utf-8')
      else:
        self.name = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.approach_pose.position.x, _x.approach_pose.position.y, _x.approach_pose.position.z, _x.approach_pose.orientation.x, _x.approach_pose.orientation.y, _x.approach_pose.orientation.z, _x.approach_pose.orientation.w,) = _get_struct_7d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_4d = None
def _get_struct_4d():
    global _struct_4d
    if _struct_4d is None:
        _struct_4d = struct.Struct("<4d")
    return _struct_4d
_struct_7d = None
def _get_struct_7d():
    global _struct_7d
    if _struct_7d is None:
        _struct_7d = struct.Struct("<7d")
    return _struct_7d
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d