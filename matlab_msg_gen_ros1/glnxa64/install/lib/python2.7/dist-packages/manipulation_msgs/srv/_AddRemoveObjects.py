# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from manipulation_msgs/AddRemoveObjectsRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import manipulation_msgs.msg
import geometry_msgs.msg

class AddRemoveObjectsRequest(genpy.Message):
  _md5sum = "c5a2c66d971abb45d9c05fd28ac08124"
  _type = "manipulation_msgs/AddRemoveObjectsRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """string inbound_box_name
manipulation_msgs/Object[] add_objects
manipulation_msgs/Object[] remove_objects

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
  __slots__ = ['inbound_box_name','add_objects','remove_objects']
  _slot_types = ['string','manipulation_msgs/Object[]','manipulation_msgs/Object[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       inbound_box_name,add_objects,remove_objects

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(AddRemoveObjectsRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.inbound_box_name is None:
        self.inbound_box_name = ''
      if self.add_objects is None:
        self.add_objects = []
      if self.remove_objects is None:
        self.remove_objects = []
    else:
      self.inbound_box_name = ''
      self.add_objects = []
      self.remove_objects = []

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
      _x = self.inbound_box_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.add_objects)
      buff.write(_struct_I.pack(length))
      for val1 in self.add_objects:
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
      length = len(self.remove_objects)
      buff.write(_struct_I.pack(length))
      for val1 in self.remove_objects:
        length = len(val1.grasping_poses)
        buff.write(_struct_I.pack(length))
        for val2 in val1.grasping_poses:
          _v7 = val2.pose
          _v8 = _v7.position
          _x = _v8
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
          _v9 = _v7.orientation
          _x = _v9
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
        _v10 = val1.pose
        _v11 = _v10.position
        _x = _v11
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v12 = _v10.orientation
        _x = _v12
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.add_objects is None:
        self.add_objects = None
      if self.remove_objects is None:
        self.remove_objects = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.inbound_box_name = str[start:end].decode('utf-8')
      else:
        self.inbound_box_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.add_objects = []
      for i in range(0, length):
        val1 = manipulation_msgs.msg.Object()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.grasping_poses = []
        for i in range(0, length):
          val2 = manipulation_msgs.msg.Grasp()
          _v13 = val2.pose
          _v14 = _v13.position
          _x = _v14
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          _v15 = _v13.orientation
          _x = _v15
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
        _v16 = val1.pose
        _v17 = _v16.position
        _x = _v17
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v18 = _v16.orientation
        _x = _v18
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        self.add_objects.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.remove_objects = []
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
        self.remove_objects.append(val1)
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
      _x = self.inbound_box_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.add_objects)
      buff.write(_struct_I.pack(length))
      for val1 in self.add_objects:
        length = len(val1.grasping_poses)
        buff.write(_struct_I.pack(length))
        for val2 in val1.grasping_poses:
          _v25 = val2.pose
          _v26 = _v25.position
          _x = _v26
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
          _v27 = _v25.orientation
          _x = _v27
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
        _v28 = val1.pose
        _v29 = _v28.position
        _x = _v29
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v30 = _v28.orientation
        _x = _v30
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
      length = len(self.remove_objects)
      buff.write(_struct_I.pack(length))
      for val1 in self.remove_objects:
        length = len(val1.grasping_poses)
        buff.write(_struct_I.pack(length))
        for val2 in val1.grasping_poses:
          _v31 = val2.pose
          _v32 = _v31.position
          _x = _v32
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
          _v33 = _v31.orientation
          _x = _v33
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
        _v34 = val1.pose
        _v35 = _v34.position
        _x = _v35
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v36 = _v34.orientation
        _x = _v36
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.add_objects is None:
        self.add_objects = None
      if self.remove_objects is None:
        self.remove_objects = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.inbound_box_name = str[start:end].decode('utf-8')
      else:
        self.inbound_box_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.add_objects = []
      for i in range(0, length):
        val1 = manipulation_msgs.msg.Object()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.grasping_poses = []
        for i in range(0, length):
          val2 = manipulation_msgs.msg.Grasp()
          _v37 = val2.pose
          _v38 = _v37.position
          _x = _v38
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          _v39 = _v37.orientation
          _x = _v39
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
        _v40 = val1.pose
        _v41 = _v40.position
        _x = _v41
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v42 = _v40.orientation
        _x = _v42
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        self.add_objects.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.remove_objects = []
      for i in range(0, length):
        val1 = manipulation_msgs.msg.Object()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.grasping_poses = []
        for i in range(0, length):
          val2 = manipulation_msgs.msg.Grasp()
          _v43 = val2.pose
          _v44 = _v43.position
          _x = _v44
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          _v45 = _v43.orientation
          _x = _v45
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
        _v46 = val1.pose
        _v47 = _v46.position
        _x = _v47
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v48 = _v46.orientation
        _x = _v48
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        self.remove_objects.append(val1)
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
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from manipulation_msgs/AddRemoveObjectsResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class AddRemoveObjectsResponse(genpy.Message):
  _md5sum = "450796f7fe6bdeb692c1c6881b3b415f"
  _type = "manipulation_msgs/AddRemoveObjectsResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int8 results
int8 Success=0
int8 ObjectNotFound=-1


"""
  # Pseudo-constants
  Success = 0
  ObjectNotFound = -1

  __slots__ = ['results']
  _slot_types = ['int8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       results

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(AddRemoveObjectsResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.results is None:
        self.results = 0
    else:
      self.results = 0

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
      buff.write(_get_struct_b().pack(self.results))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 1
      (self.results,) = _get_struct_b().unpack(str[start:end])
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
      buff.write(_get_struct_b().pack(self.results))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 1
      (self.results,) = _get_struct_b().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_b = None
def _get_struct_b():
    global _struct_b
    if _struct_b is None:
        _struct_b = struct.Struct("<b")
    return _struct_b
class AddRemoveObjects(object):
  _type          = 'manipulation_msgs/AddRemoveObjects'
  _md5sum = '6ba7cf8734f51f28c5c0c6aba89a6f3a'
  _request_class  = AddRemoveObjectsRequest
  _response_class = AddRemoveObjectsResponse
