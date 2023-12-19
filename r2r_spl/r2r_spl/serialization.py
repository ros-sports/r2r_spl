import construct
import rosidl_parser.definition

from r2r_spl.to_struct import to_struct

class Serialization:

  def __init__(self, msg_class):
    self.msg_class = msg_class
    self.struct = to_struct(msg_class)

  def serialize(self, msg_instance):
    """Serialize a message to a byte array"""
    values = {}
    for s, t in zip(msg_instance.get_fields_and_field_types().keys(), msg_instance.SLOT_TYPES):
        # Check if namespaced type
        if isinstance(t, rosidl_parser.definition.NamespacedType):
            field = getattr(msg_instance, s)
            values[s] = create_container(field)
        # Check if basic type
        if isinstance(t, rosidl_parser.definition.BasicType):
            values[s] = getattr(msg_instance, s)
    container = construct.Container(**values)
    return self.struct.build(container)

  def deserialize(self, serialized):
    """Deserialize a byte array to a message"""
    # Parse and build ros message
    # We need to do something recursive here too.......
    parsed = self.struct.parse(serialized)
    msg_map = {sc.name : getattr(parsed, sc.name) for sc in self.struct.subcons}
    return self.msg_class(**msg_map)