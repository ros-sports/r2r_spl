import construct
import rosidl_parser.definition

class Serialization:

    def __init__(self, msg_class):
        self.msg_class = msg_class
        self.struct = to_struct(msg_class)

    def serialize(self, msg_instance):
        """Serialize a message to a byte array"""
        container = to_container(msg_instance)
        return self.struct.build(container)

    def deserialize(self, serialized):
        """Deserialize a byte array to a message"""
        # Parse and build ros message
        # We need to do something recursive here too.......
        parsed = self.struct.parse(serialized)
        msg_map = {sc.name : getattr(parsed, sc.name) for sc in self.struct.subcons}
        return self.msg_class(**msg_map)

basic_type_conversion = {
    'float': construct.Float32l,
    'double': construct.Float64l,
    'boolean': construct.Flag,
    'octet': construct.Bytes(1),
    'int8': construct.Int8sl,
    'uint8': construct.Int8ul,
    'int16': construct.Int16sl,
    'uint16': construct.Int16ul,
    'int32': construct.Int32sl,
    'uint32': construct.Int32ul,
    'int64': construct.Int64sl,
    'uint64': construct.Int64ul,
}

def to_struct(msg_class) -> construct.Struct:
    """Convert a message to a construct struct"""

    members = []
    for s, t in zip(msg_class.get_fields_and_field_types().keys(), msg_class.SLOT_TYPES):
        # Nested Type
        if isinstance(t, rosidl_parser.definition.NamespacedType):
            mod = __import__('.'.join(t.namespaces), fromlist=[t.name])
            klass = getattr(mod, t.name)
            members.append(s / to_struct(klass))

        # Array
        elif isinstance(t, rosidl_parser.definition.Array):
            if isinstance(t.value_type, rosidl_parser.definition.NamespacedType):
                mod = __import__('.'.join(t.value_type.namespaces), fromlist=[t.value_type.name])
                klass = getattr(mod, t.value_type.name)
                tmp_type = to_struct(klass)
            elif isinstance(t.value_type, rosidl_parser.definition.BasicType):
                tmp_type = basic_type_conversion[t.value_type.typename]
            else:
                tmp_type = to_struct(t.value_type)
            members.append(s / construct.Array(t.size, tmp_type))

        # Unbounded sequence
        # Bounded sequence
        elif isinstance(t, rosidl_parser.definition.UnboundedSequence) or \
            isinstance(t, rosidl_parser.definition.BoundedSequence):
            if isinstance(t.value_type, rosidl_parser.definition.NamespacedType):
                mod = __import__('.'.join(t.value_type.namespaces), fromlist=[t.value_type.name])
                klass = getattr(mod, t.value_type.name)
                tmp_type = to_struct(klass)
            elif isinstance(t.value_type, rosidl_parser.definition.BasicType):
                tmp_type = basic_type_conversion[t.value_type.typename]
            else:
                tmp_type = to_struct(t.value_type)
            members.append(s / construct.PrefixedArray(construct.VarInt, tmp_type))

        # Unbounded string
        # Bounded string
        elif isinstance(t, rosidl_parser.definition.UnboundedString) or \
            isinstance(t, rosidl_parser.definition.BoundedString):
            members.append(s / construct.PascalString(construct.VarInt, 'utf8'))

        # Unbounded wstring
        # Bounded wstring
        elif isinstance(t, rosidl_parser.definition.UnboundedWString) or \
            isinstance(t, rosidl_parser.definition.BoundedWString):
            members.append(s / construct.PascalString(construct.VarInt, 'utf16'))

        # Basic type
        elif isinstance(t, rosidl_parser.definition.BasicType):
            members.append(s / basic_type_conversion[t.typename])
    return construct.Struct(*members)

def to_container(msg_instance) -> construct.Container:
    values = {}
    for s, t in zip(msg_instance.get_fields_and_field_types().keys(), msg_instance.SLOT_TYPES):
        # Check if namespaced type
        if isinstance(t, rosidl_parser.definition.NamespacedType):
            field = getattr(msg_instance, s)
            values[s] = to_container(field)
        # Check if array type
        elif isinstance(t, rosidl_parser.definition.Array) or \
            isinstance(t, rosidl_parser.definition.UnboundedSequence) or \
            isinstance(t, rosidl_parser.definition.BoundedSequence):
            field = getattr(msg_instance, s)
            if isinstance(t.value_type, rosidl_parser.definition.NamespacedType):
                values[s] = [to_container(f) for f in field]
            elif isinstance(t.value_type, rosidl_parser.definition.BasicType):
                if t.value_type.typename in rosidl_parser.definition.INTEGER_TYPES or \
                    t.value_type.typename in rosidl_parser.definition.FLOATING_POINT_TYPES:
                    values[s] = field.tolist()
                else:
                    values[s] = field
        # Check if string type
        elif isinstance(t, rosidl_parser.definition.UnboundedString) or \
            isinstance(t, rosidl_parser.definition.BoundedString):
            values[s] = getattr(msg_instance, s)
        # Check if wstring type
        elif isinstance(t, rosidl_parser.definition.UnboundedWString) or \
            isinstance(t, rosidl_parser.definition.BoundedWString):
            values[s] = getattr(msg_instance, s)
        # Check if basic type
        elif isinstance(t, rosidl_parser.definition.BasicType):
            values[s] = getattr(msg_instance, s)
    return construct.Container(**values)
