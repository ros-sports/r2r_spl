import construct
import rosidl_parser.definition

basic_type_conversion = {
    'float': construct.Float32l,
    'double': construct.Float64l,
    'char': construct.Int8ul,
    'boolean': construct.Flag,
    'octet': construct.Byte,
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
        # Check if namespaced type
        if isinstance(t, rosidl_parser.definition.NamespacedType):
            mod = __import__('.'.join(t.namespaces), fromlist=[t.name])
            klass = getattr(mod, t.name)
            members.append(s / get_construct_type(klass))

        # Check if array

        # Check if unbounded sequence

        # Check if bounded sequence

        # Check if unbounded string

        # Check if bounded string

        # Check if unbounded wstring

        # Check if bounded wstring

        # Check if basic type
        if isinstance(t, rosidl_parser.definition.BasicType):
            members.append(s / basic_type_conversion[t.typename])
    return construct.Struct(*members)