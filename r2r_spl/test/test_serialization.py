# Copyright 2023 Kenji Brameld
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from construct import Container

from r2r_spl_test_interfaces.msg import ArrayTypes, BasicTypes, NestedTypes
from r2r_spl.serialization import Serialization

import unittest

import numpy.testing

class TestSerialization(unittest.TestCase):

    def test_basic_types(self):
        """Test serialization and deserialization of basic types"""
        # Initialize serialization object
        serialization = Serialization(BasicTypes)

        # Create message instance
        msg_instance = BasicTypes()
        msg_instance.val_bool = True
        msg_instance.val_float32 = 0.01
        msg_instance.val_float64 = 0.02
        msg_instance.val_int8 = -1
        msg_instance.val_uint8 = 2
        msg_instance.val_int16 = -3
        msg_instance.val_uint16 = 4
        msg_instance.val_int32 = -5
        msg_instance.val_uint32 = 6
        msg_instance.val_int64 = -7
        msg_instance.val_uint64 = 8

        # Serialize
        serialized = serialization.serialize(msg_instance)

        # Deserialize
        deserialized = serialization.deserialize(serialized)

        # Check types match original message
        self.assertEqual(type(msg_instance), type(deserialized))

        # Check values match original message
        self.assertEqual(deserialized.val_bool, True)
        # self.assertEqual(deserialized.val_byte, )  # Some issues with this one, look into it later
        self.assertAlmostEqual(deserialized.val_float32, 0.01)
        self.assertAlmostEqual(deserialized.val_float64, 0.02)
        self.assertEqual(deserialized.val_int8, -1)
        self.assertEqual(deserialized.val_uint8, 2)
        self.assertEqual(deserialized.val_int16, -3)
        self.assertEqual(deserialized.val_uint16, 4)
        self.assertEqual(deserialized.val_int32, -5)
        self.assertEqual(deserialized.val_uint32, 6)
        self.assertEqual(deserialized.val_int64, -7)
        self.assertEqual(deserialized.val_uint64, 8)

    def test_array_types(self):
        """Test serialization and deserialization of array types"""
        # Initialize serialization object
        serialization = Serialization(ArrayTypes)

        # Create message instance
        msg_instance = ArrayTypes()
        msg_instance.data_int8_static = [1, 2, 3]
        msg_instance.data_bool_static = [False, True]
        msg_instance.data_int8_unbounded_dynamic = [10, 20, 30, 40]
        msg_instance.data_bool_unbounded_dynamic = [True, False, True, False, True]
        msg_instance.data_int8_bounded_dynamic = [70, 80]
        msg_instance.data_bool_bounded_dynamic = [False]

        # Serialize
        serialized = serialization.serialize(msg_instance)

        # Deserialize
        deserialized = serialization.deserialize(serialized)

        # Check types match original message
        self.assertEqual(type(msg_instance), type(deserialized))

        # Check values match original message
        numpy.testing.assert_array_equal(deserialized.data_int8_static, [1, 2, 3])
        self.assertEqual(deserialized.data_bool_static, [False, True])
        numpy.testing.assert_array_equal(deserialized.data_int8_unbounded_dynamic, [10, 20, 30, 40])
        self.assertEqual(deserialized.data_bool_unbounded_dynamic, [True, False, True, False, True])
        numpy.testing.assert_array_equal(deserialized.data_int8_bounded_dynamic, [70, 80])
        self.assertEqual(deserialized.data_bool_bounded_dynamic, [False])

    def test_nested_types(self):
        """Test serialization and deserialization of nested types"""
        # Initialize serialization object
        serialization = Serialization(NestedTypes)

        # Create message instance
        msg_instance = NestedTypes()

        # Serialize
        serialized = serialization.serialize(msg_instance)

        # Deserialize
        deserialized = serialization.deserialize(serialized)

        # Check types match original message
        self.assertEqual(type(msg_instance), type(deserialized))

        # Check values match original message
        # ....

if __name__ == '__main__':
    unittest.main()
