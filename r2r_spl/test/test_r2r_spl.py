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

import socket
import time

from construct import Container

from r2r_spl.r2r_spl import R2RSPL
from r2r_spl.serialization import Serialization

import rclpy
from rclpy.parameter import Parameter

from r2r_spl_test_interfaces.msg import ArrayTypes, BasicTypes, NestedTypes

import unittest


class TestR2RSPL(unittest.TestCase):
    """Tests against R2RSPL."""

    received = None

    team_num = 1
    player_num = 1

    parameter_overrides = [
        Parameter('team_num', value=team_num),
        Parameter('player_num', value=player_num),
        Parameter('msg_type', value='r2r_spl_test_interfaces.msg.BasicTypes')]

    def setUp(self):
        rclpy.init()

    def tearDown(self):
        rclpy.shutdown()

    def _callback_msg(self, msg):
        self.received = msg

    def test_receiving(self):
        """Test receiving UDP package from teammate."""
        # Setup nodes
        r2r_spl_node = R2RSPL(parameter_overrides=self.parameter_overrides)
        test_node = rclpy.node.Node('test')
        subscription = test_node.create_subscription(BasicTypes, 'r2r/recv', self._callback_msg, 10)

        # Example message from another player on same team
        serialization = Serialization(BasicTypes)
        serialized = serialization.serialize(BasicTypes())

        # UDP - adapted from https://github.com/ninedraft/python-udp/blob/8/server.py
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.sendto(serialized, ('', 10000 + self.team_num))

        # Wait for R2RSPL to receive packet over UDP, and publish a ROS msg
        time.sleep(0.1)

        # Check if message has been received on topic
        rclpy.spin_once(test_node, timeout_sec=0)
        self.assertIsNotNone(self.received)

    def test_not_receiving_myself(self):
        """Test ignoring UDP package sent from myself."""
        pass

    def test_sending(self):
        """Test sending UDP package to teammate."""
        r2r_spl_node = R2RSPL(parameter_overrides=self.parameter_overrides)
        test_node = rclpy.node.Node('test')
        publisher = test_node.create_publisher(BasicTypes, 'r2r/send', 10)

        # UDP - adapted from https://github.com/ninedraft/python-udp/blob/8/server.py
        sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind(('', 10000 + self.team_num))
        sock.settimeout(0.1)

        # Publish SPLSM to r2r_spl_node
        publisher.publish(BasicTypes())

        # Wait before spinning for the msg arrive in r2r_spl_node's subscription
        time.sleep(0.01)

        # Spin r2r_spl_node to process incoming message and send out UDP message
        rclpy.spin_once(r2r_spl_node, timeout_sec=0)

        # Check if packet has arrived
        try:
            _ = sock.recv(1024)
        except TimeoutError:
            self.fail("TimeoutError, did not receive expected UDP packet")

        # Close socket
        sock.close()

    def test_invalid_msg_type(self):
        """Test msg type parameter that is in the wrong format.
           - Not containing a dot (eg. r2r_spl_test_interfaces/msg/BasicTypes)
           - Ending with a dot (eg. r2r_spl_test_interfaces.msg.)
        """
        with self.assertRaises(AssertionError):
            R2RSPL(parameter_overrides=[
                Parameter('msg_type', value='r2r_spl_test_interfaces/msg/BasicTypes')])
        with self.assertRaises(AssertionError):
            R2RSPL(parameter_overrides=[
                Parameter('msg_type', value='r2r_spl_test_interfaces.msg.')])

    def test_msg_type_not_found(self):
        """Test msg type parameter with non-existent message type"""
        with self.assertRaises(ModuleNotFoundError):
            R2RSPL(parameter_overrides=[
                Parameter('msg_type', value='NonExistentPackage.msg.BasicTypes')])
        with self.assertRaises(AttributeError):
            R2RSPL(parameter_overrides=[
                Parameter('msg_type', value='r2r_spl_test_interfaces.msg.NonExistentType')])

    def test_wrong_packet_size(self):
        """Test receiving UDP package of incorrect size (incompatible message type)."""
        # Setup nodes
        r2r_spl_node = R2RSPL(parameter_overrides=self.parameter_overrides)
        test_node = rclpy.node.Node('test')

        # Incorrect message type from another player on same team
        serialization = Serialization(ArrayTypes)
        serialized = serialization.serialize(ArrayTypes())

        # UDP - adapted from https://github.com/ninedraft/python-udp/blob/8/server.py
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.sendto(serialized, ('', 10000 + self.team_num))

        # Wait for R2RSPL to receive packet over UDP
        time.sleep(0.1)

        # Check if message has been received on topic
        rclpy.spin_once(test_node, timeout_sec=0)

        # Expect no message published
        self.assertIsNone(self.received)

if __name__ == '__main__':
    unittest.main()
