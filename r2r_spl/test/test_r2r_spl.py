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
import unittest

from game_controller_spl_interfaces.msg import RCGCD15
from r2r_spl.r2r_spl import R2RSPL
from r2r_spl.serialization import Serialization
from r2r_spl_test_interfaces.msg import ArrayTypes, BasicTypes
import rclpy
from rclpy.parameter import Parameter


class TestR2RSPL(unittest.TestCase):
    """Tests against R2RSPL."""

    received = None

    team_num = 1
    player_num = 1

    parameter_overrides = [
        Parameter('team_num', value=team_num),
        Parameter('player_num', value=player_num),
        Parameter('msg_type', value='r2r_spl_test_interfaces.msg.ArrayTypes')]

    def _callback_msg(self, msg):
        self.received = msg

    def test_receiving(self):
        """Test receiving UDP package from teammate."""
        rclpy.init()

        # Setup nodes
        r2r_spl_node = R2RSPL(parameter_overrides=self.parameter_overrides)  # noqa: F841
        test_node = rclpy.node.Node('test')
        subscription = test_node.create_subscription(  # noqa: F841
            ArrayTypes, 'r2r/recv', self._callback_msg, 10)

        # Example message from another player on same team
        serialization = Serialization(ArrayTypes)
        serialized = serialization.serialize(ArrayTypes())

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

        # Shutdown, then ensure thread is joined
        rclpy.shutdown()
        r2r_spl_node._loop_thread.join()

    def test_filter_own(self):
        """
        Test filtering of packages, when filter_own param is set to True.

        Check:
        - Packet sent by myself is ignored
        - Packet from teammate is processed
        """
        rclpy.init()

        r2r_spl_node = R2RSPL(parameter_overrides=[  # noqa: F841
            Parameter('team_num', value=self.team_num),
            Parameter('player_num', value=self.player_num),
            Parameter('msg_type', value='r2r_spl_test_interfaces.msg.ArrayTypes'),
            Parameter('filter_own', value=True)])
        test_node = rclpy.node.Node('test')
        subscription = test_node.create_subscription(  # noqa: F841
            ArrayTypes, 'r2r/recv', self._callback_msg, 10)

        # Send message from myself
        serialization = Serialization(ArrayTypes, player_num=self.player_num)
        serialized = serialization.serialize(ArrayTypes())
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

        # Send message from another player on same team
        serialization = Serialization(ArrayTypes, player_num=5)
        serialized = serialization.serialize(ArrayTypes())
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.sendto(serialized, ('', 10000 + self.team_num))

        # Wait for R2RSPL to receive packet over UDP
        time.sleep(0.1)

        # Check if message has been received on topic
        rclpy.spin_once(test_node, timeout_sec=0)

        # Expect message to be published
        self.assertIsNotNone(self.received)

        # Shutdown, then ensure thread is joined
        rclpy.shutdown()
        r2r_spl_node._loop_thread.join()

    def test_sending(self):
        """Test sending UDP package to teammate."""
        rclpy.init()

        r2r_spl_node = R2RSPL(parameter_overrides=self.parameter_overrides)  # noqa: F841
        test_node = rclpy.node.Node('test')
        publisher = test_node.create_publisher(ArrayTypes, 'r2r/send', 10)

        # UDP - adapted from https://github.com/ninedraft/python-udp/blob/8/server.py
        sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind(('', 10000 + self.team_num))
        sock.settimeout(0.1)

        # Publish ArrayTypes to r2r_spl_node
        publisher.publish(ArrayTypes())

        # Wait before spinning for the msg arrive in r2r_spl_node's subscription
        time.sleep(0.1)

        # Spin r2r_spl_node to process incoming message and send out UDP message
        rclpy.spin_once(r2r_spl_node, timeout_sec=0)

        # Check if packet has arrived
        try:
            _ = sock.recv(1024)
        except TimeoutError:
            self.fail('TimeoutError, did not receive expected UDP packet')

        # Close socket
        sock.close()

        # Shutdown, then ensure thread is joined
        rclpy.shutdown()
        r2r_spl_node._loop_thread.join()

    def test_invalid_msg_type(self):
        """
        Test msg type parameter that is in the wrong format.

        Check:
        - Msg type not containing a dot (eg. r2r_spl_test_interfaces/msg/ArrayTypes)
        - Msg type ending with a dot (eg. r2r_spl_test_interfaces.msg.)
        """
        rclpy.init()

        with self.assertRaises(AssertionError):
            R2RSPL(parameter_overrides=[
                Parameter('msg_type', value='r2r_spl_test_interfaces/msg/ArrayTypes')])
        with self.assertRaises(AssertionError):
            R2RSPL(parameter_overrides=[
                Parameter('msg_type', value='r2r_spl_test_interfaces.msg.')])
        rclpy.shutdown()

    def test_msg_type_not_found(self):
        """Test msg type parameter with non-existent message type."""
        rclpy.init()

        with self.assertRaises(ModuleNotFoundError):
            R2RSPL(parameter_overrides=[
                Parameter('msg_type', value='NonExistentPackage.msg.ArrayTypes')])
        with self.assertRaises(AttributeError):
            R2RSPL(parameter_overrides=[
                Parameter('msg_type', value='r2r_spl_test_interfaces.msg.NonExistentType')])
        rclpy.shutdown()

    def test_wrong_packet_size(self):
        """Test receiving UDP package of incorrect size (incompatible message type)."""
        rclpy.init()

        # Setup nodes
        r2r_spl_node = R2RSPL(parameter_overrides=self.parameter_overrides)  # noqa: F841
        test_node = rclpy.node.Node('test')

        # Incorrect message type from another player on same team
        serialization = Serialization(BasicTypes)
        serialized = serialization.serialize(BasicTypes())

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

        # Shutdown, then ensure thread is joined
        rclpy.shutdown()
        r2r_spl_node._loop_thread.join()

    def test_message_budget(self):
        """
        Test to ensure the SPL message budget is not exceeded.

        Check:
        - Sending stops when message budget is low
        - Sending restarts when extra budget is added
        """
        rclpy.init()

        r2r_spl_node = R2RSPL(parameter_overrides=self.parameter_overrides)  # noqa: F841
        test_node = rclpy.node.Node('test')
        publisher = test_node.create_publisher(ArrayTypes, 'r2r/send', 10)
        publisher_rcgcd = test_node.create_publisher(RCGCD15, 'gc/data', 10)

        # Publish RCGCD with low (<10) message budget
        rcgcd_msg = RCGCD15()
        rcgcd_msg.teams[0].team_number = self.team_num
        rcgcd_msg.teams[0].message_budget = 5
        publisher_rcgcd.publish(rcgcd_msg)

        # Wait before spinning for the msg arrive in r2r_spl_node's subscription
        time.sleep(0.1)

        # Spin r2r_spl_node to process incoming message
        rclpy.spin_once(r2r_spl_node, timeout_sec=0)

        # UDP - adapted from https://github.com/ninedraft/python-udp/blob/8/server.py
        sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind(('', 10000 + self.team_num))
        sock.settimeout(0.1)

        # Publish ArrayTypes to r2r_spl_node
        publisher.publish(ArrayTypes())

        # Wait before spinning for the msg arrive in r2r_spl_node's subscription
        time.sleep(0.1)

        # Spin r2r_spl_node to process incoming message and send out UDP message
        rclpy.spin_once(r2r_spl_node, timeout_sec=0)

        # Check to see that packet didn't arrive
        with self.assertRaises(TimeoutError):
            _ = sock.recv(1024)

        # Publish RCGCD with increased message budget (simulating extra time rule)
        rcgcd_msg.teams[0].message_budget = 60
        publisher_rcgcd.publish(rcgcd_msg)

        # Wait before spinning for the msg arrive in r2r_spl_node's subscription
        time.sleep(0.1)

        # Spin r2r_spl_node to process incoming message
        rclpy.spin_once(r2r_spl_node, timeout_sec=0)

        # Publish ArrayTypes to r2r_spl_node
        publisher.publish(ArrayTypes())

        # Wait before spinning for the msg arrive in r2r_spl_node's subscription
        time.sleep(0.1)

        # Spin r2r_spl_node to process incoming message and send out UDP message
        rclpy.spin_once(r2r_spl_node, timeout_sec=0)

        # Check if packet has arrived
        try:
            _ = sock.recv(1024)
        except TimeoutError:
            self.fail('TimeoutError, did not receive expected UDP packet')

        # Close socket
        sock.close()

        # Shutdown, then ensure thread is joined
        rclpy.shutdown()
        r2r_spl_node._loop_thread.join()

    def test_msg_size_exceeding_128_bytes(self):
        """Test to ensure we don't send messages exceeding 128 bytes."""
        rclpy.init()
        r2r_spl_node = R2RSPL(parameter_overrides=self.parameter_overrides)  # noqa: F841
        test_node = rclpy.node.Node('test')
        publisher = test_node.create_publisher(ArrayTypes, 'r2r/send', 10)

        # UDP - adapted from https://github.com/ninedraft/python-udp/blob/8/server.py
        sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind(('', 10000 + self.team_num))
        sock.settimeout(0.1)

        # Publish ArrayTypes to r2r_spl_node
        publisher.publish(ArrayTypes(data_string='a' * 129))

        # Wait before spinning for the msg arrive in r2r_spl_node's subscription
        time.sleep(0.1)

        # Spin r2r_spl_node to process incoming message and send out UDP message
        rclpy.spin_once(r2r_spl_node, timeout_sec=0)

        # Check to see that packet didn't arrive
        with self.assertRaises(TimeoutError):
            _ = sock.recv(1024)

        # Expect no message published
        self.assertIsNone(self.received)

        # Close socket
        sock.close()

        # Shutdown, then ensure thread is joined
        rclpy.shutdown()
        r2r_spl_node._loop_thread.join()


if __name__ == '__main__':
    unittest.main()
