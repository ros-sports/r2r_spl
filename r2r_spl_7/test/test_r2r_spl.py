# Copyright 2022 Kenji Brameld
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

from r2r_spl_7.r2r_spl import R2RSPL

import rclpy
from rclpy.parameter import Parameter

from splsm_7.msg import SPLSM

from splsm_7_conversion.spl_standard_message import SPLStandardMessage


class TestR2RSPL:
    """Tests against R2RSPL."""

    received = None

    team_num = 1
    player_num = 1

    parameter_overrides = [
        Parameter('team_num', value=team_num),
        Parameter('player_num', value=player_num)]

    def _callback_msg(self, msg):
        self.received = msg

    def test_receiving(self):
        """Test receiving UDP package from teammate."""
        # Setup nodes
        rclpy.init()
        r2r_spl_node = R2RSPL(    # noqa: F841
          parameter_overrides=self.parameter_overrides)
        test_node = rclpy.node.Node('test')
        subscription = test_node.create_subscription(  # noqa: F841
            SPLSM, 'r2r/recv', self._callback_msg, 10)

        # Example message from player 2 on same team
        msg = SPLStandardMessage.build(
            Container(playerNum=2, teamNum=self.team_num))

        # UDP - adapted from https://github.com/ninedraft/python-udp/blob/master/server.py
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.sendto(msg, ('', 10000 + self.team_num))

        # Wait for R2RSPL to receive packet over UDP, and publish a ROS msg
        time.sleep(0.1)

        # Check if message has been received on topic
        rclpy.spin_once(test_node, timeout_sec=0)
        assert self.received is not None

        # Shutdown
        rclpy.shutdown()

    def test_receiving_myself(self):
        """Test ignoring UDP package sent from myself."""
        # Setup nodes
        rclpy.init()
        r2r_spl_node = R2RSPL(    # noqa: F841
          parameter_overrides=self.parameter_overrides)
        test_node = rclpy.node.Node('test')
        subscription = test_node.create_subscription(  # noqa: F841
            SPLSM, 'r2r/recv', self._callback_msg, 10)

        # Example message from player 2 on same team
        msg = SPLStandardMessage.build(
            Container(playerNum=self.player_num, teamNum=self.team_num))

        # UDP - adapted from https://github.com/ninedraft/python-udp/blob/master/server.py
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.sendto(msg, ('', 10000 + self.team_num))

        # Wait for R2RSPL to receive packet over UDP, and publish any ROS msgs
        time.sleep(0.1)

        # Make sure message was ignored and not published on topic
        rclpy.spin_once(test_node, timeout_sec=0)
        assert self.received is None

        # Shutdown
        rclpy.shutdown()

    def test_sending(self):
        """Test sending UDP package to teammate."""
        # Setup nodes
        rclpy.init()
        r2r_spl_node = R2RSPL(    # noqa: F841
          parameter_overrides=self.parameter_overrides)
        test_node = rclpy.node.Node('test')
        publisher = test_node.create_publisher(SPLSM, 'r2r/send', 10)

        # UDP - adapted from https://github.com/ninedraft/python-udp/blob/master/server.py
        sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind(('', 10000 + self.team_num))
        sock.settimeout(0.1)

        # Publish SPLSM to r2r_spl_node
        publisher.publish(SPLSM())

        # Wait before spinning for the msg arrive in r2r_spl_node's subscription
        time.sleep(0.01)

        # Spin r2r_spl_node to process incoming message and send out UDP message
        rclpy.spin_once(r2r_spl_node, timeout_sec=0)

        # Check if packet has arrived
        try:
            _ = sock.recv(1024)
        except TimeoutError:
            assert False

        # Shutdown
        rclpy.shutdown()
