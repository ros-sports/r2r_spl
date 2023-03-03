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
from threading import Thread

import rclpy
from rclpy.node import Node

from splsm_8.msg import SPLSM
from splsm_8_conversion.conversion import splsm_data_to_msg, splsm_msg_to_data


class R2RSPL(Node):
    """Node that runs on the robot to communicate with teammates (Robot-To-Robot) in SPL."""

    _loop_thread = None
    _sock = None
    _publisher = None

    def __init__(self, node_name='r2r_spl', **kwargs):
        super().__init__(node_name, **kwargs)

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('team_num', 0),
                ('player_num', 0)
            ]
        )

        # Read and log parameters
        self.team_num = self.get_parameter('team_num').value
        self.get_logger().debug('team_num: {}'.format(self.team_num))

        self.player_num = self.get_parameter('player_num').value
        self.get_logger().debug('player_num: {}'.format(self.player_num))

        # Setup publisher
        self._publisher = self.create_publisher(SPLSM, 'r2r/recv', 10)

        # Setup subscriber
        self._subscriber = self.create_subscription(
            SPLSM, 'r2r/send', self._topic_callback, 10)

        # UDP Client - adapted from https://github.com/ninedraft/python-udp/blob/master/client.py
        self._sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
        # This has to be SO_REUSEADDR instead of SO_REUSEPORT to work with TCM
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._sock.bind(('', 10000 + self.team_num))
        # Set timeout so _loop can constantly check for rclpy.ok()
        self._sock.settimeout(0.1)

        # Start thread to continuously poll
        self._loop_thread = Thread(target=self._loop)
        self._loop_thread.start()

    def _loop(self):
        while rclpy.ok():
            try:
                data, _ = self._sock.recvfrom(1024)
                self.get_logger().debug('received: "%s"' % data)

                # Convert data to ROS msg
                msg = splsm_data_to_msg(data)

                # Publish it if its not from myself
                if msg.player_num != self.player_num:
                    self._publisher.publish(msg)
            except TimeoutError:
                pass

    def _topic_callback(self, msg):

        data = splsm_msg_to_data(msg)

        # Broadcast data on team's UDP port
        self._sock.sendto(data, ('', 10000 + self.team_num))


def main(args=None):
    rclpy.init(args=args)
    r2r_spl = R2RSPL()
    rclpy.spin(r2r_spl)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
