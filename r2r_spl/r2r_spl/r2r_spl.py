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
from threading import Thread

import rclpy
from rclpy.node import Node

from r2r_spl.serialization import Serialization

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
                ('player_num', 0),
                ('msg_type', ''),
            ]
        )

        # Read and log parameters
        self.team_num = self.get_parameter('team_num').value
        self.get_logger().debug('team_num: {}'.format(self.team_num))

        self.player_num = self.get_parameter('player_num').value
        self.get_logger().debug('player_num: {}'.format(self.player_num))

        self.msg_type = self.get_parameter('msg_type').value
        self.get_logger().debug('msg_type: {}'.format(self.msg_type))

        # Evalulate and import message type
        index_last_dot = self.msg_type.rfind('.')
        assert index_last_dot != -1, f'msg_type must be in the form "package_name.<namespace>.MsgName" (eg. geometry_msgs.msg.PoseStamped). Got: {self.msg_type}'
        assert index_last_dot != len(self.msg_type) - 1, f'msg_type must be in the form "package_name.<namespace>.MsgName" (eg. geometry_msgs.msg.PoseStamped). Got: {self.msg_type}'

        class_name = self.msg_type[index_last_dot + 1:]
        mod = __import__(self.msg_type[:index_last_dot], fromlist=[class_name])
        msg_class = getattr(mod, class_name)

        # Setup serialization
        self._serialization = Serialization(msg_class)

        # Setup publisher
        self._publisher = self.create_publisher(msg_class, 'r2r/recv', 10)

        # Setup subscriber
        self._subscriber = self.create_subscription(
            msg_class, 'r2r/send', self._topic_callback, 10)

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
                msg = self._serialization.deserialize(data)

                # Publish
                self._publisher.publish(msg)
            except TimeoutError:
                pass

    def _topic_callback(self, msg):

        data = self._serialization.serialize(msg)

        # Broadcast data on team's UDP port
        self._sock.sendto(data, ('', 10000 + self.team_num))


def main(args=None):
    rclpy.init(args=args)
    r2r_spl = R2RSPL()
    rclpy.spin(r2r_spl)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
