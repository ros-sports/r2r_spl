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

import construct
from game_controller_spl_interfaces.msg import RCGCD15
from r2r_spl.exceptions import ParameterNotSetException
from r2r_spl.serialization import Serialization
import rclpy
from rclpy.node import Node

MAX_ALLOWED_MSG_SIZE = 128


class R2RSPL(Node):
    """Node that runs on the robot to communicate with teammates (Robot-To-Robot) in SPL."""

    _loop_thread = None
    _sock = None
    _publisher = None
    _team_num = None
    _budget_reached = False

    def __init__(self, node_name='r2r_spl', **kwargs):
        super().__init__(node_name, **kwargs)

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('team_num', 0),
                ('player_num', 0),
                ('msg_type', ''),
                ('filter_own', False),
            ]
        )

        # Read and log parameters
        self._team_num = self.get_parameter('team_num').value
        self.get_logger().debug('team_num: {}'.format(self._team_num))
        if self._team_num == 0:
            self.get_logger().warn('"team_num" parameter is 0. This is problematic in a game.')

        self.player_num = self.get_parameter('player_num').value
        self.get_logger().debug('player_num: {}'.format(self.player_num))
        if self.player_num == 0:
            self.get_logger().warn('"player_num" parameter is 0. This is problematic in a game.')

        self.msg_type = self.get_parameter('msg_type').value
        self.get_logger().debug('msg_type: {}'.format(self.msg_type))

        self.filter_own = self.get_parameter('filter_own').value
        self.get_logger().debug('filter_own: {}'.format(self.filter_own))

        # Setup subscriber that listens to message budget
        self._subscriber_rcgcd = self.create_subscription(
            RCGCD15, 'gc/data', self._rcgcd_callback, 10)

        # Evalulate and import message type
        if self.msg_type == '':
            raise ParameterNotSetException('"msg_type" parameter must be set.')
        index_last_dot = self.msg_type.rfind('.')
        assert index_last_dot != -1, \
            f'msg_type must be in the form "package_name.<namespace>.MsgName" ' \
            f'(eg. geometry_msgs.msg.PoseStamped). Got: {self.msg_type}'
        assert index_last_dot != len(self.msg_type) - 1, \
            f'msg_type must be in the form "package_name.<namespace>.MsgName" ' \
            f'(eg. geometry_msgs.msg.PoseStamped). Got: {self.msg_type}'
        class_name = self.msg_type[index_last_dot + 1:]
        mod = __import__(self.msg_type[:index_last_dot], fromlist=[class_name])
        msg_class = getattr(mod, class_name)

        # Setup serialization
        self._serialization = Serialization(
            msg_class, player_num=self.player_num if self.filter_own else None)

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
        self._sock.bind(('', 10000 + self._team_num))
        # Set timeout so _loop can constantly check for rclpy.ok()
        self._sock.settimeout(0.1)

        # Start thread to continuously poll
        self._loop_thread = Thread(target=self._loop)
        self._loop_thread.start()

    def _loop(self):
        while rclpy.ok():
            try:
                data, _ = self._sock.recvfrom(1024)
                self.get_logger().debug(f'received: {data}')

                # Convert data to ROS msg
                try:
                    msg = self._serialization.deserialize(data)

                    if msg:
                        # Publish. Make sure we haven't shutdown, since node could have been
                        # shutdown while waiting for the packet.
                        if rclpy.ok():
                            self._publisher.publish(msg)
                    else:
                        # Reaches here if we filtered out our own message
                        # (only if filtering is enabled via filter_own parameter)
                        pass

                except construct.core.StreamError:
                    # Deserialization failed
                    self.get_logger().error(
                        f'deserialization failed, please ensure other robots are using the '
                        f'matching message type {self.msg_type}', once=True)

            except TimeoutError:
                pass

    def _topic_callback(self, msg):
        if not self._budget_reached:
            data = self._serialization.serialize(msg)

            if len(data) > MAX_ALLOWED_MSG_SIZE:
                self.get_logger().error(
                    f'Cannot send message of size {len(data)} bytes. Maximum size is 128 bytes.')
            else:
                # Broadcast data on team's UDP port
                self._sock.sendto(data, ('<broadcast>', 10000 + self._team_num))

    def _rcgcd_callback(self, msg):
        team_found = False
        for team in msg.teams:
            if team.team_number == self._team_num:
                team_found = True

                if not self._budget_reached and team.message_budget < 10:
                    self.get_logger().info('Budget almost reached, not sending anymore messages')
                    self._budget_reached = True
                elif self._budget_reached and team.message_budget > 10:
                    self.get_logger().info('Extra budget available, sending messages again')
                    self._budget_reached = False

        if not team_found:
            self.get_logger().warn(
                f'Received messages from Game Controller about teams {msg.teams[0].team_number} '
                f'and {msg.teams[1].team_number}, but team_num parameter is {self._team_num}. '
                f'This is problematic if in a game.', once=True)


def main(args=None):
    rclpy.init(args=args)
    r2r_spl = R2RSPL()
    rclpy.spin(r2r_spl)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
