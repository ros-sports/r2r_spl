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

from construct import Container

from splsm_7.msg import SPLSM

from splsm_7_conversion.spl_standard_message import SPLStandardMessage


def splsm_msg_to_data(msg: SPLSM) -> bytes:
    """Convert SPLSM ROS msg to binary data."""
    container = Container(
        playerNum=msg.player_num,
        teamNum=msg.team_num,
        fallen=msg.fallen,
        pose=msg.pose,
        ballAge=msg.ball_age,
        ball=msg.ball,
        numOfDataBytes=msg.num_of_data_bytes,
        data=msg.data
    )
    data = SPLStandardMessage.build(container)
    return data


def splsm_data_to_msg(data: bytes) -> SPLSM:
    """Convert binary data to SPLSM ROS msg."""
    parsed = SPLStandardMessage.parse(data)
    msg = SPLSM()
    msg.player_num = parsed.playerNum
    msg.team_num = parsed.teamNum
    msg.fallen = parsed.fallen
    msg.pose = parsed.pose
    msg.ball_age = parsed.ballAge
    msg.ball = parsed.ball
    msg.num_of_data_bytes = parsed.numOfDataBytes
    msg.data = parsed.data
    return msg
