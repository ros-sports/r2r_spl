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

from splsm_8.msg import SPLSM

from splsm_8_conversion.conversion import splsm_data_to_msg, splsm_msg_to_data
from splsm_8_conversion.spl_standard_message import \
    SPL_STANDARD_MESSAGE_DATA_SIZE, SPLStandardMessage


def test_splsm_msg_to_data():
    """Test conversion of SPLSM msg to binary data."""
    msg = SPLSM()
    msg.player_num = 1
    msg.team_num = 2
    msg.fallen = 1
    msg.pose = [1.0, 2.0, 3.0]
    msg.ball_age = 20.0
    msg.ball = [10.0, 20.0]
    msg.data = [0, 1, 255]
    data = splsm_msg_to_data(msg)

    parsed = SPLStandardMessage.parse(data)
    assert parsed.playerNum == 1
    assert parsed.teamNum == 2
    assert parsed.fallen == 1
    assert parsed.pose == [1.0, 2.0, 3.0]
    assert parsed.ballAge == 20.0
    assert parsed.ball == [10.0, 20.0]
    assert parsed.numOfDataBytes == 3
    assert parsed.data[0] == 0
    assert parsed.data[1] == 1
    assert parsed.data[2] == 255


def test_splsm_data_to_msg():
    """Test conversion of binary data to SPLSM msg."""
    # Fill data array with my data
    data = [0, 1, 255]
    data_ = [0] * SPL_STANDARD_MESSAGE_DATA_SIZE
    for idx, item in enumerate(data):
        data_[idx] = item

    splsm_data = SPLStandardMessage.build(
        Container(
            playerNum=1,
            teamNum=2,
            fallen=1,
            pose=[1.0, 2.0, 3.0],
            ballAge=20.0,
            ball=[10.0, 20.0],
            numOfDataBytes=len(data),
            data=data_,
        )
    )

    msg = splsm_data_to_msg(splsm_data)
    assert msg.player_num == 1
    assert msg.team_num == 2
    assert msg.fallen == 1
    assert (msg.pose == [1.0, 2.0, 3.0]).all()  # Check numpy array
    assert msg.ball_age == 20.0
    assert (msg.ball == [10.0, 20.0]).all()  # Check numpy array
    assert len(msg.data) == len(data)
    for a, b in zip(msg.data, data):
        assert a == b
