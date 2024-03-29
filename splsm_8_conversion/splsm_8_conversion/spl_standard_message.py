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

from construct import Array, Byte, Const, Default, Float32l, Int16ul, Struct

SPL_STANDARD_MESSAGE_STRUCT_HEADER = b'SPL '
SPL_STANDARD_MESSAGE_STRUCT_VERSION = 8

SPL_STANDARD_MESSAGE_DATA_SIZE = 128

# Important remarks about units:
#   For each parameter, the respective comments describe its unit.
#   The following units are used:
#     - Distances:  Millimeters (mm)
#     - Angles:     Radian
#     - Time:       Seconds (s)
SPLStandardMessage = Struct(
    'header' / Const(SPL_STANDARD_MESSAGE_STRUCT_HEADER),  # "SPL "
    'version' / Const(SPL_STANDARD_MESSAGE_STRUCT_VERSION, Byte),  # has to be set to SPL_STANDARD_MESSAGE_STRUCT_VERSION  # noqa: E501
    'playerNum' / Default(Byte, 0),  # [MANDATORY FIELD] 1-7
    'teamNum' / Default(Byte, 0),  # [MANDATORY FIELD] the number of the team (as provided by the organizers)  # noqa: E501
    'fallen' / Default(Byte, 255),  # [MANDATORY FIELD] 1 means that the robot is fallen, 0 means that the robot can play  # noqa: E501
    # [MANDATORY FIELD]
    # position and orientation of robot
    # coordinates in millimeters
    # 0,0 is in center of field
    # +ve x-axis points towards the goal we are attempting to score on
    # +ve y-axis is 90 degrees counter clockwise from the +ve x-axis
    # angle in radians, 0 along the +x axis, increasing counter clockwise
    'pose' / Default(Array(3, Float32l), [0, 0, 0]),  # x,y,theta
    # ball information
    'ballAge' / Default(Float32l, -1),  # seconds since this robot last saw the ball. -1.f if we haven't seen it  # noqa: E501
    # position of ball relative to the robot
    # coordinates in millimeters
    # 0,0 is in center of the robot
    # +ve x-axis points forward from the robot
    # +ve y-axis is 90 degrees counter clockwise from the +ve x-axis
    'ball' / Default(Array(2, Float32l), [0, 0]),
    # number of bytes that is actually used by the data array
    'numOfDataBytes' / Default(Int16ul, 0),
    # buffer for arbitrary data, teams do not need to send more than specified in numOfDataBytes
    'data' / Default(Array(SPL_STANDARD_MESSAGE_DATA_SIZE, Byte), [0]*SPL_STANDARD_MESSAGE_DATA_SIZE)  # noqa: E501
)
