"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
the code is used to define the robot signal type
"""

from enum import Enum


class SignalType(Enum):
    SIGNAL_TYPE_DI = 1
    SIGNAL_TYPE_DO = 2
    SIGNAL_TYPE_UI = 3
    SIGNAL_TYPE_UO = 4
    SIGNAL_TYPE_RI = 5
    SIGNAL_TYPE_RO = 6
    SIGNAL_TYPE_GI = 7
    SIGNAL_TYPE_GO = 8
