"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
the code is used to define the soft mode status of the robotic arm
"""

from enum import Enum


class ControllerStatusEnum(Enum):
    """Controller status code enum"""

    CTRL_INIT = (0, "cn:控制器初始化|en:Controller init")
    CTRL_ENGAGED = (1, "cn:控制器使能|en:Controller engaged")
    CTRL_ESTOP = (2, "cn:控制器急停|en:Controller estop")
    CTRL_TERMINATED = (3, "cn:控制器中止|en:Controller terminated")
    CTRL_ANY_TO_ESTOP = (
        101,
        "cn:控制器中间状态 其他转换为急停|en:Controller any to estop",
    )
    CTRL_ESTOP_TO_ENGAGED = (
        102,
        "cn:控制器中间状态 急停到使能|en:Controller estop to engaged",
    )
    CTRL_ESTOP_TO_TERMINATED = (
        103,
        "cn:控制器中间状态 急停到中止|en:Controller estop to terminated",
    )
    CTRL_UNKNOWN = (-1, "cn:未知的控制器状态|en:Unknown controller status")

    @property
    def code(self):
        """attribute code"""
        return self.value[0]

    @property
    def errmsg_cn(self):
        """get status code description(Chinese)"""
        return self.value[1].split("|")[0].replace("cn:", "")

    @property
    def errmsg_en(self):
        """get status code description(English)"""
        return self.value[1].split("|")[1].replace("en:", "")

    @classmethod
    def from_id(cls, id):
        """get enum item by code"""
        for item in cls:
            if item.code == id:
                return item
        return None
