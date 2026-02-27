"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
the code is used to define the robot status enum
"""

from enum import Enum


class RobotStatusEnum(Enum):
    ROBOT_IDLE = 0, "cn:机器人空闲|en:Robot idle"
    ROBOT_RUNNING = 1, "cn:机器人运行中|en:Robot running"
    ROBOT_TEACHING = 2, "cn:机器人示教中|en:Robot teaching"
    ROBOT_IDLE_TO_RUNNING = (
        101,
        "cn:机器人中间状态 空闲转换为运行|en:Robot intermediate state from idle to running",
    )
    ROBOT_IDLE_TO_TEACHING = (
        102,
        "cn:机器人中间状态 空闲转换为示教|en:Robot intermediate state from idle to teaching",
    )
    ROBOT_RUNNING_TO_IDLE = (
        103,
        "cn:机器人中间状态 运行转换为空闲|en:Robot intermediate state from running to idle",
    )
    ROBOT_TEACHING_TO_IDLE = (
        104,
        "cn:机器人中间状态 示教转换为空闲|en:Robot intermediate state from teaching to idle",
    )
    ROBOT_UNKNOWN = -1, "cn:机器人状态未知|en:Robot_UNKNOWN"

    @property
    def code(self):

        return self.value[0]

    @property
    def errmsg_cn(self):
        return self.value[1].split("|")[0].replace("cn:", "")

    @property
    def errmsg_en(self):
        return self.value[1].split("|")[1].replace("en:", "")

    @classmethod
    def from_id(cls, id):
        """get enum item by id"""
        for item in cls:
            if item.code == id:
                return item
        return None
