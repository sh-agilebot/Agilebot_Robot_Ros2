"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
the code is used to define the status of the servo controller
"""

from enum import Enum


class ServoStatusEnum(Enum):
    """Controller status enum type"""

    SERVO_IDLE = (1, "cn:伺服控制器空闲|en:Servo controller idle")
    SERVO_RUNNING = (2, "cn:伺服控制器运行中|en:Servo controller running")
    SERVO_DISABLE = (3, "cn:伺服控制器关闭|en:Servo controller disable")
    SERVO_WAIT_READY = (4, "cn:伺服控制器等待就绪|en:Servo controller wait ready")
    SERVO_WAIT_DOWN = (5, "cn:伺服控制器等待关闭|en:Servo controller wait down")
    SERVO_INIT = (10, "cn:伺服控制器初始化|en:Servo controller init")
    SERVO_UNKNOWN = (-1, "cn:未知的伺服控制器状态|en:Unknown servo controller status")

    @property
    def code(self):
        return self.value[0]

    @property
    def errmsg_cn(self):
        """get status code description(Chinese)"""
        return self.value[1].split("|")[0].replace("cn:", "")

    @property
    def errmsg_en(self):
        """get status code description"""
        return self.value[1].split("|")[1].replace("en:", "")

    @classmethod
    def from_id(cls, id):
        """get item by id"""
        for item in cls:
            if item.code == id:
                return item
        return None
