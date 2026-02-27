"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
the code is used to define the soft mode status of the robotic arm
"""

from enum import Enum


class SoftModeEnum(Enum):
    """Soft mode status of the robotic arm"""

    UNKNOWN = (0, "cn:未知模式|en:Unknown Mode")
    AUTO = (1, "cn:自动模式|en:Auto Mode")  # Auto mode
    MANUAL_LIMIT = (
        2,
        "cn:手动限速模式|en:Manual Limit Mode",
    )  # Manual limited speed mode
    MANUAL = (3, "cn:手动模式|en:Manual Mode")  # Manual mode

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
        """Get the enum instance by id"""
        for item in cls:
            if item.code == id:
                return item
        return None
