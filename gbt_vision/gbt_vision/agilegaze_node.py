"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
 AgileGaze_node.py is a node for calling the AgileGaze service. AgileGaze is the vision processing software of Agilebot Robotics Ltd., please contact for details.
"""

import json
import math
import socket

import rclpy
from gbt_stacking_interface.msg import AgileGaze, VRItem
from gbt_stacking_interface.srv import GetAgileGaze
from rclpy.node import Node

# fake data
uf1_fake_pose = [
    [20.0, 5.1510000000000105, 0.0, 180, 0, 45],
    [-180.0, 125.0, 0.0, 180, 0, 0],
    [120.0, -75.0, 0.0, 180, 0, 45],
    [120.0, -295.0, 0.0, 180, 0, -45],
]
vr_list = [
    {
        "model_id": 0,
        "coordinate_type": 3,
        "coordinate_id": 0,
        "x": uf1_fake_pose[i][0],
        "y": uf1_fake_pose[i][1],
        "c": math.radians(uf1_fake_pose[i][-1]),
    }
    for i in range(len(uf1_fake_pose))
]
# print(vr_list)
fake_AgileGaze = {
    "code": 0,
    "message": "OK",
    "process_name": "a1",
    "quantity": len(vr_list),
    "vr_list": vr_list,
}


class AgileGazeServiceNode(Node):
    def __init__(self):
        super().__init__("AgileGaze_service_node")
        # declare parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("host", "172.17.26.57"),
                ("port", 5622),
                ("cmd", "RUN_FIND, a1\n"),
                ("srv_name", "/gbt_vision/service/AgileGaze"),
                ("fake", True),
            ],
        )
        self.host = self.get_parameter("host").value
        self.port = int(self.get_parameter("port").value)
        cmd = self.get_parameter("cmd").value
        # replace \\n to \n
        cmd = cmd.replace("\\n", "\n")
        if not cmd.endswith("\n"):
            cmd += "\n"
        self.cmd = cmd
        self.fake = self.get_parameter("fake").value

        # create service
        self.srv = self.create_service(
            GetAgileGaze,
            self.get_parameter("srv_name").value,
            self.handle_get_AgileGaze,
        )
        mode = "fake" if self.fake else f"real ({self.host}:{self.port})"
        self.get_logger().info(
            f"Service '{self.get_parameter('srv_name').value}' ready in {mode} mode."
        )

    def handle_get_AgileGaze(self, request, response):
        # if fake, return fake data
        if self.fake:
            data = fake_AgileGaze
        else:
            try:
                raw = self.send_run_find(self.host, self.port, self.cmd)
                parsed = json.loads(raw)
                data = parsed.get("data", {})
                code = parsed.get("code", -1)
                message = parsed.get("message", "")
            except Exception as e:
                self.get_logger().error(f"Socket or JSON error: {e}")
                response.agile_gaze.code = -1
                response.agile_gaze.message = f"Error: {e}"
                return response

        vp = AgileGaze()
        if self.fake:
            vp.code = data.get("code", -1)
            vp.message = data.get("message", "")
            vp.process_name = data.get("process_name", "")
            vp.quantity = data.get("quantity", 0)
            vr_items = data.get("vr_list", [])
        else:
            vp.code = code
            vp.message = message
            vp.process_name = data.get("process_name", "")
            vp.quantity = data.get("quantity", 0)
            vr_items = data.get("vr_list", [])

        for item in vr_items:
            vr = VRItem()
            vr.model_id = item.get("model_id", 0)
            vr.coordinate_type = item.get("coordinate_type", 0)
            vr.coordinate_id = item.get("coordinate_id", 0)
            vr.x = item.get("x", 0.0)
            vr.y = item.get("y", 0.0)
            vr.c = item.get("c", 0.0)
            self.get_logger().info(f"[DEBUG] VRItem raw c = {item.get('c')}")

            vp.vr_list.append(vr)

        self.get_logger().info(f"return AgileGaze msg: {vp}")
        response.agile_gaze = vp
        self.get_logger().info(
            f"Returning AgileGaze: code={vp.code}, quantity={vp.quantity}"
        )
        return response

    def send_run_find(self, host: str, port: int, message: str) -> str:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(1.0)
            sock.connect((host, port))
            sock.sendall(message.encode("utf-8"))
            resp = sock.recv(8192)
        return resp.decode("utf-8")


def main(args=None):
    rclpy.init(args=args)
    node = AgileGazeServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
