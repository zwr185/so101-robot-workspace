#!/usr/bin/env python3
"""启动时把 world 里的场景物体写入 MoveIt 的 Planning Scene。

木板、障碍物和小方块只存在于 Gazebo 的 world 中，MoveIt 默认看不到它们。
这个节点在 move_group 就绪后调用 /apply_planning_scene，把同一批几何体作为
CollisionObject 加进规划场景，让规划阶段能检测到它们。

注意：target_block 在 Gazebo 里是动态物体，进入 Planning Scene 后只是一个
静态的碰撞几何。真正夹取时需要把它从场景中移除，或者作为 AttachedCollisionObject
附加到夹爪上，否则夹爪会被它挡住。
"""

import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA

# 与 worlds/so101_empty.sdf 中的物体一一对应。
# 木板顶面在规划场景里降到 0.095，给底座碰撞网格（最低点约 0.0976）留余量，
# 避免加进去就报起始碰撞。
OBJECTS = [
    {
        "id": "wood_board",
        "size": (2.0, 2.0, 0.095),
        "pose": (0.0, 0.0, 0.0475),
        "color": (0.5625, 0.325, 0.125, 1.0),
    },
    {
        "id": "obstacle_box",
        "size": (0.06, 0.18, 0.10),
        "pose": (0.18, 0.0, 0.15),
        "color": (0.5625, 0.325, 0.125, 1.0),
    },
    {
        "id": "target_block",
        "size": (0.04, 0.04, 0.05),
        "pose": (0.30, 0.0, 0.125),
        "color": (0.1, 0.2, 0.8, 1.0),
    },
]


def make_box(spec):
    """按配置生成一个 CollisionObject 及对应的颜色。"""
    box = CollisionObject()
    # planning frame 是 URDF 的根节点 world
    box.header.frame_id = "world"
    box.id = spec["id"]
    box.operation = CollisionObject.ADD

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(spec["size"])
    box.primitives.append(primitive)

    pose = Pose()
    pose.position.x = spec["pose"][0]
    pose.position.y = spec["pose"][1]
    pose.position.z = spec["pose"][2]
    pose.orientation.w = 1.0
    box.primitive_poses.append(pose)

    color = ObjectColor()
    color.id = spec["id"]
    color.color = ColorRGBA(
        r=spec["color"][0],
        g=spec["color"][1],
        b=spec["color"][2],
        a=spec["color"][3],
    )
    return box, color


class ApplySceneObjects(Node):
    def __init__(self):
        super().__init__("apply_scene_objects")
        # 抓取演示时把它设为 false：不把待夹取方块写入规划场景，
        # 否则夹爪无法规划到方块所在位置。
        self.declare_parameter("include_target_block", True)
        self.client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )

    def apply(self) -> bool:
        if not self.client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error(
                "等待 /apply_planning_scene 服务超时，move_group 是否已启动？"
            )
            return False

        include_block = bool(
            self.get_parameter("include_target_block").value
        )
        specs = [
            spec
            for spec in OBJECTS
            if include_block or spec["id"] != "target_block"
        ]

        scene = PlanningScene()
        scene.is_diff = True

        for spec in specs:
            box, color = make_box(spec)
            scene.world.collision_objects.append(box)
            scene.object_colors.append(color)

        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        result = future.result()
        if result is None:
            self.get_logger().error("应用 Planning Scene 超时")
            return False
        if not result.success:
            self.get_logger().error("move_group 拒绝应用 Planning Scene")
            return False

        names = ", ".join(spec["id"] for spec in specs)
        self.get_logger().info(f"已加入 Planning Scene：{names}")
        return True


def main():
    rclpy.init()
    node = ApplySceneObjects()
    try:
        ok = node.apply()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
