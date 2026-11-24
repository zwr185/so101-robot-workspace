#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""URDF 正运动学、位置反解和工作空间采样。

公开 API 的长度单位是 cm，角度单位是度。
"""

import math
import xml.etree.ElementTree as ET

import numpy as np

MOVABLE_JOINT_TYPES = ("revolute", "continuous", "prismatic")


def _vector(text, default=(0.0, 0.0, 0.0)):
    if not text:
        return np.array(default, dtype=float)
    return np.array([float(value) for value in text.split()], dtype=float)


def translation(vector):
    matrix = np.eye(4)
    matrix[:3, 3] = vector
    return matrix


def rpy_to_matrix(rpy_deg):
    roll, pitch, yaw = np.radians(rpy_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rot_x = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    rot_y = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rot_z = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rot_z @ rot_y @ rot_x


def transform(xyz_cm, rpy_deg):
    matrix = np.eye(4)
    matrix[:3, :3] = rpy_to_matrix(rpy_deg)
    matrix[:3, 3] = xyz_cm
    return matrix


def axis_rotation(axis, angle_deg):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm == 0.0:
        return np.eye(4)

    angle = math.radians(angle_deg)
    x, y, z = axis / norm
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    one_c = 1.0 - cos_a

    matrix = np.eye(4)
    matrix[:3, :3] = np.array(
        [
            [x * x * one_c + cos_a, x * y * one_c - z * sin_a,
             x * z * one_c + y * sin_a],
            [y * x * one_c + z * sin_a, y * y * one_c + cos_a,
             y * z * one_c - x * sin_a],
            [z * x * one_c - y * sin_a, z * y * one_c + x * sin_a,
             z * z * one_c + cos_a],
        ]
    )
    return matrix


class Joint:
    """一个 URDF 关节；内部统一使用 cm 和度。"""

    def __init__(self, element):
        self.name = element.get("name")
        self.type = element.get("type")

        origin = element.find("origin")
        xyz_m = _vector(origin.get("xyz") if origin is not None else None)
        rpy_rad = _vector(origin.get("rpy") if origin is not None else None)
        self.xyz = xyz_m * 100.0
        self.rpy = np.degrees(rpy_rad)

        self.parent = element.find("parent").get("link")
        self.child = element.find("child").get("link")

        axis = element.find("axis")
        self.axis = _vector(
            axis.get("xyz") if axis is not None else None, (1.0, 0.0, 0.0)
        )

        limit = element.find("limit")
        self.lower = None
        self.upper = None
        if limit is not None and limit.get("lower") is not None:
            self.lower = math.degrees(float(limit.get("lower")))
            self.upper = math.degrees(float(limit.get("upper")))

    def origin_matrix(self):
        return transform(self.xyz, self.rpy)


class UrdfModel:
    """URDF 关节树，支持正解、目标链查询和工作空间采样。"""

    def __init__(self, root):
        self.joints = {}
        for element in root.findall("joint"):
            joint = Joint(element)
            self.joints[joint.name] = joint

        self.child_to_joint = {
            joint.child: joint for joint in self.joints.values()
        }

        links = [element.get("name") for element in root.findall("link")]
        self.base_link = next(
            (link for link in links if link not in self.child_to_joint), None
        )
        if self.base_link is None:
            raise ValueError("无法确定基座链接，URDF 中可能存在闭环")

    @classmethod
    def from_string(cls, urdf_xml):
        return cls(ET.fromstring(urdf_xml))

    @classmethod
    def from_file(cls, path):
        with open(path, "r", encoding="utf-8") as urdf_file:
            text = urdf_file.read()

        if "xacro:" in text or "${" in text:
            import xacro

            root = xacro.process_file(path).getroot()
        else:
            root = ET.fromstring(text)
        return cls(root)

    def chain(self, target_link, base_link=None):
        base_link = base_link or self.base_link
        joints = []
        link = target_link
        while link != base_link:
            joint = self.child_to_joint.get(link)
            if joint is None:
                raise ValueError(
                    f"找不到从 {base_link} 到 {target_link} 的运动链，"
                    f"当前链接: {link}"
                )
            joints.append(joint)
            link = joint.parent
        joints.reverse()
        return joints

    def movable_joint_names(self, target_link, base_link=None):
        return [
            joint.name
            for joint in self.chain(target_link, base_link)
            if joint.type in MOVABLE_JOINT_TYPES
        ]

    def forward_kinematics(
        self, joint_values=None, target_link="gripper_frame_link",
        base_link=None,
    ):
        joint_values = joint_values or {}
        matrix = np.eye(4)
        for joint in self.chain(target_link, base_link):
            matrix = matrix @ joint.origin_matrix()
            value = float(joint_values.get(joint.name, 0.0))

            if joint.type in ("revolute", "continuous"):
                matrix = matrix @ axis_rotation(joint.axis, value)
            elif joint.type == "prismatic":
                matrix = matrix @ translation(joint.axis * value)
        return matrix


def joint_limits(model, names):
    lower = []
    upper = []
    for name in names:
        joint = model.joints[name]
        lower.append(-180.0 if joint.lower is None else joint.lower)
        upper.append(180.0 if joint.upper is None else joint.upper)
    return np.array(lower, dtype=float), np.array(upper, dtype=float)


def sample_workspace(
    model, target_link="gripper_frame_link", samples=1200, seed=0,
):
    """随机采样关节空间，返回 target_link 的三维位置点（cm）。"""
    names = model.movable_joint_names(target_link)
    if not names:
        return np.empty((0, 3))

    lower, upper = joint_limits(model, names)
    rng = np.random.default_rng(seed)
    points = np.empty((samples, 3), dtype=float)

    for index in range(samples):
        values = {
            name: float(rng.uniform(low, high))
            for name, low, high in zip(names, lower, upper)
        }
        points[index] = model.forward_kinematics(
            values, target_link
        )[:3, 3]

    return points


def _position_m(model, values_rad, names, target_link):
    values_deg = {
        name: math.degrees(float(value))
        for name, value in zip(names, values_rad)
    }
    return model.forward_kinematics(values_deg, target_link)[:3, 3] / 100.0


def _jacobian_m(model, values_rad, names, target_link, epsilon=1e-4):
    base = _position_m(model, values_rad, names, target_link)
    jacobian = np.zeros((3, len(names)))
    for index in range(len(names)):
        perturbed = values_rad.copy()
        perturbed[index] += epsilon
        jacobian[:, index] = (
            _position_m(model, perturbed, names, target_link) - base
        ) / epsilon
    return jacobian


def _solve_once(
    model, target_m, names, lower, upper, initial, target_link,
    max_iterations, tolerance_m, damping, step_limit,
):
    values = np.clip(initial, lower, upper)

    for _ in range(max_iterations):
        difference = target_m - _position_m(model, values, names, target_link)
        if np.linalg.norm(difference) <= tolerance_m:
            break

        jacobian = _jacobian_m(model, values, names, target_link)
        matrix = jacobian @ jacobian.T + (damping ** 2) * np.eye(3)
        step = jacobian.T @ np.linalg.solve(matrix, difference)
        step = np.clip(step, -step_limit, step_limit)
        values = np.clip(values + step, lower, upper)

    error_m = float(
        np.linalg.norm(
            target_m - _position_m(model, values, names, target_link)
        )
    )
    return values, error_m


def inverse_kinematics(
    model, target_position_cm, target_link="gripper_frame_link",
    initial=None, max_iterations=300, tolerance_cm=0.5,
    damping=0.05, step_limit_deg=17.0, restarts=10, seed=0,
):
    """位置反解，返回 (关节角度字典, 位置误差 cm)。

    这里只约束末端位置，不约束姿态；对于 SO-101 的 5 个臂关节，
    位置目标通常有多个解，求解器会从多个随机种子中选误差最小的。
    """
    names = model.movable_joint_names(target_link)
    if not names:
        raise ValueError(f"{target_link} 的运动链上没有可动关节")

    lower_deg, upper_deg = joint_limits(model, names)
    lower = np.radians(lower_deg)
    upper = np.radians(upper_deg)
    target_m = np.asarray(target_position_cm, dtype=float) / 100.0
    tolerance_m = float(tolerance_cm) / 100.0

    if initial is not None:
        start = np.radians(
            [float(initial.get(name, 0.0)) for name in names]
        )
    else:
        start = np.zeros(len(names))

    step_limit_rad = math.radians(step_limit_deg)
    rng = np.random.default_rng(seed)
    best_values = None
    best_error_m = float("inf")

    for attempt in range(max(restarts, 1)):
        if attempt == 0:
            guess = start
        else:
            guess = rng.uniform(lower, upper)

        values, error_m = _solve_once(
            model, target_m, names, lower, upper, guess, target_link,
            max_iterations, tolerance_m, damping, step_limit_rad,
        )
        if error_m < best_error_m:
            best_values = values
            best_error_m = error_m
        if best_error_m <= tolerance_m:
            break

    best_degrees = {
        name: math.degrees(float(value))
        for name, value in zip(names, best_values)
    }
    return best_degrees, best_error_m * 100.0
