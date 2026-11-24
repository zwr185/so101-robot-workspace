"""Parse a URDF/xacro model and compute forward kinematics.

Lengths are in centimeters, joint angles are in degrees.
All positions are expressed in the base_link frame (机械臂底座原点).
"""

import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

MOVABLE_JOINT_TYPES = ('revolute', 'continuous', 'prismatic')


def _vector(text, default=(0.0, 0.0, 0.0)):
    """Parse a space separated vector, falling back to a default."""
    if not text:
        return np.array(default, dtype=float)
    return np.array([float(value) for value in text.split()], dtype=float)


def translation(vector):
    """Build a 4x4 homogeneous translation matrix."""
    matrix = np.eye(4)
    matrix[:3, 3] = vector
    return matrix


def rpy_to_matrix(rpy):
    """Convert roll/pitch/yaw in degrees into a 3x3 rotation matrix."""
    roll, pitch, yaw = np.radians(rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rot_x = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    rot_y = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rot_z = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rot_z @ rot_y @ rot_x


def transform(xyz, rpy):
    """Build a 4x4 homogeneous transform from xyz (cm) and rpy (degrees)."""
    matrix = np.eye(4)
    matrix[:3, :3] = rpy_to_matrix(rpy)
    matrix[:3, 3] = xyz
    return matrix


def axis_rotation(axis, angle):
    """Rotation matrix around an arbitrary axis, angle in degrees."""
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm == 0.0:
        return np.eye(4)

    angle = math.radians(angle)
    x, y, z = axis / norm
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    one_c = 1.0 - cos_a

    matrix = np.eye(4)
    matrix[:3, :3] = np.array([
        [x * x * one_c + cos_a, x * y * one_c - z * sin_a,
         x * z * one_c + y * sin_a],
        [y * x * one_c + z * sin_a, y * y * one_c + cos_a,
         y * z * one_c - x * sin_a],
        [z * x * one_c - y * sin_a, z * y * one_c + x * sin_a,
         z * z * one_c + cos_a],
    ])
    return matrix


class Joint:
    """One URDF joint with origin, axis and limits.

    The URDF file stores xyz in meters, rpy and limits in radians. They are
    converted to centimeters and degrees here, so every public length and
    angle of this module uses cm and degrees.
    """

    def __init__(self, element):
        """Read joint data from a URDF joint element."""
        self.name = element.get('name')
        self.type = element.get('type')

        origin = element.find('origin')
        xyz_m = _vector(origin.get('xyz') if origin is not None else None)
        rpy = _vector(origin.get('rpy') if origin is not None else None)
        self.xyz = xyz_m * 100.0
        self.rpy = np.degrees(rpy)

        self.parent = element.find('parent').get('link')
        self.child = element.find('child').get('link')

        axis = element.find('axis')
        self.axis = _vector(
            axis.get('xyz') if axis is not None else None, (1.0, 0.0, 0.0)
        )

        limit = element.find('limit')
        if limit is not None:
            self.lower = math.degrees(float(limit.get('lower')))
            self.upper = math.degrees(float(limit.get('upper')))
        else:
            self.lower = None
            self.upper = None

    def origin_matrix(self):
        """Return the fixed transform of this joint origin."""
        return transform(self.xyz, self.rpy)


class UrdfModel:
    """URDF model providing the joint tree and forward kinematics."""

    def __init__(self, path):
        """Load the URDF/xacro file and index all joints by child link."""
        self.path = str(path)
        root = self._load_root(self.path)

        self.joints = {}
        for element in root.findall('joint'):
            joint = Joint(element)
            self.joints[joint.name] = joint
        self.child_to_joint = {
            joint.child: joint for joint in self.joints.values()
        }

        links = [element.get('name') for element in root.findall('link')]
        self.base_link = next(
            (link for link in links if link not in self.child_to_joint), None
        )
        if self.base_link is None:
            raise ValueError('无法确定基座链接，URDF 中可能存在闭环')

    @staticmethod
    def _load_root(path):
        """Load the model root, expanding xacro constructs when present."""
        with open(path, 'r', encoding='utf-8') as urdf_file:
            text = urdf_file.read()

        if 'xacro:' in text or '${' in text:
            import xacro
            return xacro.process_file(path).getroot()
        return ET.fromstring(text)

    def chain(self, target_link, base_link=None):
        """Return the joints from base_link to target_link, in order."""
        base_link = base_link or self.base_link
        joints = []
        link = target_link
        while link != base_link:
            joint = self.child_to_joint.get(link)
            if joint is None:
                raise ValueError(
                    f'找不到从 {base_link} 到 {target_link} 的运动链，'
                    f'当前链接: {link}'
                )
            joints.append(joint)
            link = joint.parent
        joints.reverse()
        return joints

    def movable_joint_names(self, target_link, base_link=None):
        """Return the names of the movable joints on the chain."""
        return [
            joint.name
            for joint in self.chain(target_link, base_link)
            if joint.type in MOVABLE_JOINT_TYPES
        ]

    def forward_kinematics(
        self, joint_values=None, target_link='gripper_frame_link',
        base_link=None,
    ):
        """Compute the 4x4 pose of target_link in the base frame.

        Joint values are in degrees, the translation part is in centimeters.
        """
        joint_values = joint_values or {}
        matrix = np.eye(4)
        for joint in self.chain(target_link, base_link):
            matrix = matrix @ joint.origin_matrix()
            value = float(joint_values.get(joint.name, 0.0))

            if joint.type in ('revolute', 'continuous'):
                matrix = matrix @ axis_rotation(joint.axis, value)
            elif joint.type == 'prismatic':
                matrix = matrix @ translation(joint.axis * value)
        return matrix


def load_model(path):
    """Load a URDF/xacro file into an UrdfModel."""
    return UrdfModel(path)


def main(argv=None):
    """Print the kinematic chain and a sample forward kinematics result."""
    argv = sys.argv if argv is None else argv
    default_path = Path(__file__).with_name('arm.urdf.xacro')
    path = argv[1] if len(argv) > 1 else str(default_path)

    model = load_model(path)
    target_link = 'gripper_frame_link'

    print(f'模型文件: {model.path}')
    print(f'基座链接: {model.base_link}')
    print(f'运动链 {model.base_link} -> {target_link}:')
    for joint in model.chain(target_link):
        limits = ''
        if joint.type in MOVABLE_JOINT_TYPES:
            limits = f'  限位 [{joint.lower:.2f}, {joint.upper:.2f}] 度'
        print(
            f'  {joint.name:14s} {joint.type:10s} '
            f'{joint.parent} -> {joint.child}{limits}'
        )

    movable = model.movable_joint_names(target_link)
    zero_values = {name: 0.0 for name in movable}
    zero_pose = model.forward_kinematics(zero_values, target_link)
    print(f'\n关节顺序: {movable}')
    print(f'零位末端位置 (cm): {np.round(zero_pose[:3, 3], 2).tolist()}')

    sample_values = dict(zero_values)
    sample_values['shoulder_pan'] = 30.0
    sample_values['elbow_flex'] = 20.0
    sample_pose = model.forward_kinematics(sample_values, target_link)
    print(f'\n示例关节值 (度): {sample_values}')
    print(f'示例末端位置 (cm): {np.round(sample_pose[:3, 3], 2).tolist()}')
    print('示例末端姿态矩阵:')
    print(np.round(sample_pose[:3, :3], 4))


if __name__ == '__main__':
    main()
