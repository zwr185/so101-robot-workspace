"""Parse a URDF model and solve inverse kinematics for the arm end effector.

Targets and positions are in centimeters, joint angles are in degrees.
All positions are expressed in the base_link frame (机械臂底座原点).
"""

import math
from pathlib import Path
import sys

from arm_pkg.urdf_fk import load_model
import numpy as np


def _joint_limits(model, names):
    """Collect the lower and upper limits (degrees) of the given joints."""
    lower = []
    upper = []
    for name in names:
        joint = model.joints[name]
        lower.append(joint.lower)
        upper.append(joint.upper)
    return np.array(lower, dtype=float), np.array(upper, dtype=float)


def _position_m(model, values_rad, names, target_link):
    """Return the end effector position in meters (internal use)."""
    values_deg = {
        name: math.degrees(float(value))
        for name, value in zip(names, values_rad)
    }
    return model.forward_kinematics(values_deg, target_link)[:3, 3] / 100.0


def _jacobian_m(model, values_rad, names, target_link, epsilon=1e-4):
    """Build the 3xN position Jacobian in meters with finite differences."""
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
    """Run one damped least squares loop, angles in radians, lengths in m."""
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
        np.linalg.norm(target_m - _position_m(model, values, names, target_link))
    )
    return values, error_m


def inverse_kinematics(
    model, target_position, target_link='gripper_frame_link',
    initial=None, max_iterations=300, tolerance=0.01,
    damping=0.05, step_limit=17.0, restarts=10, seed=0, limits=None,
):
    """Solve joint angles (degrees) so the target link reaches the position.

    target_position is in centimeters, step_limit is the maximum joint
    change per iteration in degrees, tolerance is in centimeters.
    limits optionally maps joint names to (lower, upper) limits in degrees
    and overrides the URDF limits.
    Returns (joint_values, position_error) with error in centimeters.
    """
    names = model.movable_joint_names(target_link)
    lower_deg, upper_deg = _joint_limits(model, names)
    if limits:
        for index, name in enumerate(names):
            if name in limits:
                lower_deg[index], upper_deg[index] = limits[name]
    lower = np.radians(lower_deg)
    upper = np.radians(upper_deg)
    target_m = np.asarray(target_position, dtype=float) / 100.0
    tolerance_m = float(tolerance) / 100.0

    if initial is not None:
        start = np.radians(
            [float(initial.get(name, 0.0)) for name in names]
        )
    else:
        start = np.zeros(len(names))

    step_limit_rad = math.radians(step_limit)
    rng = np.random.default_rng(seed)
    best_values = None
    best_error_m = float('inf')

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


def main(argv=None):
    """Solve a demo target and print the recovered joint angles."""
    argv = sys.argv if argv is None else argv
    default_path = Path(__file__).with_name('arm.urdf.xacro')
    path = argv[1] if len(argv) > 1 else str(default_path)

    model = load_model(path)
    target_link = 'gripper_frame_link'

    if len(argv) >= 5:
        target = np.array([float(value) for value in argv[2:5]])
        print(f'目标位置 (cm): {np.round(target, 2).tolist()}')
    else:
        sample_values = {
            'shoulder_pan': 20.0,
            'shoulder_lift': -15.0,
            'elbow_flex': 30.0,
            'wrist_flex': 10.0,
            'wrist_roll': 0.0,
        }
        target = model.forward_kinematics(sample_values, target_link)[:3, 3]
        print(f'目标位置 (cm, 由正解生成): {np.round(target, 2).tolist()}')

    values, error = inverse_kinematics(model, target, target_link)
    print('解出的关节角度 (度):')
    for name, value in values.items():
        print(f'  {name:14s} {value: .4f}')
    print(f'位置误差 (cm): {error:.4f}')

    reached = model.forward_kinematics(values, target_link)[:3, 3]
    print(f'正解复核位置 (cm): {np.round(reached, 2).tolist()}')


if __name__ == '__main__':
    main()
