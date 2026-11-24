"""Fit a pixel-to-base homography for box_tf_node.

用法（相机和 box_detect_node 运行时）：
    ros2 run vision_pkg box_tf_calibrate
把球依次放在 4 个以上已知位置，输入该点相对机械臂底座的 x y（cm），
回车采集当前 /box_center 像素。输入 q 结束，自动拟合透视变换并保存。

也可以用 CSV 批量拟合（每行 x,y,u,v）：
    ros2 run vision_pkg box_tf_calibrate --input points.csv
"""

import argparse
from datetime import datetime
import json
import os
import select
import sys
import time

from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.node import Node


def _normalize(points):
    """Normalize 2D points for a numerically stable DLT."""
    points = np.asarray(points, dtype=float)
    centroid = points.mean(axis=0)
    shifted = points - centroid
    mean_distance = np.sqrt((shifted ** 2).sum(axis=1)).mean()
    scale = np.sqrt(2.0) / mean_distance if mean_distance > 1e-12 else 1.0
    transform = np.array([
        [scale, 0.0, -scale * centroid[0]],
        [0.0, scale, -scale * centroid[1]],
        [0.0, 0.0, 1.0],
    ])
    homogeneous = np.hstack([points, np.ones((len(points), 1))])
    normalized = (transform @ homogeneous.T).T[:, :2]
    return normalized, transform


def fit_homography(pixels, bases):
    """Fit the homography mapping pixel centers to base XY in centimeters."""
    pixels = np.asarray(pixels, dtype=float)
    bases = np.asarray(bases, dtype=float)
    if len(pixels) < 4:
        raise ValueError('至少需要 4 组对应点')

    normalized_pixels, pixel_transform = _normalize(pixels)
    normalized_bases, base_transform = _normalize(bases)

    rows = []
    for (u, v), (x, y) in zip(normalized_pixels, normalized_bases):
        rows.append([-u, -v, -1.0, 0.0, 0.0, 0.0, u * x, v * x, x])
        rows.append([0.0, 0.0, 0.0, -u, -v, -1.0, u * y, v * y, y])
    matrix = np.asarray(rows, dtype=float)

    _, _, vh = np.linalg.svd(matrix)
    normalized_h = vh[-1].reshape(3, 3)
    homography = (
        np.linalg.inv(base_transform) @ normalized_h @ pixel_transform
    )
    if abs(homography[2, 2]) < 1e-12:
        raise ValueError('拟合失败：透视变换矩阵的 H33 为 0')
    return homography / homography[2, 2]


def project(homography, pixel):
    """Project one pixel center to base XY with the homography."""
    vector = homography @ np.array([pixel[0], pixel[1], 1.0], dtype=float)
    if abs(vector[2]) < 1e-12:
        return None
    return vector[:2] / vector[2]


class BoxTfCalibrator(Node):
    """Keep the latest detected pixel center from box_detect_node."""

    def __init__(self, pixel_topic='/box_center', timeout=2.0):
        """Create the pixel subscription."""
        super().__init__('box_tf_calibrate')
        self.timeout = float(timeout)
        self.latest_pixel = None
        self.latest_time = 0.0
        self.subscription = self.create_subscription(
            PointStamped, pixel_topic, self._on_pixel, 10
        )

    def _on_pixel(self, msg):
        """Store the newest detected pixel center."""
        self.latest_pixel = (float(msg.point.x), float(msg.point.y))
        self.latest_time = time.monotonic()

    def capture(self):
        """Return the latest pixel if it is fresh enough."""
        if self.latest_pixel is None:
            return None
        if time.monotonic() - self.latest_time > self.timeout:
            return None
        return self.latest_pixel


def report_and_save(pairs, output, pixel_topic):
    """Fit, print the residuals and save the homography file."""
    pixels = [pair[1] for pair in pairs]
    bases = [pair[0] for pair in pairs]
    homography = fit_homography(pixels, bases)

    errors = []
    print('\n采集点与残差:')
    print('  序号   底座 x,y (cm)        像素 u,v           误差 (cm)')
    for index, (base, pixel) in enumerate(pairs):
        projected = project(homography, pixel)
        error = float(np.linalg.norm(projected - np.array(base)))
        errors.append(error)
        print(
            f'  {index + 1:>3d}   ({base[0]:7.2f}, {base[1]:7.2f})   '
            f'({pixel[0]:7.1f}, {pixel[1]:7.1f})   {error:7.3f}'
        )

    rmse = float(np.sqrt(np.mean(np.square(errors))))
    maximum = float(np.max(errors))
    print(f'\nRMSE: {rmse:.3f} cm，最大误差: {maximum:.3f} cm')
    if maximum > 2.0:
        print('提示: 最大误差超过 2 cm，建议检查采集点是否准确或重新取点。')

    output = os.path.expanduser(output)
    os.makedirs(os.path.dirname(output) or '.', exist_ok=True)
    payload = {
        'homography': homography.tolist(),
        'rmse_cm': rmse,
        'max_error_cm': maximum,
        'pixel_topic': pixel_topic,
        'created': datetime.now().isoformat(timespec='seconds'),
        'points': [
            {
                'base': [float(base[0]), float(base[1])],
                'pixel': [float(pixel[0]), float(pixel[1])],
                'error_cm': float(error),
            }
            for base, pixel, error in zip(bases, pixels, errors)
        ],
    }
    with open(output, 'w', encoding='utf-8') as file:
        json.dump(payload, file, indent=4, ensure_ascii=False)
    print(f'\n已保存: {output}')
    print('重启 box_tf_node 即可生效（mode 为 auto 或 homography）。')


def load_csv(path):
    """Read x,y,u,v rows from a CSV or whitespace separated text file."""
    pairs = []
    with open(os.path.expanduser(path), 'r', encoding='utf-8') as file:
        for line_number, line in enumerate(file, start=1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.replace(',', ' ').split()
            if len(parts) != 4:
                raise ValueError(f'第 {line_number} 行需要 4 个数值: x y u v')
            base_x, base_y, pixel_u, pixel_v = (float(v) for v in parts)
            pairs.append((
                (base_x, base_y), (pixel_u, pixel_v)
            ))
    return pairs


def interactive(pixel_topic, timeout):
    """Collect point pairs from the terminal until the user quits."""
    rclpy.init()
    node = BoxTfCalibrator(pixel_topic=pixel_topic, timeout=timeout)
    pairs = []
    print(
        '把球放在已知位置（相对机械臂底座的 x y，单位 cm），\n'
        '输入 "x y" 并回车采集当前像素；也可输入 "x y u v" 手动录入像素；\n'
        '输入 q 结束并拟合。至少需要 4 个点。\n'
    )
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue

            line = sys.stdin.readline()
            if not line:
                break
            line = line.strip()
            if line.lower() == 'q':
                break
            parts = line.replace(',', ' ').split()
            if len(parts) == 2:
                try:
                    base = (float(parts[0]), float(parts[1]))
                except ValueError:
                    print('格式错误：请输入 "x y" 或 "x y u v"')
                    continue
                pixel = node.capture()
                if pixel is None:
                    print('当前没有新的 /box_center 数据，请确认相机和检测节点已启动')
                    continue
            elif len(parts) == 4:
                base_x, base_y, pixel_u, pixel_v = (float(v) for v in parts)
                base = (base_x, base_y)
                pixel = (pixel_u, pixel_v)
            else:
                print('格式错误：请输入 "x y" 或 "x y u v"')
                continue

            pairs.append((base, pixel))
            print(
                f'已采集第 {len(pairs)} 点: 底座 ({base[0]:.2f}, '
                f'{base[1]:.2f}) cm, 像素 ({pixel[0]:.1f}, {pixel[1]:.1f})'
            )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return pairs


def main(args=None):
    """Run the interactive or CSV based homography calibration."""
    parser = argparse.ArgumentParser(
        description='拟合 box_tf_node 的像素→底座坐标透视变换'
    )
    parser.add_argument(
        '--output',
        default=(
            '/home/zwr185/dev_ws/src/vision_pkg/json/'
            'box_tf_homography.json'
        ),
        help='输出 JSON 路径',
    )
    parser.add_argument(
        '--input', default='',
        help='CSV 文件（每行 x,y,u,v），提供后不进入交互模式',
    )
    parser.add_argument(
        '--pixel-topic', default='/box_center', help='像素中心话题',
    )
    parser.add_argument(
        '--timeout', type=float, default=2.0,
        help='采集时允许的最大像素数据延迟（秒）',
    )
    parsed, _ = parser.parse_known_args(args)

    try:
        if parsed.input:
            pairs = load_csv(parsed.input)
        else:
            pairs = interactive(parsed.pixel_topic, parsed.timeout)
    except ValueError as exc:
        print(f'错误: {exc}')
        return 1

    if len(pairs) < 4:
        print(f'错误: 只采集了 {len(pairs)} 个点，至少需要 4 个')
        return 1

    report_and_save(pairs, parsed.output, parsed.pixel_topic)
    return 0


if __name__ == '__main__':
    sys.exit(main())
