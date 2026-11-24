from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'arm_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zwr185',
    maintainer_email='zwr185@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arm_joint_node = arm_pkg.arm_joint_node:main',
            'arm_read_angle = arm_pkg.arm_read_angle:main',
            'arm_record_node = arm_pkg.arm_record_node:main',
            'arm_playback_node = arm_pkg.arm_playback_node:main',
            'arm_record_playback_node = arm_pkg.arm_record_playback_node:main',
            'gui_record_playback_node = arm_pkg.gui_record_playback_node:main',
            'arm_catch_box_node = arm_pkg.arm_catch_box_node:main',
            'arm_track_box_node = arm_pkg.arm_track_box_node:main',
            'arm_catch_joint_node = arm_pkg.arm_catch_joint_node:main',
            'arm_playback_joint_node = arm_pkg.arm_playback_joint_node:main',
        ],
    },
)
