from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'vision_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
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
            'camera_native_node = vision_pkg.camera_native_node:main',
            'hsv_image_node = vision_pkg.hsv_image_node:main',
            'box_detect_node = vision_pkg.box_detect_node:main',
            'box_tf_node = vision_pkg.box_tf_node:main',
            'box_tf_calibrate = vision_pkg.box_tf_calibrate:main',
        ],
    },
)
