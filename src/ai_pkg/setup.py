import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ai_pkg'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zwr185',
    maintainer_email='zwr185@todo.todo',
    description='AI package: MCP server node and LLM agent node',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mcp_server_node = ai_pkg.mcp_server_node:main',
            'llm_agent_node = ai_pkg.llm_agent_node:main',
            'robot_nod_node = ai_pkg.robot_nod_node:main',
            'robot_shake_head_node = ai_pkg.robot_shake_head_node:main',
            'robot_remote_control_node = ai_pkg.robot_remote_control_node:main',
            'robot_action_manager_node = ai_pkg.robot_action_manager_node:main',
            'robot_catch_box_node = ai_pkg.robot_catch_box_node:main',
        ],
    },
)
