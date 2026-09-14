from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'xbox_one_s_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fibo',
    maintainer_email='fibo@todo.todo',
    description='Xbox One S (1708) Bluetooth teleop: /cmd_vel and /body_pose',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xbox_teleop_node = xbox_one_s_teleop.teleop_node:main',
        ],
    },
)
