from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'unitree_rl_deploy'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'policies'), glob('policies/README.md')),
    ],
    install_requires=['setuptools', 'numpy', 'pyyaml'],
    zip_safe=True,
    maintainer='fibo',
    maintainer_email='fibo@todo.todo',
    description='RL policy deploy (MuJoCo + Unitree LowCmd). No training.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'policy_runner = unitree_rl_deploy.runner:main',
        ],
    },
)
