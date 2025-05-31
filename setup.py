from setuptools import setup
import os
from glob import glob

package_name = 'automated_robot_planning'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'srv'),
         glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fariha',
    maintainer_email='farihaal.ferdous@unitn.it',
    description='Automated robot planning package',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'position_manager = automated_robot_planning.position_manager:main',
        ],
    },
)
