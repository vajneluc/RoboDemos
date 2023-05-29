import os
from glob import glob
from setuptools import setup

package_name = 'ros2_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='julius',
    maintainer_email='tomsa.julius@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_teleop_pymoveit2 = ros2_teleop.ros2_teleop_pymoveit2:main',
            'keyboard_control = ros2_teleop.keyboard_control:main',
            'waypoints = ros2_teleop.waypoints:main '
     ],
    },
)
