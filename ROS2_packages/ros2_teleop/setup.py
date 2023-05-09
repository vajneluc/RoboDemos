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
            'ros2_teleop_node = ros2_teleop.ros2_teleop_node:main',
            'ros2_teleop_pymoveit2 = ros2_teleop.ros2_teleop_pymoveit2:main'
     ],
    },
)
