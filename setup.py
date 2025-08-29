from setuptools import setup

package_name = 'dwa_local_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dwa_planner.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='A minimal custom DWA local planner for TurtleBot3 in Gazebo (ROS 2 Humble).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dwa_planner = dwa_local_planner.dwa_planner:main',
        ],
    },
)
