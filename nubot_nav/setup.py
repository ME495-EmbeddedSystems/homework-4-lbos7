from setuptools import find_packages, setup

package_name = 'nubot_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/manual_explore.launch.xml', 'launch/explore.launch.xml']),
        ('share/' + package_name + '/config', ['config/nubot_nav_params.yaml']),
        ('share/' + package_name + '/config', ['config/nubot_nav_rviz_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Logan Boswell',
    maintainer_email='loganstuartboswell@gmail.com',
    description='ROS2 package for exploring a world in Gazebo and mapping it',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explore = nubot_nav.explore:main'
        ],
    },
)
