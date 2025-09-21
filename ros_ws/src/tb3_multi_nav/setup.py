from setuptools import find_packages, setup

package_name = 'tb3_multi_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/robot1_bridge.yaml']),
        ('share/' + package_name + '/config', ['config/robot2_bridge.yaml']),
        ('share/' + package_name + '/config', ['config/nav2_waffle_robot1.yaml']),
        ('share/' + package_name + '/config', ['config/nav2_waffle_robot2.yaml']),
        ('share/' + package_name + '/config', ['config/nav2_waffle.yaml']),
        ('share/' + package_name + '/config', ['config/map.yaml']),
        ('share/' + package_name + '/config', ['config/multi_nav.rviz']),
        ('share/' + package_name + '/launch', ['launch/multi_tb3.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_republisher = tb3_multi_nav.map_republisher:main',  
        ],
    },
)