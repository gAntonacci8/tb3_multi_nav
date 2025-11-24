from setuptools import find_packages, setup

package_name = 'tb3_multi_nav'                                                      # package name to be used 

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/multi_tb3.launch.py']),      # to get main launch available
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={          
        'console_scripts': [                                                        # nodes available in the package   
            "orchestrator_node = tb3_multi_nav.orchestrator_node_5:main",           # tag game logic
            "robot1_publisher_node = tb3_multi_nav.robot1_publisher_node:main",     # robot1 position publisher
            "robot2_publisher_node = tb3_multi_nav.robot2_publisher_node:main"      # robot2 position publisher
        ],
    },
)
