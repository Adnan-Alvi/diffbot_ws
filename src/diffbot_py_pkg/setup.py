from setuptools import find_packages, setup

package_name = 'diffbot_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adnan',
    maintainer_email='adnanalvi17092001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "simple_publisher = diffbot_py_pkg.simple_publisher_example:main",
            "simple_subscriber = diffbot_py_pkg.simple_subscriber_example:main",
            "simple_parameter = diffbot_py_pkg.simple_parameter:main",
            "simple_action_server = diffbot_py_pkg.simple_action_server:main",
            "simple_action_client = diffbot_py_pkg.simple_action_client:main",
            "simple_turtlesim_kinematics = diffbot_py_pkg.simple_turtlesim_kinematics:main",
            "simple_turtlesim_odometry = diffbot_py_pkg.simple_turtlesim_odometry:main",
        ],
    },
)
