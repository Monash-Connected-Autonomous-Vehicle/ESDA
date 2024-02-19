from setuptools import find_packages, setup

package_name = 'esda_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/parallelogram_steering_controller.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiawei',
    maintainer_email='jiaweiliao01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parallelogram_steering_controller = esda_control.parallelogram_steering_controller:main',
            'dynamixel_sdk_node = dynamixel_sdk_examples.read_write_mode:main',
            'mcu_interface = esda_control.mcu_interface:main'
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    },
)
