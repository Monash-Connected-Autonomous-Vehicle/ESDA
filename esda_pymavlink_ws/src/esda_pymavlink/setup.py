from setuptools import find_packages, setup

package_name = 'esda_pymavlink'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pymav_sender_launch.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samuel Tri',
    maintainer_email='stri0019@student.monash.edu',
    description='A ROS 2 package to interface with pymavlink',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pymavlink_driver = esda_pymavlink.pymavlink_driver:main',
        ],
    },
)
