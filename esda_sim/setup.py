import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'esda_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),        
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'description'), glob(os.path.join('description', 'esda.urdf.xacro'))),
        (os.path.join('share', package_name, 'description'), glob(os.path.join('description', 'inertial_macros.xacro')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mcav',
    maintainer_email='s.a.baaset.moslih@gmail.com',
    description='hosts the udrf for the robot for sim and tf purposes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
