from setuptools import find_packages, setup

package_name = 'esda_zed_cam'

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
    maintainer='jokua',
    maintainer_email='williamtioe2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_interface = esda_zed_cam.camera_interface:main',
            'lane_detection = esda_zed_cam.lane_detection:main'
        ],
    },
)
