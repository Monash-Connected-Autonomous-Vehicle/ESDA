from setuptools import find_packages, setup

package_name = 'occupancy_fuser'

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
    maintainer='mcav',
    maintainer_email='zoon0002@student.monash.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'occupancy_fuser = occupancy_fuser.occupancy_fuser_node:main',
            'dummy_grid = occupancy_fuser.dummy_grid_pub:main'
        ],
    },
)
