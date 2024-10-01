from glob import glob
from setuptools import setup

package_name = 'diff_drive_asterius_hardware_interface'

setup(
    name=package_name,
    ...
    data_files=[
        ('share/{}'.format(package_name), glob('launch/*.launch.py')),
        ...
    ],
)
