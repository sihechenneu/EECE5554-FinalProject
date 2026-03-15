import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'oakd_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/oakd_bringup']),
        ('share/oakd_bringup', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    entry_points={'console_scripts': []},
)
