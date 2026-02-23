from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'px100_control_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools', 'flask'],
    entry_points={
        'console_scripts': [
            'ui_node = px100_control_ui.ui_node:main',
        ],
    },
    zip_safe=True,
    maintainer='EECE5554',
    maintainer_email='user@example.com',
    description='Web UI to control PincherX 100 robot arm',
    license='BSD',
)
