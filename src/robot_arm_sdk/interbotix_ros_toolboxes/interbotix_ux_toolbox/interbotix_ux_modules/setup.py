from setuptools import find_packages, setup

package_name = 'interbotix_ux_modules'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src', exclude=['test']),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luke Schmitt',
    maintainer_email='trsupport@trossenrobotics.com',
    description='The interbotix_ux_modules package (ROS 2)',
    license='BSD',
    tests_require=['pytest'],
    entry_points={},
)
