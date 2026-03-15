from setuptools import find_packages, setup

package_name = 'spatial_calculator'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/spatial_calculator.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='martin',
    maintainer_email='chen.sihe1@northeastern.edu',
    description='Projects 2D YOLO detections to 3D using aligned depth and camera intrinsics',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'spatial_calculator = spatial_calculator.spatial_calculator_node:main',
        ],
    },
)
