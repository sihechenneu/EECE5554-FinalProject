from setuptools import find_packages, setup

package_name = 'interbotix_landmark_modules'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/landmarks', ['landmarks/landmarks.yaml']),
    ],
    install_requires=['setuptools', 'tf_transformations'],
    zip_safe=True,
    maintainer='Luke Schmitt',
    maintainer_email='trsupport@trossenrobotics.com',
    description='The interbotix_landmark_modules package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_map_to_landmark = interbotix_landmark_modules.tf_map_to_landmark:main',
            'landmark_manager = interbotix_landmark_modules.landmark_manager:main',
            'landmark_finder = interbotix_landmark_modules.landmark_finder:main',
        ],
    },
)
