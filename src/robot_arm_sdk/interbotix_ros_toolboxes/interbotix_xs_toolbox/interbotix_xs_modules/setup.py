from setuptools import find_packages, setup

package_name = 'interbotix_xs_modules'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luke Schmitt',
    maintainer_email='trsupport@trossenrobotics.com',
    description='The interbotix_xs_modules package',
    license='BSD',
)
