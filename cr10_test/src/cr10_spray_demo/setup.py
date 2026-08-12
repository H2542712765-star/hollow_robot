from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'cr10_spray_demo'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hollow',
    maintainer_email='hollow@example.com',
    description='TF-validated autonomous raster spray demo for CR10 using MoveIt 2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spray_demo_node = cr10_spray_demo.spray_demo_node:main',
            'tf_check_node = cr10_spray_demo.tf_check_node:main',
        ],
    },
)
