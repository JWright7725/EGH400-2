import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'offboard_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='James Wright',
    maintainer_email='jw.wright@connect.qut.edu.au',
    description='Offboard Control Package for EGH400',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'UAV_control = offboard_control.UAV_control:main',
        ],
    },
)
