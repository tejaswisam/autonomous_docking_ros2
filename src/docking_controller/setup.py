from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'docking_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    package_data={
        '': ['msg/*.msg'],
    },
    zip_safe=True,
    maintainer='tejaswi',
    maintainer_email='tejaswisamavedula@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = docking_controller.controller_node:main',
            'pose_estimator_node = docking_controller.pose_estimator:main',
        ],
    },
)
