import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'roomba'

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
    maintainer='calpoly',
    maintainer_email='calpoly@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "run_roomba = roomba.roomba_fsm:main",
            "follow_simon = roomba.follow_path:main",
            "follow_aruco = roomba.final_project_robot:main"
        ],
    },
)
