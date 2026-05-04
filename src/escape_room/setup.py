from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'escape_room'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch') + glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.ttm')),
        (os.path.join('share', package_name, 'scenarios'), glob('scenarios/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gianluca Viviano',
    maintainer_email='gianluca.viviano@usi.ch',
    description='Escape room mission for RoboMaster EP',
    license='MIT',
    entry_points={
        'console_scripts': [
            'door_controller = escape_room.nodes.door_controller:main',
        ],
    },
)