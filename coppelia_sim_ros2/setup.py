from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'coppelia_sim_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jjj',
    maintainer_email='jjjau03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'puzzlebot_coppelia = coppelia_sim_ros2.puzzlebot_coppelia:main'
        ],
    },
)
