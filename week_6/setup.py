from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'week_6'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.sdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='pedro.ribeiro@york.ac.uk',
    description='Compares wheel odometry with ground truth for a simulated TurtleBot3',
    license='EPL-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_detector = week_6.aruco_detector:main',
            'path_publisher = week_6.path_publisher:main',
            'true_frames = week_6.true_frames:main',
            'visual_sensor = week_6.visual_sensor:main',
        ],
    },
)
