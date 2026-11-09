from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'solution'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Anything the launch file needs has to be installed, to be found by package name
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Solution to the assessment',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'task1 = solution.task1:main',
            'task2 = solution.task2:main',
        ],
    },
)
