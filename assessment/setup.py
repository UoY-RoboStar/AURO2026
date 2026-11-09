from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'assessment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'), ['worlds/assessment.world']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'worlds/meshes'),
         [f for ext in ('stl', 'dae', 'png', 'jpg')
          for f in glob(os.path.join('worlds/meshes', '*.' + ext))]),
        # ArUco tags: their meshes and the images they show
        (os.path.join('share', package_name, 'worlds/tags'),
         [f for ext in ('dae', 'png') for f in glob(os.path.join('worlds/tags', '*.' + ext))]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'reactor_monitor = assessment.reactor_monitor:main',
            'simulation_monitor = assessment.simulation_monitor:main',
            'nav2_startup = assessment.nav2_startup:main',
            'end_effector_manager = assessment.end_effector_manager:main',
            'description_publisher = assessment.description_publisher:main',
            'power_display = assessment.power_display:main',
            'sim_time_log = assessment.sim_time_log:main',
            'time_limit = assessment.time_limit:main',
        ],
    },
)
