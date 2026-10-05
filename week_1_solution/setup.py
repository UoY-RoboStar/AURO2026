from setuptools import find_packages, setup

package_name = 'week_1_solution'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pedro Ribeiro',
    maintainer_email='pedro.ribeiro@york.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'task5_publisher = week_1_solution.task5_publisher:main',
            'task5_graceful_publisher = week_1_solution.task5_graceful_publisher:main',
            'task9_subscriber = week_1_solution.task9_subscriber:main',
            'task14 = week_1_solution.task14:main'
        ],
    },
)
