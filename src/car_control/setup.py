from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'car_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
          ('share/' + package_name, ['package.xml']),
          (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
          (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
          (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rony',
    maintainer_email='rony@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'spawn_car = car_control.spawn_car:main',
		'car_controller = car_control.car_controller:main',
		'waypoint_follower = car_control.waypoint_follower:main',
        ],
    },
)
