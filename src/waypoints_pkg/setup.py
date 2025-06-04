from setuptools import setup
import os
from glob import glob

package_name = 'waypoints_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),  # ← corrige warning 2
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # ← corrige warning 1
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ton_nom',
    maintainer_email='ton_email@exemple.com',
    description='Waypoint manager package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'waypoints = waypoints_pkg.waypoints:get_waypoints'
        ],
    },
)

