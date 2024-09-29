from setuptools import setup
import os
from glob import glob

package_name = 'saws_handeye_calibration'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    py_modules=[
        'saws_handeye_calibration.calibration_node',
        'saws_handeye_calibration.axbsolver'
    ],
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'), ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shashank Goyal',
    maintainer_email='sgoyal18@jhu.edu',
    description='A ROS2 package for hand-eye calibration using ArUco markers.',
    entry_points={
        'console_scripts': [
            'calibration = saws_handeye_calibration.calibration_node:main',
        ],
    },
)
