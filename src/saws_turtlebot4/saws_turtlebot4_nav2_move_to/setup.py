from setuptools import setup
import os
from glob import glob

package_name = 'saws_turtlebot4_nav2_move_to'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    py_modules=[
        'saws_turtlebot4_nav2_move_to.navigate_to_goal',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tarun',
    maintainer_email='tarunprasadoff@gmail.com',
    description='ROS2 script to navigate to a goal using TurtleBot4Navigator.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'navigate_to_goal = saws_turtlebot4_nav2_move_to.navigate_to_goal:main'
        ],
    },
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
)
