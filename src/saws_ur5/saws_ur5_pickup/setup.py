from setuptools import find_packages, setup

package_name = 'saws_ur5_pickup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/handeye.yaml']),
        ('share/' + package_name + '/launch', ['launch/saws_pickup.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'saws_pickup = saws_ur5_pickup.saws_pickup:main'
        ],
    },
)
