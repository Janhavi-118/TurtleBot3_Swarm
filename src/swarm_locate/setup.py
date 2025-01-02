from setuptools import find_packages, setup

package_name = 'swarm_locate'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/spawn_turtles.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='janhavi',
    maintainer_email='janhavi@todo.todo',
    description='Swarm robotics project for locating a target using TurtleBot3 robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
