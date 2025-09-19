from setuptools import find_packages, setup

package_name = 'motor_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='null',
    maintainer_email='null@todo.todo',
    description='Motor driver node for controlling robot via cmd_vel and L298N.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_node = motor_driver.driver_node:main',
        ],
    },
)

