from setuptools import find_packages, setup

package_name = 'lidar_gui'

setup(
    name=package_name,
    version='0.1.0',
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
    description='LiDAR GUI with interactive path drawing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_node = lidar_gui.gui_node:main'
        ],
    },
)

