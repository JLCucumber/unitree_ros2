from setuptools import find_packages, setup

package_name = 'unitree_examples_py'

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
    maintainer='jlcucumber',
    maintainer_email='jlcucumber@gmail.com',
    description='Python examples for Unitree Go2 & Go2-W',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'read_motion_state = unitree_examples_py.read_motion_state:main',
            'cmd_vel_to_unitree = unitree_examples_py.cmd_vel_to_unitree:main',
        ],
    },
)
