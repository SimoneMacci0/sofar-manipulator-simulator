import os
from glob import glob
from setuptools import setup

package_name = 'sofar_manipulator_simulator'
lib = 'sofar_manipulator_simulator/lib'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, lib],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "resource"), glob("resource/*.png")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simone',
    maintainer_email='simone.maccio2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manipulator_sim_node = sofar_manipulator_simulator.manipulator_sim_node:main',
            'controller_node = sofar_manipulator_simulator.controller_node:main',
            'logic_node = sofar_manipulator_simulator.robot_logic:main'
        ],
    },
)
