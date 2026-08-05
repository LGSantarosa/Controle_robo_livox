import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_motion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luiz Santarosa',
    maintainer_email='luizgustavo.santarosa@gmail.com',
    description='Movimentação do robô 2: controlador de rumo por lei de frenagem',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # decisão 011: entra cmd_vel desejado, sai cmd_vel que anda reto
            'compensador_rumo = robot_motion.compensador_rumo:main',
            # o freio de mao do humano (twist_mux, prio 90)
            'teleop_teclado = robot_motion.teleop_teclado:main',
            'heading_controller = robot_motion.heading_controller:main',
            'path_follower = robot_motion.path_follower:main',
            # aposentado em 28-07 (decisão 008); sai numa fatia própria
            'goal_navigator = robot_motion.goal_navigator:main',
        ],
    },
)
