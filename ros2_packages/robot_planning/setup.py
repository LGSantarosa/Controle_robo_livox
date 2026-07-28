import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_planning'

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
    description='Bancada do planner do robô 2: compara caminhos do Nav2 sem mover o robô',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bancada_planner = robot_planning.bancada_planner:main',
        ],
    },
)
