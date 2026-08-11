from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'description'), glob('description/*.xacro')),
        # Mundos vivem na raiz do repo (worlds/), junto com os do robô 1, mas
        # precisam estar no share para o launch achá-los depois de instalado.
        (os.path.join('share', package_name, 'worlds'), glob('../../worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luiz Santarosa',
    maintainer_email='luizgustavo.santarosa@gmail.com',
    description='Base do robô 2: tração (hoverboard/ros2_control) e localização (Mid-360 + FAST-LIO)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'placa_simulada = robot_base.placa_simulada:main',
            'tf_odom = robot_base.tf_odom:main',
            'nuvem_pontos = robot_base.nuvem_pontos:main',
        ],
    },
)
