"""A nuvem 3D vira a fatia 2D que a localização contra mapa consome.

    /livox/pontos  (PointCloud2, 3D)  ->  /scan  (LaserScan, 2D)

Sobe nos DOIS mundos, e é por isso que mora num arquivo só: o simulador e o
robô consomem `/livox/pontos` desde a decisão 017, e os números da fatia
(`config/scan_2d.yaml`) são geometria da máquina — não podem divergir entre
onde se testa e onde se roda. Divergir aqui é o defeito da bitola de 29-07 de
novo, e a forma dele é sempre a mesma: a bancada parece boa e o robô piora.

⚠️ Depende do pacote `ros-jazzy-pointcloud-to-laserscan`, que NÃO vem no clone.
No NUC:

    sudo apt install ros-jazzy-pointcloud-to-laserscan

Conferido antes de escolher esta dependência: `apt-get install --dry-run` diz
`0 upgraded, 1 newly installed, 0 to remove`. Ela não arrasta upgrade de lib
nenhuma — o perigo provado em 05-08 (o símbolo do `diagnostic_updater` que
DESAPARECE entre versões) é de upgrade, e não tem essa forma aqui.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('robot_base'),
                          'config', 'scan_2d.yaml')
    sim = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            # O nome bate com a chave do YAML de propósito: parâmetro que não
            # casa com o nome do nó é silenciosamente ignorado, e o nó sobe com
            # os defaults do upstream — fatia de −1 a +1 m, que lê o chão e o
            # teto. Falha sem sintoma, do tipo que este projeto já pagou.
            name='scan_2d',
            parameters=[params, {'use_sim_time': sim}],
            remappings=[('cloud_in', '/livox/pontos'),
                        ('scan', '/scan')],
            output='both',
        ),
    ])
