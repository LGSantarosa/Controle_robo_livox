"""Tração do robô 2: placa hover via ros2_control + controlador diferencial.

Sobe o `controller_manager` com a interface de hardware `hoverboard_driver`
(serial direta com a placa, protocolo 0xABCD) e ativa o
`hoverboard_base_controller` (diff_drive_controller).

Depois disso o robô aceita comandos em `/hoverboard_base_controller/cmd_vel`
(TwistStamped, unidades SI: m/s e rad/s). Nada além disso é responsabilidade
deste launch — quem decide PARA ONDE ir é outra camada.
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # URDF vem do xacro do próprio driver — é lá que ficam o wheel_radius e o
    # device serial usados pela interface de hardware.
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('hoverboard_driver'), 'urdf', 'diffbot.urdf.xacro'
        ]),
    ])
    # O launch_ros >=0.26 lê parâmetro como YAML por padrão; um URDF cru falha
    # ('<?xml ...' não é YAML). value_type=str força string — correto no
    # launch_ros novo e no antigo.
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('hoverboard_driver'), 'config', 'hoverboard_controllers.yaml'
    ])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
    )

    # Publica o TF das juntas a partir do URDF. Sem remapping: o exemplo
    # upstream remapeava um tópico do controlador aqui, o que não tem efeito
    # (o robot_state_publisher não fala cmd_vel) e só confundia a leitura.
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hoverboard_base_controller', '--controller-manager', '/controller_manager'],
    )

    # O controlador diferencial só sobe depois do broadcaster de juntas, senão
    # ele reclama de interfaces de estado ainda não disponíveis.
    base_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[base_controller_spawner],
        )
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        base_after_broadcaster,
    ])
