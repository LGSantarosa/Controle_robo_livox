#!/usr/bin/env python3
"""Converte a nuvem do Livox (`CustomMsg`) para `PointCloud2`.

    ros2 run robot_base nuvem_pontos
    ros2 run robot_base nuvem_pontos --ros-args -p entrada:=/livox/lidar \\
                                                -p saida:=/livox/pontos

## O defeito que ele conserta (medido no robô em 10-08)

    /livox/lidar   publisher:  livox_ros_driver2/msg/CustomMsg   (1 publisher)
    costmaps, collision_monitor e o pré-voo assinam  sensor_msgs/PointCloud2

Quem assina PointCloud2 nunca recebeu nada. O FAST-LIO lê CustomMsg, então a
localização ia bem e a percepção era zero — os dois costmaps com **0 células
letais** enquanto o `ros2 topic hz /livox/lidar` mostrava 9,96 Hz. O reflexo de
colisão nunca funcionou no robô pela mesma razão, o que explica o teste D de
06-08 sem precisar de outra hipótese.

## O contrato, depois desta mudança

    /livox/lidar    CRU, e o tipo depende do mundo:
                    robô = CustomMsg (é o que o FAST-LIO come)
                    simulador = PointCloud2 (o `gpu_lidar` do Gazebo)
    /livox/pontos   PointCloud2 SEMPRE — é o que a percepção consome

⚠️ O erro que criou o problema foi dar **o mesmo nome** a coisas de tipos
diferentes, achando que isso tornava os dois mundos iguais ("o MESMO tópico do
driver real — quem consome não sabe a diferença", dizia o comentário da ponte do
Gazebo). Um tópico é (nome, tipo): igualar só o nome esconde a diferença em vez
de eliminá-la. No simulador quem publica em `/livox/pontos` é a própria ponte;
este nó só roda no robô.

## O que ele NÃO resolve, e é preciso saber antes de testar

⚠️ **Sozinho, ele não faz os costmaps marcarem.** Há um segundo defeito em
série, medido no mesmo dia: com o `tf_odom` compondo a pose do sensor, o `odom`
fica na ALTURA DO SENSOR (`tf2_echo odom base_link` deu z = −0,477 m), o chão vai
para z ≈ −0,42 e a faixa de altura da camada de obstáculo (0,10–0,50 m, aplicada
no frame global) rejeita tudo. Ver a decisão 017.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

from robot_base.nuvem import CAMPOS, PASSO_PONTO, empacota


class NuvemPontos(Node):
    def __init__(self):
        super().__init__('nuvem_pontos')
        p = self.declare_parameters('', [
            ('entrada', '/livox/lidar'),
            ('saida', '/livox/pontos'),
            # [m] raio do CILINDRO em torno do eixo do sensor cujos pontos são
            # do próprio robô e não podem virar obstáculo. Ver a decisão 027 e
            # o docstring de `ponto_valido`: com 0,0 o nó volta ao
            # comportamento de antes de 12-08, que é como se MEDE se a peça ao
            # lado do sensor era a culpada.
            ('raio_cego', 0.15),
        ])
        self.par = {x.name: x.value for x in p}

        # Importado AQUI, e não no topo, de propósito: o `livox_ros_driver2` só
        # existe onde o driver foi compilado (o NUC). Importar no topo do módulo
        # tornaria o pacote inteiro impossível de carregar na máquina de dev.
        try:
            from livox_ros_driver2.msg import CustomMsg
        except ImportError as e:  # noqa: BLE001
            self.get_logger().error(
                f'sem `livox_ros_driver2.msg`: {e}\n'
                'Este nó só roda onde o driver do Livox foi compilado. No '
                'simulador ele NÃO deve subir — lá a ponte do Gazebo já '
                'publica PointCloud2 direto em `saida`.')
            raise

        self.pub = self.create_publisher(
            PointCloud2, self.par['saida'], qos_profile_sensor_data)
        self.create_subscription(
            CustomMsg, self.par['entrada'], self.passo,
            qos_profile_sensor_data)

        self.campos = [PointField(name=n, offset=o, datatype=d, count=c)
                       for n, o, d, c in CAMPOS]
        self.n = 0
        # O pré-voo e a bancada procuram esta linha: sem ela não há como saber,
        # olhando o robô rodando, se o filtro do corpo está valendo ou não.
        self.get_logger().warn(
            f"raio cego do proprio robo: {self.par['raio_cego']:.3f} m "
            '(decisão 027 — pontos mais perto que isto do EIXO do sensor são '
            'peça do robô e saem da nuvem). Com 0,000 o reflexo volta a poder '
            'disparar contra o próprio robô, que foi o defeito de 12-08.')
        self.get_logger().warn(
            f"nuvem_pontos: {self.par['entrada']} (CustomMsg) -> "
            f"{self.par['saida']} (PointCloud2). Sem esta ponte os costmaps e o "
            f"collision_monitor não recebem nuvem nenhuma no robô — medido em "
            f"10-08, com os dois costmaps em 0 células letais.")

    def passo(self, msg):
        dados, quantos, fora = empacota(
            ((p.x, p.y, p.z, float(p.reflectivity)) for p in msg.points),
            raio_cego=self.par['raio_cego'])

        fora_msg = PointCloud2()
        fora_msg.header = msg.header
        fora_msg.height = 1
        fora_msg.width = quantos
        fora_msg.fields = self.campos
        fora_msg.is_bigendian = False
        fora_msg.point_step = PASSO_PONTO
        fora_msg.row_step = PASSO_PONTO * quantos
        fora_msg.data = dados
        fora_msg.is_dense = True      # os inválidos já saíram no `empacota`
        self.pub.publish(fora_msg)

        self.n += 1
        if self.n == 1:
            self.get_logger().warn(
                f'primeira nuvem convertida: {quantos} pontos '
                f'({fora} inválidos descartados), frame '
                f'"{msg.header.frame_id}". Se o frame não for `livox_frame`, a '
                f'nuvem inteira sai deslocada e a TF não conserta.')
        # Diagnóstico contínuo e barato: quadro que chega com muito ponto
        # zerado é sensor obstruído, e isso não pode virar silêncio.
        self.get_logger().info(
            f'{quantos} pontos, {fora} inválidos', throttle_duration_sec=10.0)


def main():
    rclpy.init()
    no = NuvemPontos()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        no.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
