#!/usr/bin/env python3
"""Placa do hoverboard, fingida — o pedaço de robô ruim que faltava no Gazebo.

O simulador obedece qualquer comando, por menor que seja. A placa real **não**:
abaixo de uma certa velocidade de roda ela simplesmente ignora, e a roda não
sai do lugar. Isso não é detalhe — é o que já deixou o robô plantado no chão
sem erro nenhum no log (BO-3), e é o que decide se o robô consegue girar no
próprio eixo devagar ou não.

Este nó fica **entre** o controlador e o simulador e reproduz esse defeito:

    heading_controller -> /cmd_vel_bruto -> [placa_simulada] -> cmd_vel do
                                                                controlador

A zona morta age na **roda**, não no comando de alto nível — é lá que ela mora
de verdade. Então o nó converte o twist em velocidade de cada roda (a mesma
conta do `diff_drive_controller`), zera a que for pequena demais, e converte de
volta. Andando reto as duas rodas estão longe do limiar e nada acontece;
girando no lugar, as duas ficam pequenas ao mesmo tempo, e é aí que morde.

`zona_morta: 0.0` transforma o nó em fio: passa tudo adiante, sem alterar.

⚠️ O valor real deste robô é DESCONHECIDO (BO-3). O que está no launch é chute.
A graça de ter isto aqui é justamente poder desenvolver o controle contra uma
zona morta plausível e, quando a bancada medir a de verdade, trocar só o
número — nos dois lados.
"""
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class PlacaSimulada(Node):
    def __init__(self):
        super().__init__('placa_simulada')

        p = self.declare_parameters('', [
            # Velocidade de roda abaixo da qual a placa ignora o comando [m/s].
            # 0 = fio (sem zona morta).
            ('zona_morta', 0.10),
            # Têm que bater com o diff_drive_controller, senão a conversão
            # mente e o defeito simulado não é o defeito de verdade.
            # MEDIDA COM TRENA 2026-07-29 (era 0.20, herdada e nunca medida).
            ('bitola', 0.270),
            ('taxa_avisos', 2.0),
        ])
        self.par = {x.name: x.value for x in p}

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(
            TwistStamped, '/hoverboard_base_controller/cmd_vel', qos)
        self.create_subscription(TwistStamped, '/cmd_vel_bruto', self.cb, qos)

        self.get_logger().warn(
            f"placa simulada com zona morta de {self.par['zona_morta']} m/s "
            f"de roda (bitola {self.par['bitola']} m). "
            'Valor CHUTADO — o real sai do tools/banco (BO-3).')
        if self.par['zona_morta'] > 0.0:
            wz_min = 2.0 * self.par['zona_morta'] / self.par['bitola']
            self.get_logger().warn(
                f'com isso, girar parado abaixo de {wz_min:.2f} rad/s é '
                'impossível neste simulador — como no robô real')

    def cb(self, msg):
        v = msg.twist.linear.x
        wz = msg.twist.angular.z
        zm = self.par['zona_morta']
        meia = self.par['bitola'] / 2.0

        if zm > 0.0:
            ve = v - wz * meia
            vd = v + wz * meia
            engoliu = False
            if abs(ve) < zm:
                ve, engoliu = 0.0, True
            if abs(vd) < zm:
                vd, engoliu = 0.0, True
            v = (ve + vd) / 2.0
            wz = (vd - ve) / self.par['bitola']
            if engoliu and (abs(msg.twist.linear.x) > 1e-3
                            or abs(msg.twist.angular.z) > 1e-3):
                self.get_logger().warn(
                    f'engoli comando: pediram v={msg.twist.linear.x:.3f} '
                    f'wz={msg.twist.angular.z:.3f}, entregando v={v:.3f} '
                    f'wz={wz:.3f}',
                    throttle_duration_sec=1.0 / self.par['taxa_avisos'])

        fora = TwistStamped()
        fora.header = msg.header
        fora.twist.linear.x = float(v)
        fora.twist.angular.z = float(wz)
        self.pub.publish(fora)


def main():
    rclpy.init()
    no = PlacaSimulada()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        no.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
