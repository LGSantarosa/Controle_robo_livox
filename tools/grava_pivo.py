#!/usr/bin/env python3
"""Grava, com carimbo de chegada, o que decide o puxão do robô 3 (sem mexer na pilha).

Só leitura, ao lado da pilha do Xbox (`bash bin/sobe-robo3`):
  rodas  /hoverboard/wheel_velocities  rpm da placa, FL = canal L, FR = canal R
  mega   /mega/debug                   steer, speed que a MEGA ESCREVEU na placa
  cmd    /cmd_vel                      linear.x, angular.z que o PC pediu
  bat    /battery/front                tensão das rodas (compara corridas)

O `/mega/debug` é o que tira o pacote ROS da conta: é o comando no fio.

Uso (no notebook, pilha no ar):
    python3 tools/grava_pivo.py ~/bancada_robo3/pivo_chao_<hora>.csv 300

Nasceu em 15-09 como script solto em ~/bancada_robo3 (pivo_chao_170422.csv).
"""

import csv
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64MultiArray, Int32MultiArray


def main():
    if len(sys.argv) < 3:
        sys.exit('uso: grava_pivo.py <saida.csv> <segundos>')
    rclpy.init()
    n = rclpy.create_node('grava_pivo')
    f = open(sys.argv[1], 'w', newline='')
    w = csv.writer(f)
    w.writerow(['t', 'topico', 'FL', 'FR', 'steer', 'speed', 'vx', 'wz', 'bat_V'])
    t0 = time.time()

    def agora():
        return f'{time.time() - t0:.3f}'

    # sensor_data (best effort) casa com publicador confiável ou não.
    n.create_subscription(
        Float64MultiArray, 'hoverboard/wheel_velocities',
        lambda m: w.writerow([agora(), 'rodas', m.data[0], m.data[1], '', '', '', '', '']),
        qos_profile_sensor_data)
    n.create_subscription(
        Int32MultiArray, 'mega/debug',
        lambda m: w.writerow([agora(), 'mega', '', '', m.data[0], m.data[1], '', '', '']),
        qos_profile_sensor_data)
    n.create_subscription(
        Twist, 'cmd_vel',
        lambda m: w.writerow([agora(), 'cmd', '', '', '', '', m.linear.x, m.angular.z, '']),
        10)
    n.create_subscription(
        BatteryState, 'battery/front',
        lambda m: w.writerow([agora(), 'bat', '', '', '', '', '', '', f'{m.voltage:.2f}']),
        qos_profile_sensor_data)

    fim = time.time() + float(sys.argv[2])
    try:
        while time.time() < fim:
            rclpy.spin_once(n, timeout_sec=0.1)
            f.flush()
    except KeyboardInterrupt:
        pass
    finally:
        f.close()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
