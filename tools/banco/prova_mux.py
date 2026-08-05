#!/usr/bin/env python3
"""Prova a arbitragem do `twist_mux` — o freio de mão — SEM robô e SEM humano.

    ros2 run twist_mux twist_mux --ros-args \\
        --params-file install/robot_motion/share/robot_motion/config/twist_mux.yaml \\
        -r /cmd_vel_out:=/compensador_rumo/cmd_vel
    python3 tools/banco/prova_mux.py

## Por que isto existe

O teste C do `docs/PLANO_TESTE_ROBO.md` ("o freio de mão funciona") tem três
itens, e **dois deles não precisam de robô nem de dedo no teclado**:

    2. com a autonomia comandando, apertar w/s VENCE — o humano manda;
    3. soltar devolve o comando à autonomia.

São afirmações sobre arbitragem de tópico, não sobre movimento. Provar isso na
bancada é de graça, e deixa para o laboratório só o que **precisa** de máquina:
o item 1 (o homem-morto para o robô em 0,4 s), que é sobre a roda parar.

Fazer isso aqui também protege contra o modo de falha mais chato do mux: ele
não reclama quando ninguém escuta. Se o tipo da mensagem divergir (`Twist` cru
contra `TwistStamped`, que é toda a cadeia deste robô), o DDS rejeita por
type hash, o mux publica no vazio e **o log fica limpo**. Um robô com freio de
mão que não freia, sem uma linha de erro.

## O que ele mede, e o que NÃO mede

Mede: quem ganha a saída quando duas fontes falam junto, e se o comando volta.
NÃO mede: se o robô obedece. Isso é o item 1, e é hands-on.

⚠️ **O regime é o que vale, não a transição.** O mux age na chegada da
mensagem, então nos primeiros instantes em que o humano começa a falar ainda
pode sair uma mensagem da autonomia — ela foi processada antes de o
handler do teclado ficar ativo. Por isso a leitura separa `tudo` (a fase
inteira) de `regime` (a segunda metade). Julgue pelo regime; a primeira versão
deste script reprovou uma arbitragem CORRETA por não fazer essa distinção.
"""
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

AUTONOMIA = 0.10      # o que a pilha do Nav2 estaria pedindo
HUMANO = 0.90         # o que o dedo no teclado pede
SAIDA = '/compensador_rumo/cmd_vel'   # onde o mux entrega (ver pilha.launch.py)


class ProvaDoMux(Node):
    def __init__(self):
        super().__init__('prova_mux')
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.auto = self.create_publisher(TwistStamped, '/auto_vel', q)
        self.key = self.create_publisher(TwistStamped, '/key_vel', q)
        self.saida = []
        self.create_subscription(
            TwistStamped, SAIDA,
            lambda m: self.saida.append(m.twist.linear.x), q)

    def manda(self, pub, v):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = float(v)
        pub.publish(m)

    def fase(self, nome, segundos, fontes):
        self.saida.clear()
        fim = time.time() + segundos
        while time.time() < fim:
            for pub, v in fontes:
                self.manda(pub, v)
            rclpy.spin_once(self, timeout_sec=0.02)
        v = [round(x, 2) for x in self.saida]
        regime = sorted(set(v[len(v) // 2:]))
        print(f'  {nome:<34} tudo={sorted(set(v))}  regime={regime}  '
              f'({len(v)} msgs)')
        return regime


def main():
    rclpy.init()
    p = ProvaDoMux()
    time.sleep(1.0)
    print(f'autonomia pede {AUTONOMIA} (prio 10), humano pede {HUMANO} '
          f'(prio 90)\n')

    a = p.fase('1. só a autonomia', 1.5, [(p.auto, AUTONOMIA)])
    b = p.fase('2. autonomia + humano juntos', 1.5,
               [(p.auto, AUTONOMIA), (p.key, HUMANO)])
    c = p.fase('3. só a autonomia de novo', 1.5, [(p.auto, AUTONOMIA)])

    p.saida.clear()
    fim = time.time() + 1.5
    while time.time() < fim:
        rclpy.spin_once(p, timeout_sec=0.02)
    mudo = not [x for x in p.saida[-5:] if abs(x) > 1e-9]
    print(f'  {"4. ninguém publica (timeout 0,5 s)":<34} '
          f'{len(p.saida)} msgs depois do silêncio')

    print()
    veredito = [
        (a == [AUTONOMIA], 'sozinha, a autonomia passa'),
        (b == [HUMANO], f'o humano VENCE a autonomia ({HUMANO}, e nada de '
                        f'{AUTONOMIA})'),
        (c == [AUTONOMIA], 'soltando o teclado, o comando VOLTA para a autonomia'),
        (mudo, 'com todos calados, comando velho NÃO é repetido'),
    ]
    for ok, texto in veredito:
        print(f'  {"✓" if ok else "✗ FALHOU:"} {texto}')

    tudo_ok = all(ok for ok, _ in veredito)
    print(f'\n  RESULTADO: {"TUDO OK" if tudo_ok else "TEM FALHA"}')
    print('\n  Falta o item 1 do teste C, que PRECISA do robô: soltar o '
          'teclado\n  e o robô parar sozinho em 0,4 s (homem-morto).')

    p.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if tudo_ok else 1)


if __name__ == '__main__':
    main()
