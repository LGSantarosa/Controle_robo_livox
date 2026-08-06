#!/usr/bin/env python3
"""Mede o **homem-morto**: soltou o teclado, quanto tempo até a roda parar?

    # robô LIGADO, base e pilha de pé, e num terminal SEPARADO: bin/robot-key
    python3 tools/banco/homem_morto.py --csv docs/dados/AAAA-MM-DD-.../homem-morto-a.csv

## Por que isto existe

O teste C item 1 do `docs/PLANO_TESTE_ROBO.md` é o único da lista que o plano
manda verificar **a olho** ("solte o teclado: ele para sozinho em 0,4 s"). Mas
0,4 s não se mede a olho — a olho só dá para dizer *que* parou, não *quando*.
Para um teste de segurança isso já vale alguma coisa; para um número que entra
no artigo, não vale nada.

E há um motivo mais forte: **não existe UM tempo de parada, existem três**, em
série, e só o do meio é o homem-morto propriamente dito. Confundi-los é fácil e
já rendeu uma acusação errada nesta sessão (ver `docs/DIARIO.md`, 06-08).

## Os três intervalos, e de quem é cada um

    tecla solta ──[A]──> /key_vel zera ──[B]──> mux entrega zero ──[C]──> roda para

    [A]  o homem-morto do TELEOP: parâmetro `solta` do teleop_teclado,
         default 0,4 s. É o que o plano chama de "para sozinho em 0,4 s".
         Este script mede [A] pelo tópico: o intervalo entre a ÚLTIMA mensagem
         não-nula em /key_vel e a PRIMEIRA nula.

         ⚠️ **[A] tem viés de +0 a +0,05 s, por construção.** O teleop publica
         a 20 Hz (`taxa`), então a última mensagem não-nula que se observa de
         fora pode estar até um período ANTES da tecla ter sido de fato solta.
         O valor honesto a esperar é **0,40 a 0,45 s**, e um 0,44 não é o
         teleop atrasando — é a régua. Só se pode acusar o `solta` acima disso.

    [B]  o mux repassando. Deve ser ~1 ciclo. NÃO confundir com o `timeout:
         0.5` do twist_mux.yaml: aquele é o fallback para o teleop MORRER
         (fonte que cala é fonte que some). Enquanto o teleop vive, ele publica
         ZERO explícito e o timeout do mux nunca entra em jogo.

    [C]  a placa e a inércia. Medido em 04-08: ~0,52 s de latência de desliga.
         É o pedaço que nenhum software conserta.

Um teleop que falhasse o homem-morto daria [A] crescendo sem parar — o robô
seguiria comandado depois de a pessoa largar o teclado. É esse o modo de falha
que manda PARAR TUDO.

## Como se opera

Este script só ESCUTA. Ele não publica nada e não move o robô. Quem move é o
dedo no `bin/robot-key`, num terminal separado. O protocolo é:

    1. sobe este gravador;
    2. no outro terminal, aperta `w` e segura ~2 s (o robô anda);
    3. SOLTA o teclado e não toca em mais nada por ~3 s;
    4. Ctrl-C aqui.

Repetir 3x. Uma solta por corrida — duas soltas no mesmo CSV se confundem.

⚠️ **O robô ANDA.** `v` do teleop é 0,25 m/s e ele vai andar enquanto a tecla
estiver sendo repetida. Espaço à frente como em qualquer corrida.
"""
import argparse
import csv
import sys

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# m/s — abaixo disso a roda está parada. O encoder do hoverboard tem ruído de
# quantização perto de zero; 0,02 fica acima dele e bem abaixo do patamar de
# 0,25 que o teleop comanda.
PARADO = 0.02


class Gravador(Node):
    def __init__(self, saida):
        super().__init__('homem_morto')
        self.linhas = []
        self.t0 = None
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.estado = {'key': 0.0, 'mux': 0.0, 'roda': 0.0}
        self.create_subscription(
            TwistStamped, '/key_vel',
            lambda m: self.grava('key', m.twist.linear.x), qos)
        self.create_subscription(
            TwistStamped, '/compensador_rumo/cmd_vel',
            lambda m: self.grava('mux', m.twist.linear.x), qos)
        self.create_subscription(
            Odometry, '/hoverboard_base_controller/odom',
            lambda m: self.grava('roda', m.twist.twist.linear.x), qos)
        self.saida = saida
        print('gravando. Aperte `w` no outro terminal, segure ~2 s, SOLTE.')
        print('Ctrl-C para fechar o arquivo.\n')

    def grava(self, campo, valor):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = t
        self.estado[campo] = valor
        self.linhas.append({
            't': round(t - self.t0, 4), 'fonte': campo,
            'key': round(self.estado['key'], 4),
            'mux': round(self.estado['mux'], 4),
            'roda': round(self.estado['roda'], 4),
        })

    def fecha(self):
        if not self.linhas:
            print('nada gravado — a pilha estava de pé?')
            return
        with open(self.saida, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=['t', 'fonte', 'key', 'mux', 'roda'])
            w.writeheader()
            w.writerows(self.linhas)
        print(f'\n{len(self.linhas)} amostras -> {self.saida}')
        self.leitura()

    def leitura(self):
        """Os três intervalos. Só faz sentido com UMA solta no arquivo."""
        def ultimo_nao_nulo(campo):
            r = [l for l in self.linhas if abs(l[campo]) > PARADO]
            return r[-1]['t'] if r else None

        def primeiro_nulo_depois(campo, t):
            r = [l for l in self.linhas
                 if l['t'] > t and abs(l[campo]) <= PARADO]
            return r[0]['t'] if r else None

        tk = ultimo_nao_nulo('key')
        if tk is None:
            print('nenhum comando não-nulo em /key_vel — a tecla chegou ao nó?')
            return
        zk = primeiro_nulo_depois('key', tk)
        zm = primeiro_nulo_depois('mux', tk) if zk else None
        zr = primeiro_nulo_depois('roda', tk) if zk else None

        print('\n  intervalo                                     medido   esperado')
        if zk:
            a = zk - tk
            veredito = 'ok' if a <= 0.45 else '🛑 ACIMA — ver o parâmetro `solta`'
            print(f'  [A] tecla solta -> /key_vel zera            {a:6.3f} s  0,40-0,45  {veredito}')
        if zk and zm:
            print(f'  [B] /key_vel zera -> mux entrega zero       {zm - zk:6.3f} s    ~0,05')
        if zk and zr:
            print(f'  [C] mux zera -> roda para                   {zr - (zm or zk):6.3f} s    ~0,52')
            print(f'  ------------------------------------------------------')
            print(f'  TOTAL da solta até a roda parar             {zr - tk:6.3f} s')
        if not zr:
            print('  🛑 A RODA NUNCA PAROU dentro do arquivo. Se isso se repetir,')
            print('     é o modo de falha que manda PARAR TUDO.')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', required=True)
    a = ap.parse_args()
    rclpy.init()
    no = Gravador(a.csv)
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        no.fecha()
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
