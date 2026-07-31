#!/usr/bin/env python3
"""Placa do hoverboard, fingida — agora com o atuador MEDIDO no robô (31-07).

O simulador obedece qualquer comando. A placa real não — e o que ela faz é
muito pior do que "ignorar comando pequeno", que era o modelo até 31-07.

## O que a bancada de 31-07 mediu, e que este nó reproduz

A cadeia real tem uma **compensação de zona morta dentro do driver**
(`hoverboard_driver.cpp`): quando a maior das duas rodas fica abaixo de
`deadband_speed`, ela multiplica AS DUAS por `k = deadband_speed/mx`. Preserva a
curva — e destrói a magnitude. Consequência medida:

    comando 0,10 m/s  ->  0,28 m/s de borda de roda   (2,8x)
    comando 0,25 m/s  ->  a MESMA coisa

Todo comando entre ~0,008 e ~0,838 m/s chega na placa como o mesmo número. Nessa
faixa **`cmd_vel` escolhe SENTIDO, não módulo** — e essa faixa é toda a faixa que
a navegação usa. É o defeito central deste robô, e o simulador tem de tê-lo,
senão a movimentação é ajustada contra uma máquina que não existe.

Três outras coisas medidas entram junto:

- **latência de ~0,27 s** entre o comando e a roda sair do lugar (n=4, faixa
  0,24–0,31). Não é filtro: é o motor vencendo a inércia. Muda o que um
  controlador a 10 Hz consegue fazer.
- **a escala do firmware não é a que o driver pensa.** O driver converte
  rad/s -> unidades por `/0,10472`; medido, 100 unidades entregam 3,72 rad/s de
  roda, ou seja **0,0372 rad/s por unidade**. O driver superestima a roda em
  **2,8x**. É por isso que o robô sempre andou mais rápido do que se pediu.
- **assimetria esquerda/direita dependente de sentido**: indo para a frente a
  esquerda gira 11–13% mais que a direita (n=2); de ré elas empatam (+6,7% e
  −0,9%, n=2). É o desvio de rumo do robô, medido no atuador.

## Os dois regimes

    compensacao LIGADA  (como o robô está hoje)
        mx <= 1 unidade   -> não anda
        acima disso       -> PATAMAR: sempre a mesma velocidade
    compensacao DESLIGADA
        abaixo da zona morta física -> não anda
        acima                       -> proporcional, na escala real

`modelo: ideal` transforma o nó em fio, para comparar contra a máquina perfeita.

Fonte dos números: `docs/MODELO_ROBO2.md` e a 4ª leva de 31-07 no `docs/DIARIO.md`.
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
            # 'medido'  — a placa de 31-07, com compensação e patamar (padrão);
            # 'cru'     — compensação desligada: zona morta física, proporcional;
            # 'ideal'   — fio, obedece tudo (para comparar).
            ('modelo', 'medido'),

            # --- o que o driver faz (hoverboard_driver.cpp) ---
            ('deadband_speed', 100.0),      # unidades de set_speed
            ('escala_driver', 0.10472),     # rad/s por unidade que o driver ASSUME

            # --- o que a placa faz de verdade (medido 31-07) ---
            # 100 unidades -> 3,72 rad/s de roda (n=4, faixa 3,56–3,82).
            ('escala_real', 0.0372),        # rad/s por unidade, MEDIDO
            ('latencia', 0.27),             # s até a roda sair do lugar (n=4)
            # Assimetria: fração a MAIS que a roda esquerda entrega.
            ('assimetria_frente', 0.12),    # 11–13% medidos (n=2)
            ('assimetria_re', 0.0),         # empatam (n=2)
            # Zona morta FÍSICA, só usada com modelo 'cru'. NÃO medida: a
            # bancada só bracketou entre 0,25 e 0,5 m/s de borda. Chute
            # conservador no meio da faixa, e é o principal furo do modelo.
            ('zona_morta_crua', 0.30),

            # Geometria — tem que bater com o diff_drive_controller.
            ('bitola', 0.270),
            ('raio', 0.080),
            ('taxa_avisos', 2.0),
        ])
        self.par = {x.name: x.value for x in p}

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(
            TwistStamped, '/hoverboard_base_controller/cmd_vel', qos)
        self.create_subscription(TwistStamped, '/cmd_vel_bruto', self.cb, qos)

        self.t_pedido = None      # quando o comando saiu de zero (latência)
        self.parado = True

        self._anuncia()

    def _anuncia(self):
        m = self.par['modelo']
        if m == 'ideal':
            self.get_logger().warn(
                'placa IDEAL: passa tudo adiante. O robô do Gazebo vai obedecer '
                'o que o robô real NÃO obedece — use só para comparação.')
            return
        if m == 'cru':
            self.get_logger().warn(
                f"placa CRUA (compensação desligada): zona morta de "
                f"{self.par['zona_morta_crua']:.2f} m/s de borda. Este número "
                f"NÃO foi medido — só bracketado entre 0,25 e 0,5.")
            return
        piso, teto = self.faixa_do_patamar()
        self.get_logger().warn(
            f'placa MEDIDA (31-07): patamar de {piso:.3f} a {teto:.3f} m/s de '
            f'comando vira SEMPRE {self.patamar():.3f} m/s de borda. '
            f'Nessa faixa cmd_vel escolhe sentido, não módulo.')
        self.get_logger().warn(
            f"latência {self.par['latencia']:.2f} s, esquerda "
            f"{100 * self.par['assimetria_frente']:.0f}% mais forte indo à "
            f"frente, escala real {self.par['escala_real']:.4f} rad/s/unidade "
            f"({self.par['escala_driver'] / self.par['escala_real']:.1f}x menor "
            f"que a assumida pelo driver).")

    # ------------------------------------------------------------ conversões
    #
    # A conta é a MESMA do driver, de propósito: se ela divergir, o simulador
    # deixa de reproduzir o defeito e passa a ter um defeito próprio.

    def unidades(self, v_borda):
        """m/s de borda de roda -> unidades de set_speed, como o driver faz."""
        return v_borda / self.par['raio'] / self.par['escala_driver']

    def patamar(self):
        """Velocidade de borda que o patamar entrega [m/s]."""
        return (self.par['deadband_speed'] * self.par['escala_real']
                * self.par['raio'])

    def faixa_do_patamar(self):
        """Comandos (em m/s de borda) que caem dentro do patamar."""
        e = self.par['raio'] * self.par['escala_driver']
        return 1.0 * e, self.par['deadband_speed'] * e

    # ------------------------------------------------------------------ laço

    def cb(self, msg):
        v, wz = msg.twist.linear.x, msg.twist.angular.z
        modelo = self.par['modelo']

        if modelo == 'ideal':
            return self.publica(msg, v, wz)

        meia = self.par['bitola'] / 2.0
        ve, vd = v - wz * meia, v + wz * meia          # borda de cada roda [m/s]

        # Latência: o motor não sai do lugar no instante do comando. Zerar o
        # cronômetro só quando o comando volta a zero é o que reproduz o
        # arranque a cada nova ordem, que é onde ela aparece.
        agora = self.get_clock().now().nanoseconds * 1e-9
        pedindo = max(abs(ve), abs(vd)) > 1e-6
        if not pedindo:
            self.t_pedido, self.parado = None, True
        else:
            if self.t_pedido is None:
                self.t_pedido = agora
            if self.parado and agora - self.t_pedido < self.par['latencia']:
                return self.publica(msg, 0.0, 0.0)
            self.parado = False

        if modelo == 'cru':
            ve, vd, engoliu = self.cru(ve, vd)
        else:
            ve, vd, engoliu = self.medido(ve, vd)

        # Assimetria: a esquerda entrega mais indo para a frente. Depois da
        # zona morta, porque é ganho do atuador, não limiar.
        assim = (self.par['assimetria_frente'] if (ve + vd) >= 0
                 else self.par['assimetria_re'])
        ve *= (1.0 + assim)

        v = (ve + vd) / 2.0
        wz = (vd - ve) / self.par['bitola']

        if engoliu and (abs(msg.twist.linear.x) > 1e-3
                        or abs(msg.twist.angular.z) > 1e-3):
            self.get_logger().warn(
                f'engoli comando: pediram v={msg.twist.linear.x:.3f} '
                f'wz={msg.twist.angular.z:.3f}, entregando v={v:.3f} '
                f'wz={wz:.3f}',
                throttle_duration_sec=1.0 / self.par['taxa_avisos'])
        self.publica(msg, v, wz)

    def medido(self, ve, vd):
        """A placa como ela está hoje: compensação ligada, e o patamar.

        Reproduz `hoverboard_driver.cpp` linha a linha — escala as DUAS rodas
        pelo mesmo `k` até a maior vencer o deadband — e então aplica a escala
        REAL do firmware, que não é a que o driver supõe.
        """
        s_e, s_d = self.unidades(ve), self.unidades(vd)
        mx = max(abs(s_e), abs(s_d))

        if mx <= 1.0:
            # Abaixo disso o driver não compensa e a placa não anda. É este o
            # "limiar" que a bancada mediu (0,095 rad/s no giro) — aritmética
            # do driver mais a latência, NÃO atrito.
            return 0.0, 0.0, mx > 1e-9

        if mx < self.par['deadband_speed']:
            k = self.par['deadband_speed'] / mx
            s_e, s_d = s_e * k, s_d * k

        # unidades -> rad/s de roda, na escala MEDIDA -> m/s de borda
        e = self.par['escala_real'] * self.par['raio']
        return s_e * e, s_d * e, False

    def cru(self, ve, vd):
        """Compensação desligada: a zona morta física aparece, e o comando
        volta a ser proporcional — na escala real, que segue 2,8x menor."""
        zm = self.par['zona_morta_crua']
        razao = self.par['escala_real'] / self.par['escala_driver']
        ve, vd = ve * razao, vd * razao
        engoliu = False
        if abs(ve) < zm:
            ve, engoliu = 0.0, True
        if abs(vd) < zm:
            vd, engoliu = 0.0, True
        return ve, vd, engoliu

    def publica(self, msg, v, wz):
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
