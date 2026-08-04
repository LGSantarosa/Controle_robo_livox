"""A malha fechada de rumo em reta — lógica pura, sem ROS.

Está separada do nó (`compensador_rumo.py`) pelo padrão da casa: é ela que
carrega a decisão técnica (`docs/decisoes/011-malha-fechada-de-rumo-em-reta.md`)
e é ela que precisa ser testável sem subir simulador nenhum.

O que ela corrige: este robô comandado a ir reto descreve um círculo
(−0,817 1/m de frente, −0,098 de ré — medido em 04-08). O que a torna
possível: a compensação de zona morta do driver preserva a RAZÃO entre as
rodas, então a curvatura comandada sobrevive ao patamar — há autoridade
contínua sobre o rumo, mesmo sem autoridade nenhuma sobre a velocidade.

    wz_saida = wz_ff + Kp·e + Ki·∫e          e = norm(rumo_ref − yaw)
    wz_ff    = −curvatura_medida(sentido) · |v_cmd|

PI, não PID: o viés é constante (caso de livro do integrador) e um D
derivaria pose de 10 Hz contra um atuador com ~0,27 s de latência — só
amplificaria ruído.
"""
import math


def norm_ang(a):
    """Traz um ângulo para (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class MalhaDeReta:
    """Segura o rumo capturado enquanto o comando pedir reta.

    Uma instância por robô: ela carrega estado (referência de rumo e
    integrador), e o estado tem regras próprias de descarte — ver `passo`.

    Parâmetros são grandezas com unidade, não ganhos soltos:
      curv_frente, curv_re  [1/m]   curvatura que o robô descreve comandado
                                    reto — a MEDIDA da bancada, com sinal.
      kp                    [1/s]   quanto de wz por rad de erro de rumo.
      ki                    [1/s²]  quanto de wz por rad·s acumulado.
      wz_max                [rad/s] grampo da CORREÇÃO (não do comando).
      int_max               [rad·s] grampo do integrador (anti-windup).
      limiar_curva          [rad/s] |wz_cmd| a partir do qual o comando é
                                    curva de verdade e passa intocado.
    """

    def __init__(self, curv_frente=-0.817, curv_re=-0.098, kp=1.0, ki=0.5,
                 wz_max=0.6, int_max=0.6, limiar_curva=0.05):
        if kp < 0.0 or ki < 0.0:
            raise ValueError('kp e ki não podem ser negativos')
        self.curv_frente = curv_frente
        self.curv_re = curv_re
        self.kp = kp
        self.ki = ki
        self.wz_max = wz_max
        self.int_max = int_max
        self.limiar_curva = limiar_curva
        self.rumo_ref = None      # capturado ao entrar em reta
        self.integral = 0.0       # [rad·s]
        self.sentido = 0          # +1 frente, -1 ré, 0 parado

    def passo(self, v_cmd, wz_cmd, yaw, dt):
        """Um ciclo: devolve o wz corrigido para (v_cmd, wz_cmd) dados.

        `yaw` é o rumo atual [rad] (LIO no robô, pose verdadeira no Gazebo);
        `dt` é o tempo desde o último passo [s]. `v_cmd` sai como entrou —
        esta lei não toca na velocidade (não teria autoridade: patamar).
        """
        # Curva pedida de verdade passa INTOCADA e rearma a captura: curva é
        # assunto do comandante. Corrigi-la aqui brigaria com quem pediu.
        if abs(wz_cmd) >= self.limiar_curva:
            self._descarta()
            return wz_cmd

        # Parado não há rumo a segurar — e referência velha é pior que
        # nenhuma: o robô pode ter sido girado no chão enquanto esperava.
        if abs(v_cmd) < 1e-9:
            self._descarta()
            return wz_cmd

        sentido = 1 if v_cmd > 0.0 else -1
        if sentido != self.sentido:
            # O viés da frente (−0,82) não é o da ré (−0,10): integrador
            # carregado do sentido errado viraria chicote na troca.
            self._descarta()
            self.sentido = sentido
        if self.rumo_ref is None:
            self.rumo_ref = yaw

        e = norm_ang(self.rumo_ref - yaw)

        # dt não-positivo (relógio andou para trás, primeira amostra) não
        # pode envenenar o integrador; o termo P segue valendo.
        if dt > 0.0:
            self.integral = max(-self.int_max,
                                min(self.int_max, self.integral + e * dt))

        curv = self.curv_frente if sentido > 0 else self.curv_re
        wz_ff = -curv * abs(v_cmd)

        wz = wz_ff + self.kp * e + self.ki * self.integral
        return max(-self.wz_max, min(self.wz_max, wz))

    def _descarta(self):
        self.rumo_ref = None
        self.integral = 0.0
        self.sentido = 0
