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

    # ⚠️ `kp` e `ki` reduzidos 4,1x em 05-08 (eram 1,0 e 0,5), porque com os
    # valores antigos o robô OSCILA e a oscilação CRESCE 2,07x por meio-período.
    # É tempo morto: 0,94 s de atraso efetivo no laço, confirmado por duas
    # rotas independentes (a frequência da oscilação medida, e a soma dos
    # atrasos de liga e desliga da placa). O racional completo, com os números
    # e o preço, está no `compensador_rumo.py`, ao lado do `declare_parameters`.
    #
    # ⚠️ Estes defaults têm de acompanhar os do nó — há um teste que compara os
    # dois, porque default duplicado é default que deriva.
    def __init__(self, curv_frente=-0.817, curv_re=-0.098, kp=0.25, ki=0.12,
                 wz_max=0.6, int_max=0.6, limiar_curva=0.05,
                 segura_rumo=True):
        if kp < 0.0 or ki < 0.0:
            raise ValueError('kp e ki não podem ser negativos')
        # `segura_rumo=False` deixa só o FEEDFORWARD: cancela o arco do corpo
        # e não escolhe rumo nenhum. É o modo para quando há um controlador de
        # rumo ACIMA (o `heading_controller`, na pilha). Com ele ligado os dois
        # disputam: este nó segura o rumo que CAPTUROU, que não é o rumo que o
        # caminho quer, e o de cima tem de pivotar para desfazer — foi o
        # sintoma que o dono viu em 05-08 ("parece que ele está sem o PID, tá
        # pendendo pra direita, aí o pivô para tendo que arrumar isso").
        self.segura_rumo = segura_rumo
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

    def passo(self, v_cmd, wz_cmd, yaw, dt, v_real=None):
        """Um ciclo: devolve o wz corrigido para (v_cmd, wz_cmd) dados.

        `yaw` é o rumo atual [rad] (LIO no robô, pose verdadeira no Gazebo);
        `dt` é o tempo desde o último passo [s]. `v_real` é a velocidade
        MEDIDA — ver abaixo. `v_cmd` sai como entrou: esta lei não toca na
        velocidade (não teria autoridade: patamar).

        ⚠️ O feedforward escala com a velocidade **REAL**, não com a pedida.
        O arco é curvatura × distância percorrida, e quem decide a distância é
        o patamar da placa, não o comando. Medido em 05-08 dentro da pilha: o
        seguidor pedia 0,500 m/s, o robô andava 0,299, e o ff saía +0,408
        rad/s onde bastavam +0,244 — 67% a mais. Sem `v_real` a lei cai no
        comando, que é o certo só quando os dois coincidem.
        """
        v_ff = abs(v_real) if v_real is not None else abs(v_cmd)

        # Parado não há arco: ele nasce do movimento. E não há rumo a segurar
        # — referência velha é pior que nenhuma, o robô pode ter sido girado
        # no chão enquanto esperava.
        if abs(v_cmd) < 1e-9:
            self._descarta()
            return wz_cmd

        sentido = 1 if v_cmd > 0.0 else -1
        curv = self.curv_frente if sentido > 0 else self.curv_re
        ff = -curv * v_ff

        # O arco existe girando também: ele é do CORPO, não do comando. Por
        # isso o ff entra sempre, inclusive na curva pedida. O que a curva
        # dispensa é a malha de rumo — essa sim é assunto do comandante.
        if abs(wz_cmd) >= self.limiar_curva or not self.segura_rumo:
            if abs(wz_cmd) >= self.limiar_curva:
                self._descarta()
            return wz_cmd + ff
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

        wz = ff + self.kp * e + self.ki * self.integral
        return max(-self.wz_max, min(self.wz_max, wz))

    def _descarta(self):
        self.rumo_ref = None
        self.integral = 0.0
        self.sentido = 0
