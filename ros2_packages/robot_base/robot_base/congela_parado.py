#!/usr/bin/env python3
"""Congela o `odom → base_link` enquanto as rodas estão paradas — sem ROS.

## Por que (decisão 050)

O AMCL só corrige a pose depois que o `odom` anda `update_min_d`/`update_min_a`.
No robô 1 o `odom` é de encoder: roda parada = deslocamento zero, e o AMCL
fica quieto. No robô 2 o `odom` é o LIO, que oscila e deriva com o robô
parado; somado, isso passa do limiar e o AMCL reamostra com o robô parado —
a pose pula no mapa.

Aqui a gente imita o robô 1: com as DUAS rodas paradas há `espera` segundos,
a TF publicada fica parada, e o que o LIO andar nesse meio-tempo vai para uma
correção `C`. Quando as rodas voltam, publica `C ∘ LIO` — continua de onde
parou, sem degrau.

Roda sem leitura (driver caído, simulador) = NÃO congela: o LIO é a verdade, e
travar a TF de um robô que talvez esteja andando é pior que o pulo.
"""

try:
    from robot_base.transformadas import compoe, inverte
except ImportError:  # teste sem ROS: o diretório do módulo está no sys.path
    from transformadas import compoe, inverte

IDENT_T = (0.0, 0.0, 0.0)
IDENT_Q = (0.0, 0.0, 0.0, 1.0)


class CongelaParado:
    def __init__(self, limiar=0.05, espera=0.5, validade=0.5):
        self.limiar = limiar        # rad/s; a placa mede em passos de 0,105
        self.espera = espera        # s parado antes de congelar
        self.validade = validade    # s sem leitura = leitura velha
        self.c_t, self.c_q = IDENT_T, IDENT_Q
        self.ultima = None          # última TF publicada (t, q)
        self.t_roda = None          # instante da última leitura de roda
        self.parado_desde = None

    def rodas(self, agora, v_esq, v_dir):
        self.t_roda = agora
        if abs(v_esq) < self.limiar and abs(v_dir) < self.limiar:
            if self.parado_desde is None:
                self.parado_desde = agora
        else:
            self.parado_desde = None

    def congelado(self, agora):
        return (self.t_roda is not None
                and agora - self.t_roda <= self.validade
                and self.parado_desde is not None
                and agora - self.parado_desde >= self.espera
                and self.ultima is not None)

    def passo(self, agora, t, q):
        """Recebe a pose do LIO (já no base_link) e devolve a que se publica."""
        if self.congelado(agora):
            # C = última ∘ LIO⁻¹, para que C ∘ LIO continue dando a última.
            t_i, q_i = inverte(t, q)
            self.c_t, self.c_q = compoe(*self.ultima, t_i, q_i)
            return self.ultima
        self.ultima = compoe(self.c_t, self.c_q, t, q)
        return self.ultima
