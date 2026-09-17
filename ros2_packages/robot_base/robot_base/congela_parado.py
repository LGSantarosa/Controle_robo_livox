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
    """Estado POR RODA: uma leitura nunca fala pela outra.

    🔴 Corrigido em 17-09. A primeira versão guardava UM `t_roda` para as duas
    rodas, e o `tf_odom` chamava `rodas()` a cada mensagem de QUALQUER lado,
    passando o valor em CACHE do outro (que nascia 0,0). Consequência: se o
    stream de uma roda nunca chegasse — driver caído, cabo solto —, a outra
    publicando zero mantinha a trava armada contra uma leitura que nunca
    existiu. A TF congelava com o robô possivelmente ANDANDO, empurrado pela
    roda muda, e a pose mentia sem sintoma.

    Agora cada roda tem valor e instante próprios, e só congela com as DUAS
    vistas e as DUAS frescas.
    """

    def __init__(self, limiar=0.05, espera=0.5, validade=0.5):
        self.limiar = limiar        # rad/s; a placa mede em passos de 0,105
        self.espera = espera        # s parado antes de congelar
        self.validade = validade    # s sem leitura = leitura velha
        self.c_t, self.c_q = IDENT_T, IDENT_Q
        self.ultima = None          # última TF publicada (t, q)
        self.t_roda = [None, None]  # instante da última leitura, POR RODA
        self.v_roda = [None, None]  # última velocidade lida, POR RODA
        self.parado_desde = None

    def roda(self, agora, i, v):
        """Uma leitura de UMA roda (0 = esquerda, 1 = direita)."""
        self.t_roda[i] = agora
        self.v_roda[i] = v
        self._reavalia(agora)

    def rodas(self, agora, v_esq, v_dir):
        """As duas rodas no mesmo instante — é o que os ensaios sintéticos usam.

        No nó real NÃO se usa isto: as mensagens chegam separadas, e juntá-las
        aqui reintroduziria o defeito de uma roda falar pela outra.
        """
        self.t_roda = [agora, agora]
        self.v_roda = [v_esq, v_dir]
        self._reavalia(agora)

    def _reavalia(self, agora):
        if self._parado():
            if self.parado_desde is None:
                self.parado_desde = agora
        else:
            self.parado_desde = None

    def _parado(self):
        return all(v is not None and abs(v) < self.limiar for v in self.v_roda)

    def _frescas(self, agora):
        return all(t is not None and agora - t <= self.validade
                   for t in self.t_roda)

    def congelado(self, agora):
        # 🔴 O CRONÔMETRO ZERA QUANDO A LEITURA ENVELHECE (corrigido 17-09).
        # `_reavalia` só roda quando CHEGA mensagem de roda, então durante um
        # apagão de um dos lados o `parado_desde` ficava congelado no valor
        # antigo. Quando a roda sumida voltava com zero, `_frescas` virava True
        # e o `agora - parado_desde >= espera` já estava satisfeito com o
        # carimbo VELHO: congelava no primeiro pacote, sem reobservar os 0,5 s
        # de robô parado. Reproduzido antes de consertar.
        #
        # Este método é chamado a cada pose do LIO, então serve de tique: sem
        # as duas leituras frescas, o robô volta a ter de PROVAR que está
        # parado desde agora.
        if not self._frescas(agora):
            self.parado_desde = None
            return False
        return (self.parado_desde is not None
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
