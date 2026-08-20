"""O seguidor segue o plano SUAVIZADO, e cai para o cru só se ele calar (042).

Por que isto é travado por teste e não só por configuração: o defeito que a
042 conserta era exatamente uma fiação silenciosa — a árvore suavizava desde a
decisão 026 e o seguidor lia o tópico do PLANEJADOR, então o trabalho do
suavizador ia para o lixo sem uma linha de log. Medido nas 5 corridas do
protocolo de 14-08, na aproximação da porta:

    /plan            raio mínimo exigido  0,215 a 0,275 m   a máquina fecha 0,37
    /plan_smoothed                        0,402 a 0,477 m

⚠️ E a QUEDA para o cru é tão importante quanto a preferência pelo suave: sem
ela, um `SmoothPath` que recusa deixa o robô parado com objetivo aceito e plano
desenhado — o sintoma de 13-08 que não acusa ninguém no log.

Mesma técnica do `test_re_desligada`: `cb_plano` é chamada como função solta,
com um `self` de mentira, porque o que se quer travar é o RAMO de decisão.
"""
from robot_motion.path_follower import PathFollower


class LoggerFalso:
    def __init__(self):
        self.avisos = []

    def warn(self, msg, **kwargs):
        self.avisos.append(msg)


class PoseFalsa:
    class _P:
        x = y = 0.0

    class _O:
        x = y = z = 0.0
        w = 1.0

    def __init__(self, x, y):
        self.position = PoseFalsa._P()
        self.position.x, self.position.y = x, y
        self.orientation = PoseFalsa._O()


class MsgFalsa:
    def __init__(self, pontos, frame='map'):
        self.poses = [type('S', (), {'pose': PoseFalsa(x, y)})()
                      for (x, y) in pontos]
        self.header = type('H', (), {'frame_id': frame})()


class SeguidorFalso:
    """Só o que a `cb_plano` toca."""

    def __init__(self, t=100.0):
        self.par = {'timeout_plano': 2.0, 'aceita_plano_cru': False}
        self.t = t
        self.plano = []
        self.plano_frame = None
        self.t_plano = None
        self.t_plano_suave = None
        self.aceita_replano = True
        self.passagem_ativa = None
        self.passagem_fase = ''
        self.rumo_objetivo = None
        self.res_sem_plano = 7
        self.estado = 'ocioso'
        self._log = LoggerFalso()
        self.progresso = type('P', (), {'reinicia': lambda _s: None})()
        self.objetivo_vivo = True

    def agora(self):
        return self.t

    def get_logger(self):
        return self._log

    def tem_objetivo(self):
        return self.objetivo_vivo

    def atualiza_passagens(self):
        pass


CRU = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.9)]        # a quina que a máquina não fecha
SUAVE = [(0.0, 0.0), (1.0, 0.2), (2.0, 0.9)]


def chama(s, pontos, suave):
    PathFollower.cb_plano(s, MsgFalsa(pontos), suave=suave)


def test_o_suavizado_e_aceito():
    s = SeguidorFalso()
    chama(s, SUAVE, suave=True)
    assert s.plano == SUAVE
    assert s.t_plano_suave == s.t
    assert s.estado == 'seguindo'


def test_replano_do_mesmo_objetivo_nao_troca_rota_que_esta_funcionando():
    s = SeguidorFalso()
    chama(s, SUAVE, suave=True)
    rota_estavel = list(s.plano)
    s.t += 1.0
    outra_rota = [(0.0, 0.0), (1.0, -0.8), SUAVE[-1]]
    chama(s, outra_rota, suave=True)

    assert s.plano == rota_estavel
    assert s.t_plano == s.t, 'replano recusado ainda prova que o planner vive'


def test_falta_de_progresso_libera_uma_substituicao_de_rota():
    s = SeguidorFalso()
    chama(s, SUAVE, suave=True)
    s.aceita_replano = True
    outra_rota = [(0.0, 0.0), (1.0, -0.8), SUAVE[-1]]
    chama(s, outra_rota, suave=True)

    assert s.plano == outra_rota
    assert s.aceita_replano is False


def test_dentro_do_gargalo_nem_recuperacao_troca_o_eixo_no_meio():
    s = SeguidorFalso()
    chama(s, SUAVE, suave=True)
    s.passagem_ativa = object()
    s.aceita_replano = True
    outra_rota = [(0.0, 0.0), (1.0, -0.8), SUAVE[-1]]
    chama(s, outra_rota, suave=True)

    assert s.plano == SUAVE


def test_objetivo_novo_entra_imediatamente_mesmo_com_rota_travada():
    s = SeguidorFalso()
    chama(s, SUAVE, suave=True)
    novo_objetivo = [(0.0, 0.0), (0.0, 1.0), (8.0, 8.0)]
    chama(s, novo_objetivo, suave=True)

    assert s.plano == novo_objetivo


def test_o_cru_e_DESCARTADO_enquanto_o_suavizado_esta_vivo():
    """O ponto inteiro da 042: os dois tópicos carregam a MESMA missão, e o
    cru pede curva que a máquina não fecha."""
    s = SeguidorFalso()
    chama(s, SUAVE, suave=True)
    s.t += 1.0                      # dentro do `timeout_plano` de 2,0 s
    chama(s, CRU, suave=False)
    assert s.plano == SUAVE, 'o cru sobrescreveu o suavizado'


def test_o_cru_ASSUME_quando_o_suavizador_cala():
    """Sem esta queda, `SmoothPath` recusando = robô parado sem culpado."""
    s = SeguidorFalso()
    s.par['aceita_plano_cru'] = True
    chama(s, SUAVE, suave=True)
    s.t += 2.5                      # passou do `timeout_plano`
    chama(s, CRU, suave=False)
    assert s.plano == CRU
    assert any('CRU' in m for m in s._log.avisos), (
        'assumir o plano cru é degradação e tem de aparecer no log')


def test_sem_suavizador_nenhum_o_cru_vale_de_primeira():
    """Bancada e qualquer pilha sem `smoother_server`: nunca veio suavizado,
    então não há o que preferir e o robô tem de andar."""
    s = SeguidorFalso()
    s.par['aceita_plano_cru'] = True
    chama(s, CRU, suave=False)
    assert s.plano == CRU
    assert s._log.avisos == [], 'não é degradação: não havia suavizador'


def test_producao_nunca_congela_o_cru_que_chega_antes_do_suave():
    """Regressão da batida no aperto em 20-08: diferença medida de 8 ms."""
    s = SeguidorFalso()
    chama(s, CRU, suave=False)
    assert s.plano == []

    chama(s, SUAVE, suave=True)
    assert s.plano == SUAVE


def test_o_default_do_topico_e_o_suavizado():
    """Se o default voltar para `/plan`, a 042 é desfeita em silêncio."""
    import re
    src = open(PathFollower.__module__.replace('.', '/') + '.py').read() \
        if False else open(
            'ros2_packages/robot_motion/robot_motion/path_follower.py').read()
    m = re.search(r"\('topico_plano',\s*'([^']+)'\)", src)
    assert m and m.group(1) == '/plan_smoothed', (
        f'`topico_plano` é {m.group(1) if m else "indefinido"}, não o suavizado')
