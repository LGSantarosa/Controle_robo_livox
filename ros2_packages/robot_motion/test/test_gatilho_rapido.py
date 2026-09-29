"""O gatilho RÁPIDO da recuperação — os dois portões do mapa (29-09).

Porte do recovery contextual do robô 1 (`stuck_timeout_mapped`, 2026-06-22).
Bloqueio que o mapa JÁ conhecia autoriza decidir cedo: não há informação nova
por vir esperando. Novidade só do sensor continua pagando o teto cheio.

🔴 O DEFEITO QUE ESTE ARQUIVO EXISTE PARA TRAVAR, e ele custou uma corrida.

A 1ª leva portou só o portão FRONTAL, e na volta da corrida das 15:26 o robô
parou na porta 2 com o log dizendo *"EMPERRADO com frente livre (4,38 m)"*: o
corredor à frente estava limpo por quatro metros, porque quem segura o robô na
porta é a OMBREIRA AO LADO. A sonda frontal caía no vazio, respondia "não é
parede mapeada", o teto cheio de 4,0 s voltava a valer e a recuperação só vinha
6,84 s depois do `STOP`. O robô 1 sempre teve os dois portões, e o raio do
segundo foi de 0,35 para 0,6 m em 2026-06-28 pelo BO *"demorou ~15 s pra
desencalhar do conhecido"* — a mesma queixa.

O que se trava aqui é a fronteira, nos dois sentidos: parede ao lado ENCURTA, e
rearme entre rés NÃO encurta nunca.
"""
import math
import os
import sys
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'robot_motion'))

from robot_motion.path_follower import PathFollower     # noqa: E402


class LoggerFalso:
    def __init__(self):
        self.avisos = []

    def warn(self, msg, **kwargs):
        self.avisos.append(msg)

    def info(self, msg, **kwargs):
        pass


def _mapa(ocupadas, largura=120, altura=120, resolucao=0.05, frame='odom'):
    """OccupancyGrid mínimo. `ocupadas` em (linha, coluna), origem em (0, 0)."""
    dados = [0] * (largura * altura)
    for lin, col in ocupadas:
        dados[lin * largura + col] = 100
    return NS(header=NS(frame_id=frame),
              data=dados,
              info=NS(width=largura, height=altura, resolution=resolucao,
                      origin=NS(position=NS(x=0.0, y=0.0),
                                orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))))


def _celulas_em(x, y, resolucao=0.05):
    return int(y / resolucao), int(x / resolucao)


class SeguidorFalso:
    """O mínimo que `bloqueio_mapeado` e `teto_de_emperramento` tocam."""

    def __init__(self, mapa=None, **par):
        base = {
            're_parado_s': 4.0,
            're_parado_s_mapeado': 2.0,
            're_mapeado_alcance': 0.50,
            're_mapeado_raio_perto': 0.6,
            're_mapeado_vizinhanca': 0.22,
            're_mapeado_limiar': 65,
            'avanco_para_choque': 0.28,
        }
        base.update(par)
        self.par = base
        self.mapa = mapa
        self.pose = NS(header=NS(frame_id='odom'))
        self.res_seguidas = 0
        self.logger = LoggerFalso()
        self.tf_buffer = None       # só é tocado se os frames diferirem

    def get_logger(self):
        return self.logger

    def bloqueio_mapeado(self, x, y, rumo):
        # A implementação DE VERDADE: o teto e o portão têm de ser medidos
        # juntos, senão o teste do rearme passaria com um dublê que sempre diz
        # "é parede" — e não travaria nada.
        return PathFollower.bloqueio_mapeado(self, x, y, rumo)


def _bloqueio(seg, x=1.0, y=1.0, rumo=0.0):
    return PathFollower.bloqueio_mapeado(seg, x, y, rumo)


def _teto(seg, x=1.0, y=1.0, rumo=0.0):
    return PathFollower.teto_de_emperramento(seg, x, y, rumo)


# ── o portão que faltava: ombreira AO LADO ───────────────────────────────────

def test_parede_SO_AO_LADO_encurta_o_relogio():
    """🔴 A REGRESSÃO DA PORTA 2. Frente livre e ombreira a 0,40 m do centro:
    era isso que o robô via quando esperou 6,84 s parado."""
    seg = SeguidorFalso(_mapa([_celulas_em(1.0, 1.40)]))   # 0,40 m ao LADO
    assert _bloqueio(seg), 'ombreira ao lado tem de contar como conhecida'
    assert _teto(seg) == pytest.approx(2.0)


def test_parede_so_na_FRENTE_continua_encurtando():
    """O portão da 1ª leva não pode ter sido perdido: parede a 0,78 m à frente
    (para-choque 0,28 + alcance 0,50) segue valendo."""
    seg = SeguidorFalso(_mapa([_celulas_em(1.78, 1.0)]))
    seg.par['re_mapeado_raio_perto'] = 0.0      # desliga o portão novo
    assert _bloqueio(seg)
    assert _teto(seg) == pytest.approx(2.0)


def test_area_livre_nao_encurta_nada():
    """Sem parede em lugar nenhum, o teto é o cheio — é o comportamento de
    antes, e é o que impede o gatilho rápido de virar "dispara sempre"."""
    seg = SeguidorFalso(_mapa([]))
    assert not _bloqueio(seg)
    assert _teto(seg) == pytest.approx(4.0)


def test_parede_longe_dos_dois_portoes_nao_encurta():
    """1,2 m de lado está fora do raio de 0,6 e fora da sonda frontal."""
    seg = SeguidorFalso(_mapa([_celulas_em(1.0, 1.0 + 1.2)]))
    assert not _bloqueio(seg)


# ── a fronteira do rearme: o que a medida de 12-08 protege ───────────────────

def test_rearme_entre_res_NAO_encurta_nem_com_parede_mapeada():
    """🔴 O teto cheio existe contra realimentação positiva: a ré dura 1,6–2,8 s
    e recua 0,30 m, então relógio curto rearma a ré antes de o robô ter tempo
    físico de aproveitar a anterior. Enquanto uma ré não pagou o que gastou
    (`res_seguidas > 0`), o mapa não compra desconto nenhum."""
    seg = SeguidorFalso(_mapa([_celulas_em(1.0, 1.40)]))
    assert _teto(seg) == pytest.approx(2.0)
    seg.res_seguidas = 1
    assert _teto(seg) == pytest.approx(4.0), 'o rearme tem de ficar nos 4,0 s'


def test_curto_maior_ou_igual_ao_cheio_e_ignorado():
    """Sintonia incoerente não pode ATRASAR a decisão: o curto só entra se for
    mais curto mesmo."""
    seg = SeguidorFalso(_mapa([_celulas_em(1.0, 1.40)]))
    for v in (4.0, 9.0, 0.0, -1.0):
        seg.par['re_parado_s_mapeado'] = v
        assert _teto(seg) == pytest.approx(4.0), v


# ── sem mapa, sem pose, sem TF: nunca adianta ────────────────────────────────

def test_sem_mapa_fica_o_teto_cheio():
    """É o caso do robô REAL, que roda `mapa:=nenhum`. Sem mapa o gatilho
    rápido não existe — e é assim que ele não vaza para onde não foi medido."""
    seg = SeguidorFalso(None)
    assert not _bloqueio(seg)
    assert _teto(seg) == pytest.approx(4.0)


def test_sem_pose_nao_consulta_o_mapa():
    seg = SeguidorFalso(_mapa([_celulas_em(1.0, 1.40)]))
    seg.pose = None
    assert not _bloqueio(seg)


def test_frame_diferente_sem_TF_nao_adianta_e_avisa():
    """🔴 O defeito de 14-08 de novo: comparar `map` com `odom` sem a
    transformada é comparar coisa com coisa diferente. Sem TF, teto cheio."""
    class BufferQueFalha:
        def lookup_transform(self, *a, **k):
            from tf2_ros import TransformException
            raise TransformException('sem TF no teste')

    seg = SeguidorFalso(_mapa([_celulas_em(1.0, 1.40)], frame='map'))
    seg.tf_buffer = BufferQueFalha()
    assert not _bloqueio(seg)
    assert _teto(seg) == pytest.approx(4.0)
    assert any('não consulto o mapa' in m for m in seg.logger.avisos)


def test_frame_diferente_COM_TF_aplica_a_translacao():
    """Com TF, a sonda vai para onde o mapa realmente está. Deslocando o mapa
    2 m em x, a mesma parede só é achada depois de aplicar a transformada."""
    class BufferQueDesloca:
        def lookup_transform(self, *a, **k):
            return NS(transform=NS(
                translation=NS(x=2.0, y=0.0),
                rotation=NS(x=0.0, y=0.0, z=0.0, w=1.0)))

    # parede em (3,0; 1,40) no frame do MAPA = (1,0; 1,40) no frame da pose
    seg = SeguidorFalso(_mapa([_celulas_em(3.0, 1.40)], frame='map'))
    seg.tf_buffer = BufferQueDesloca()
    assert _bloqueio(seg), 'a translação do TF tem de ter sido aplicada'
    seg2 = SeguidorFalso(_mapa([_celulas_em(1.0, 1.40)], frame='map'))
    seg2.tf_buffer = BufferQueDesloca()
    assert not _bloqueio(seg2), 'sem aplicar o TF sondaria o lugar errado'


def test_limiar_e_respeitado_no_portao_de_perto():
    """Custo intermediário não é parede do mapa estático."""
    m = _mapa([])
    lin, col = _celulas_em(1.0, 1.40)
    m.data[lin * m.info.width + col] = 50
    seg = SeguidorFalso(m)
    assert not _bloqueio(seg)
    m.data[lin * m.info.width + col] = 70
    assert _bloqueio(seg)
