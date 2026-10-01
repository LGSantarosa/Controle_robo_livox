"""Objetivo encerrado para o seguidor imediatamente (A2 da revisão)."""
from types import SimpleNamespace

import pytest

from robot_motion.path_follower import PathFollower


class Contador:
    def __init__(self):
        self.chamadas = 0

    def reinicia(self):
        self.chamadas += 1

    def reset(self):
        self.chamadas += 1


class LoggerFalso:
    def __init__(self):
        self.infos = []

    def info(self, msg, **kwargs):
        self.infos.append(msg)


class SeguidorFalso:
    ATIVOS = PathFollower.ATIVOS
    tem_objetivo = PathFollower.tem_objetivo
    encerra_objetivo = PathFollower.encerra_objetivo
    para = PathFollower.para

    def __init__(self, estado='seguindo'):
        posicao = SimpleNamespace(x=1.0, y=2.0)
        orientacao = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        pose = SimpleNamespace(position=posicao, orientation=orientacao)
        self.pose = SimpleNamespace(pose=SimpleNamespace(pose=pose))
        self._objetivo = {
            'navigate_to_pose/_action/status': True,
            'navigate_through_poses/_action/status': False,
        }
        self.plano = [(0.0, 0.0), (3.0, 0.0)]
        self.estado = estado
        self.progresso = Contador()
        self.correcao = Contador()
        self.publicados = []
        self.desencalhe = []
        self.logger = LoggerFalso()

    def publica(self, rumo, velocidade):
        self.publicados.append((rumo, velocidade))

    def publica_desencalhe(self, velocidade, wz=0.0):
        self.desencalhe.append((velocidade, wz))

    def get_logger(self):
        return self.logger


def mensagem_status(*status):
    return SimpleNamespace(
        status_list=[SimpleNamespace(status=valor) for valor in status])


@pytest.mark.parametrize('estado', ['seguindo', 're', 'pivo_escape'])
def test_fim_do_objetivo_para_todos_os_canais_e_descarta_plano(estado):
    seguidor = SeguidorFalso(estado)

    PathFollower.cb_status(
        seguidor, mensagem_status(6), 'navigate_to_pose/_action/status')

    assert seguidor.desencalhe == [(0.0, 0.0)]
    assert seguidor.publicados[-1][1] == 0.0
    assert seguidor.estado == 'ocioso'
    assert seguidor.plano == []


def test_outro_tipo_de_objetivo_ativo_impede_parada():
    seguidor = SeguidorFalso()
    seguidor._objetivo['navigate_through_poses/_action/status'] = True
    plano = list(seguidor.plano)

    PathFollower.cb_status(
        seguidor, mensagem_status(5), 'navigate_to_pose/_action/status')

    assert seguidor.plano == plano
    assert seguidor.publicados == []
    assert seguidor.desencalhe == []


def test_bancada_sem_action_nao_e_interrompida_por_status_inicial_inativo():
    seguidor = SeguidorFalso()
    seguidor._objetivo = {}
    plano = list(seguidor.plano)

    PathFollower.cb_status(
        seguidor, mensagem_status(4), 'navigate_to_pose/_action/status')

    assert seguidor.plano == plano
    assert seguidor.publicados == []
    assert seguidor.desencalhe == []


def test_canceling_ja_para_o_seguidor():
    """Decisão 059: cancelamento pedido é ordem de parar; não espera CANCELED."""
    seguidor = SeguidorFalso('re')

    PathFollower.cb_status(
        seguidor, mensagem_status(3), 'navigate_to_pose/_action/status')

    assert seguidor.desencalhe == [(0.0, 0.0)]
    assert seguidor.publicados[-1][1] == 0.0
    assert seguidor.estado == 'ocioso'
    assert seguidor.plano == []


def test_canceling_numa_action_com_a_outra_viva_nao_para():
    seguidor = SeguidorFalso()
    seguidor._objetivo['navigate_through_poses/_action/status'] = True
    plano = list(seguidor.plano)

    PathFollower.cb_status(
        seguidor, mensagem_status(3), 'navigate_to_pose/_action/status')

    assert seguidor.plano == plano
    assert seguidor.publicados == []


def test_canceling_nao_autoriza_movimento():
    assert 3 not in PathFollower.ATIVOS
    assert PathFollower.ATIVOS == {1, 2}
