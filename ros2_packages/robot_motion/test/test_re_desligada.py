"""A ré desligada não pode MATAR o seguidor — regressão de 13-08.

O defeito, medido no robô: com `re_max_seguidas: 0` a guarda de teto de rés é
verdadeira já na primeira vez que o robô emperra, e a mensagem de erro formata
`dist_antes_da_re` — que só existe DEPOIS de uma ré ter acontecido. Com a ré
desligada nunca há primeira ré:

    TypeError: unsupported format string passed to NoneType.__format__
    process has died [pid 16332, exit code 1]

Sintoma no robô: objetivo aceito pelo `bt_navigator`, plano desenhado na tela
do serviço web, e o robô PARADO — sem nenhuma mensagem culpando ninguém.
Seguidor morto não dirige, e nó morto não reclama.

⚠️ Estes testes chamam `entra_na_re` como função solta, com um `self` de
mentira. É de propósito: subir o nó de verdade exigiria ROS de pé, e o que se
quer travar aqui é o RAMO de decisão, que é lógica pura.
"""
import pytest

from robot_motion.path_follower import PathFollower


class LoggerFalso:
    """Só registra o que foi dito, para o teste poder afirmar sobre a mensagem."""

    def __init__(self):
        self.avisos = []
        self.erros = []

    def warn(self, msg, **kwargs):
        self.avisos.append(msg)

    def error(self, msg, **kwargs):
        self.erros.append(msg)


class ProgressoFalso:
    def __init__(self):
        self.reiniciado = 0

    def reinicia(self):
        self.reiniciado += 1


class SeguidorFalso:
    """O mínimo que `entra_na_re` toca antes de decidir se recua."""

    def __init__(self, **par):
        base = {
            're_habilitada': True,
            're_max_seguidas': 2,
            're_scan_velho_s': 0.8,
            're_folga': 0.30,
            're_orcamento_cego': 0.30,
        }
        base.update(par)
        self.par = base
        self.logger = LoggerFalso()
        self.progresso = ProgressoFalso()
        self.res_seguidas = 0
        self.dist_antes_da_re = None
        self.vao_pedido = 0

    def get_logger(self):
        return self.logger

    def vao_traseiro(self):
        # Se o código chegar aqui, a ré NÃO foi barrada — é o que os testes de
        # desligamento afirmam que não pode acontecer.
        self.vao_pedido += 1
        return 1.0


def test_teto_zero_nao_mata_o_seguidor():
    """O defeito exato de 13-08: teto 0 com `dist_antes_da_re` ainda None."""
    seg = SeguidorFalso(re_max_seguidas=0)

    PathFollower.entra_na_re(seg, t=1.0, x=0.0, y=0.0, dist=2.0)

    assert seg.vao_pedido == 0, 'com teto zero a ré não pode nem medir o vão'
    assert seg.logger.avisos, 'desligar a ré em silêncio é o pior dos mundos'


def test_re_desligada_pelo_knob_proprio_tambem_avisa():
    """`re_habilitada: False` é o interruptor certo, e ele já existia."""
    seg = SeguidorFalso(re_habilitada=False)

    PathFollower.entra_na_re(seg, t=1.0, x=0.0, y=0.0, dist=2.0)

    assert seg.vao_pedido == 0
    assert seg.progresso.reiniciado == 1, (
        'sem reiniciar o contador de progresso o seguidor volta aqui a cada '
        'passo e enche o log')


def test_teto_atingido_com_distancia_conhecida_nao_estoura():
    """O caminho normal: teto 2, duas rés feitas, distância gravada."""
    seg = SeguidorFalso(re_max_seguidas=2)
    seg.res_seguidas = 2
    seg.dist_antes_da_re = 2.50

    PathFollower.entra_na_re(seg, t=1.0, x=0.0, y=0.0, dist=3.0)

    assert seg.vao_pedido == 0, 'teto atingido não pode virar mais uma ré'
    assert seg.logger.erros, 'teto atingido é ERRO, não aviso'
    assert '2.50' in seg.logger.erros[0]


def test_teto_atingido_sem_distancia_gravada_nao_estoura():
    """O cinto: `dist_antes_da_re` None nesta guarda não pode matar o nó.

    Não deveria acontecer (quem incrementa `res_seguidas` também grava a
    distância), mas formatar None mata o processo — e nó morto não dirige.
    """
    seg = SeguidorFalso(re_max_seguidas=1)
    seg.res_seguidas = 1
    seg.dist_antes_da_re = None

    PathFollower.entra_na_re(seg, t=1.0, x=0.0, y=0.0, dist=3.0)

    assert seg.vao_pedido == 0
    assert seg.logger.erros


@pytest.mark.parametrize('teto', [0, -1])
def test_qualquer_teto_nao_positivo_desliga_a_re(teto):
    """Teto negativo é erro de digitação, e tem de cair no ramo seguro."""
    seg = SeguidorFalso(re_max_seguidas=teto)

    PathFollower.entra_na_re(seg, t=1.0, x=0.0, y=0.0, dist=2.0)

    assert seg.vao_pedido == 0
