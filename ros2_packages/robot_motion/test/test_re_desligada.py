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
            're_exige_objetivo': True,
            'desencalhe_frente_dist': 0.20,
            'desencalhe_frente_folga': 0.10,
            're_bloqueio_frente_max': 0.20,
        }
        base.update(par)
        self.par = base
        self.logger = LoggerFalso()
        self.progresso = ProgressoFalso()
        self.res_seguidas = 0
        self.dist_antes_da_re = None
        self.vao_pedido = 0
        # Os testes de desligamento existiam antes da guarda de objetivo, e o
        # que eles travam é OUTRO ramo: nasce com objetivo vivo para seguirem
        # medindo o que sempre mediram.
        self.objetivo_vivo = True

    def get_logger(self):
        return self.logger

    def tem_objetivo(self):
        return self.objetivo_vivo

    def vao_traseiro(self):
        # Se o código chegar aqui, a ré NÃO foi barrada — é o que os testes de
        # desligamento afirmam que não pode acontecer.
        self.vao_pedido += 1
        return 1.0

    def vao_frente(self):
        # Obstáculo frontal confirmado: mantém a ré legítima como padrão dos
        # testes antigos. Casos sem bloqueio sobrescrevem este método.
        return 0.0


def test_sem_bloqueio_frontal_nao_da_re_por_falta_de_progresso():
    seg = SeguidorFalso()
    seg.vao_frente = lambda: 1.0

    PathFollower.entra_na_re(seg, t=1.0, x=0.0, y=0.0, dist=2.0)

    assert not hasattr(seg, 're_desde')
    assert seg.progresso.reiniciado == 1
    assert 'mero sintoma' in seg.logger.avisos[-1]


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


# --------------------------------- a ré sem objetivo vivo (14-08, decisão 031)
#
# O defeito, nas palavras do dono: *"ontem ela ativava do nada sem nada estar
# acontecendo, e pior, aconteceu no gazebo também"*. Mecanismo: o `/plan` fica
# RETIDO depois que o objetivo morre, o robô parado passa `re_parado_s` sem
# avançar, e o gatilho de emperramento dispara com ninguém tendo pedido nada.


def test_sem_objetivo_vivo_a_re_nao_acontece():
    """A guarda nova: plano retido + robô parado NÃO é motivo para recuar."""
    seg = SeguidorFalso()
    seg.objetivo_vivo = False

    PathFollower.entra_na_re(seg, t=1.0, x=0.0, y=0.0, dist=2.0)

    assert seg.vao_pedido == 0, 'sem objetivo a ré não pode nem medir o vão'
    assert seg.logger.avisos, 'recusar em silêncio é o que esconde o defeito'
    assert 'objetivo' in seg.logger.avisos[0].lower()
    assert seg.progresso.reiniciado == 1, (
        'sem reiniciar o progresso a guarda dispara em todo ciclo e o log '
        'vira enxurrada a 20 Hz')


def test_com_objetivo_vivo_a_re_segue_o_caminho_normal():
    """O contrapeso: a guarda não pode matar a ré legítima.

    Com objetivo vivo, `entra_na_re` tem de chegar até a MEDIDA DO VÃO — que é
    a primeira coisa concreta que ela faz depois das guardas.
    """
    seg = SeguidorFalso()
    seg.objetivo_vivo = True

    PathFollower.entra_na_re(seg, t=1.0, x=0.0, y=0.0, dist=2.0)

    assert seg.vao_pedido == 1, 'a ré com objetivo vivo tem de medir o vão'


def test_o_knob_permite_a_re_sem_objetivo_para_bancada():
    """`re_exige_objetivo: False` devolve o comportamento antigo.

    Existe para a bancada, onde o seguidor é dirigido por `/plan` cru e não há
    ação do Nav2 nenhuma. Não é o default — o default é o caso seguro (019).
    """
    seg = SeguidorFalso(re_exige_objetivo=False)
    seg.objetivo_vivo = False

    PathFollower.entra_na_re(seg, t=1.0, x=0.0, y=0.0, dist=2.0)

    assert seg.vao_pedido == 1


class SeguidorEmRe:
    """O mínimo que `passo_de_re` toca para terminar uma manobra."""

    def __init__(self):
        self.par = {'re_folga': 0.30, 're_orcamento_cego': 0.30,
                    're_teto_s': 8.0, 'v_piso': 0.203}
        self.logger = LoggerFalso()
        self.progresso = ProgressoFalso()
        self.estado = 're'
        self.re_desde = 0.0
        self.re_origem = (0.0, 0.0)
        self.desencalhe = []
        self.publicado = []

    def get_logger(self):
        return self.logger

    def vao_traseiro(self):
        return 1.0

    def publica_desencalhe(self, v):
        self.desencalhe.append(v)

    def publica(self, rumo, v):
        self.publicado.append((rumo, v))

    def registra(self, *a, **k):
        pass


def test_terminada_a_re_o_seguidor_VOLTA_A_SEGUIR():
    """A outra metade do requisito do dono: *"é pra desencalhar E ir até um
    ponto"*.

    Desencalhar e ficar parado ali não é recuperação — é o robô de costas no
    meio da sala. Terminado o recuo, o estado tem de voltar a `seguindo`, que
    é o que faz o próximo ciclo pegar o plano e retomar o caminho ao objetivo.
    """
    seg = SeguidorEmRe()

    # recuou 0,30 m (o orçamento cego inteiro) -> manobra esgotada
    PathFollower.passo_de_re(seg, t=2.0, x=-0.30, y=0.0, rumo=0.0, dist=2.3)

    assert seg.estado == 'seguindo', (
        'ré que termina e não devolve o seguidor ao plano deixa o robô parado '
        'de costas — desencalhou e não foi a lugar nenhum')
    assert seg.desencalhe[-1] == 0.0, (
        'sair da manobra sem zerar o canal deixa ré órfã até o timeout do mux')
    assert seg.progresso.reiniciado == 1
