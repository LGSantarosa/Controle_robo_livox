"""A geometria não pode divergir entre os arquivos que a repetem.

Este teste existe por causa de 2026-07-29. A bitola aparecia em quatro arquivos
e estava com valores DIFERENTES: 0,20 no simulador e 0,32 no robô, nenhum dos
dois medido, e os dois errados em sentidos opostos (a trena disse 0,270).
Sintonizar o rumo na bancada e levar para o robô teria errado duas vezes, em
direções contrárias — a bancada pareceria boa e o robô pioraria.

Não é zelo de estilo: é o defeito mais caro que este projeto teve, e ele não dá
sintoma. Um número copiado que envelhece sozinho não quebra nada visivelmente;
ele só faz a bancada medir uma máquina que não existe.
"""
import os
import re

import pytest

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
PRODUCAO = os.path.join(RAIZ, 'ros2_packages', 'robot_motion', 'config', 'nav2.yaml')
BANCADA = os.path.join(RAIZ, 'ros2_packages', 'robot_planning', 'config',
                       'bancada_planner.yaml')


def valor(caminho, chave):
    """Primeiro valor numérico da chave, ignorando comentários.

    Lido por texto e não por parser de YAML de propósito: os comentários destes
    arquivos são metade do valor deles, e um teste que exigisse reserializar
    convidaria alguém a jogá-los fora.
    """
    with open(caminho) as f:
        for linha in f:
            corte = linha.split('#')[0]
            m = re.search(rf'^\s*{re.escape(chave)}:\s*([-\d.]+)\s*$', corte)
            if m:
                return float(m.group(1))
    raise AssertionError(f'{chave} não encontrada em {caminho}')


def texto(caminho, chave):
    with open(caminho) as f:
        for linha in f:
            corte = linha.split('#')[0]
            m = re.search(rf'^\s*{re.escape(chave)}:\s*"?([A-Z_]+)"?\s*$', corte)
            if m:
                return m.group(1)
    raise AssertionError(f'{chave} não encontrada em {caminho}')


# `inflation_radius` NÃO está aqui, e a ausência é deliberada. Ele é ESCOLHA,
# não medida: a bancada julgou os planners com 0,45 (é o número por trás dos 48
# planos da decisão 008) e a produção roda com 0,30, porque 0,45 fazia o robô
# travar DENTRO da porta de 0,90 m — o replanejamento falha com "Start occupied"
# quando o vão inteiro está inflado. Divergir aqui é a decisão; divergir nos de
# baixo é o defeito da bitola.
@pytest.mark.parametrize('chave', [
    'minimum_turning_radius',   # o argumento da decisão 008; refém da zona morta
    'robot_radius',             # footprint: geometria da máquina, não política
])
def test_geometria_igual_na_bancada_e_na_producao(chave):
    p, b = valor(PRODUCAO, chave), valor(BANCADA, chave)
    assert p == b, (
        f'{chave} diverge: produção {p} × bancada {b}. Julgar o planner com um '
        'número e rodar o robô com outro é o erro da bitola de 29-07 de novo — '
        'a bancada parece boa e o robô piora.')


def test_o_modelo_de_movimento_e_o_mesmo_nos_dois():
    """Decisão 009: o plano não dá ré. Se a bancada julgar em Reeds-Shepp e o
    robô rodar em Dubins, o que foi aprovado não é o que anda."""
    p, b = texto(PRODUCAO, 'motion_model_for_search'), texto(
        BANCADA, 'motion_model_for_search')
    assert p == b == 'DUBIN', f'produção {p} × bancada {b}'


def test_o_raio_de_chegada_do_nav2_bate_com_o_do_seguidor():
    """Quem chega é o nosso seguidor; quem declara sucesso é o Nav2.

    Se a tolerância do `goal_checker` for menor que o `raio_chegada` do
    seguidor, o robô para e a árvore de comportamento continua pedindo — fica
    "navegando" parado até estourar o tempo. O padrão do seguidor está no
    `path_follower.py`; aqui trava-se o lado do Nav2.
    """
    from robot_motion import path_follower  # noqa: F401  (só para existir)
    assert valor(PRODUCAO, 'xy_goal_tolerance') == 0.25


# ------------------------------ o árbitro de comando (levantamento da 010)

def _mux():
    import yaml
    p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     'config', 'twist_mux.yaml')
    return yaml.safe_load(open(p))['twist_mux']['ros__parameters']


def test_humano_vence_a_autonomia_sempre():
    """O princípio que o CLAUDE.md diz valer nos DOIS robôs. Se alguém inverter
    isto, o robô deixa de ser controlável por uma pessoa — e o sintoma só
    aparece no pior momento possível, com ele indo para a parede."""
    t = _mux()['topics']
    auto = t['autonomia']['priority']
    for nome in ('teclado', 'web'):
        assert t[nome]['priority'] > auto, f'{nome} tem de vencer a autonomia'


def test_o_mux_fala_stamped_como_o_resto_da_cadeia():
    """A cadeia do robô 2 é TwistStamped de ponta a ponta. O robô 1 usava
    Twist cru, e copiar aquele valor faria o DDS rejeitar por type hash: o mux
    publicaria e ninguém consumiria — falha silenciosa, a classe de defeito que
    mais custou tempo neste projeto (BO-3)."""
    assert _mux()['use_stamped'] is True


def test_toda_fonte_tem_timeout():
    """Fonte sem timeout é comando velho vivo para sempre. Mesma regra do
    `timeout_alvo` da movimentação e do `timeout_plano` do seguidor: é o
    timeout que faz o robô PARAR quando quem comandava morreu."""
    for nome, cfg in _mux()['topics'].items():
        assert cfg.get('timeout', 0) > 0, f'{nome} sem timeout'
        assert cfg['timeout'] <= 1.0, f'{nome} com timeout longo demais'


def test_a_cadeia_da_launch_bate_com_a_config():
    """Os nomes de tópico vivem em dois arquivos (YAML e launch) e se
    divergirem o robô não anda — ou pior, anda sem árbitro. Este teste é o que
    impede a divergência de virar depuração de campo."""
    p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     'launch', 'pilha.launch.py')
    launch = open(p).read()
    topicos = {c['topic'] for c in _mux()['topics'].values()}
    assert 'auto_vel' in topicos
    # Desde 05-08 o heading_controller alimenta o CRU: entre ele e o mux
    # entrou o reflexo de colisão. Quem entrega `auto_vel` ao mux é o
    # collision_monitor, pela config dele — não por remap na launch.
    assert "'/auto_vel_raw'" in launch, \
        'o heading_controller tem de alimentar a entrada do reflexo'
    assert 'nav2_collision_monitor' in launch, 'o reflexo tem de subir'
    assert "'/cmd_vel_out', '/compensador_rumo/cmd_vel'" in launch, \
        'a saída do mux tem de entrar no compensador'


# ---------------------------------- o reflexo de colisão (levantamento da 010)

def _cm():
    import yaml
    p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     'config', 'collision_monitor.yaml')
    return yaml.safe_load(open(p))['collision_monitor']['ros__parameters']


def test_o_reflexo_filtra_a_autonomia_e_nao_o_humano():
    """A entrada dele é a autonomia CRUA e a saída é o que vai para o mux. Se
    alguém ligar a saída direto no atuador, o humano deixa de furar o reflexo —
    e some a única forma de tirar um robô que o próprio reflexo prendeu contra
    uma parede."""
    cm = _cm()
    assert cm['cmd_vel_in_topic'] == 'auto_vel_raw'
    assert cm['cmd_vel_out_topic'] == 'auto_vel'
    mux_topicos = {c['topic'] for c in _mux()['topics'].values()}
    assert 'auto_vel' in mux_topicos, 'a saída do reflexo tem de entrar no mux'
    assert 'auto_vel_raw' not in mux_topicos, 'o mux não pode pegar o cru'


def test_so_existe_acao_de_PARAR():
    """Desacelerar não é ação que este atuador saiba executar: a compensação
    entrega um patamar de ~0,30 m/s e não há velocidade entre 0 e isso. O robô
    1 tem um polígono de `limit`; copiar aquilo aqui seria configurar uma ação
    que a máquina não tem."""
    cm = _cm()
    for nome in cm['polygons']:
        assert cm[nome]['action_type'] == 'stop', f'{nome} não é stop'


def test_o_poligono_cobre_a_distancia_de_parada_MEDIDA():
    """A frente do polígono não é gosto: é meia caixa + o quanto o robô anda
    depois do corte, com os números de 31-07/01-08/04-08.

        0,298 × 0,5 (a placa empurra)  +  0,298²/(2·0,373)  +  0,433/2
    """
    frente_min = 0.298 * 0.5 + 0.298 ** 2 / (2 * 0.373) + 0.433 / 2
    pontos = eval(_cm()['PolygonStop']['points'])  # noqa: S307 (lista literal)
    assert max(x for x, _ in pontos) >= frente_min - 0.01, \
        f'o polígono tem de chegar a {frente_min:.3f} m à frente'


def test_o_reflexo_ignora_o_chao_e_o_que_passa_por_cima():
    """Abaixo de ~0,05 m o próprio chão vira obstáculo (a nuvem erra até
    3,8 cm em rasância, medido 05-08) e o robô se trancaria sozinho. Acima de
    0,50 ele passa por baixo, e marcar isso o faria recusar portas."""
    fonte = _cm()[_cm()['observation_sources'][0]]
    assert fonte['min_height'] >= 0.08
    assert fonte['max_height'] <= 0.60
    assert fonte['topic'] == '/livox/lidar'


def test_o_reflexo_fala_stamped_como_o_resto_da_cadeia():
    assert _cm()['enable_stamped_cmd_vel'] is True
