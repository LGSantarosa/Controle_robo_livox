"""Um contrato só na cadeia do robô 3 — etapa 5, passo 4 (PLANO_ETAPA5_ROBO3.md).

Tipo trocado entre quem publica e quem assina não dá erro: o nó simplesmente
não recebe, e o robô fica parado em silêncio. Por isso o corte é ATÔMICO e a
coerência é travada aqui, dos dois lados de cada tópico da cadeia do robô 3:

    joy_vel   teleop_twist_joy (publish_stamped_twist) → twist_mux (use_stamped)
    dpad_vel  dpad_reto (o tipo que ele publica)       → twist_mux (use_stamped)
    cmd_vel   twist_mux (use_stamped)                  → cmd_vel_to_wheels (use_stamped)

E o caminho LEGADO (`robot.launch.py`, `config/twist_mux.yaml`, usado pelo
`launch.sh`) continua cru: mexer nele viraria a cadeia do robô 2 sem ninguém
pedir (DIARIO 1331, decisão 049).
"""
import ast
import os

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import get_launch_description_from_python_launch_file
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest
import yaml

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
CONFIG = os.path.join(PKG, 'config')
FONTE = os.path.join(PKG, 'robot_nav')
FRAME = 'base_link'


def _yaml(nome):
    with open(os.path.join(CONFIG, nome)) as f:
        return yaml.safe_load(f)


def _params_do_no(nome_do_no, launch='controle_robo3.launch.py', monkeypatch=None):
    """Os parâmetros que a launch entrega ao nó, num LaunchContext real."""
    ld = get_launch_description_from_python_launch_file(os.path.join(PKG, 'launch', launch))
    ctx = LaunchContext()
    acoes = []
    for e in ld.entities:
        if isinstance(e, DeclareLaunchArgument):
            e.visit(ctx)
        elif isinstance(e, OpaqueFunction):
            if monkeypatch is not None:
                monkeypatch.setitem(e._OpaqueFunction__function.__globals__,
                                    'escolhe_joystick', lambda: (0, 'fingido (teste)'))
            acoes.extend(e.visit(ctx) or [])
        else:
            acoes.append(e)
    no = [a for a in acoes if isinstance(a, Node) and a._Node__node_name == nome_do_no]
    assert len(no) == 1, f'{nome_do_no}: {len(no)} nós'
    valores = {}
    for p in evaluate_parameters(ctx, no[0]._Node__parameters):
        if isinstance(p, dict):
            valores.update(p)
    return valores


def _tipo_publicado(modulo, topico):
    """O tipo que o nó cria como publisher do tópico (AST do fonte)."""
    arvore = ast.parse(open(os.path.join(FONTE, modulo)).read())
    for n in ast.walk(arvore):
        if (isinstance(n, ast.Call) and getattr(n.func, 'attr', '') == 'create_publisher'
                and len(n.args) >= 2 and isinstance(n.args[1], ast.Constant)
                and n.args[1].value == topico):
            return n.args[0].id
    return None


# ─── a cadeia do robô 3: TwistStamped dos dois lados ─────────────────────────

def test_teleop_publica_stamped_com_frame_explicito():
    p = _yaml('teleop_xbox_robo3.yaml')['teleop_twist_joy_node']['ros__parameters']
    assert p['publish_stamped_twist'] is True
    assert p.get('frame') == FRAME, 'o frame_id do TwistStamped, explícito'


def test_dpad_reto_publica_twiststamped_com_header():
    fonte = open(os.path.join(FONTE, 'dpad_reto.py')).read()
    assert _tipo_publicado('dpad_reto.py', 'dpad_vel') == 'TwistStamped'
    arvore = ast.parse(fonte)
    publica = [n for n in ast.walk(arvore)
               if isinstance(n, ast.Call) and getattr(n.func, 'attr', '') == 'publish']
    assert len(publica) >= 2, 'o comando e o zero ao soltar'
    # stamp e frame_id em TODA publicação (inclusive o zero de soltura): quem
    # monta a mensagem é sempre a mesma função.
    assert fonte.count('_mensagem(') >= 3, 'uma função só monta o TwistStamped'
    assert 'get_clock().now().to_msg()' in fonte and f"frame_id = '{FRAME}'" in fonte


@pytest.mark.parametrize('config, esperado', [
    ('twist_mux_robo3.yaml', True),     # a cadeia do robô 3
    ('twist_mux.yaml', False),          # LEGADO (robot.launch.py): continua cru
])
def test_mux_use_stamped(config, esperado):
    assert _yaml(config)['twist_mux']['ros__parameters']['use_stamped'] is esperado


def test_controle_robo3_manda_use_stamped_ao_cmd_vel_to_wheels(monkeypatch):
    p = _params_do_no('cmd_vel_to_wheels', monkeypatch=monkeypatch)
    assert p.get('use_stamped') is True


def test_legado_continua_cru():
    """`robot.launch.py` não passa `use_stamped`, e o default do nó é cru."""
    fonte = open(os.path.join(PKG, 'launch', 'robot.launch.py')).read()
    assert 'use_stamped' not in fonte
    arvore = ast.parse(open(os.path.join(FONTE, 'cmd_vel_to_wheels.py')).read())
    for n in ast.walk(arvore):
        if (isinstance(n, ast.Call) and getattr(n.func, 'attr', '') == 'declare_parameter'
                and n.args[0].value == 'use_stamped'):
            assert n.args[1].value is False, 'o default é cru — o legado depende disso'
            return
    raise AssertionError('cmd_vel_to_wheels sem o parâmetro use_stamped')


# ─── coerência dos dois lados de cada tópico ─────────────────────────────────

def _stamped_para_twist(stamped):
    return 'TwistStamped' if stamped else 'Twist'


def test_coerencia_dos_dois_lados_de_cada_topico(monkeypatch):
    mux = _yaml('twist_mux_robo3.yaml')['twist_mux']['ros__parameters']
    teleop = _yaml('teleop_xbox_robo3.yaml')['teleop_twist_joy_node']['ros__parameters']
    faixas = {f['topic'] for f in mux['topics'].values()}
    assert faixas == {'joy_vel', 'dpad_vel'}, faixas
    tipo_mux = _stamped_para_twist(mux['use_stamped'])

    publicado = {
        'joy_vel': _stamped_para_twist(teleop['publish_stamped_twist']),
        'dpad_vel': _tipo_publicado('dpad_reto.py', 'dpad_vel'),
    }
    for faixa, tipo in publicado.items():
        assert tipo == tipo_mux, f'{faixa}: publica {tipo}, o mux espera {tipo_mux}'

    # a saída do mux (cmd_vel) e quem assina
    assinado = _stamped_para_twist(
        _params_do_no('cmd_vel_to_wheels', monkeypatch=monkeypatch).get('use_stamped', False))
    assert assinado == tipo_mux, f'cmd_vel: o mux publica {tipo_mux}, o nó assina {assinado}'
