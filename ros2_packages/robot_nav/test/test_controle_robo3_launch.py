"""O `controle_robo3` sobe o RSP do robô 3 — etapa 4, passo 6b (D4, plano §7).

Sem `robot_state_publisher` não existe `base_link → livox_frame`, e sem essa TF
nada do Livox chega ao corpo. O contrato, na launch de verdade:

- `robot_state_publisher` exatamente uma vez, com o URDF do `share/` do
  `robot_base` (`robo3.urdf.xacro`, `sim:=false`) — conferido por IGUALDADE
  com o xacro renderizado, e diferente do `sim:=true` (senão a conferência não
  distinguiria nada);
- os seis nós de hoje exatamente uma vez cada, e nenhum outro — em particular
  nenhum `joint_state_publisher`;
- os cinco argumentos e os defaults deles intactos;
- `robot_nav` declara `robot_base` como dependência de execução.

O `OpaqueFunction` roda num `LaunchContext` real e devolve as ações SEM
executá-las: nenhum processo ROS sobe. O `escolhe_joystick` é trocado por um
falso para o teste não abrir `/dev/input/js*` da máquina.
"""
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import get_launch_description_from_python_launch_file
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest
import xacro

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
LAUNCH = os.path.join(PKG, 'launch', 'controle_robo3.launch.py')
PACKAGE_XML = os.path.join(PKG, 'package.xml')

SEIS = ('mega_bridge', 'cmd_vel_to_wheels', 'joy_node', 'teleop_twist_joy_node',
        'dpad_reto', 'twist_mux')
ARGUMENTOS = {'porta': '/dev/ttyACM0', 'sinal': '1.0', 'frente': '-1.0',
              'bitola': '0.320', 'escala': '400.0'}


@pytest.fixture
def montado(monkeypatch):
    """(contexto, nós) — os argumentos com default e o `OpaqueFunction` rodado."""
    ld = get_launch_description_from_python_launch_file(LAUNCH)
    ctx = LaunchContext()
    acoes = []
    for e in ld.entities:
        if isinstance(e, DeclareLaunchArgument):
            e.visit(ctx)
        elif isinstance(e, OpaqueFunction):
            monkeypatch.setitem(e._OpaqueFunction__function.__globals__,
                                'escolhe_joystick', lambda: (0, 'fingido (teste)'))
            acoes.extend(e.visit(ctx) or [])
        else:
            acoes.append(e)
    return ctx, [a for a in acoes if isinstance(a, Node)]


def _nome(no):
    return no._Node__node_name


def _por_nome(nos, nome):
    return [n for n in nos if _nome(n) == nome]


def _descricao_de(ctx, no):
    params = evaluate_parameters(ctx, no._Node__parameters)
    valores = {}
    for p in params:
        if isinstance(p, dict):
            valores.update(p)
    return valores.get('robot_description')


def _xacro_robo3(sim):
    caminho = os.path.join(get_package_share_directory('robot_base'), 'description',
                           'robo3.urdf.xacro')
    return xacro.process_file(caminho, mappings={'sim': sim}).toxml()


# ─── o RSP ───────────────────────────────────────────────────────────────────

def test_robot_state_publisher_exatamente_uma_vez(montado):
    _, nos = montado
    rsp = [n for n in nos if n.node_package == 'robot_state_publisher']
    assert len(rsp) == 1, f'RSP no controle_robo3: {len(rsp)}'
    assert rsp[0].node_executable == 'robot_state_publisher'
    assert _nome(rsp[0]) == 'robot_state_publisher', \
        'nome fixo: é o que o sobe-robo3 procura no grafo'


def test_urdf_e_o_robo3_do_share_do_robot_base_com_sim_false(montado):
    ctx, nos = montado
    rsp = _por_nome(nos, 'robot_state_publisher')
    assert len(rsp) == 1
    descricao = _descricao_de(ctx, rsp[0])
    assert isinstance(descricao, str) and descricao.strip(), 'robot_description vazio'
    esperado, simulado = _xacro_robo3('false'), _xacro_robo3('true')
    assert esperado != simulado, 'montagem: sim:=true e sim:=false têm de diferir'
    assert descricao == esperado, 'não é o robo3.urdf.xacro do robot_base com sim:=false'


def test_urdf_tem_a_tf_do_livox(montado):
    """A razão de ser do RSP aqui: `base_link → livox_frame` existir."""
    ctx, nos = montado
    rsp = _por_nome(nos, 'robot_state_publisher')
    assert len(rsp) == 1
    urdf = ET.fromstring(_descricao_de(ctx, rsp[0]))
    # `findall` e não `iter`: o bloco <ros2_control> também tem <joint>, sem
    # parent/child.
    juntas = {(j.find('parent').get('link'), j.find('child').get('link'))
              for j in urdf.findall('joint')}
    assert ('base_link', 'livox_frame') in juntas


def test_nenhum_joint_state_publisher(montado):
    _, nos = montado
    assert not [n for n in nos if 'joint_state_publisher' in (n.node_package or '')
                or 'joint_state_publisher' in (n.node_executable or '')]


# ─── o que já existia, intacto ───────────────────────────────────────────────

@pytest.mark.parametrize('nome', SEIS)
def test_os_seis_nos_de_hoje_exatamente_uma_vez(montado, nome):
    _, nos = montado
    assert len(_por_nome(nos, nome)) == 1, nome


def test_nenhum_no_alem_dos_seis_e_do_rsp(montado):
    _, nos = montado
    assert sorted(_nome(n) for n in nos) == sorted(SEIS + ('robot_state_publisher',))


def test_os_cinco_argumentos_e_defaults_intactos():
    ld = get_launch_description_from_python_launch_file(LAUNCH)
    ctx = LaunchContext()
    declarados = {e.name: perform_substitutions(ctx, e.default_value)
                  for e in ld.entities if isinstance(e, DeclareLaunchArgument)}
    assert declarados == ARGUMENTOS


# ─── dependência ─────────────────────────────────────────────────────────────

def test_robot_nav_declara_robot_base_para_execucao():
    raiz = ET.parse(PACKAGE_XML).getroot()
    execucao = {d.text.strip() for tag in ('exec_depend', 'depend')
                for d in raiz.findall(tag)}
    assert 'robot_base' in execucao, 'o launch lê o share/ do robot_base'
