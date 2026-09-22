"""`cmd_vel_to_wheels` nos dois contratos — etapa 5, passo 3 (PLANO_ETAPA5_ROBO3.md D1).

O nó de VERDADE, dentro do processo, num domínio isolado (só localhost):

- sem parâmetro: assina `Twist` (conferido NO GRAFO) e produz os mesmos
  `WheelSpeeds` de sempre — é o que mantém o `robot.launch.py` funcionando;
- `use_stamped:=true`: assina `TwistStamped` e faz a MESMA conta;
- tipo incompatível não recebe — só depois do controle positivo (o tipo certo
  chega) e da barreira de descoberta (o publicador errado aparece no grafo);
  o errado vive noutro contexto rclpy (outro participante DDS);
- `frame_id` e `stamp` não mudam a conversão;
- um caminho só para a cinemática (AST).
"""
import ast
import os
import struct
import time

from geometry_msgs.msg import Twist, TwistStamped
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from robot_nav.cmd_vel_to_wheels import CmdVelToWheels
from wheel_msgs.msg import WheelSpeeds

FONTE = os.path.join(os.path.dirname(__file__), '..', 'robot_nav', 'cmd_vel_to_wheels.py')
PARAMS = {'wheel_base': 0.320, 'linear_scale': 400.0, 'left_wheel_sign': 1.0,
          'right_wheel_sign': 1.0, 'linear_sign': -1.0, 'max_output': 1000.0}
# (linear, angular): frente, ré, giro dos dois lados, arco, e um que satura.
ENTRADAS = [(0.30, 0.0), (-0.30, 0.0), (0.0, 4.0), (0.0, -1.5), (0.2, 0.7), (3.0, 2.0)]


def esperado(linear, angular, p=PARAMS):
    """Refaz a conta de hoje aqui, sem importar do nó."""
    linear *= p['linear_sign']
    left = (linear - angular * p['wheel_base'] / 2.0) * p['linear_scale'] * p['left_wheel_sign']
    right = (linear + angular * p['wheel_base'] / 2.0) * p['linear_scale'] * p['right_wheel_sign']
    pico = max(abs(left), abs(right))
    if pico > p['max_output']:
        left, right = left * p['max_output'] / pico, right * p['max_output'] / pico
    # WheelSpeeds tem campos float32: o esperado passa pela mesma conversão, e a
    # comparação é de IGUALDADE (não de tolerância).
    return tuple(struct.unpack('f', struct.pack('f', v))[0] for v in (left, right))


def _args(extra):
    a = ['--ros-args']
    for k, v in {**PARAMS, **extra}.items():
        a += ['-p', f'{k}:={str(v).lower() if isinstance(v, bool) else v}']
    return a


def _espera(cond, ex, prazo=5.0):
    fim = time.monotonic() + prazo
    while time.monotonic() < fim:
        if cond():
            return True
        for e in (ex if isinstance(ex, (list, tuple)) else [ex]):
            e.spin_once(timeout_sec=0.02)
    return cond()


class Bancada:
    def __init__(self, extra):
        rclpy.init(args=_args(extra))
        self.no = CmdVelToWheels()
        self.aux = Node('_bancada_cmd_vel')
        self.saidas = []
        self.aux.create_subscription(WheelSpeeds, 'wheel_vel_setpoints',
                                     lambda m: self.saidas.append(
                                         (m.left_wheel, m.right_wheel)), 10)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.no)
        self.ex.add_node(self.aux)
        self.outro = None

    def tipo_assinado(self):
        infos = [i for i in self.aux.get_subscriptions_info_by_topic('/cmd_vel')
                 if i.node_name == 'cmd_vel_to_wheels']
        return [i.topic_type for i in infos]

    def publicador(self, tipo, contexto_proprio=False):
        if not contexto_proprio:
            return self.aux.create_publisher(tipo, 'cmd_vel', 10), self.ex
        ctx = Context()
        rclpy.init(context=ctx)
        no = Node('_bancada_tipo_errado', context=ctx)
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(no)
        self.outro = (ctx, no, ex)
        return no.create_publisher(tipo, 'cmd_vel', 10), ex

    def fecha(self):
        if self.outro:
            ctx, no, ex = self.outro
            ex.shutdown()
            no.destroy_node()
            rclpy.shutdown(context=ctx)
        self.ex.shutdown()
        self.no.destroy_node()
        self.aux.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def bancada(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '72')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    criadas = []

    def cria(**extra):
        b = Bancada(extra)
        criadas.append(b)
        return b
    yield cria
    for b in criadas:
        b.fecha()


def _msg(tipo, linear, angular, stamp_s=0, frame=''):
    t = Twist()
    t.linear.x, t.angular.z = float(linear), float(angular)
    if tipo is Twist:
        return t
    m = TwistStamped()
    m.twist = t
    m.header.stamp.sec = stamp_s
    m.header.frame_id = frame
    return m


def _manda(b, pub, ex, tipo, entradas, **kw):
    assert _espera(lambda: pub.get_subscription_count() >= 1, [b.ex, ex]), \
        'barreira: o publicador do tipo certo nunca casou com o nó'
    antes = len(b.saidas)
    for linear, angular in entradas:
        pub.publish(_msg(tipo, linear, angular, **kw))
        assert _espera(lambda n=len(b.saidas): len(b.saidas) > n, [b.ex, ex]), (linear, angular)
    return b.saidas[antes:]


# ─── os dois contratos, a mesma conta ────────────────────────────────────────

def test_default_assina_twist_e_faz_a_conta_de_sempre(bancada):
    b = bancada()
    assert _espera(lambda: b.tipo_assinado(), b.ex)
    assert b.tipo_assinado() == ['geometry_msgs/msg/Twist']
    pub, ex = b.publicador(Twist)
    saidas = _manda(b, pub, ex, Twist, ENTRADAS)
    for (l, r), e in zip(saidas, ENTRADAS):
        assert (l, r) == esperado(*e), e


def test_use_stamped_false_explicito_e_o_default(bancada):
    b = bancada(use_stamped=False)
    assert _espera(lambda: b.tipo_assinado(), b.ex)
    assert b.tipo_assinado() == ['geometry_msgs/msg/Twist']


def test_use_stamped_assina_twiststamped_e_faz_a_mesma_conta(bancada):
    b = bancada(use_stamped=True)
    assert _espera(lambda: b.tipo_assinado(), b.ex)
    assert b.tipo_assinado() == ['geometry_msgs/msg/TwistStamped']
    pub, ex = b.publicador(TwistStamped)
    saidas = _manda(b, pub, ex, TwistStamped, ENTRADAS)
    for (l, r), e in zip(saidas, ENTRADAS):
        assert (l, r) == esperado(*e), e


def test_frame_id_e_stamp_nao_mudam_a_conversao(bancada):
    b = bancada(use_stamped=True)
    pub, ex = b.publicador(TwistStamped)
    a = _manda(b, pub, ex, TwistStamped, ENTRADAS[:3], stamp_s=0, frame='')
    c = _manda(b, pub, ex, TwistStamped, ENTRADAS[:3], stamp_s=12345, frame='base_link')
    assert a == c


# ─── tipo incompatível não recebe (controle positivo + barreira) ─────────────

@pytest.mark.parametrize('stamped, certo, errado', [
    (False, Twist, TwistStamped),
    (True, TwistStamped, Twist),
])
def test_tipo_incompativel_nao_recebe(bancada, stamped, certo, errado):
    b = bancada(use_stamped=stamped)
    pub_ok, ex_ok = b.publicador(certo)
    assert _manda(b, pub_ok, ex_ok, certo, ENTRADAS[:1]), 'controle positivo'
    pub_x, ex_x = b.publicador(errado, contexto_proprio=True)
    tipo_errado = errado.__module__.split('.')[0] + '/msg/' + errado.__name__
    assert _espera(lambda: any(i.topic_type == tipo_errado for i in
                               b.aux.get_publishers_info_by_topic('/cmd_vel')),
                   [b.ex, ex_x]), 'barreira: o publicador errado nunca apareceu no grafo'
    antes = len(b.saidas)
    for _ in range(10):
        pub_x.publish(_msg(errado, 0.30, 1.0))
        _espera(lambda: False, [b.ex, ex_x], prazo=0.1)
    assert len(b.saidas) == antes, 'o tipo errado chegou ao nó'
    assert pub_x.get_subscription_count() == 0


# ─── um caminho só para a cinemática ─────────────────────────────────────────

def test_um_caminho_so_para_a_cinematica():
    arvore = ast.parse(open(FONTE).read())
    classe = next(n for n in arvore.body if isinstance(n, ast.ClassDef))
    usa_bitola = [f.name for f in classe.body if isinstance(f, ast.FunctionDef)
                  and f.name != '__init__'
                  and any(isinstance(n, ast.Attribute) and n.attr == 'wheel_base'
                          for n in ast.walk(f))]
    assert len(usa_bitola) == 1, f'cinemática em mais de um lugar: {usa_bitola}'
