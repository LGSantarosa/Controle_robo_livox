"""O joystick não pode ficar incoerente com a máquina nem com o mux.

Este arquivo existe por causa de um modo de falha que NÃO dá sintoma, e é o
mesmo tipo de defeito do `test_configs_coerentes.py`: um número copiado de
outro robô que continua parecendo certo.

O caso central é o `publish_stamped_twist`. O mux do robô 2 roda com
`use_stamped: true`; o `teleop_xbox.yaml` do robô 1, de onde este veio, traz
`false`. Com os dois em desacordo:

    o `joy_node` sobe                        ✅
    o `/joy` publica a 17-20 Hz              ✅
    o `/joy_vel` aparece em `topic list`     ✅
    o DDS recusa a ligação por type hash     ❌ e é só isto que se vê
    o robô não reage ao controle             ❌ sem UMA linha de erro

Ninguém acha isso olhando; se acha depois de uma sessão perdida no robô. É
exatamente a classe de defeito que custou 14-08 (frame do seguidor) e 29-07
(bitola divergente): a peça está certa em si e errada em relação à vizinha.
"""
import os
import re

import pytest
import yaml

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
CFG = os.path.join(RAIZ, 'ros2_packages', 'robot_motion', 'config')
TELEOP = os.path.join(CFG, 'teleop_xbox.yaml')
MUX = os.path.join(CFG, 'twist_mux.yaml')
MOVIMENTACAO = os.path.join(CFG, 'movimentacao.yaml')
LAUNCH = os.path.join(RAIZ, 'ros2_packages', 'robot_motion', 'launch',
                      'joystick.launch.py')


def carrega(caminho):
    with open(caminho) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def teleop():
    return carrega(TELEOP)['teleop_twist_joy_node']['ros__parameters']


@pytest.fixture(scope='module')
def mux():
    return carrega(MUX)['twist_mux']['ros__parameters']


@pytest.fixture(scope='module')
def limites():
    return carrega(MOVIMENTACAO)['heading_controller']['ros__parameters']


@pytest.fixture(scope='module')
def launch_txt():
    with open(LAUNCH) as f:
        return f.read()


# ---------------------------------------------------------------- o silencioso

def test_stamped_do_teleop_casa_com_o_do_mux(teleop, mux):
    """O par que decide se o mux ESCUTA o controle. Ver docstring do módulo."""
    assert teleop['publish_stamped_twist'] == mux['use_stamped'], (
        'teleop_xbox.yaml e twist_mux.yaml discordam sobre TwistStamped. '
        'O DDS recusa por type hash e o robô ignora o controle EM SILÊNCIO.')


# ------------------------------------------------------- o robô é este, não o 1

def test_escalas_cabem_nos_tetos_da_maquina(teleop, limites):
    """Mão humana não estreia velocidade que a autonomia não usa.

    O robô 1 tinha `scale_angular: 6.0` — knob anti-skid do chassi de 4 rodas,
    que o `CLAUDE.md` proíbe herdar por escrito. Aqui o teto é o da máquina.
    """
    assert teleop['scale_linear']['x'] <= limites['v_max']
    assert teleop['scale_linear_turbo']['x'] <= limites['v_max']
    assert teleop['scale_angular']['yaw'] <= limites['wz_max']
    assert teleop['scale_angular_turbo']['yaw'] <= limites['wz_max']


def test_turbo_nao_e_menor_que_o_normal(teleop):
    """Turbo abaixo do normal já aconteceu: o default do pacote é 1.0 e, se
    alguém apagar a linha do turbo angular, ele CAI para o default em vez de
    herdar o normal. O robô 1 tem um comentário sobre isso no teleop_ps4."""
    assert teleop['scale_linear_turbo']['x'] >= teleop['scale_linear']['x']
    assert teleop['scale_angular_turbo']['yaw'] >= teleop['scale_angular']['yaw']


# ------------------------------------------------------------ o homem-morto

def test_homem_morto_exigido(teleop):
    """Sem isto o canal de prioridade 100 dirige o robô com o botão solto."""
    assert teleop['require_enable_button'] is True


def test_dead_man_e_turbo_sao_botoes_distintos(teleop):
    assert teleop['enable_button'] != teleop['enable_turbo_button']


def test_sticky_buttons_desligado(launch_txt):
    """`sticky_buttons` transforma o LB em liga/desliga — o oposto de um
    homem-morto. O robô continuaria andando com o botão solto."""
    assert re.search(r"'sticky_buttons':\s*False", launch_txt), (
        'joystick.launch.py precisa cravar sticky_buttons=False')


def test_autorepeat_ligado(launch_txt):
    """O `joy_node` só publica quando algo MUDA. Analógico segurado parado num
    ângulo não gera mensagem nova, o `timeout` do mux expira e o mux devolve o
    robô para a autonomia com o operador ainda segurando o LB."""
    m = re.search(r"'autorepeat_rate':\s*([\d.]+)", launch_txt)
    assert m, 'joystick.launch.py precisa declarar autorepeat_rate'
    taxa = float(m.group(1))
    assert taxa > 0.0, 'autorepeat_rate 0 = o mux larga o operador'


def test_autorepeat_alimenta_o_timeout_do_mux(mux, launch_txt):
    """E não basta ser > 0: tem que caber DENTRO do timeout da faixa."""
    taxa = float(re.search(r"'autorepeat_rate':\s*([\d.]+)", launch_txt).group(1))
    timeout = mux['topics']['joystick']['timeout']
    assert 1.0 / taxa < timeout, (
        f'autorepeat de {taxa} Hz ({1/taxa:.2f} s) não sustenta um timeout '
        f'de {timeout} s — o mux soltaria o controle entre duas mensagens')


# ------------------------------------------------------------------ o mux

def test_joystick_e_a_maior_prioridade(mux):
    """Quem está com o homem-morto na mão está olhando para o robô."""
    prios = {k: v['priority'] for k, v in mux['topics'].items()}
    assert prios['joystick'] == max(prios.values())
    assert prios['joystick'] > prios['teclado'] > prios['web'] > prios['autonomia']


def test_topico_do_mux_casa_com_o_remap_da_launch(mux, launch_txt):
    """Tópico divergente = mux escutando um nome que ninguém publica."""
    topico = mux['topics']['joystick']['topic'].lstrip('/')
    assert re.search(rf"'/{re.escape(topico)}'", launch_txt), (
        f"o mux escuta {topico!r}, mas joystick.launch.py não remapeia para ele")


def test_nome_do_no_casa_com_a_chave_do_yaml(launch_txt):
    """Nome diferente = YAML ignorado em silêncio = defaults do pacote
    (enable_button 0), ou seja, homem-morto num botão que ninguém aperta."""
    chave = next(iter(carrega(TELEOP)))
    assert re.search(rf"name='{re.escape(chave)}'", launch_txt), (
        f"o teleop_xbox.yaml é endereçado a {chave!r}; a launch tem de nomear "
        f'o nó exatamente assim')
