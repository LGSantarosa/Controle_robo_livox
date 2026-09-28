#!/usr/bin/env python3
"""As medidas de uma subida do robô 3 (decisão 061) — puras, sem ROS.

🔴 NASCEM VERMELHOS: `mede.py` ainda não existe. As linhas de log abaixo são
copiadas das corridas reais de 28-09: `~/etapa7/20260928_114902` (nominal) e
`~/etapa7/20260928_114620` (o broadcaster não ativou).
"""
import importlib.util
import os

import pytest

AQUI = os.path.dirname(os.path.abspath(__file__))
MEDE = os.path.join(AQUI, 'mede.py')


@pytest.fixture(scope='module')
def mede():
    if not os.path.exists(MEDE):
        pytest.fail(f'{MEDE} ainda não existe — decisão 061, passo 2: os '
                    'testes vêm antes do código.')
    spec = importlib.util.spec_from_file_location('mede_subidas', MEDE)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


# ─── o launch.log ────────────────────────────────────────────────────────────

LOG_NOMINAL = '\n'.join([
    "[gazebo-1] [INFO] [1790606952.810389440] [controller_manager]: Loading controller : 'joint_state_broadcaster' of type 'joint_state_broadcaster/JointStateBroadcaster'",
    "[gazebo-1] [INFO] [1790606953.260870651] [controller_manager]: Activating controllers: [ joint_state_broadcaster ]",
    "[spawner-20] [INFO] [1790606954.055156104] [spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster",
    "[gazebo-1] [INFO] [1790606955.527720645] [controller_manager]: Loading controller : 'hoverboard_base_controller' of type 'diff_drive_controller/DiffDriveController'",
    "[gazebo-1] [INFO] [1790606955.810001944] [controller_manager]: Activating controllers: [ hoverboard_base_controller ]",
    "[spawner-21] [INFO] [1790606956.036623859] [spawner_hoverboard_base_controller]: Configured and activated hoverboard_base_controller",
    "[robot_state_publisher-2] [INFO] [1790607012.315095577] [rclcpp]: signal_handler(SIGINT/SIGTERM)",
    "[ERROR] [placa_simulada-3]: process has died [pid 573768, exit code 1, cmd 'x']",
    ''])

LOG_FALHA = '\n'.join([
    "[gazebo-1] [INFO] [1790606802.642888730] [controller_manager]: Loading controller : 'joint_state_broadcaster' of type 'joint_state_broadcaster/JointStateBroadcaster'",
    "[gazebo-1] [INFO] [1790606803.224943703] [controller_manager]: Activating controllers: [ joint_state_broadcaster ]",
    "[gazebo-1] [ERROR] [1790606808.225453647] [controller_manager]: Switch controller timed out after 5 seconds!",
    "[spawner-20] [ERROR] [1790606808.230409477] [spawner_joint_state_broadcaster]: Failed to activate controller : joint_state_broadcaster",
    "[ERROR] [spawner-20]: process has died [pid 562426, exit code 1, cmd '/opt/ros/jazzy/lib/controller_manager/spawner joint_state_broadcaster']",
    "[gazebo-1] [INFO] [1790606811.007216060] [controller_manager]: Activating controllers: [ hoverboard_base_controller ]",
    "[spawner-21] [INFO] [1790606816.064721843] [spawner_hoverboard_base_controller]: Configured and activated hoverboard_base_controller",
    "[robot_state_publisher-2] [INFO] [1790606872.481458530] [rclcpp]: signal_handler(SIGINT/SIGTERM)",
    "[ERROR] [compensador_rumo-17]: process has died [pid 1, exit code 1, cmd 'x']",
    ''])


def test_log_nominal(mede):
    a = mede.analisa_log(LOG_NOMINAL)
    assert a['switch_timeout'] == 0 and a['falha_ativar'] == 0
    assert a['died_antes_do_fim'] == 0
    assert a['t_ativacao_jsb_s'] == pytest.approx(0.794, abs=1e-3)
    assert a['t_ativacao_base_s'] == pytest.approx(0.227, abs=1e-3)


def test_log_da_falha_de_11h46(mede):
    """O caso real: switch estourou, spawner morreu ANTES do encerramento, e
    o controlador da base levou 5,06 s — colado no limite."""
    a = mede.analisa_log(LOG_FALHA)
    assert a['switch_timeout'] == 1 and a['falha_ativar'] == 1
    assert a['died_antes_do_fim'] == 1
    assert a['t_ativacao_jsb_s'] is None
    assert a['t_ativacao_base_s'] == pytest.approx(5.058, abs=1e-3)


def test_o_que_vem_depois_do_sigint_nao_conta(mede):
    """As mortes do teardown (decisão 057) não são falha de subida."""
    assert mede.analisa_log(LOG_NOMINAL)['died_antes_do_fim'] == 0


def test_log_vazio_nao_inventa_ativacao(mede):
    a = mede.analisa_log('')
    assert a['t_ativacao_jsb_s'] is None and a['t_ativacao_base_s'] is None
    assert a['switch_timeout'] == 0


def test_timeout_depois_do_sigint_nao_conta(mede):
    log = LOG_NOMINAL + ("[gazebo-1] [ERROR] [1790607013.0] [controller_manager]: "
                         "Switch controller timed out after 5 seconds!\n")
    assert mede.analisa_log(log)['switch_timeout'] == 0


# ─── o list_controllers ──────────────────────────────────────────────────────

LISTA = ("joint_state_broadcaster     joint_state_broadcaster/JointStateBroadcaster  active\n"
         "hoverboard_base_controller  diff_drive_controller/DiffDriveController      active\n")


def test_lista_de_controladores(mede):
    assert mede.controladores(LISTA) == {
        'joint_state_broadcaster': 'active',
        'hoverboard_base_controller': 'active'}


def test_lista_com_cor_ansi(mede):
    """O `ros2 control` pode colorir o estado; a cor não pode virar estado."""
    cor = LISTA.replace('active\n', '\x1b[92mactive\x1b[0m\n')
    assert set(mede.controladores(cor).values()) == {'active'}


def test_controlador_inativo_e_lido_como_inativo(mede):
    texto = LISTA.replace('JointStateBroadcaster  active',
                          'JointStateBroadcaster  inactive')
    assert mede.controladores(texto)['joint_state_broadcaster'] == 'inactive'


def test_lixo_na_saida_nao_vira_controlador(mede):
    assert mede.controladores('Could not contact service\n\n') == {}


# O `ros2 control` (ros2controlcli) NÃO está instalado neste PC (28-09): a
# leitura é pelo serviço do controller_manager, cuja resposta sai como a
# representação da mensagem.
SERVICO = (
    "requester: making request: controller_manager_msgs.srv.ListControllers_Request()\n\n"
    "response:\n"
    "controller_manager_msgs.srv.ListControllers_Response(controller=["
    "controller_manager_msgs.msg.ControllerState(name='joint_state_broadcaster', "
    "state='active', type='joint_state_broadcaster/JointStateBroadcaster', "
    "is_async=False, update_rate=10, claimed_interfaces=[], "
    "required_command_interfaces=[], required_state_interfaces=['a/position']), "
    "controller_manager_msgs.msg.ControllerState(name='hoverboard_base_controller', "
    "state='inactive', type='diff_drive_controller/DiffDriveController', "
    "is_async=False, update_rate=10, claimed_interfaces=[])])\n")


def test_resposta_do_servico_list_controllers(mede):
    assert mede.controladores(SERVICO) == {
        'joint_state_broadcaster': 'active',
        'hoverboard_base_controller': 'inactive'}


def test_servico_sem_resposta_nao_vira_controlador(mede):
    texto = ("requester: making request: controller_manager_msgs.srv."
             "ListControllers_Request()\n")
    assert mede.controladores(texto) == {}


# ─── /joint_states e o amostrador ────────────────────────────────────────────

def test_conta_mensagens_do_echo(mede):
    texto = '1790\n---\n1790\n---\n1791\n---\n'
    assert mede.conta_mensagens(texto) == 3
    assert mede.conta_mensagens('') == 0


def test_resumo_do_amostrador(mede):
    """Linhas `epoch load1 some_avg10 some_total_us`, uma por segundo."""
    texto = ('1790.0 1.50 0.00 1000\n'
             '1791.0 3.20 12.50 900000\n'
             '1792.0 2.10 8.00 1400000\n')
    r = mede.resume_amostras(texto)
    assert r['load1_max'] == pytest.approx(3.20)
    assert r['psi_cpu_some_avg10_max'] == pytest.approx(12.50)
    assert r['psi_cpu_some_us'] == 1399000
    assert r['amostras'] == 3


def test_amostrador_vazio_ou_quebrado_nao_inventa(mede):
    r = mede.resume_amostras('lixo\n')
    assert r['amostras'] == 0 and r['load1_max'] is None
    assert r['psi_cpu_some_us'] is None


# ─── a linha do CSV e o que é nominal ────────────────────────────────────────

def _medidas(**sobre):
    m = {'nav2_tf_pronto': 1, 't_nav2_tf_s': 30.0, 'js_janela_s': 5,
         'limpeza_ok': 1}
    m.update(sobre)
    return m


def test_subida_nominal(mede):
    l = mede.linha(3, _medidas(), LOG_NOMINAL, LISTA, '1\n---\n' * 40,
                   '1790.0 1.0 0.0 0\n')
    assert l['subida'] == 3 and l['subida_ok'] == 1
    assert l['controladores_ok'] == 1 and l['joint_states_msgs'] == 40
    assert set(mede.COLUNAS) == set(l)


@pytest.mark.parametrize('caso, argumentos', [
    ('sem Nav2/TF', dict(medidas=_medidas(nav2_tf_pronto=0))),
    ('broadcaster inativo', dict(lista=LISTA.replace(
        'JointStateBroadcaster  active', 'JointStateBroadcaster  inactive'))),
    ('controlador ausente', dict(lista=LISTA.splitlines()[1] + '\n')),
    ('sem joint_states', dict(echo='')),
    ('switch estourou', dict(log=LOG_FALHA)),
])
def test_o_que_nao_e_nominal(mede, caso, argumentos):
    a = dict(medidas=_medidas(), log=LOG_NOMINAL, lista=LISTA,
             echo='1\n---\n' * 40)
    a.update(argumentos)
    l = mede.linha(1, a['medidas'], a['log'], a['lista'], a['echo'], '')
    assert l['subida_ok'] == 0, caso


def test_a_limpeza_fica_em_coluna_propria_e_nao_muda_a_subida(mede):
    """Subida e limpeza são perguntas separadas: a limpeza que falha INTERROMPE
    a bateria (é do wrapper), mas não reescreve o que a subida foi."""
    l = mede.linha(1, _medidas(limpeza_ok=0), LOG_NOMINAL, LISTA,
                   '1\n---\n' * 40, '')
    assert l['subida_ok'] == 1 and l['limpeza_ok'] == 0


# ─── o veredito da bateria ───────────────────────────────────────────────────

def _linhas(n, falhas=()):
    return [{'subida': i, 'subida_ok': 0 if i in falhas else 1,
             'limpeza_ok': 1} for i in range(1, n + 1)]


def test_vinte_de_vinte_e_estavel_com_o_limite_escrito(mede):
    v, texto = mede.veredito(_linhas(20), 20, interrompida=False)
    assert v == 'ESTÁVEL'
    assert '20/20' in texto and '13,9%' in texto


def test_uma_falha_e_instavel(mede):
    v, texto = mede.veredito(_linhas(20, falhas={7}), 20, interrompida=False)
    assert v == 'INSTÁVEL' and '19/20' in texto and '7' in texto


def test_interrompida_e_incompleta_mesmo_sem_falha(mede):
    v, _ = mede.veredito(_linhas(12), 20, interrompida=True)
    assert v == 'INCOMPLETA'


def test_menos_linhas_que_o_pedido_e_incompleta(mede):
    v, _ = mede.veredito(_linhas(19), 20, interrompida=False)
    assert v == 'INCOMPLETA'


def test_limpeza_reprovada_nunca_e_estavel(mede):
    linhas = _linhas(20)
    linhas[-1]['limpeza_ok'] = 0
    v, _ = mede.veredito(linhas, 20, interrompida=False)
    assert v != 'ESTÁVEL'
