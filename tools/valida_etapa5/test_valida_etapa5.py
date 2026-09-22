"""A bancada da etapa 5 diz a verdade — sem ROS de pé (PLANO_ETAPA5_ROBO3.md §3.3).

Os números de fases.yaml são RECALCULADOS das configs (teleop, dpad_reto,
defaults do launch, fórmulas do cmd_vel_to_wheels e do mega_bridge); cada
conferência (igual, zero, timeout, perda) tem caso que aprova e caso que
reprova; e o orquestrador novo segue as proibições da etapa 4.
"""
import ast
import importlib.util
import os
import re
import subprocess

import pytest
import yaml

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.abspath(os.path.join(AQUI, '..', '..'))
NAV = os.path.join(RAIZ, 'ros2_packages', 'robot_nav')
ORQ = os.path.join(RAIZ, 'bin', 'valida-etapa5')
LIB = os.path.join(AQUI, 'lib.sh')


def _mod(nome, caminho):
    spec = importlib.util.spec_from_file_location(nome, caminho)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


mf = _mod('mega_fingida5', os.path.join(AQUI, 'mega_fingida5.py'))
pr = _mod('processos5', os.path.join(AQUI, 'processos.py'))
CFG = yaml.safe_load(open(os.path.join(AQUI, 'fases.yaml')))


# ─── os números vêm das configs ──────────────────────────────────────────────

def _defaults_do_launch():
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.launch_description_sources import \
        get_launch_description_from_python_launch_file
    from launch.utilities import perform_substitutions
    ld = get_launch_description_from_python_launch_file(
        os.path.join(NAV, 'launch', 'controle_robo3.launch.py'))
    ctx = LaunchContext()
    return {e.name: perform_substitutions(ctx, e.default_value)
            for e in ld.entities if isinstance(e, DeclareLaunchArgument)}


def _vel_do_dpad():
    """O default `vel` declarado no dpad_reto (o launch não o sobrescreve)."""
    arvore = ast.parse(open(os.path.join(NAV, 'robot_nav', 'dpad_reto.py')).read())
    for no in ast.walk(arvore):
        if (isinstance(no, ast.Call) and getattr(no.func, 'attr', '') == 'declare_parameter'
                and no.args[0].value == 'vel'):
            return no.args[1].value
    raise AssertionError('dpad_reto sem parâmetro vel')


def _frame(args, lin, ang):
    sinal, frente = float(args['sinal']), float(args['frente'])
    bitola, escala = float(args['bitola']), float(args['escala'])
    lin *= frente
    left = (lin - ang * bitola / 2) * escala * sinal
    right = (lin + ang * bitola / 2) * escala * sinal
    return {'steer': int(round((left - right) / 2)), 'speed': int(round((left + right) / 2))}


@pytest.mark.parametrize('caso', sorted(CFG['cenarios']))
def test_frames_recalculados_das_configs(caso):
    pytest.importorskip('launch')
    c = CFG['cenarios'][caso]
    args = _defaults_do_launch()
    for a in c['argumentos']:
        k, v = a.split(':=')
        assert k in args
        args[k] = v
    tele = yaml.safe_load(open(os.path.join(NAV, 'config', 'teleop_xbox_robo3.yaml')))
    p = tele['teleop_twist_joy_node']['ros__parameters']
    calc = {'lb_frente': _frame(args, p['scale_linear']['x'], 0.0),
            'lb_esquerda': _frame(args, 0.0, p['scale_angular']['yaw'] * c['eixo_giro']),
            'dpad': _frame(args, _vel_do_dpad(), 0.0)}
    for nome, e in c['frames'].items():
        assert e == calc[nome], (caso, nome)


def test_frente_mais_1_so_difere_de_padrao_hoje_na_frente():
    a, b = CFG['cenarios']['padrao_hoje'], CFG['cenarios']['frente_mais_1']
    assert b['argumentos'] == ['frente:=1.0'] and a['argumentos'] == []
    assert a['eixo_giro'] == b['eixo_giro']
    assert b['frames']['lb_frente']['speed'] == -a['frames']['lb_frente']['speed'] != 0
    assert b['frames']['lb_esquerda'] == a['frames']['lb_esquerda'], 'o steer não vira'


def test_regressao_do_passo7_com_os_mesmos_numeros():
    p7 = yaml.safe_load(open(os.path.join(RAIZ, 'tools/valida_etapa4/frames_esperados.yaml')))
    for caso in ('reproducao_14_09', 'padrao_hoje'):
        velho, novo = p7['cenarios'][caso], CFG['cenarios'][caso]
        assert velho['argumentos'] == novo['argumentos']
        assert velho['eixo_giro'] == novo['eixo_giro']
        for fase in ('lb_frente', 'lb_esquerda'):
            assert velho['fases'][fase] == novo['frames'][fase], (caso, fase)


def test_toda_espera_tem_frame_no_cenario_que_a_usa():
    for c in CFG['cenarios'].values():
        for f in CFG['sequencias'][c['sequencia']]:
            if f.get('espera'):
                assert f['espera'] in c['frames'], (f['nome'], f['espera'])


# ─── as conferências mordem ──────────────────────────────────────────────────

F = mf.frame_set_speed(0, -120)
G = mf.frame_set_speed(-256, 0)
Z = mf.frame_set_speed(0, 0)
FR = {'steer': 0, 'speed': -120}
GI = {'steer': -256, 'speed': 0}


def _c(tipo, frames, esperado, t0=10.0, t1=13.0):
    return mf.confere({'nome': 'x', 'confere': tipo}, t0, t1, frames, esperado, CFG)


def test_igual():
    bom = [(10.1, G)] + [(10.7 + i / 20, F) for i in range(20)]
    assert _c('igual', bom, FR)['veredito'] == 'APROVADO'
    assert _c('igual', [(10.7, F), (11.0, G)], FR)['veredito'] == 'REPROVADO'
    assert _c('igual', [], FR)['veredito'] == 'REPROVADO'


def test_zero():
    assert _c('zero', [(9.0, F), (10.05, Z)], None)['veredito'] == 'APROVADO'
    assert _c('zero', [(9.0, F)], None)['veredito'] == 'REPROVADO'
    assert _c('zero', [(10.05, Z), (11.0, F)], None)['veredito'] == 'REPROVADO'


def test_timeout_exige_zero_depois_silencio_depois_a_faixa_menor():
    bom = [(10.05, Z)] + [(10.36 + i / 20, G) for i in range(30)]
    r = _c('timeout', bom, GI)
    assert r['veredito'] == 'APROVADO' and r['atraso_da_volta_s'] == pytest.approx(0.31)
    cedo = [(10.05, Z)] + [(10.1 + i / 20, G) for i in range(30)]
    assert _c('timeout', cedo, GI)['veredito'] == 'REPROVADO', 'voltou antes do timeout'
    sem_zero = [(10.36 + i / 20, G) for i in range(30)]
    assert _c('timeout', sem_zero, GI)['veredito'] == 'REPROVADO'
    intruso = [(10.05, Z), (10.2, F)] + [(10.36 + i / 20, G) for i in range(30)]
    assert _c('timeout', intruso, GI)['veredito'] == 'REPROVADO'


def test_perda_exige_silencio_e_ultimo_nao_zero():
    r = _c('perda', [(9.9, F), (10.05, F)], FR, t1=12.5)
    assert r['veredito'] == 'APROVADO' and r['ultimo_hex'] == F.hex()
    assert _c('perda', [(10.05, F), (11.0, F)], FR, t1=12.5)['veredito'] == 'REPROVADO'
    assert _c('perda', [(9.9, F), (10.05, Z)], FR, t1=12.5)['veredito'] == 'REPROVADO'


def test_comando_da_fase():
    caso = CFG['cenarios']['padrao_hoje']
    pub, lb, e = mf.comando_da_fase({'lb': True, 'giro': True, 'dpad': True}, caso)
    assert pub and lb and e[0] == 1.0 and e[7] == 1.0 and e[1] == 0.0
    pub, lb, e = mf.comando_da_fase({'lb': True, 'frente': True, 'publica': False}, caso)
    assert not pub


def test_externos_sao_opcionais_na_copia():
    antes = [{'pid': 10, 'ppid': 1, 'pgid': 10, 'sid': 10, 'starttime': 100,
              'estado': 'S', 'marcado': True, 'argv': 'x'}]
    r = pr.compara(antes, [], [(10, 100, 'launch')], [], 10_000)
    assert r['veredito'] == 'APROVADO', r['itens']


# ─── o orquestrador novo ─────────────────────────────────────────────────────

def _codigo(caminho):
    with open(caminho) as f:
        return '\n'.join(re.sub(r'(^|\s)#.*$', '', linha) for linha in f.read().splitlines())


@pytest.mark.parametrize('arquivo', [ORQ, LIB])
@pytest.mark.parametrize('proibido', [
    r'\bpkill\b', r'\bkillall\b', r'\bpgrep\b', r'\bxargs\b[^\n|;]*\bkill\b',
    r'/(usr/)?bin/kill\b', r'\bbuiltin\s+kill\b', r'\bcommand\s+kill\b',
    r'\bkill\b[^\n]*--\s*"?-', r'docs/dados/[^\s"]*\s*>', r'valida_etapa4/'])
def test_nao_mata_por_nome_nem_toca_na_etapa4(arquivo, proibido):
    assert re.search(proibido, _codigo(arquivo)) is None, (arquivo, proibido)


def test_toda_consulta_ao_grafo_testa_o_codigo():
    for c in [linha.strip() for linha in _codigo(ORQ).splitlines()
              if re.search(r'\bnos_do_dominio\s+"', linha)]:
        assert re.match(r'(if\s+!?\s*nos_do_dominio|nos_do_dominio .*;\s*rc_\w+=\$\?)', c), c


def test_limpeza_instalada_logo_depois_da_marca():
    c = _codigo(ORQ)
    marca = c.index('export VALIDA_ETAPA4_MARCA')
    assert c.index('instala_limpeza', marca) < c.index('colcon build')


def test_subida_que_falha_encerra():
    c = _codigo(ORQ)
    i = c.index('sobe-robo3 subiu (placa, bateria, /joy)" REPROVADO')
    assert re.match(r'[^\n]*\n\s*return\b', c[i:])


def test_sintaxe_e_executaveis():
    assert subprocess.run(['bash', '-n', ORQ]).returncode == 0
    for f in (ORQ, os.path.join(AQUI, 'mega_fingida5.py'), os.path.join(AQUI, 'processos.py')):
        assert os.access(f, os.X_OK), f
