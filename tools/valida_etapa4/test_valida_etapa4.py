"""Os coletores do passo 7 dizem a verdade — etapa 4 (§10), sem ROS de pé.

Nada aqui sobe nó, Gazebo ou pty de verdade: são as contas e os vereditos
que a validação vai usar, testados antes de ela rodar. Os números da tabela
de frames são RECALCULADOS das configs: config mudou, a tabela reprova aqui.
"""
import importlib.util
import os
import re
import struct
import subprocess
import sys

import pytest
import yaml

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.abspath(os.path.join(AQUI, '..', '..'))
NAV = os.path.join(RAIZ, 'ros2_packages', 'robot_nav')


def _mod(nome, caminho):
    spec = importlib.util.spec_from_file_location(nome, caminho)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


mf = _mod('mega_fingida', os.path.join(AQUI, 'mega_fingida.py'))
pr = _mod('processos', os.path.join(AQUI, 'processos.py'))
cr = _mod('compara_robo2', os.path.join(AQUI, 'compara_robo2.py'))
ln = _mod('lista_nos', os.path.join(AQUI, 'lista_nos.py'))
FRAMES = yaml.safe_load(open(os.path.join(AQUI, 'frames_esperados.yaml')))


# ─── o frame é o do mega_bridge ──────────────────────────────────────────────

def test_frame_igual_ao_do_mega_bridge():
    mb = pytest.importorskip('robot_nav.mega_bridge')
    for steer, speed in [(0, -120), (97, 0), (-256, 0), (0, 0), (-32000, 32000)]:
        payload = struct.pack('<hhhh', steer, speed, steer, speed)
        assert mf.frame_set_speed(steer, speed) == mb._build_frame(mb.FT_SET_SPEED, payload)
    assert mf.FT_SET_SPEED == mb.FT_SET_SPEED and mf.FT_STATE == mb.FT_STATE


def test_state_fingido_tem_o_layout_que_o_mega_bridge_le():
    q = mf.frame_state(3650)
    assert q[:2] == b'\xaa\x55' and q[2] == 0x81 and q[3] == 16
    rpm_fl, rpm_fr, rpm_rl, rpm_rr, bat_f, bat_r = struct.unpack('<hhhhhh', q[4:16])
    assert (bat_f, bat_r) == (3650, 3650)
    assert q[16] & 0x01 == 0 and q[17] & 0x01 == 0, 'falha bit 0 = stale: tem de ser 0'


def test_decodificador_acha_frames_no_meio_de_lixo_e_marca_chk_ruim():
    bom = mf.frame_set_speed(97, 0)
    ruim = bytearray(mf.frame_set_speed(0, -120))
    ruim[-1] ^= 0xFF
    fluxo = b'\x00\x13\xaa' + bom + b'\xaa\xaa' + bom[1:] + bytes(ruim)
    dec, achados = mf.Decodificador(), []
    for b in fluxo:
        r = dec.alimenta(b)
        if r:
            achados.append(r)
    assert [(t, q, ok) for t, _, q, ok in achados] == [
        (0x01, bom, True), (0x01, bom, True), (0x01, bytes(ruim), False)]


# ─── a avaliação das fases ───────────────────────────────────────────────────

ESP = {'lb_frente': {'steer': 0, 'speed': -120}, 'lb_solto': {'steer': 0, 'speed': 0}}
F, Z = mf.frame_set_speed(0, -120), mf.frame_set_speed(0, 0)


def test_transitorio_antes_da_janela_nao_conta():
    frames = [(1.05, Z), (1.2, Z)] + [(1.0 + 0.7 + i * 0.05, F) for i in range(20)]
    r = mf.avalia(frames, [('lb_frente', 1.0, 3.0)], ESP, 0.6)
    assert r[0]['veredito'] == 'APROVADO' and r[0]['frames_na_janela'] == 20


def test_um_frame_errado_na_janela_reprova():
    frames = [(1.7 + i * 0.05, F) for i in range(20)] + [(2.5, mf.frame_set_speed(1, -120))]
    r = mf.avalia(frames, [('lb_frente', 1.0, 3.0)], ESP, 0.6)
    assert r[0]['veredito'] == 'REPROVADO' and len(r[0]['diferentes_hex']) == 1


def test_fase_com_lb_sem_frame_reprova():
    r = mf.avalia([], [('lb_frente', 1.0, 3.0)], ESP, 0.6)
    assert r[0]['veredito'] == 'REPROVADO'


def test_solto_exige_o_ultimo_frame_zero_e_nenhum_nao_zero_na_janela():
    ok = mf.avalia([(0.5, F), (3.1, Z)], [('lb_solto', 3.0, 5.0)], ESP, 0.6)
    assert ok[0]['veredito'] == 'APROVADO', 'um zero ao soltar e silêncio'
    sem_zero = mf.avalia([(0.5, F)], [('lb_solto', 3.0, 5.0)], ESP, 0.6)
    assert sem_zero[0]['veredito'] == 'REPROVADO', 'último frame ainda andando'
    volta = mf.avalia([(3.1, Z), (4.0, F)], [('lb_solto', 3.0, 5.0)], ESP, 0.6)
    assert volta[0]['veredito'] == 'REPROVADO'


# ─── a tabela de frames bate com as configs ──────────────────────────────────

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


def _frame_da_cadeia(args, eixo_frente, eixo_giro, fase):
    """Passa pela cadeia teleop → cmd_vel_to_wheels → mega_bridge (fórmulas dos nós)."""
    tele = yaml.safe_load(open(os.path.join(NAV, 'config', 'teleop_xbox_robo3.yaml')))
    p = tele['teleop_twist_joy_node']['ros__parameters']
    lin = p['scale_linear']['x'] * (eixo_frente if fase == 'lb_frente' else 0.0)
    ang = p['scale_angular']['yaw'] * (eixo_giro if fase == 'lb_esquerda' else 0.0)
    sinal, frente = float(args['sinal']), float(args['frente'])
    bitola, escala = float(args['bitola']), float(args['escala'])
    lin *= frente
    left = (lin - ang * bitola / 2) * escala * sinal
    right = (lin + ang * bitola / 2) * escala * sinal
    return int(round((left - right) / 2)), int(round((left + right) / 2))


@pytest.mark.parametrize('caso', sorted(FRAMES['cenarios']))
def test_tabela_de_frames_recalculada_das_configs(caso):
    pytest.importorskip('launch')
    c = FRAMES['cenarios'][caso]
    args = _defaults_do_launch()
    for a in c['argumentos']:
        k, v = a.split(':=')
        assert k in args, f'argumento {k} não existe no launch'
        args[k] = v
    for fase, e in c['fases'].items():
        steer, speed = _frame_da_cadeia(args, c['eixo_frente'], c['eixo_giro'], fase)
        assert (steer, speed) == (e['steer'], e['speed']), (caso, fase)


def test_reproducao_de_14_09_e_a_tabela_do_diario():
    """Confere a tabela de 14-09: frente −120, esquerda steer 97, solto 0 (DIARIO)."""
    f = FRAMES['cenarios']['reproducao_14_09']['fases']
    assert (f['lb_frente'], f['lb_esquerda'], f['lb_solto']) == (
        {'steer': 0, 'speed': -120}, {'steer': 97, 'speed': 0}, {'steer': 0, 'speed': 0})


def test_fases_do_joy_cobrem_toda_expectativa():
    nomes = {f['nome'] for f in FRAMES['fases']}
    for c in FRAMES['cenarios'].values():
        assert set(c['fases']) <= nomes


# ─── TF: o que o URDF manda ──────────────────────────────────────────────────

def test_junta_livox_do_urdf_instalado_tem_yaw_zero():
    pytest.importorskip('xacro')
    xyz, ang = mf.junta_livox(mf._urdf_instalado())
    assert ang == [0.0, 0.0, 0.0], 'o §10.5 espera yaw 0 na TF viva'
    assert len(xyz) == 3


def test_rpy_de_quaternion():
    import math
    assert mf.rpy(0, 0, 0, 1) == (0.0, 0.0, 0.0)
    s = math.sin(math.radians(30) / 2)
    assert mf.rpy(0, 0, s, math.cos(math.radians(30) / 2))[2] == pytest.approx(math.radians(30))


# ─── quem morreu (prova grupo a grupo) ───────────────────────────────────────

def _p(pid, pgid, start, argv='x', marcado=False):
    return {'pid': pid, 'ppid': 1, 'pgid': pgid, 'sid': pgid, 'starttime': start,
            'estado': 'S', 'marcado': marcado, 'argv': argv}


ANTES = [_p(10, 10, 100, marcado=True), _p(11, 10, 101, marcado=True),
         _p(20, 20, 50, 'joy_node'), _p(30, 30, 40, 'bash')]
GRUPOS = [(10, 100, 'launch')]
EXT = [(20, 20, 50, 'joy_node')]
T = 10_000


def test_so_o_grupo_registrado_morreu():
    r = pr.compara(ANTES, [ANTES[2], ANTES[3]], GRUPOS, EXT, T)
    assert r['veredito'] == 'APROVADO', r['itens']


def test_membro_do_grupo_vivo_reprova():
    r = pr.compara(ANTES, [ANTES[1], ANTES[2], ANTES[3]], GRUPOS, EXT, T)
    assert not r['itens']['cada_grupo_registrado_valido_e_morto']
    assert r['grupos'][0]['membros_vivos_depois'] == [11]


def test_outro_processo_morto_reprova():
    r = pr.compara(ANTES, [ANTES[2]], GRUPOS, EXT, T)
    assert not r['itens']['nenhum_outro_processo_morreu']


def test_externo_com_pid_reusado_reprova():
    reusado = _p(20, 20, 999, 'outro')
    r = pr.compara(ANTES, [reusado, ANTES[3]], GRUPOS, EXT, T)
    assert not r['itens']['externos_vivos_depois_mesmo_starttime']


def test_processo_de_vida_curta_que_some_e_listado_mas_nao_reprova():
    curto = _p(40, 40, T - 10)
    r = pr.compara(ANTES + [curto], [ANTES[2], ANTES[3]], GRUPOS, EXT, T)
    assert r['veredito'] == 'APROVADO' and r['vida_curta_que_sumiu'] == [curto]


def test_dois_grupos_registrados_e_um_ausente_reprova():
    """`bool(do_grupo)` de antes passaria: um grupo com membros bastava."""
    r = pr.compara(ANTES, [ANTES[2], ANTES[3]], GRUPOS + [(77, 5, 'bag')], EXT, T)
    assert not r['itens']['cada_grupo_registrado_valido_e_morto']
    assert [(g['pgid'], g['classe'], g['ok']) for g in r['grupos']] == [
        (10, 'nosso', True), (77, 'vazio', False)]


def test_pgid_reusado_na_foto_nao_e_nosso_e_reprova():
    """Líder vivo com starttime diferente do registrado: o grupo é de outro.

    O que morreu nele NÃO conta como nosso — reprova duas vezes.
    """
    r = pr.compara(ANTES, [ANTES[2], ANTES[3]], [(10, 555, 'launch')], EXT, T)
    assert r['grupos'][0]['classe'] == 'reusado' and not r['grupos'][0]['ok']
    assert not r['itens']['nenhum_outro_processo_morreu']


def test_orfao_so_e_nosso_com_todos_os_membros_marcados():
    orfao = [_p(12, 10, 102, marcado=True), _p(13, 10, 103, marcado=True)]
    r = pr.compara(orfao + ANTES[2:], ANTES[2:], GRUPOS, EXT, T)
    assert r['grupos'][0]['classe'] == 'nosso_orfao' and r['veredito'] == 'APROVADO'
    alheio = [_p(12, 10, 102, marcado=True), _p(13, 10, 103, marcado=False)]
    r = pr.compara(alheio + ANTES[2:], ANTES[2:], GRUPOS, EXT, T)
    assert r['grupos'][0]['classe'] == 'orfao_alheio' and r['veredito'] == 'REPROVADO'


def test_membro_sem_marca_em_grupo_nosso_reprova():
    antes = [_p(10, 10, 100, marcado=True), _p(11, 10, 101, marcado=False)] + ANTES[2:]
    r = pr.compara(antes, ANTES[2:], GRUPOS, EXT, T)
    assert r['grupos'][0]['membros_sem_marca'] == [11] and r['veredito'] == 'REPROVADO'


# ─── sinal só com identidade (processos reais, isolados pela marca) ──────────

class _Bancada:
    """Processos `sleep` do teste, em sessão própria, com ou sem a marca."""

    def __init__(self, marca):
        self.marca, self.filhos = marca, []

    def sobe(self, marcado=True, novo_grupo=True):
        env = dict(os.environ)
        env.pop(pr.MARCA, None)
        if marcado:
            env[pr.MARCA] = self.marca
        p = subprocess.Popen(['sleep', '300'], env=env, start_new_session=novo_grupo)
        self.filhos.append(p)
        return p

    def limpa(self):
        for p in self.filhos:
            if p.poll() is None:
                p.kill()
            p.wait()


@pytest.fixture
def banc(tmp_path):
    b = _Bancada(str(tmp_path))
    yield b
    b.limpa()


def _start(pid):
    return pr._stat(pid)[4]


def _vivo(p):
    return p.poll() is None


def test_sinal_em_grupo_nosso_chega(banc):
    p = banc.sobe()
    log = []
    pr.sinaliza_grupos([(p.pid, _start(p.pid), 'teste')], pr.signal.SIGKILL,
                       banc.marca, log.append)
    p.wait(timeout=5)
    assert 'sinalizado' in log[0]


def test_pgid_reusado_nao_recebe_sinal(banc):
    """Não sinaliza PGID reusado: o registrado morreu e o número é de outro.

    O starttime do líder não bate — nada é enviado, mesmo com a marca.
    """
    alheio = banc.sobe(marcado=True)
    log = []
    pr.sinaliza_grupos([(alheio.pid, _start(alheio.pid) + 1, 'launch')],
                       pr.signal.SIGKILL, banc.marca, log.append)
    assert _vivo(alheio) and 'reusado' in log[0]


def test_membro_sem_marca_nao_recebe_sinal_nem_em_grupo_nosso(banc):
    p = banc.sobe(marcado=False)
    log = []
    pr.sinaliza_grupos([(p.pid, _start(p.pid), 'launch')], pr.signal.SIGKILL,
                       banc.marca, log.append)
    assert _vivo(p) and 'sem marca' in log[0]


def test_orfao_com_membro_sem_marca_nao_recebe_sinal(banc, tmp_path):
    """Líder morto; um membro marcado e um sem marca no MESMO grupo."""
    lider = subprocess.Popen(
        ['bash', '-c', f'{pr.MARCA}={banc.marca} sleep 300 & sleep 300 & echo $!; read -r _'],
        start_new_session=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
        env={k: v for k, v in os.environ.items() if k != pr.MARCA})
    banc.filhos.append(lider)
    sem_marca = int(lider.stdout.readline())
    pgid, start = lider.pid, _start(lider.pid)
    lider.stdin.close()
    lider.wait()
    try:
        log = []
        pr.sinaliza_grupos([(pgid, start, 'launch')], pr.signal.SIGKILL,
                           banc.marca, log.append)
        assert 'orfao_alheio' in log[0]
        assert pr._stat(sem_marca)[0] not in ('Z', 'X'), 'o sem-marca levou sinal'
    finally:
        os.killpg(pgid, pr.signal.SIGKILL)


def test_varredura_por_marca_so_pega_marcado(banc):
    marcado, limpo = banc.sobe(True), banc.sobe(False)
    pr.sinaliza_marcados(pr.signal.SIGKILL, banc.marca, log=lambda _: None)
    marcado.wait(timeout=5)
    assert _vivo(limpo)


def test_varredura_reconfere_o_starttime_antes_do_sinal(banc):
    p = banc.sobe(True)
    falso = {'pid': p.pid, 'starttime': _start(p.pid) + 1}
    assert pr._mata_se_ainda_for(falso, pr.signal.SIGKILL, banc.marca) == 'outro processo'
    assert _vivo(p)


# ─── a lib do orquestrador: limpeza em EXIT e consulta ROS falhando ─────────

LIB = os.path.join(AQUI, 'lib.sh')


def _harness(tmp_path, ros2_rc, corpo):
    """Roda a lib com um `ros2` falso (sem saída, código ros2_rc)."""
    falso = tmp_path / 'bin'
    falso.mkdir()
    (falso / 'ros2').write_text(f'#!/usr/bin/env bash\nexit {ros2_rc}\n')
    os.chmod(falso / 'ros2', 0o755)
    saida = tmp_path / 'rodada'
    saida.mkdir()
    script = f"""set +u
SAIDA={saida}; GRUPOS=$SAIDA/grupos_wrapper.txt; RESULTADO=$SAIDA/resultado.txt
V={AQUI}; DOMINIOS_USADOS="99"; : > "$GRUPOS"; : > "$RESULTADO"
source {LIB}
export VALIDA_ETAPA4_MARCA="$SAIDA"
instala_limpeza
{corpo}
"""
    env = {k: v for k, v in os.environ.items() if k != pr.MARCA}
    env['PATH'] = f'{falso}:{env["PATH"]}'
    r = subprocess.run(['bash', '-c', script], env=env, capture_output=True, text=True,
                       timeout=90)
    return r, saida


def _marcados(marca):
    return [p for p in pr.foto(marca) if p['marcado']]


def test_saida_antecipada_com_processo_marcado_passa_pela_limpeza(tmp_path):
    r, saida = _harness(tmp_path, 0, """
sobe_grupo teste "$SAIDA/teste.log" sleep 300
echo "$ULTIMO_PID" > "$SAIDA/pid"
exit 1   # como um build que falha
""")
    try:
        assert r.returncode == 1
        pid = int((saida / 'pid').read_text())
        assert not os.path.exists(f'/proc/{pid}') or pr._stat(pid)[0] in ('Z', 'X')
        assert _marcados(str(saida)) == []
        assert 'limpeza: nada marcado, domínios vazios' in (saida / 'resultado.txt').read_text()
        assert 'APROVADO' in (saida / 'resultado.txt').read_text()
    finally:
        pr.sinaliza_marcados(pr.signal.SIGKILL, str(saida), log=lambda _: None)


def test_consulta_ros_falhando_vazia_reprova_a_limpeza(tmp_path):
    r, saida = _harness(tmp_path, 1, 'exit 0')
    resultado = (saida / 'resultado.txt').read_text()
    assert 'REPROVADO' in resultado, 'saída vazia com erro não é "domínio vazio"'
    assert 'FALHOU (código 1)' in (saida / 'limpeza.txt').read_text()


def test_limpeza_e_idempotente(tmp_path):
    r, saida = _harness(tmp_path, 0, 'limpa; limpa; exit 0')
    assert (saida / 'resultado.txt').read_text().count('limpeza:') == 1


# ─── robô 2: réguas ──────────────────────────────────────────────────────────

P3 = os.path.join(RAIZ, 'docs/dados/2026-09-21-passo3-robo2/captura/parametros_normalizados.yaml')


def _nz():
    return cr._nz()


def test_comparador_aprova_a_captura_do_passo3():
    nz = _nz()
    r = cr.confere(nz.le(P3), nz.le(cr.V2), nz.le(cr.ORIGINAL),
                   nz.le_permitidas(cr.PERMITIDA_P2), nz)
    assert r['veredito'] == 'APROVADO', r


def test_comparador_reprova_diferenca_e_padding():
    nz = _nz()
    nova = nz.le(P3)
    nova['/path_follower']['v_max'] = 99.0
    r = cr.confere(nova, nz.le(cr.V2), nz.le(cr.ORIGINAL),
                   nz.le_permitidas(cr.PERMITIDA_P2), nz)
    assert not r['itens']['A_v2_sem_permissao'] and not r['itens']['B_original_com_passo2']
    nova = nz.le(P3)
    nova['/local_costmap/local_costmap']['footprint_padding'] = 0.01
    r = cr.confere(nova, nz.le(cr.V2), nz.le(cr.ORIGINAL),
                   nz.le_permitidas(cr.PERMITIDA_P2), nz)
    assert not r['itens']['C_padding_dos_dois_costmaps']


def test_comparador_reprova_permissao_sem_uso():
    nz = _nz()
    nova = nz.le(P3)
    del nova['/path_follower']['avanco_para_choque']
    r = cr.confere(nova, nz.le(cr.V2), nz.le(cr.ORIGINAL),
                   nz.le_permitidas(cr.PERMITIDA_P2), nz)
    assert not r['itens']['B_original_com_passo2']


# ─── lista de nós ────────────────────────────────────────────────────────────

def test_lista_de_nos_pega_duplicado_faltando_e_sobrando():
    cap = ln._captura()
    cfg = cap.le_configuracao(os.path.join(AQUI, 'esperados_robo3_mega.yaml'))
    certo = cfg['nos'] + ['/_oculto_qualquer']
    assert ln.avalia(certo, cfg, cap)['veredito'] == 'APROVADO'
    dup = ln.avalia(certo + ['/twist_mux'], cfg, cap)
    assert dup['veredito'] == 'REPROVADO' and dup['uniq_d'] == ['/twist_mux']
    falta = ln.avalia([n for n in cfg['nos'] if n != '/robot_state_publisher'], cfg, cap)
    assert falta['faltando'] == ['/robot_state_publisher']
    assert ln.avalia(cfg['nos'] + ['/intruso'], cfg, cap)['sobrando'] == ['/intruso']


def test_robo3_sim_aceita_exatamente_um_listener_do_scan_2d():
    """A família volátil do scan_2d: uma ocorrência aprova; zero e duas reprovam."""
    cap = ln._captura()
    cfg = cap.le_configuracao(os.path.join(AQUI, 'esperados_robo3_sim.yaml'))
    um = '/transform_listener_impl_5a6bfed37c90'
    outro = '/transform_listener_impl_5eeb28ab2770'
    assert ln.avalia(cfg['nos'] + [um], cfg, cap)['veredito'] == 'APROVADO'
    zero = ln.avalia(cfg['nos'], cfg, cap)
    assert zero['veredito'] == 'REPROVADO' and zero['volateis_invalidos']
    dois = ln.avalia(cfg['nos'] + [um, outro], cfg, cap)
    assert dois['veredito'] == 'REPROVADO' and dois['volateis_invalidos']
    nome_torto = ln.avalia(cfg['nos'] + ['/transform_listener_impl_XYZ'], cfg, cap)
    assert nome_torto['veredito'] == 'REPROVADO', 'fora do padrão ancorado é nó a mais'


# ─── /dev/input/js*: só mouse explícito passa ────────────────────────────────

def _classifica(tmp_path, corpo_udevadm):
    falso = tmp_path / 'bin'
    falso.mkdir(exist_ok=True)
    (falso / 'udevadm').write_text('#!/usr/bin/env bash\n' + corpo_udevadm + '\n')
    os.chmod(falso / 'udevadm', 0o755)
    env = dict(os.environ, PATH=f'{falso}:{os.environ["PATH"]}')
    r = subprocess.run(['bash', '-c', f'source {LIB}; classifica_js /dev/input/js9'],
                       env=env, capture_output=True, text=True)
    return r.stdout.strip()


@pytest.mark.parametrize('props, veredito', [
    ('ID_INPUT=1\nID_INPUT_JOYSTICK=1', 'JOYSTICK'),
    ('ID_INPUT=1\nID_INPUT_MOUSE=1\nID_INPUT_JOYSTICK=1', 'JOYSTICK'),
    ('ID_INPUT=1\nID_INPUT_MOUSE=1', 'MOUSE'),
    ('ID_INPUT=1', 'INCONCLUSIVO'),
    ('ID_INPUT=1\nID_INPUT_MOUSE=0', 'INCONCLUSIVO'),
])
def test_classificacao_do_js_pelo_udev(tmp_path, props, veredito):
    assert _classifica(tmp_path, f"printf '{props}\\n'") == veredito


def test_udevadm_falhando_e_inconclusivo(tmp_path):
    assert _classifica(tmp_path, 'echo "erro" >&2; exit 1') == 'INCONCLUSIVO'
    assert _classifica(tmp_path, 'exit 0') == 'INCONCLUSIVO', 'sem saída não é mouse'


@pytest.mark.parametrize('item', ['recusa: listou os 4', 'sobe-robo3 subiu (placa'])
def test_falha_da_recusa_ou_da_subida_encerra_o_cenario(item):
    """Nada de rodada da MEGA por cima: o REPROVADO é seguido de `return`."""
    codigo = _codigo()
    i = codigo.index(f'anota "$caso: {item}' if 'subiu' in item else f'anota "{item}')
    reprovado = codigo.index('REPROVADO', codigo.index(item, i + 1))
    assert re.match(r'[^\n]*\n\s*return\b', codigo[reprovado:]), item


def test_sobe_robo3_consulta_o_grafo_sem_daemon():
    with open(os.path.join(RAIZ, 'bin', 'sobe-robo3')) as f:
        fonte = '\n'.join(re.sub(r'(^|\s)#.*$', '', linha) for linha in f.read().splitlines())
    consultas = re.findall(r'ros2 node list[^)\n]*', fonte)
    assert consultas == ['ros2 node list --no-daemon --spin-time 5 2>&1'], consultas


def test_pre_condicao_so_deixa_passar_mouse():
    codigo = _codigo()
    bloco = codigo[codigo.index('JS_INFO="nenhum'):codigo.index('registro do sobe-robo3')]
    assert 'classifica_js' in bloco and '!= MOUSE' in bloco and 'exit 1' in bloco


# ─── o orquestrador ──────────────────────────────────────────────────────────

ORQ = os.path.join(RAIZ, 'bin', 'valida-etapa4')


def _codigo():
    with open(ORQ) as f:
        return '\n'.join(re.sub(r'(^|\s)#.*$', '', linha) for linha in f.read().splitlines())


def _codigo_de(caminho):
    with open(caminho) as f:
        return '\n'.join(re.sub(r'(^|\s)#.*$', '', linha) for linha in f.read().splitlines())


@pytest.mark.parametrize('arquivo', [ORQ, LIB])
@pytest.mark.parametrize('proibido', [
    r'\bpkill\b', r'\bkillall\b', r'\bpgrep\b', r'\bxargs\b[^\n|;]*\bkill\b',
    r'/(usr/)?bin/kill\b', r'\bbuiltin\s+kill\b', r'\bcommand\s+kill\b',
    r'\bkill\b[^\n]*--\s*"?-',          # sinal a grupo cru: só pelo processos.py
    r'docs/dados/[^\s"]*\s*>', r'>\s*"?\$R/docs/dados'])
def test_orquestrador_nao_mata_por_nome_nem_escreve_nas_baselines(arquivo, proibido):
    assert re.search(proibido, _codigo_de(arquivo)) is None, (arquivo, proibido)


def test_toda_consulta_ao_grafo_testa_o_codigo():
    chamadas = [linha.strip() for linha in _codigo_de(ORQ).splitlines()
                if re.search(r'\bnos_do_dominio\s+"', linha)]
    assert chamadas, 'montagem: o orquestrador consulta o grafo'
    for c in chamadas:
        assert re.match(r'(if\s+!?\s*nos_do_dominio|nos_do_dominio .*;\s*rc_\w+=\$\?)', c), c


def test_limpeza_instalada_logo_depois_da_marca():
    codigo = _codigo()
    marca = codigo.index('export VALIDA_ETAPA4_MARCA')
    trap = codigo.index('instala_limpeza', marca)
    assert codigo.index('colcon build') > trap, 'build antes da limpeza instalada'


def test_orquestrador_sintaxe():
    assert subprocess.run(['bash', '-n', ORQ]).returncode == 0


def test_marca_so_depois_das_pre_condicoes():
    codigo = _codigo()
    assert codigo.index('export VALIDA_ETAPA4_MARCA') > codigo.index('VALIDA_ETAPA4_MARCA=" /proc')


def test_scripts_executaveis():
    for f in (ORQ, *[os.path.join(AQUI, n) for n in
                     ('processos.py', 'mega_fingida.py', 'externo.py', 'compara_robo2.py',
                      'lista_nos.py')]):
        assert os.access(f, os.X_OK), f


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-q']))
