#!/usr/bin/env python3
"""A ponte pasta → juiz (`corrida.py`) e o `bin/valida-etapa7`, sem Gazebo.

Duas partes. A primeira exercita o `corrida.py` offline — pose e objetivo, a
assinatura do gravador, a junção dos brutos da pasta e a tabela de vereditos —
com o `le` do bag INJETADO, então nada aqui precisa de ROS. A segunda são
travas estáticas do wrapper, no molde das do `bin/valida-etapa6`: elas leem o
texto do script e conferem a ordem e as flags que a decisão 060 exige.
"""
import importlib.util
import math
import os
import re
import subprocess

import pytest
import yaml

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(os.path.dirname(AQUI))
WRAPPER = os.path.join(RAIZ, 'bin', 'valida-etapa7')


def _carrega(nome_arquivo, nome):
    spec = importlib.util.spec_from_file_location(
        nome, os.path.join(AQUI, nome_arquivo))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


@pytest.fixture(scope='module')
def corrida():
    return _carrega('corrida.py', 'corrida_etapa7')


@pytest.fixture(scope='module')
def fixtures():
    """Os brutos sintéticos do teste do montador, reaproveitados: a mesma
    corrida boa, agora escrita como PASTA."""
    return _carrega('test_monta_etapa7.py', 'fixtures_monta_etapa7')


# ─── pose e objetivo ─────────────────────────────────────────────────────────

def _pose(yaw, x=2.0, y=5.0):
    return {'position': {'x': x, 'y': y, 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw / 2),
                            'w': math.cos(yaw / 2)}}


@pytest.mark.parametrize('yaw, esperado', (
    (0.0, (3.0, 5.0)), (math.pi / 2, (2.0, 6.0)),
    (math.pi, (1.0, 5.0)), (-math.pi / 2, (2.0, 4.0))))
def test_objetivo_e_1m_no_rumo_atual(corrida, yaw, esperado):
    """Os quatro quatérnios conhecidos, a partir do spawn (2,0; 5,0)."""
    alvo, _ = corrida.objetivo(_pose(yaw), 1.0)
    assert alvo['x'] == pytest.approx(esperado[0], abs=1e-9)
    assert alvo['y'] == pytest.approx(esperado[1], abs=1e-9)


def test_echo_com_separador_final_e_um_documento(corrida):
    texto = yaml.safe_dump(_pose(0.0)) + '---\n'
    assert corrida.um_documento(texto)['position']['x'] == 2.0


@pytest.mark.parametrize('texto', ('---\n', '',
                                   'x: 1\n---\nx: 2\n---\n'))
def test_zero_ou_dois_documentos_recusa(corrida, texto):
    with pytest.raises(ValueError):
        corrida.um_documento(texto)


def test_poses_normalizadas_gravadas_para_auditoria(corrida, tmp_path, capsys):
    bruta = tmp_path / 'pose_inicial_bruta.yaml'
    bruta.write_text(yaml.safe_dump(_pose(0.0)) + '---\n')
    final = tmp_path / 'pose_final_bruta.yaml'
    final.write_text(yaml.safe_dump({'x': 2.92, 'y': 5.0, 'z': 0.0}) + '---\n')
    poses = tmp_path / 'poses.yaml'
    assert corrida.main(['objetivo', str(bruta), '1.0', str(poses)]) == 0
    assert corrida.main(['pose_final', str(final), str(poses)]) == 0
    assert yaml.safe_load(poses.read_text()) == {
        'goal': {'x': 3.0, 'y': 5.0}, 'pose_inicial': {'x': 2.0, 'y': 5.0},
        'pose_final': {'x': 2.92, 'y': 5.0}}
    distancia = capsys.readouterr().out.split()[-1]
    assert float(distancia) == pytest.approx(0.08)


# ─── o alvo enviado É o alvo gravado (revisão do b11f966, ponto B) ───────────
#
# O wrapper manda no `send_goal` o texto que o `corrida.py objetivo` imprime, e
# o juiz compara a pose final com o `goal` do `poses.yaml`. Com `%.4f` os dois
# diferiam em até ~7e-5 m. E `repr` puro não serve: o `ros2 action send_goal`
# lê o objetivo com `yaml.safe_load` (YAML 1.1), que lê `1e-05` como TEXTO.
# O texto impresso tem de ser o escalar YAML do float, o mesmo que o
# `poses.yaml` contém.

def _objetivo_impresso(corrida, tmp_path, capsys, pose):
    bruta = tmp_path / 'pose_inicial_bruta.yaml'
    bruta.write_text(yaml.safe_dump(pose) + '---\n')
    poses = tmp_path / 'poses.yaml'
    capsys.readouterr()
    assert corrida.main(['objetivo', str(bruta), '1.0', str(poses)]) == 0
    campos = capsys.readouterr().out.split()
    return campos[2], campos[3], poses.read_text()


@pytest.mark.parametrize('pose', (
    _pose(0.3, x=2.123456789, y=5.987654321),      # alvo com 16 dígitos
    _pose(math.pi / 2, x=0.0, y=0.0),              # alvo x = 6.1e-17
    _pose(-2.0, x=-1.5, y=0.25)))
def test_o_alvo_impresso_e_o_texto_gravado_no_poses_yaml(
        corrida, tmp_path, capsys, pose):
    meta_x, meta_y, gravado = _objetivo_impresso(corrida, tmp_path, capsys, pose)
    goal = yaml.safe_load(gravado)['goal']
    # o mesmo TEXTO, byte a byte, nas duas pontas
    assert f'x: {meta_x}\n' in gravado, (meta_x, gravado)
    assert f'y: {meta_y}\n' in gravado, (meta_y, gravado)
    # e o ROS relê o mesmo double que o juiz vai usar
    enviado = yaml.safe_load('{x: %s, y: %s}' % (meta_x, meta_y))
    assert type(enviado['x']) is float and type(enviado['y']) is float
    assert enviado['x'] == goal['x'] and enviado['y'] == goal['y']
    esperado, _ = corrida.objetivo(pose, 1.0)
    assert goal == esperado


@pytest.mark.parametrize('valor', (1e-05, 6.123233995736766e-17, 3.0,
                                   -0.1, 1e16, 2.123456789012345))
def test_o_texto_do_alvo_e_float_para_o_yaml_do_ros(corrida, valor):
    """`repr(1e-05)` é `1e-05`, que o YAML 1.1 lê como texto; o escalar tem de
    voltar float e idêntico."""
    texto = corrida.escalar_yaml(valor)
    lido = yaml.safe_load('{x: %s}' % texto)['x']
    assert type(lido) is float and lido == valor, (texto, lido)


# ─── o gravador assina? ──────────────────────────────────────────────────────

def test_gravador_assinante_e_reconhecido(corrida, fixtures):
    assert corrida.gravador_assina(fixtures._grafo())


def test_sem_o_gravador_entre_os_assinantes_nao_assina(corrida, fixtures):
    assert not corrida.gravador_assina(
        fixtures._grafo(subs=('hoverboard_base_controller',)))


@pytest.mark.parametrize('texto', (None, 'Unknown topic', ''))
def test_topic_info_ilegivel_nao_assina(corrida, texto):
    assert not corrida.gravador_assina(texto)


# ─── a pasta vira brutos, e os brutos viram vereditos ────────────────────────

def _pasta_boa(tmp_path, fixtures):
    b = fixtures._brutos()
    (tmp_path / 'objetivo.log').write_text(b['objetivo_log'])
    (tmp_path / 'placa_simulada.yaml').write_text(b['dump_placa'])
    (tmp_path / 'controller_server.yaml').write_text(b['dump_controller'])
    (tmp_path / 'grafo_antes.txt').write_text(b['grafo_antes'])
    (tmp_path / 'grafo_depois.txt').write_text(b['grafo_depois'])
    (tmp_path / 'corrida_2026-09-25_120000').mkdir()
    (tmp_path / 'corrida_2026-09-25_120000' / 'perfil_nav2.yaml').write_text(
        b['nav2_materializado'])
    (tmp_path / 'poses.yaml').write_text(yaml.safe_dump(
        {k: b[k] for k in ('goal', 'pose_inicial', 'pose_final')}))

    def le(caminho):
        assert caminho == str(tmp_path / 'bag')
        return {'amostras': b['amostras'], 'status': b['status']}
    return le


def _julga_pasta(corrida, pasta, le):
    brutos, erro_bag = corrida.brutos_da_pasta(str(pasta), le)
    evidencia, falhas = corrida.monta.monta(brutos)
    return corrida.vereditos(evidencia, falhas, erro_bag, corrida.julga.avalia)


def test_pasta_boa_aprova_os_cinco_de_coleta_e_os_oito_do_juiz(
        corrida, fixtures, tmp_path):
    linhas = _julga_pasta(corrida, tmp_path, _pasta_boa(tmp_path, fixtures))
    assert len(linhas) == 5 + 8, linhas
    assert all(v == 'APROVADO' for _, v, _ in linhas), linhas


def test_bag_ilegivel_e_item_de_coleta_proprio(corrida, fixtures, tmp_path):
    _pasta_boa(tmp_path, fixtures)

    def le(_):
        raise RuntimeError('mcap sem metadata')
    linhas = dict((i, (v, d)) for i, v, d in
                  _julga_pasta(corrida, tmp_path, le))
    assert linhas['coleta: bag legível'][0] == 'REPROVADO'
    assert 'mcap sem metadata' in linhas['coleta: bag legível'][1]
    # sem status, a ação também reprova — pelo nome, não por fallback
    assert linhas['coleta: ação e janela (status × objetivo.log)'][0] == \
        'REPROVADO'


def test_arquivo_ausente_reprova_a_fonte_dele(corrida, fixtures, tmp_path):
    le = _pasta_boa(tmp_path, fixtures)
    (tmp_path / 'placa_simulada.yaml').unlink()
    linhas = dict((i, v) for i, v, _ in _julga_pasta(corrida, tmp_path, le))
    assert linhas['coleta: parâmetros da placa'] == 'REPROVADO'
    assert linhas['coleta: ação e janela (status × objetivo.log)'] == 'APROVADO'


def test_dois_materializados_nao_escolhe_um(corrida, fixtures, tmp_path):
    le = _pasta_boa(tmp_path, fixtures)
    outra = tmp_path / 'corrida_2026-09-25_130000'
    outra.mkdir()
    (outra / 'perfil_nav2.yaml').write_text(
        (tmp_path / 'corrida_2026-09-25_120000' / 'perfil_nav2.yaml')
        .read_text())
    linhas = dict((i, v) for i, v, _ in _julga_pasta(corrida, tmp_path, le))
    assert linhas['coleta: tolerância viva (× materializado)'] == 'REPROVADO'


# ─── o wrapper (travas estáticas) ────────────────────────────────────────────

def _texto():
    return open(WRAPPER).read()


def _codigo():
    return [l for l in _texto().splitlines() if not l.lstrip().startswith('#')]


def _primeira(trecho):
    achados = [i for i, l in enumerate(_codigo()) if trecho in l]
    assert achados, f'não achei {trecho!r} no código do wrapper'
    return achados[0]


def test_o_wrapper_nunca_usa_pkill():
    assert not [l for l in _codigo() if 'pkill' in l]


def test_o_wrapper_nao_apaga_a_pasta_de_evidencia():
    assert 'rm -rf' not in _texto()


def test_a_pilha_sobe_headless_robo3_fixa_e_sem_o_gravador_da_launch():
    texto = _texto()
    for arg in ('robo:=3', 'sim:=true', 'gui:=false', 'rviz:=false',
                'localizacao:=fixa', 'bag:=false'):
        assert arg in texto, arg


def test_o_gravador_grava_ocultos_em_tempo_simulado():
    i = _primeira('ros2 bag record --all-topics')
    linha = _codigo()[i] + _codigo()[i + 1]
    for flag in ('--all-topics', '--include-hidden-topics', '--use-sim-time'):
        assert flag in linha, flag


def test_dominio_proprio():
    assert 'DOMINIO=49' in _texto()


def test_a_arvore_suja_barra():
    bloco = _texto().split('status --porcelain)" ]; then', 1)[1][:300]
    assert 'exit 1' in bloco


def test_a_ordem_da_corrida():
    """build → pilha → gravador → assinou → leituras vivas e grafo de antes →
    goal → grafo de depois → fecha o bag → julga."""
    ordem = [_primeira('colcon build'),
             _primeira('ros2 launch robot_motion'),
             _primeira('ros2 bag record --all-topics'),
             _primeira('corrida.py" assina'),
             _primeira('param dump --timeout 10 /placa_simulada'),
             _primeira('param dump --timeout 10 /controller_server'),
             _primeira('grafo_antes.txt'),
             _primeira('ros2 action send_goal'),
             _primeira('grafo_depois.txt'),
             _primeira('corrida.py" pose_final'),
             [i for i, l in enumerate(_codigo()) if l.strip() == 'fecha_gravador'][0],
             _primeira('corrida.py" julga')]
    assert ordem == sorted(ordem), ordem


def test_sem_assinatura_do_gravador_nao_manda_goal():
    bloco = _texto().split('if [ "$ASSINOU" -ne 1 ]; then', 1)[1][:400]
    assert 'REPROVADO' in bloco and 'exit 1' in bloco


def test_o_sinal_dirigido_ao_gravador_e_term():
    dirigidos = [l for l in _codigo() if 'sinaliza_um' in l]
    assert dirigidos
    for l in dirigidos:
        assert ' TERM' in l and ' INT' not in l, l


def test_os_wrappers_congelados_nao_sao_chamados():
    texto = _texto()
    assert 'bin/valida-etapa6' not in texto.replace('`bin/valida-etapa6`', '')
    assert 'explora-objetivo-robo3 ' not in texto


# ─── o teto no wrapper (revisão do b11f966, ponto A) ─────────────────────────
#
# O laço de espera é EXECUTADO, não só lido: o trecho entre os dois marcadores
# roda num bash com `relogio`, `anota` e o PID do goal trocados por calços.
# Nada de ROS: o "goal" é um `sleep`.

INICIO_ESPERA = '# ── esperar a ação'
FIM_ESPERA = '# O `objetivo.log` só se lê depois de o send_goal SAIR'


def _trecho_de_espera():
    texto = _texto()
    assert INICIO_ESPERA in texto and FIM_ESPERA in texto
    return texto.split(INICIO_ESPERA, 1)[1].split(FIM_ESPERA, 1)[0]


def _roda_espera(tmp_path, relogio, teto, goal='sleep 30'):
    anotado = tmp_path / 'anotado.txt'
    script = f"""
set +u
anota() {{ printf '%s|%s|%s\n' "$1" "$2" "${{3:-}}" >> '{anotado}'; }}
relogio() {{ {relogio}; }}
TETO_SIMULADO={teto}
T0_SIM=100
{goal} &
PID_GOAL=$!
sleep 0.2
# {_trecho_de_espera()}
kill -TERM "$PID_GOAL" 2>/dev/null
wait "$PID_GOAL" 2>/dev/null
exit 0
"""
    r = subprocess.run(['bash', '-c', script], capture_output=True, text=True,
                       timeout=60)
    assert r.returncode == 0, r.stderr
    return [l.split('|') for l in anotado.read_text().splitlines()]


def test_goal_vivo_depois_do_relogio_passar_de_60s_nao_reprova(tmp_path):
    """O relógio lido pelo wrapper já passou 61 s do seu marco, mas o goal ainda
    está vivo (aceite atrasado em relação ao marco, ou cliente ainda saindo
    depois de um terminal em 60 s exatos). A duração canônica pode caber no
    teto — só o juiz, pelo bag, sabe. O wrapper espera e não reprova."""
    linhas = _roda_espera(tmp_path, 'echo 161', 60, goal='sleep 1')
    assert linhas and all(l[1] != 'REPROVADO' for l in linhas), linhas


def test_o_wrapper_nao_tem_corte_por_relogio_simulado():
    """O teto de 60 s simulados é só do juiz. Um relógio colhido pelo wrapper
    antes do goal não é autoridade sobre a duração da ação."""
    codigo = '\n'.join(_codigo())
    assert 'relogio' not in codigo
    assert 'T0_SIM' not in codigo
    assert '(/clock) respondendo' not in _texto()


def test_o_watchdog_de_parede_do_wrapper_reprova(tmp_path):
    """Goal que não fecha: quem para é o watchdog (6 × teto de parede; com
    teto 1, seis segundos), e é REPROVADO pelo próprio motivo."""
    linhas = _roda_espera(tmp_path, 'echo', 1)
    assert [l[1] for l in linhas] == ['REPROVADO'], linhas
    assert 'WATCHDOG' in linhas[0][2], linhas
    assert 'relógio' not in linhas[0][2], 'o watchdog não infere nada do /clock'


def test_a_acao_que_termina_sozinha_nao_e_reprovada_pelo_wrapper(tmp_path):
    """Terminar antes do teto não é aprovação da corrida — isso é do juiz —,
    mas o wrapper não pode inventar reprovação."""
    linhas = _roda_espera(tmp_path, 'echo 100', 60, goal='true')
    assert linhas and all(l[1] != 'REPROVADO' for l in linhas), linhas


def test_teto_do_wrapper_e_do_juiz_sao_o_mesmo_60(corrida):
    """Os dois números vivem em arquivos diferentes; divergirem em silêncio
    faria o wrapper parar numa hora e o juiz julgar noutra."""
    achado = re.findall(r'^TETO_SIMULADO=(\d+)\b', _texto(), re.M)
    assert achado == ['60'], achado
    assert corrida.julga.TETO_SIMULADO == 60
