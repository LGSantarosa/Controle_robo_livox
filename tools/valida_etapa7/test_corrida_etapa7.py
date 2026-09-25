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


def test_pasta_boa_aprova_os_cinco_de_coleta_e_os_sete_do_juiz(
        corrida, fixtures, tmp_path):
    linhas = _julga_pasta(corrida, tmp_path, _pasta_boa(tmp_path, fixtures))
    assert len(linhas) == 5 + 7, linhas
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
