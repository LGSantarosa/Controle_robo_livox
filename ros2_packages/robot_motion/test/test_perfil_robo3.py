"""O perfil do robô 3 — etapa 4, passo 4c (PLANO_ETAPA4_ROBO3.md §§2–5).

`parametros(3, share_motion, share_base=...)` monta o robô 3 sobre a MESMA
base do robô 2 (`nav2.yaml`, `collision_monitor.yaml`, defaults do
`path_follower`) mais `config/perfil_robo3.yaml`, que guarda SÓ política
(classe (c)) e referências. A geometria (classe (b)) vem do artefato da 052,
`robot_base/config/geometria_robo3.yaml`, e nunca é redigitada.

O que dá para provar nesta etapa é o que chega AO ARQUIVO que o nó leria (o
YAML reescrito) e ao dicionário do `path_follower` — não o que o nó aceita:
nenhum nó do robô 3 roda na etapa 4 (D1). O `ros2 param get` dos costmaps vivos
é critério herdado da etapa 6.

Os números esperados abaixo são a coluna "montado" do §5 do plano. A proibição
de redigitar vale para o PERFIL; o teste que o confere escreve o resultado.
"""
import importlib
import math
import os
import subprocess
import sys

import pytest
import yaml

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
BASE = os.path.abspath(os.path.join(PKG, '..', 'robot_base'))
SEGUIDOR = os.path.join(PKG, 'robot_motion', 'path_follower.py')
PERFIL3 = os.path.join(PKG, 'config', 'perfil_robo3.yaml')
GEOMETRIA = os.path.join(BASE, 'config', 'geometria_robo3.yaml')
COSTMAPS = ('global_costmap', 'local_costmap')
PADDING_D2 = 0.009999999776482582
CM_RAIZ = ('collision_monitor', 'ros__parameters')


def _perfil():
    return importlib.import_module('robot_motion.perfil')


def _p3(share_base=BASE):
    return _perfil().parametros(3, PKG, share_base=share_base)


def _le(caminho):
    with open(caminho) as f:
        return yaml.safe_load(f)


def _folhas(dados, prefixo=()):
    if isinstance(dados, dict):
        out = {}
        for k, v in dados.items():
            out.update(_folhas(v, prefixo + (k,)))
        return out
    return {prefixo: dados}


def _montado(p):
    """(nav2 reescrito, collision_monitor reescrito) — o que os nós leriam."""
    ap = _perfil().aplica_reescritas
    return (ap(_le(p['nav2']), p['nav2_rewrites']),
            ap(_le(p['collision_monitor']), p['collision_monitor_rewrites']))


def _cm(cm):
    return cm['collision_monitor']['ros__parameters']


def _mesmos_vertices(texto, esperado):
    """Mesma ORDEM (frente-esq, frente-dir, trás-dir, trás-esq — a do robô 2),
    vértice a vértice: o `approx` desta máquina não aceita lista aninhada."""
    assert isinstance(texto, str), 'o Nav2 exige os pontos como TEXTO'
    obtido = yaml.safe_load(texto)
    assert len(obtido) == len(esperado)
    for o, e in zip(obtido, esperado):
        assert o == pytest.approx(e), (obtido, esperado)


def _declarados_do_seguidor():
    import ast
    with open(SEGUIDOR) as f:
        arvore = ast.parse(f.read())
    return {no.elts[0].value for no in ast.walk(arvore)
            if isinstance(no, ast.Tuple) and len(no.elts) == 2
            and isinstance(no.elts[0], ast.Constant)
            and isinstance(no.elts[0].value, str)}


# ─── a interface ─────────────────────────────────────────────────────────────

def test_robo3_monta():
    p = _p3()
    assert set(p) == {'nav2', 'nav2_rewrites', 'collision_monitor',
                      'collision_monitor_rewrites', 'path_follower'}
    # A mesma base do robô 2: o robô 3 é sobreposição, não cópia.
    assert p['nav2'] == os.path.join(PKG, 'config', 'nav2.yaml')
    assert p['collision_monitor'] == os.path.join(PKG, 'config', 'collision_monitor.yaml')


def test_robo3_sem_share_base_recusa():
    """Sem o share do robot_base não há artefato — erro que diz isso, e não um
    perfil sem geometria."""
    with pytest.raises(ValueError, match='share_base'):
        _perfil().parametros(3, PKG)


def test_share_base_so_por_nome():
    """⚠️ ANTECIPADAMENTE VERDE: antes da 4c passa pela aridade antiga (o
    argumento nem existe); depois, guarda o `*` da assinatura nova."""
    with pytest.raises(TypeError):
        _perfil().parametros(3, PKG, BASE)   # noqa: posicional de propósito


def test_robo2_continua_sem_share_base():
    p = _perfil().parametros(2, PKG)
    assert p['nav2_rewrites'] == {} and p['collision_monitor_rewrites'] == {}


def test_montar_os_dois_perfis_e_puro():
    """CHAMA `parametros` (2 e 3), não só importa: montar não pode puxar ROS,
    launch, ament_index nem xacro."""
    codigo = (
        'import sys; from robot_motion import perfil; '
        f'perfil.parametros(2, {PKG!r}); '
        f'perfil.parametros(3, {PKG!r}, share_base={BASE!r}); '
        'print(sorted({m.split(".")[0] for m in sys.modules} & '
        '{"rclpy", "launch", "launch_ros", "ament_index_python", "xacro"}))')
    saida = subprocess.run([sys.executable, '-c', codigo], cwd=PKG, check=True,
                           capture_output=True, text=True,
                           env={**os.environ, 'PYTHONPATH': PKG}).stdout.strip()
    assert saida == '[]'


# ─── §3: footprint canônico, sem copiar vértice ──────────────────────────────

def test_reescritas_exatas_do_nav2():
    chaves = {(c, c, 'ros__parameters', k) for c in COSTMAPS
              for k in ('footprint', 'footprint_padding')}
    assert set(_p3()['nav2_rewrites']) == chaves


def test_footprint_dos_DOIS_costmaps_e_o_artefato():
    artefato = _le(GEOMETRIA)['footprint']['poligono']
    nav2, _ = _montado(_p3())
    for c in COSTMAPS:
        fp = nav2[c][c]['ros__parameters']['footprint']
        assert isinstance(fp, str), 'o Nav2 exige o footprint como TEXTO'
        assert yaml.safe_load(fp) == artefato, c


def test_nenhuma_outra_folha_dos_dois_yamls_muda():
    p = _p3()
    nav2, cm = _montado(p)
    for base, novo, reesc in ((_le(p['nav2']), nav2, p['nav2_rewrites']),
                              (_le(p['collision_monitor']), cm,
                               p['collision_monitor_rewrites'])):
        antes, depois = _folhas(base), _folhas(novo)
        assert set(depois) == set(antes)
        assert {c for c in antes if antes[c] != depois[c]} <= set(reesc)


def _geometria_mexida(tmp_path):
    share = tmp_path / 'robot_base'
    (share / 'config').mkdir(parents=True)
    geo = _le(GEOMETRIA)
    geo['footprint']['poligono'] = [[0.10, 0.20], [0.10, -0.20],
                                    [-0.30, -0.20], [-0.30, 0.20]]
    geo['raio_varrido_pivo'] = 0.33
    (share / 'config' / 'geometria_robo3.yaml').write_text(yaml.safe_dump(geo))
    return str(share)


def test_a_geometria_vem_do_artefato_e_nao_de_copia(tmp_path):
    """Mexe no artefato (numa cópia) e TUDO o que é (b) acompanha."""
    p = _p3(_geometria_mexida(tmp_path))
    nav2, cm = _montado(p)
    for c in COSTMAPS:
        assert yaml.safe_load(nav2[c][c]['ros__parameters']['footprint']) == \
            [[0.10, 0.20], [0.10, -0.20], [-0.30, -0.20], [-0.30, 0.20]]
    _mesmos_vertices(_cm(cm)['PolygonApproach']['points'],
                     [[0.13, 0.23], [0.13, -0.23], [-0.33, -0.23], [-0.33, 0.23]])
    _mesmos_vertices(_cm(cm)['PolygonStop']['points'],
                     [[0.233, 0.25], [0.233, -0.25], [-0.35, -0.25], [-0.35, 0.25]])
    pf = p['path_follower']
    assert pf['passagem_meia_largura'] == pytest.approx(0.20)
    assert pf['avanco_para_choque'] == pytest.approx(0.10)
    assert pf['re_recuo_para_choque'] == pytest.approx(0.30)
    assert pf['re_largura'] == pytest.approx(0.50)
    assert pf['desencalhe_pivo_folga'] == pytest.approx(0.35)


def _politica_mexida(tmp_path):
    """Um `share` do robot_motion com as bases de verdade e o perfil mexido."""
    share = tmp_path / 'robot_motion'
    (share / 'config').mkdir(parents=True)
    for arq in ('nav2.yaml', 'collision_monitor.yaml'):
        (share / 'config' / arq).write_text(
            open(os.path.join(PKG, 'config', arq)).read())
    perfil = _le(PERFIL3)
    perfil['footprint_padding']['valor'] = 0.02
    perfil['approach_margem'].update(faces=0.05, time_before_collision=1.0)
    perfil['stop_margem'].update(frente=0.2, lados=0.1, tras=0.07)
    perfil['passagem_margem']['valor'] = 0.04
    perfil['desencalhe_pivo_margem']['valor'] = 0.03
    (share / 'config' / 'perfil_robo3.yaml').write_text(yaml.safe_dump(perfil))
    return str(share)


def test_a_politica_vem_do_perfil_e_nao_do_codigo(tmp_path):
    """Mexe no perfil (numa cópia) e TUDO o que é (c) acompanha."""
    p = _perfil().parametros(3, _politica_mexida(tmp_path), share_base=BASE)
    nav2, cm = _montado(p)
    for c in COSTMAPS:
        assert nav2[c][c]['ros__parameters']['footprint_padding'] == 0.02, c
    _mesmos_vertices(_cm(cm)['PolygonApproach']['points'],
                     [[0.1325, 0.24], [0.1325, -0.24], [-0.3413, -0.24], [-0.3413, 0.24]])
    assert _cm(cm)['PolygonApproach']['time_before_collision'] == 1.0
    _mesmos_vertices(_cm(cm)['PolygonStop']['points'],
                     [[0.2825, 0.29], [0.2825, -0.29], [-0.3613, -0.29], [-0.3613, 0.29]])
    pf = p['path_follower']
    assert pf['passagem_margem'] == pytest.approx(0.04)
    assert pf['re_largura'] == pytest.approx(0.58)
    assert pf['desencalhe_pivo_folga'] == pytest.approx(0.3425)
    # geometria pura não muda com a política
    assert pf['passagem_meia_largura'] == pytest.approx(0.190)
    assert pf['avanco_para_choque'] == pytest.approx(0.0825)
    assert pf['re_recuo_para_choque'] == pytest.approx(0.2913)


# ─── o arquivo do perfil: (b) só por referência, (c) em chave própria ────────

CHAVES_DE_B = {'poligono', 'footprint', 'points', 'raio_varrido_pivo',
               'passagem_meia_largura', 're_largura', 're_recuo_para_choque',
               'avanco_para_choque', 'desencalhe_pivo_folga'}


def _numeros(v):
    if isinstance(v, bool):
        return []
    if isinstance(v, (int, float)):
        return [v]
    if isinstance(v, dict):
        return [n for x in v.values() for n in _numeros(x)]
    if isinstance(v, list):
        return [n for x in v for n in _numeros(x)]
    return []


def _chaves(v):
    if isinstance(v, dict):
        return set(v) | {k for x in v.values() for k in _chaves(x)}
    if isinstance(v, list):
        return {k for x in v for k in _chaves(x)}
    return set()


def test_toda_entrada_tem_classe_e_origem():
    perfil = _le(PERFIL3)
    for nome, entrada in perfil.items():
        if nome in ('herdados_provisorios', 'independentes_do_robo'):
            continue
        assert isinstance(entrada, dict), nome
        assert entrada.get('classe') in ('b', 'c'), nome
        assert entrada.get('origem'), nome


def test_referencia_de_b_nao_tem_valor():
    """Estrutural: entrada (b) só aponta — nenhum número dentro dela."""
    perfil = _le(PERFIL3)
    bs = {n: e for n, e in perfil.items()
          if isinstance(e, dict) and e.get('classe') == 'b'}
    assert bs, 'o perfil tem de referenciar a geometria'
    for nome, entrada in bs.items():
        assert set(entrada) <= {'classe', 'origem', 'arquivo', 'decisao'}, nome
        assert _numeros(entrada) == [], nome
    assert bs['geometria']['arquivo'] == 'geometria_robo3.yaml'


def test_nenhuma_quantidade_de_b_mora_no_perfil():
    """Nem como chave, nem como lista de vértices, nem como número montado."""
    perfil = _le(PERFIL3)
    assert not (_chaves(perfil) & CHAVES_DE_B)
    montados = [0.1125, -0.3213, 0.22, 0.2155, -0.3413, 0.24, 0.19, 0.48,
                0.2913, 0.0825, 0.3125, 0.3325]
    for n in _numeros(perfil):
        for m in montados:
            assert not math.isclose(n, m, abs_tol=1e-9), \
                f'{n} é valor montado/(b) dentro do perfil'


def test_margens_de_c_em_chaves_separadas():
    perfil = _le(PERFIL3)
    for nome in ('footprint_padding', 'approach_margem', 'stop_margem',
                 'passagem_margem', 'desencalhe_pivo_margem'):
        assert perfil[nome]['classe'] == 'c', nome
    assert perfil['approach_margem'] != perfil['stop_margem']
    assert perfil['stop_margem']['frente'] == 0.133
    assert perfil['stop_margem']['lados'] == 0.05
    assert perfil['stop_margem']['tras'] == 0.05
    assert perfil['approach_margem']['faces'] == 0.03
    assert perfil['approach_margem']['time_before_collision'] == 0.75
    assert perfil['passagem_margem']['valor'] == 0.03
    assert perfil['desencalhe_pivo_margem']['valor'] == 0.020


# ─── o que o robô 3 herda do robô 2: partição FECHADA ─────────────────────────
#
# Regra de inclusão (resposta à revisão de 21-09): TODA folha numérica das três
# bases que o robô 3 consome — `nav2.yaml`, `collision_monitor.yaml` e os
# defaults declarados do `path_follower` — está em exatamente UM grupo:
#
#   sobrescrito   o perfil 3 reescreve/sobrepõe (§§3–5);
#   herdado       o valor foi medido, sintonizado ou derivado NO ROBÔ 2
#                 (dinâmica, atuador, geometria, sensor, comportamento na
#                 pista) — provisório, com a etapa que o fecha;
#   independente  não depende do robô: frequência/tempo de software, contagem,
#                 parâmetro de algoritmo, resolução, ou recurso desligado.
#
# Parâmetro novo na base sem classificação REPROVA. Bool e texto ficam fora
# (nenhum deles carrega medida). Etapas (PLANO_NAV2_ROBO3.md): 7 = pose do
# Livox; 8 = escala, dinâmica, rumo e curvatura; 9 = percepção e reflexo;
# 10 = navegação no espaço livre (comportamento: inflação, mira, passagem).

_NAV = 'nav2:'
_G = _NAV + 'global_costmap.global_costmap.ros__parameters.'
_L = _NAV + 'local_costmap.local_costmap.ros__parameters.'
_CM = 'collision_monitor:collision_monitor.ros__parameters.'
_PF = 'path_follower:'

HERDADOS = {
    # nav2 — servidores
    _NAV + 'planner_server.ros__parameters.GridBased.tolerance': 'etapa 8',
    _NAV + 'planner_server.ros__parameters.GridBased.cost_travel_multiplier': 'etapa 10',
    _NAV + 'controller_server.ros__parameters.goal_checker.xy_goal_tolerance': 'etapa 8',
    _NAV + 'controller_server.ros__parameters.FollowPath.desired_linear_vel': 'etapa 8',
    _NAV + 'controller_server.ros__parameters.FollowPath.lookahead_dist': 'etapa 8',
    _NAV + 'controller_server.ros__parameters.FollowPath.max_angular_accel': 'etapa 8',
    _NAV + 'smoother_server.ros__parameters.suave.minimum_turning_radius': 'etapa 8',
    _NAV + 'smoother_server.ros__parameters.suave.w_curve': 'etapa 10',
    _NAV + 'smoother_server.ros__parameters.suave.w_dist': 'etapa 10',
    _NAV + 'smoother_server.ros__parameters.suave.w_smooth': 'etapa 10',
    _NAV + 'smoother_server.ros__parameters.suave.w_cost': 'etapa 10',
    # nav2 — costmaps (os dois)
    **{c + k: e for c in (_G, _L) for k, e in (
        ('obstacle_layer.origin_z', 'etapa 7'),
        ('obstacle_layer.z_resolution', 'etapa 7'),
        ('obstacle_layer.z_voxels', 'etapa 7'),
        ('obstacle_layer.livox.min_obstacle_height', 'etapa 7'),
        ('obstacle_layer.livox.max_obstacle_height', 'etapa 7'),
        ('obstacle_layer.livox.obstacle_max_range', 'etapa 9'),
        ('obstacle_layer.livox.raytrace_max_range', 'etapa 9'),
        ('obstacle_layer.livox.expected_update_rate', 'etapa 9'),
        ('inflation_layer.inflation_radius', 'etapa 10'),
        ('inflation_layer.cost_scaling_factor', 'etapa 10'))},
    # collision_monitor
    _CM + 'source_timeout': 'etapa 9',
    _CM + 'PolygonApproach.min_points': 'etapa 9',
    _CM + 'PolygonStop.min_points': 'etapa 9',
    _CM + 'livox.min_height': 'etapa 7',
    _CM + 'livox.max_height': 'etapa 7',
    # path_follower
    **{_PF + n: e for n, e in (
        ('raio_min_curva', 'etapa 8'), ('lookahead_fator', 'etapa 8'),
        ('lookahead_piso', 'etapa 8'), ('k_lat', 'etapa 8'),
        ('desvio_v_ref', 'etapa 8'), ('desvio_teto_deg', 'etapa 8'),
        ('desvio_taxa_deg_s', 'etapa 8'), ('v_max', 'etapa 8'),
        ('a_lin', 'etapa 8'), ('wz_max', 'etapa 8'),
        ('passagem_v_max', 'etapa 8'), ('v_piso', 'etapa 8'),
        ('raio_chegada', 'etapa 8'), ('tolerancia_rumo_final', 'etapa 8'),
        ('re_folga', 'etapa 8'), ('re_avanco_min', 'etapa 8'),
        ('re_orcamento_cego', 'etapa 8'), ('desencalhe_pivo_angulo_deg', 'etapa 8'),
        ('desencalhe_pivo_wz', 'etapa 8'),
        ('re_scan_velho_s', 'etapa 9'),
        ('mira_longa', 'etapa 10'), ('mira_tol_estica', 'etapa 10'),
        ('mira_tol_encolhe', 'etapa 10'), ('mira_rumo_estica_deg', 'etapa 10'),
        ('mira_rumo_encolhe_deg', 'etapa 10'), ('mira_rumo_passo', 'etapa 10'),
        ('mira_folga_min', 'etapa 10'), ('passagem_largura_min', 'etapa 10'),
        ('passagem_largura_max', 'etapa 10'), ('passagem_antecipacao', 'etapa 10'),
        ('passagem_saida', 'etapa 10'), ('passagem_liberacao', 'etapa 10'),
        ('passagem_alinha_lateral', 'etapa 10'),
        ('passagem_alinha_rumo_deg', 'etapa 10'),
        ('desencalhe_frente_dist', 'etapa 10'),
        ('desencalhe_frente_folga', 'etapa 10'),
        ('re_bloqueio_frente_max', 'etapa 10'))},
}

INDEPENDENTES = {
    _NAV + 'bt_navigator.ros__parameters.bt_loop_duration',
    _NAV + 'bt_navigator.ros__parameters.default_server_timeout',
    _NAV + 'planner_server.ros__parameters.expected_planner_frequency',
    _NAV + 'planner_server.ros__parameters.GridBased.downsampling_factor',
    _NAV + 'planner_server.ros__parameters.GridBased.max_iterations',
    _NAV + 'planner_server.ros__parameters.GridBased.max_on_approach_iterations',
    _NAV + 'planner_server.ros__parameters.GridBased.max_planning_time',
    _NAV + 'planner_server.ros__parameters.GridBased.angle_quantization_bins',
    _NAV + 'planner_server.ros__parameters.GridBased.smoother.max_iterations',
    _NAV + 'planner_server.ros__parameters.GridBased.smoother.w_smooth',
    _NAV + 'planner_server.ros__parameters.GridBased.smoother.w_data',
    _NAV + 'planner_server.ros__parameters.GridBased.smoother.tolerance',
    _NAV + 'controller_server.ros__parameters.controller_frequency',
    _NAV + 'controller_server.ros__parameters.progress_checker.required_movement_radius',
    _NAV + 'controller_server.ros__parameters.progress_checker.movement_time_allowance',
    _NAV + 'controller_server.ros__parameters.goal_checker.yaw_goal_tolerance',
    _NAV + 'smoother_server.ros__parameters.suave.path_downsampling_factor',
    _NAV + 'smoother_server.ros__parameters.suave.path_upsampling_factor',
    _NAV + 'smoother_server.ros__parameters.simples.tolerance',
    _NAV + 'smoother_server.ros__parameters.simples.max_its',
    _NAV + 'smoother_server.ros__parameters.savgol.refinement_num',
    *{c + k for c in (_G, _L) for k in (
        'update_frequency', 'publish_frequency', 'resolution',
        'obstacle_layer.mark_threshold', 'obstacle_layer.combination_method')},
    _L + 'width',
    _L + 'height',
    _CM + 'transform_tolerance',
    _CM + 'stop_pub_timeout',
    _CM + 'PolygonApproach.simulation_time_step',
    *{_PF + n for n in (
        're_max_sem_plano', 're_parado_s', 're_teto_s', 'desencalhe_pivo_teto_s',
        're_max_seguidas', 'taxa', 'timeout_plano', 'log_periodo_s')},
}


def _numerica(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _universo():
    """Toda folha numérica das três bases, com o nome qualificado."""
    u = set()
    for prefixo, arq in (('nav2:', 'nav2.yaml'),
                         ('collision_monitor:', 'collision_monitor.yaml')):
        for c, v in _folhas(_le(os.path.join(PKG, 'config', arq))).items():
            if _numerica(v):
                u.add(prefixo + '.'.join(c))
    import ast
    with open(SEGUIDOR) as f:
        arvore = ast.parse(f.read())
    for no in ast.walk(arvore):
        if (isinstance(no, ast.Tuple) and len(no.elts) == 2
                and isinstance(no.elts[0], ast.Constant)
                and isinstance(no.elts[0].value, str)
                and isinstance(no.elts[1], ast.Constant)
                and _numerica(no.elts[1].value)):
            u.add(_PF + no.elts[0].value)
    return u


def _sobrescritos(p):
    return ({'nav2:' + '.'.join(c) for c in p['nav2_rewrites']}
            | {'collision_monitor:' + '.'.join(c) for c in p['collision_monitor_rewrites']}
            | {_PF + n for n in p['path_follower']})


def test_particao_fechada_das_bases_do_robo3():
    """Nada que o robô 3 herda some por omissão: cada folha numérica da base
    está em UM grupo, e parâmetro novo sem grupo reprova."""
    s, h, i = _sobrescritos(_p3()), set(HERDADOS), INDEPENDENTES
    assert not (s & h) and not (s & i) and not (h & i), (s & h, s & i, h & i)
    u = _universo()
    assert u - (s | h | i) == set(), 'sem classificação'
    # Herdado/independente é folha NUMÉRICA que existe; sobrescrito pode ser
    # texto (footprint, points) — a existência dele é o `aplica_reescritas`.
    assert (h | i) - u == set(), 'classificado mas não existe na base'


def test_herdados_provisorios_lista_exata():
    """No perfil, cada herdado com origem e a etapa EXATA que o fecha."""
    herdados = _le(PERFIL3)['herdados_provisorios']
    nomes = [h['parametro'] for h in herdados]
    assert len(nomes) == len(set(nomes)), 'herdado repetido'
    assert {h['parametro']: h['fecha'] for h in herdados} == HERDADOS
    for h in herdados:
        assert set(h) == {'parametro', 'origem', 'fecha'}, h['parametro']
        assert isinstance(h['origem'], str) and h['origem'].strip(), h['parametro']
    v_piso = next(h for h in herdados if h['parametro'] == _PF + 'v_piso')
    assert '020' in v_piso['origem'], 'v_piso vem da decisão 020, no robô 2'


def test_independentes_lista_exata_com_motivo():
    ind = _le(PERFIL3)['independentes_do_robo']
    nomes = [x['parametro'] for x in ind]
    assert len(nomes) == len(set(nomes))
    assert set(nomes) == INDEPENDENTES
    for x in ind:
        assert set(x) == {'parametro', 'motivo'}, x['parametro']
        assert isinstance(x['motivo'], str) and x['motivo'].strip(), x['parametro']


# ─── §4: footprint_padding explícito ─────────────────────────────────────────

def test_padding_declarado_nos_DOIS_costmaps_do_robo3():
    nav2, _ = _montado(_p3())
    for c in COSTMAPS:
        assert nav2[c][c]['ros__parameters']['footprint_padding'] == PADDING_D2, c
    fp = _le(PERFIL3)['footprint_padding']
    assert fp['classe'] == 'c' and 'D2' in fp['origem']
    assert fp['valor'] == PADDING_D2


# ─── §5: reflexo e path_follower com valores próprios ────────────────────────

def test_reescritas_exatas_do_reflexo():
    assert set(_p3()['collision_monitor_rewrites']) == {
        CM_RAIZ + ('PolygonApproach', 'points'),
        CM_RAIZ + ('PolygonApproach', 'time_before_collision'),
        CM_RAIZ + ('PolygonStop', 'points')}


def test_polygon_approach_robo3():
    _, cm = _montado(_p3())
    ap = _cm(cm)['PolygonApproach']
    assert isinstance(ap['points'], str)
    _mesmos_vertices(ap['points'],
                     [[0.1125, 0.22], [0.1125, -0.22], [-0.3213, -0.22], [-0.3213, 0.22]])
    assert ap['time_before_collision'] == 0.75


def test_polygon_stop_robo3():
    _, cm = _montado(_p3())
    st = _cm(cm)['PolygonStop']
    assert isinstance(st['points'], str)
    _mesmos_vertices(st['points'],
                     [[0.2155, 0.24], [0.2155, -0.24], [-0.3413, -0.24], [-0.3413, 0.24]])


def test_path_follower_robo3():
    pf = _p3()['path_follower']
    assert set(pf) == {'passagem_meia_largura', 'passagem_margem', 're_largura',
                       're_recuo_para_choque', 'avanco_para_choque',
                       'desencalhe_pivo_folga'}
    assert pf['passagem_meia_largura'] == pytest.approx(0.190)
    assert pf['passagem_margem'] == pytest.approx(0.03)
    assert pf['re_largura'] == pytest.approx(0.480)
    assert pf['re_recuo_para_choque'] == pytest.approx(0.2913)
    assert pf['avanco_para_choque'] == pytest.approx(0.0825)
    assert pf['desencalhe_pivo_folga'] == pytest.approx(0.3325)
    assert all(type(v) is float for v in pf.values()), \
        'parâmetro double do ROS recebendo int derruba o nó na subida'


def test_sobreposicao_so_usa_parametro_que_o_no_declara():
    """Parâmetro não declarado seria IGNORADO pelo nó, em silêncio."""
    assert set(_p3()['path_follower']) <= _declarados_do_seguidor()
