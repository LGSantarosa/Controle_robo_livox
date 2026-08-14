"""A geometria não pode divergir entre os arquivos que a repetem.

Este teste existe por causa de 2026-07-29. A bitola aparecia em quatro arquivos
e estava com valores DIFERENTES: 0,20 no simulador e 0,32 no robô, nenhum dos
dois medido, e os dois errados em sentidos opostos (a trena disse 0,270).
Sintonizar o rumo na bancada e levar para o robô teria errado duas vezes, em
direções contrárias — a bancada pareceria boa e o robô pioraria.

Não é zelo de estilo: é o defeito mais caro que este projeto teve, e ele não dá
sintoma. Um número copiado que envelhece sozinho não quebra nada visivelmente;
ele só faz a bancada medir uma máquina que não existe.
"""
import math
import os
import re

import pytest

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
PRODUCAO = os.path.join(RAIZ, 'ros2_packages', 'robot_motion', 'config', 'nav2.yaml')
BANCADA = os.path.join(RAIZ, 'ros2_packages', 'robot_planning', 'config',
                       'bancada_planner.yaml')


def valor(caminho, chave):
    """Primeiro valor numérico da chave, ignorando comentários.

    Lido por texto e não por parser de YAML de propósito: os comentários destes
    arquivos são metade do valor deles, e um teste que exigisse reserializar
    convidaria alguém a jogá-los fora.
    """
    with open(caminho) as f:
        for linha in f:
            corte = linha.split('#')[0]
            m = re.search(rf'^\s*{re.escape(chave)}:\s*([-\d.]+)\s*$', corte)
            if m:
                return float(m.group(1))
    raise AssertionError(f'{chave} não encontrada em {caminho}')


def texto(caminho, chave):
    with open(caminho) as f:
        for linha in f:
            corte = linha.split('#')[0]
            m = re.search(rf'^\s*{re.escape(chave)}:\s*"?([A-Z_]+)"?\s*$', corte)
            if m:
                return m.group(1)
    raise AssertionError(f'{chave} não encontrada em {caminho}')


# `inflation_radius` NÃO está aqui, e a ausência é deliberada. Ele é ESCOLHA,
# não medida: a bancada julgou os planners com 0,45 (é o número por trás dos 48
# planos da decisão 008) e a produção roda com 0,30, porque 0,45 fazia o robô
# travar DENTRO da porta de 0,90 m — o replanejamento falha com "Start occupied"
# quando o vão inteiro está inflado. Divergir aqui é a decisão; divergir nos de
# baixo é o defeito da bitola.
# ⚠️ `robot_radius` SAIU desta lista em 14-08 (decisão 032), e a saída é a
# própria descoberta: na produção ele DEIXOU de ser "geometria da máquina" e
# virou modelagem. O Theta* não conhece contorno — bloqueia célula com custo
# > 252 — então o raio do robô entra no plano só via
# `min(inflation_radius, raio inscrito)`, e o valor tem de servir a três donos
# ao mesmo tempo: o corpo, o reflexo e a porta. A bancada continua com o
# circunscrito medido, que é o que ela quer comparar.
# O que substitui a igualdade é `test_o_raio_do_planejador_fica_na_faixa_util`.
@pytest.mark.parametrize('chave', [
    'minimum_turning_radius',   # o argumento da decisão 008; refém da zona morta
])
def test_geometria_igual_na_bancada_e_na_producao(chave):
    p, b = valor(PRODUCAO, chave), valor(BANCADA, chave)
    assert p == b, (
        f'{chave} diverge: produção {p} × bancada {b}. Julgar o planner com um '
        'número e rodar o robô com outro é o erro da bitola de 29-07 de novo — '
        'a bancada parece boa e o robô piora.')


def test_o_modelo_de_movimento_e_o_mesmo_nos_dois():
    """Decisão 009: o plano não dá ré. Se a bancada julgar em Reeds-Shepp e o
    robô rodar em Dubins, o que foi aprovado não é o que anda."""
    p, b = texto(PRODUCAO, 'motion_model_for_search'), texto(
        BANCADA, 'motion_model_for_search')
    assert p == b == 'DUBIN', f'produção {p} × bancada {b}'


def test_o_raio_de_chegada_do_nav2_bate_com_o_do_seguidor():
    """Quem chega é o nosso seguidor; quem declara sucesso é o Nav2.

    Se a tolerância do `goal_checker` for menor que o `raio_chegada` do
    seguidor, o robô para e a árvore de comportamento continua pedindo — fica
    "navegando" parado até estourar o tempo. O padrão do seguidor está no
    `path_follower.py`; aqui trava-se o lado do Nav2.
    """
    from robot_motion import path_follower  # noqa: F401  (só para existir)
    assert valor(PRODUCAO, 'xy_goal_tolerance') == 0.25


# ------------------------------ o árbitro de comando (levantamento da 010)

def _nav2():
    import yaml
    return yaml.safe_load(open(PRODUCAO))


def _mux():
    import yaml
    p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     'config', 'twist_mux.yaml')
    return yaml.safe_load(open(p))['twist_mux']['ros__parameters']


def test_humano_vence_a_autonomia_sempre():
    """O princípio que o CLAUDE.md diz valer nos DOIS robôs. Se alguém inverter
    isto, o robô deixa de ser controlável por uma pessoa — e o sintoma só
    aparece no pior momento possível, com ele indo para a parede."""
    t = _mux()['topics']
    auto = t['autonomia']['priority']
    for nome in ('teclado', 'web'):
        assert t[nome]['priority'] > auto, f'{nome} tem de vencer a autonomia'


def test_o_mux_fala_stamped_como_o_resto_da_cadeia():
    """A cadeia do robô 2 é TwistStamped de ponta a ponta. O robô 1 usava
    Twist cru, e copiar aquele valor faria o DDS rejeitar por type hash: o mux
    publicaria e ninguém consumiria — falha silenciosa, a classe de defeito que
    mais custou tempo neste projeto (BO-3)."""
    assert _mux()['use_stamped'] is True


def test_toda_fonte_tem_timeout():
    """Fonte sem timeout é comando velho vivo para sempre. Mesma regra do
    `timeout_alvo` da movimentação e do `timeout_plano` do seguidor: é o
    timeout que faz o robô PARAR quando quem comandava morreu."""
    for nome, cfg in _mux()['topics'].items():
        assert cfg.get('timeout', 0) > 0, f'{nome} sem timeout'
        assert cfg['timeout'] <= 1.0, f'{nome} com timeout longo demais'


def test_a_cadeia_da_launch_bate_com_a_config():
    """Os nomes de tópico vivem em dois arquivos (YAML e launch) e se
    divergirem o robô não anda — ou pior, anda sem árbitro. Este teste é o que
    impede a divergência de virar depuração de campo."""
    p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     'launch', 'pilha.launch.py')
    launch = open(p).read()
    topicos = {c['topic'] for c in _mux()['topics'].values()}
    assert 'auto_vel' in topicos
    # Desde 05-08 o heading_controller alimenta o CRU: entre ele e o mux
    # entrou o reflexo de colisão. Quem entrega `auto_vel` ao mux é o
    # collision_monitor, pela config dele — não por remap na launch.
    assert "'/auto_vel_raw'" in launch, \
        'o heading_controller tem de alimentar a entrada do reflexo'
    assert 'nav2_collision_monitor' in launch, 'o reflexo tem de subir'
    assert "'/cmd_vel_out', '/compensador_rumo/cmd_vel'" in launch, \
        'a saída do mux tem de entrar no compensador'


# ---------------------------------- o reflexo de colisão (levantamento da 010)

def _cm():
    import yaml
    p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     'config', 'collision_monitor.yaml')
    return yaml.safe_load(open(p))['collision_monitor']['ros__parameters']


def test_o_reflexo_filtra_a_autonomia_e_nao_o_humano():
    """A entrada dele é a autonomia CRUA e a saída é o que vai para o mux. Se
    alguém ligar a saída direto no atuador, o humano deixa de furar o reflexo —
    e some a única forma de tirar um robô que o próprio reflexo prendeu contra
    uma parede."""
    cm = _cm()
    assert cm['cmd_vel_in_topic'] == 'auto_vel_raw'
    assert cm['cmd_vel_out_topic'] == 'auto_vel'
    mux_topicos = {c['topic'] for c in _mux()['topics'].values()}
    assert 'auto_vel' in mux_topicos, 'a saída do reflexo tem de entrar no mux'
    assert 'auto_vel_raw' not in mux_topicos, 'o mux não pode pegar o cru'


def test_o_reflexo_tem_as_DUAS_camadas(): 
    """Uma trava de contato e uma projeção que acompanha o arco (decisão 033).

    Com um polígono estático só, o reflexo é cego para direção — e como a 023
    tirou o pivô, toda curva deste robô é arco. Medido no Gazebo em 14-08: das
    amostras vetadas, |wz| mediano 0,51 rad/s contra 0,25 das que passaram, com
    folga parecida. Ele era vetado JUSTAMENTE quando girava.
    """
    cm = _cm()
    assert set(cm['polygons']) == {'PolygonStop', 'PolygonApproach'}
    assert cm['PolygonStop']['action_type'] == 'stop'
    assert cm['PolygonApproach']['action_type'] == 'approach', (
        'sem `approach` o reflexo volta a ser cego para direção e veta a '
        'própria manobra de contorno')


def test_a_projecao_cobre_a_parada_NO_COMANDO_TIPICO():
    """`approach` projeta pela velocidade COMANDADA, e neste robô o comando não
    diz a velocidade (pede 0,50, anda 0,298 — decisão 020: o comando escolhe o
    RAIO, a placa escolhe o módulo). Então o tempo é uma CALIBRAÇÃO, e ela é
    feita no comando típico de cruzeiro.

        parada medida + margem   0,268 + 0,085 = 0,353 m
        comando típico medido    0,47 m/s
        tempo                    0,353 / 0,47 = 0,75 s

    ⚠️ Pelo pior caso daria 1,74 s, e foi tentado: em 14-08 a projeção de 1,8 s
    valia 0,85 m no comando típico e VETOU uma curva que o robô fazia — 30 s
    parado numa quina. Projeção longa demais tem o mesmo defeito da caixa
    estática de 0,57: proíbe a manobra de contorno de um robô que só faz arco.
    """
    parada = 0.298 * 0.5 + 0.298 ** 2 / (2 * 0.373)
    V_TIPICO = 0.47
    t = _cm()['PolygonApproach']['time_before_collision']
    projecao = t * V_TIPICO
    assert projecao >= parada, (
        f'projeção de {projecao:.3f} m no comando típico não cobre a parada de '
        f'{parada:.3f} m')
    assert projecao <= parada + 0.15, (
        f'projeção de {projecao:.3f} m é longa demais: ela passa a vetar a '
        'curva do próprio robô (medido em 14-08 com 1,8 s)')


def test_a_projecao_usa_o_CORPO_e_nao_uma_caixa_de_frenagem():
    """No `approach` quem cria distância é o TEMPO. Se o polígono já for uma
    caixa de frenagem, a conta é feita duas vezes e ninguém consegue
    interpretar o número depois."""
    pontos = eval(_cm()['PolygonApproach']['points'])  # noqa: S307
    frente = max(x for x, _ in pontos)
    assert 0.2165 <= frente <= 0.2165 + 0.05, (
        f'frente {frente} não é o corpo (0,2165, medido em 29-07) mais uma '
        'margem pequena — projeção com caixa de frenagem conta a distância '
        'duas vezes')


def test_a_caixa_estatica_cobre_ao_menos_o_CORPO():
    """Ela encolheu de 0,57 para 0,30 quando a frenagem passou para a projeção,
    mas não pode encolher para dentro do robô: aí o reflexo não veria nem o que
    já está encostando no para-choque."""
    pontos = eval(_cm()['PolygonStop']['points'])  # noqa: S307
    frente = max(x for x, _ in pontos)
    assert frente >= 0.433 / 2, f'frente {frente} está DENTRO do corpo'
    assert frente <= 0.40, (
        f'frente {frente} grande demais: caixa reta larga com robô que só faz '
        'arco veta a própria curva (o impasse de 14-08)')


def test_a_lateral_do_reflexo_cabe_no_que_o_PLANEJADOR_permite():
    """Se o reflexo exigir mais folga lateral que o planejador, o Nav2 traça
    por um vão que o reflexo veta e o robô fica parado entre os dois — foi o
    que o dono previu e o que aconteceu em 14-08."""
    import yaml
    lateral = max(abs(y) for _, y in eval(_cm()['PolygonStop']['points']))  # noqa: S307
    raio = yaml.safe_load(open(PRODUCAO))['global_costmap']['global_costmap'][
        'ros__parameters']['robot_radius']
    assert lateral <= raio, (
        f'reflexo pede {lateral} de lado e o planejador só garante {raio}')


# 🔴 O TÓPICO QUE A PERCEPÇÃO CONSOME, e ele NÃO é o do driver (decisão 017).
#
# Medido no robô em 10-08: `/livox/lidar` sai do driver como
# `livox_ros_driver2/msg/CustomMsg`, e todo mundo que assinava `PointCloud2`
# recebia NADA — os dois costmaps com 0 células letais e o reflexo mudo, com o
# sensor a 9,96 Hz. No simulador o `gpu_lidar` publicava PointCloud2 no MESMO
# nome, então a decisão 014 passou lá e nunca foi exercitada.
NUVEM = '/livox/pontos'


def test_o_reflexo_ignora_o_chao_e_o_que_passa_por_cima():
    """Abaixo de ~0,05 m o próprio chão vira obstáculo (a nuvem erra até
    3,8 cm em rasância, medido 05-08) e o robô se trancaria sozinho. Acima de
    0,50 ele passa por baixo, e marcar isso o faria recusar portas."""
    fonte = _cm()[_cm()['observation_sources'][0]]
    assert fonte['min_height'] >= 0.08
    assert fonte['max_height'] <= 0.60
    assert fonte['topic'] == NUVEM


def test_o_reflexo_fala_stamped_como_o_resto_da_cadeia():
    assert _cm()['enable_stamped_cmd_vel'] is True


# --------------------------- a nuvem vista pelas duas camadas que a consomem

def _obstaculo(qual='local_costmap'):
    """A camada de obstáculo de um dos costmaps — a que faz o desvio existir."""
    import yaml
    d = yaml.safe_load(open(PRODUCAO))
    return d[qual][qual]['ros__parameters']['obstacle_layer']


AMBOS = pytest.mark.parametrize('qual', ['local_costmap', 'global_costmap'])


@AMBOS
def test_os_DOIS_costmaps_veem_a_nuvem(qual):
    """O local alimenta quem DIRIGE, o global alimenta quem PLANEJA, e faltar
    num dos dois tem sintomas diferentes.

    Só no global: o plano desvia mas o seguidor não reage a nada que apareça no
    caminho. Só no local (medido no simulador com o `pista_surpresa.sdf`): o
    plano sai RETO por cima do obstáculo — 2,20 m de caminho para 2,20 m de
    reta, folga de 0,035 m do centro da caixa — e o robô vai até lá e trava,
    porque o replanejamento a 1 Hz devolve para sempre o mesmo plano ruim.
    """
    plugins = _obstaculo(qual)  # levanta KeyError se a camada não existir
    assert plugins['plugin'] == 'nav2_costmap_2d::VoxelLayer'


def test_o_costmap_e_o_reflexo_recortam_a_MESMA_nuvem():
    """Dois consumidores da nuvem, e eles não podem discordar sobre o
    que é obstáculo.

    Se o costmap marcasse abaixo do reflexo, o planner desviaria de coisa que o
    reflexo ignora — e o robô ficaria dando volta em chão. Se marcasse acima, o
    reflexo pararia por algo que o planner nem sabe existir, e o robô travaria
    sem que ninguém replanejasse. O sintoma dos dois casos é "o robô está
    esquisito", que é o defeito mais caro de diagnosticar.

    Os números vêm da medida de 05-08 (a nuvem erra até 3,8 cm em incidência
    rasante) e da altura da caixa; o racional inteiro mora nos dois YAML.
    """
    fonte_cm = _cm()[_cm()['observation_sources'][0]]
    for qual in ('local_costmap', 'global_costmap'):
        fonte_cost = _obstaculo(qual)['livox']
        assert fonte_cost['topic'] == fonte_cm['topic'], qual
        assert fonte_cost['min_obstacle_height'] == fonte_cm['min_height'], qual
        assert fonte_cost['max_obstacle_height'] == fonte_cm['max_height'], qual


@AMBOS
def test_a_camada_de_obstaculo_e_VOXEL_por_causa_do_sensor_apontar_para_cima(qual):
    """O Mid-360 varre de −7° a +52°. Com `ObstacleLayer` (raytrace 2D) um raio
    que passa POR CIMA de uma caixa baixa limpa a célula da própria caixa, e o
    robô esquece o obstáculo que acabou de ver. O robô 1 fez essa troca ao
    contrário e estava certo LÁ: o LD06 dele é planar.

    Com o Livox a 0,42 m e feixe mais baixo a −7°, ele enxerga a altura
    `0,42 − 0,123·d`: uma caixa de 0,30 m é vista a 1–2 m e some quando o robô
    chega a 0,5 m dela — que é exatamente quando esquecer sai caro.
    """
    assert _obstaculo(qual)['plugin'] == 'nav2_costmap_2d::VoxelLayer'


def test_a_inflacao_e_a_ultima_camada():
    """Inflar antes de marcar infla um mapa sem o obstáculo que acabou de
    chegar — a célula fica ocupada e a auréola dela não, que é pior que não
    ter inflação nenhuma."""
    import yaml
    d = yaml.safe_load(open(PRODUCAO))
    for qual in ('global_costmap', 'local_costmap'):
        plugins = d[qual][qual]['ros__parameters']['plugins']
        assert plugins[-1] == 'inflation_layer', f'{qual}: {plugins}'


@AMBOS
def test_a_coluna_de_voxel_cobre_a_altura_do_sensor(qual):
    """`z_voxels × z_resolution` tem de passar dos 0,42 m do Livox (trena de
    05-08) — senão a nuvem do próprio plano do sensor cai fora da grade e a
    camada marca menos do que vê, sem dizer nada."""
    o = _obstaculo(qual)
    assert o['z_voxels'] * o['z_resolution'] > 0.42


# ------------------------------------- o perfil SEM MAPA (o do robô real)

SEM_MAPA = os.path.join(RAIZ, 'ros2_packages', 'robot_motion', 'config',
                        'nav2_sem_mapa.yaml')


def _sem_mapa(qual):
    import yaml
    return yaml.safe_load(open(SEM_MAPA))[qual][qual]['ros__parameters']


@AMBOS
def test_sem_mapa_nao_le_mapa_e_a_inflacao_segue_por_ultimo(qual):
    """`mapa:=nenhum` é o perfil do robô real: a localização é LIO (decisão 003)
    e não existe mapa do lugar onde ele anda. O mapa padrão é a planta da pista
    SIMULADA — parede onde não há nada, livre onde há parede.
    """
    plugins = _sem_mapa(qual)['plugins']
    assert 'static_layer' not in plugins, f'{qual} ainda lê mapa: {plugins}'
    assert 'obstacle_layer' in plugins, f'{qual} ficaria cego: {plugins}'
    assert plugins[-1] == 'inflation_layer', f'{qual}: {plugins}'


def test_o_perfil_sem_mapa_e_OVERLAY_e_nao_um_segundo_nav2():
    """Ele só pode sobrescrever o que MUDA sem mapa.

    Redefinir geometria aqui recria o defeito da bitola de 29-07 num lugar novo:
    dois arquivos com o mesmo número, um deles envelhecendo sozinho, e o robô
    rodando com um valor diferente do que a bancada julgou. O `nav2.yaml` é a
    fonte única — este arquivo é a diferença.
    """
    texto_overlay = open(SEM_MAPA).read()
    for proibida in ('robot_radius', 'inflation_radius', 'cost_scaling_factor',
                     'minimum_turning_radius', 'motion_model_for_search',
                     'min_obstacle_height', 'max_obstacle_height'):
        # ignora comentários: o racional PODE citar os números
        for linha in texto_overlay.splitlines():
            corte = linha.split('#')[0]
            assert f'{proibida}:' not in corte, (
                f'{proibida} redefinida no overlay — ela mora no nav2.yaml')


def test_sem_mapa_o_global_vira_janela_que_anda_com_o_robo():
    """Sem `static_layer` e sem janela rolante, o costmap global fica fixo no
    tamanho do mapa que não existe mais: o robô sairia dele e passaria a
    planejar contra uma grade vazia parada na origem."""
    g = _sem_mapa('global_costmap')
    assert g['rolling_window'] is True
    # Maior que o local (4 m): o global escolhe ROTA, e rota se decide com o
    # que está longe.
    assert g['width'] > 4 and g['height'] > 4


@AMBOS
def test_o_alarme_de_nuvem_velha_tem_folga_MEDIDA(qual):
    """`expected_update_rate` apertado desliga a percepção com um aviso amarelo
    por sintoma: buffer vencido deixa a camada não-current, e costmap
    não-current para de atualizar.

    0,30 s (10 Hz + 3 quadros) foi tentado e NÃO sobreviveu: o log encheu de
    "has not been updated for 0.43 seconds" com a nuvem chegando certinha a
    9,7 Hz e carimbo de 100,0 ms sem cauda. O atraso é do consumidor — 20 000
    pontos por quadro para transportar e transformar.
    """
    assert _obstaculo(qual)['livox']['expected_update_rate'] >= 0.5


# ------------- o ff do dia tem de CHEGAR ao compensador (decisão 013)
#
# O dono escolheu em 07-08 o caminho 3: medir a curvatura crua no começo de
# cada sessão e passar por parâmetro. A escolha ficou registrada e a plumbing
# não existia — a launch subia o compensador só com `use_sim_time` e
# `segura_rumo`, nenhum YAML carrega o número, e sobrava `ros2 param set`, que
# o roteiro lista como armadilha (não chega no nó; matar e subir). Ou seja: a
# medida do dia não tinha para onde ir.

def _launch_ast():
    import ast
    p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     'launch', 'pilha.launch.py')
    return ast.parse(open(p).read()), open(p).read()


def _nos_compensador():
    """Os `Node(...)` do compensador na pilha — são DOIS (sim e robô)."""
    import ast
    arvore, _ = _launch_ast()
    achados = []
    for no in ast.walk(arvore):
        if not (isinstance(no, ast.Call)
                and getattr(no.func, 'id', None) == 'Node'):
            continue
        kw = {k.arg: k.value for k in no.keywords}
        exe = kw.get('executable')
        if isinstance(exe, ast.Constant) and exe.value == 'compensador_rumo':
            achados.append(kw)
    return achados


@pytest.mark.parametrize('arg', ['curv_frente', 'curv_re', 'curv_medido_em'])
def test_a_launch_declara_o_ff_do_dia(arg):
    _, texto = _launch_ast()
    assert f"DeclareLaunchArgument(\n            '{arg}'" in texto \
        or f"'{arg}'" in texto, f'{arg} não é argumento da pilha'


def test_OS_DOIS_compensadores_recebem_o_ff():
    """Sim e robô. Se só um receber, a sessão de bancada mede uma coisa e o
    robô roda outra — a divergência silenciosa que este arquivo existe para
    impedir."""
    import ast
    nos = _nos_compensador()
    assert len(nos) == 2, f'esperava 2 compensadores na pilha, achei {len(nos)}'
    for kw in nos:
        fonte = ast.dump(kw['parameters'])
        assert "'curv'" in fonte or "id='curv'" in fonte, \
            'este compensador sobe sem o ff do dia'


def test_o_ff_vai_TIPADO_para_o_no():
    """Argumento de launch chega como TEXTO e o nó declarou `curv_frente` como
    double. Passar a substituição crua derruba o compensador na subida com
    "parameter type mismatch" — e compensador que não sobe é o robô arcando
    0,82 1/m com a pilha inteira de pé."""
    _, texto = _launch_ast()
    assert texto.count('value_type=float') >= 2, \
        'curv_frente e curv_re têm de ir como float'
    assert 'value_type=str' in texto, 'curv_medido_em é texto'


def test_o_default_da_launch_e_o_default_do_no():
    """Default duplicado é default que deriva (o caso da bitola, 29-07). Quem
    não passa nada tem de subir exatamente como subia antes destes argumentos
    existirem."""
    import ast
    fonte_no = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'robot_motion', 'compensador_rumo.py')
    arvore = ast.parse(open(fonte_no).read())
    par = {}
    for no in ast.walk(arvore):
        if (isinstance(no, ast.Call)
                and getattr(no.func, 'attr', None) == 'declare_parameters'):
            par = {ast.literal_eval(t.elts[0]): ast.literal_eval(t.elts[1])
                   for t in no.args[1].elts}
    _, texto = _launch_ast()
    for chave in ('curv_frente', 'curv_re'):
        assert f"default_value='{par[chave]}'" in texto, \
            f'{chave}: launch e nó divergiram ({par[chave]} não está na launch)'
    assert f"default_value='{par['curv_medido_em']}'" in texto


# ------------------------------------- o contrato da nuvem entre os DOIS mundos
#
# Este bloco existe porque o defeito de 10-08 passou por três semanas e nenhuma
# das dezenas de testes de config o pegou: eles conferiam que os consumidores
# concordavam ENTRE SI, e concordavam — todos liam `/livox/lidar`. O que
# faltava era conferir que alguém PUBLICA aquilo no formato que eles leem.

def _sim_launch():
    return open(os.path.join(RAIZ, 'ros2_packages', 'robot_base', 'launch',
                             'sim.launch.py')).read()


def _localizacao_launch():
    return open(os.path.join(RAIZ, 'ros2_packages', 'robot_base', 'launch',
                             'localizacao.launch.py')).read()


def test_o_simulador_publica_no_topico_que_a_percepcao_LE():
    """No Gazebo quem entrega a nuvem é a ponte, e o destino do remap tem de
    ser o tópico dos consumidores. Antes de 10-08 ele apontava para
    `/livox/lidar` "para ser igual ao robô" — e no robô aquele nome carrega
    CustomMsg, então a igualdade era só de nome."""
    texto = _sim_launch()
    assert f"'{NUVEM}')" in texto, (
        f'a ponte do Gazebo não remapeia para {NUVEM} — no simulador a '
        f'percepção fica sem nuvem')


def test_o_robo_sobe_a_ponte_que_converte_a_nuvem():
    """No robô quem entrega `PointCloud2` é o `nuvem_pontos`. Sem ele no
    launch, o sintoma é o de 10-08: sensor a 9,96 Hz, costmaps com 0 células
    letais e reflexo mudo — tudo com cara de config errada de altura."""
    assert 'nuvem_pontos' in _localizacao_launch(), (
        'a localização do robô não sobe o conversor: a percepção fica cega')


def test_ninguem_consome_o_topico_CRU_do_driver():
    """`/livox/lidar` é do FAST-LIO (CustomMsg no robô, PointCloud2 no
    simulador) — tipo que depende do mundo não serve de contrato. Consumidor
    que voltar a apontar para lá volta a receber nada no robô."""
    import yaml
    d = yaml.safe_load(open(PRODUCAO))
    for qual in ('local_costmap', 'global_costmap'):
        fonte = d[qual][qual]['ros__parameters']['obstacle_layer']['livox']
        assert fonte['topic'] == NUVEM, qual
    assert _cm()[_cm()['observation_sources'][0]]['topic'] == NUVEM


# ---------------------------------------------------------------------------
# OS DEFAULTS DA PILHA TÊM DE SER SEGUROS NO ROBÔ REAL (11-08)
#
# A decisão 015 resolveu o mapa fantasma criando `mapa:=nenhum` — e deixou a
# escolha como algo que o operador digita. Config que precisa ser digitada é
# config que vai ser esquecida, e o preço é silencioso: `StaticLayer` com uma
# sala de 12 × 8 m que não existe, misturada desde a 014 com marcação real do
# Livox. O robô recusa caminho livre e o sintoma não aponta para o mapa.
#
# A regra que estes testes travam: **o caso perigoso exige intenção, o seguro é
# o default.**
# ---------------------------------------------------------------------------

def _default_declarado(texto, arg):
    """O `default_value=` do DeclareLaunchArgument de `arg`, como texto.

    Ancorado no `DeclareLaunchArgument(` e não na primeira aparição do nome: o
    cabeçalho desta launch cita `mapa` e `rviz` em prosa muito antes de
    declará-los.
    """
    m = re.search(r"DeclareLaunchArgument\(\s*'" + re.escape(arg) + r"'",
                  texto)
    assert m, f'`{arg}` não é argumento declarado da pilha'
    trecho = texto[m.end():m.end() + 500]
    j = trecho.index('default_value=')
    corte = trecho.index('description=') if 'description=' in trecho else 200
    return trecho[j:corte]


@pytest.mark.parametrize('arg,perigoso', [
    ('mapa', 'nenhum'),     # sem mapa é o seguro no robô
    ('rviz', 'false'),      # o NUC não tem tela
])
def test_o_default_do_robo_real_e_o_seguro(arg, perigoso):
    _, texto = _launch_ast()
    d = _default_declarado(texto, arg)
    assert 'LaunchConfiguration' in d and "'sim'" in d, (
        f'o default de `{arg}` tem de depender do `sim` — no robô real ele '
        f'precisa valer {perigoso!r} sem ninguém digitar nada')
    assert perigoso in d, f'o ramo do robô real de `{arg}` tem de ser {perigoso!r}'


def _resolve_default(arg, sim):
    """Resolve o `default_value=` QUE ESTÁ NA LAUNCH, com o `sim` dado.

    Avalia a expressão escrita no arquivo em vez de remontá-la aqui — foi a
    primeira versão deste teste que remontava, e ela **passou** com o
    condicional invertido na launch. Teste que reconstrói o alvo não testa o
    alvo.
    """
    import ast

    from launch import LaunchContext
    from launch.substitutions import LaunchConfiguration, PythonExpression

    arvore, texto = _launch_ast()
    fonte = None
    for no in ast.walk(arvore):
        if not (isinstance(no, ast.Call)
                and getattr(no.func, 'id', None) == 'DeclareLaunchArgument'):
            continue
        if not (no.args and isinstance(no.args[0], ast.Constant)
                and no.args[0].value == arg):
            continue
        for kw in no.keywords:
            if kw.arg == 'default_value':
                fonte = ast.get_source_segment(texto, kw.value)
    assert fonte, f'`{arg}` não tem default_value declarado'

    ambiente = {
        'PythonExpression': PythonExpression,
        'LaunchConfiguration': LaunchConfiguration,
        # Só o nome importa; o valor é opaco para o sentido do condicional.
        'MAPA_PADRAO': '/QUALQUER/mapa.yaml',
    }
    expr = eval(fonte, ambiente)  # noqa: S307 - fonte é o nosso próprio repo
    ctx = LaunchContext()
    ctx.launch_configurations['sim'] = sim
    if isinstance(expr, str):
        return expr
    return expr.perform(ctx)


@pytest.mark.parametrize('arg,no_robo,no_sim', [
    ('mapa', 'nenhum', '/QUALQUER/mapa.yaml'),
    ('rviz', 'false', 'true'),
])
def test_o_default_RESOLVE_para_o_seguro_quando_sim_e_false(arg, no_robo, no_sim):
    """Não basta o texto citar `sim`: a substituição tem de RESOLVER, e no
    sentido certo. É este teste que pega o condicional escrito ao contrário —
    o modo de falha em que o robô real ganha o mapa da pista simulada e o
    simulador roda sem mapa nenhum, os dois em silêncio."""
    pytest.importorskip('launch')
    assert _resolve_default(arg, 'false') == no_robo, (
        f'com sim:=false (robô real) o default de `{arg}` tem de ser {no_robo!r}')
    assert _resolve_default(arg, 'true') == no_sim, (
        f'com sim:=true o default de `{arg}` tem de ser {no_sim!r}')


# ---------------------------------------------------------------------------
# A ZONA MORTA MEDIDA, E O QUE ELA SEGURA (decisão 020)
#
# `movimentacao.yaml` passou doze dias dizendo "NENHUM DESTES NÚMEROS FOI
# MEDIDO NESTE ROBÔ AINDA" depois de 07-31 ter medido. O chute que ficou (0,15
# contra 0,0178 reais) não ficava parado no arquivo: proibia o pivô por
# aritmética e ABRIA AS CURVAS, porque a saída da lei é acelerar — e acelerar
# muda a razão entre as rodas, que neste atuador é a única coisa obedecida.
#
# Os quatro testes abaixo travam as pontas que deixaram isso passar.
# ---------------------------------------------------------------------------

MOVIMENTACAO = os.path.join(RAIZ, 'ros2_packages', 'robot_motion', 'config',
                            'movimentacao.yaml')
SEGUIDOR = os.path.join(RAIZ, 'ros2_packages', 'robot_motion', 'robot_motion',
                        'path_follower.py')
URDF = os.path.join(RAIZ, 'ros2_packages', 'robot_base', 'description',
                    'robo2.urdf.xacro')


def _perfil_do_robo():
    """Os números do robô real, LIDOS do arquivo que o robô carrega."""
    return {c: valor(MOVIMENTACAO, c)
            for c in ('zona_morta', 'bitola', 'margem_piso', 'wz_max')}


def _default_do_seguidor(nome):
    """O default de um parâmetro do `path_follower`, lido por AST.

    Lê o ARQUIVO em vez de importar e instanciar o nó (que exigiria rclpy) e em
    vez de repetir o número aqui. A lição é a da 019: teste que reconstrói o
    alvo não testa o alvo.
    """
    import ast

    with open(SEGUIDOR) as f:
        arvore = ast.parse(f.read())
    for no in ast.walk(arvore):
        if not isinstance(no, ast.Tuple) or len(no.elts) != 2:
            continue
        chave, val = no.elts
        if (isinstance(chave, ast.Constant) and chave.value == nome
                and isinstance(val, ast.Constant)):
            return val.value
    raise AssertionError(f'`{nome}` não é parâmetro declarado do seguidor')


def test_o_piso_do_seguidor_sai_da_movimentacao():
    """O `v_piso` do seguidor é a fórmula da movimentação, não um número solto.

    O comentário no `path_follower.py` já mandava "TEM QUE BATER com o que a
    movimentação calcula" — e era só comentário. Os dois arquivos andaram
    separados por doze dias (0,335 contra os 0,203 da conta) porque nada
    conferia. O `v_piso` governa o raio de chegada mínimo: divergir aqui é o
    robô aceitando um raio que a máquina não fecha, e orbitando o ponto.
    """
    p = _perfil_do_robo()
    esperado = p['zona_morta'] + p['wz_max'] * p['bitola'] / 2.0 + p['margem_piso']
    v_piso = _default_do_seguidor('v_piso')
    assert abs(v_piso - esperado) < 5e-4, (
        f'v_piso={v_piso} no seguidor, mas a movimentação calcula '
        f'{esperado:.4f} (zona_morta {p["zona_morta"]} + wz_max·bitola/2 + '
        f'margem {p["margem_piso"]}). Um dos dois arquivos envelheceu.')


def test_o_pivo_existe_com_os_numeros_do_robo():
    """A máquina pivota — o YAML não pode dizer que não.

    Medido em 07-31: `wz=±0,30` por 1 s deu +147,7° e −150,0° de giro total. Com
    `zona_morta=0,15` a lei exigia 1,48 rad/s contra teto de 1,00 e declarava o
    pivô IMPOSSÍVEL na subida do nó — proibição que chegou a virar item de
    bancada no ESTADO_PROJETO, trabalho agendado para um defeito inexistente.
    """
    from robot_motion.lei_de_rumo import pivo_disponivel, wz_minimo_parado
    p = _perfil_do_robo()
    assert pivo_disponivel(p['zona_morta'], p['bitola'], p['margem_piso'],
                           p['wz_max']), (
        'com estes números o nó declara "pivô INDISPONÍVEL" na subida e a '
        'saída de pivô da lei fica inalcançável — contra uma máquina que gira '
        '147° com wz=0,30')
    assert wz_minimo_parado(p['zona_morta'], p['bitola']) < p['wz_max']


def test_a_curva_pedida_e_a_curva_ENTREGUE():
    """O par medido em 07-31 tem de sair da lei com o raio que a máquina fez.

    `v=0,10 · wz=0,30` foi ao chão e entregou raio 0,333 m contra 0,333
    comandado: este atuador obedece ao RAIO (a compensação do driver escala as
    duas rodas juntas, preservando a razão entre elas e destruindo o módulo).

    Com `zona_morta=0,15` a lei interceptava esse pedido e mandava 0,802 m para
    a máquina — 2,4x mais aberto. Em porta e corredor, é a diferença entre
    passar e raspar.
    """
    from robot_motion.lei_de_rumo import ajusta_para_zona_morta
    p = _perfil_do_robo()
    v, wz = ajusta_para_zona_morta(0.10, 0.30, p['zona_morta'], p['bitola'],
                                   p['margem_piso'], v_teto=0.5)
    pedido, entregue = 0.10 / 0.30, v / wz
    assert entregue < 1.10 * pedido, (
        f'a lei abre a curva: pedido {pedido:.3f} m, entregue {entregue:.3f} m '
        f'({entregue / pedido:.2f}x). Acelerar para escapar da banda muda a '
        'razão entre as rodas, e a razão é o raio.')


def test_a_zona_morta_supoe_a_compensacao_do_driver_LIGADA():
    """O par `zona_morta` × `deadband_enable` é um acoplamento entre arquivos.

    0,0178 m/s só vale porque o driver escala as rodas até a maior vencer o
    limiar do firmware — o limiar que sobra é o `mx > 1.0` dele (1 RPM ·
    roda_raio 0,080 = 0,0084 m/s). Com `deadband_enable=false`, que é o que o
    banco precisa fazer para caracterizar o atuador, a zona morta de verdade
    (0,25–0,50 m/s, MODELO_ROBO2 §2) volta e este piso fica perigosamente
    baixo: o robô não sai do lugar e o sintoma é o BO-3 clássico.

    Quem desligar a compensação derruba este teste, e é essa a intenção.
    """
    with open(URDF) as f:
        texto_urdf = f.read()
    real = texto_urdf.split('<xacro:unless value="$(arg sim)">')[-1]
    m = re.search(r'<param name="deadband_enable">\s*(\w+)\s*</param>', real)
    assert m, 'o bloco de hardware do robô real não declara `deadband_enable`'
    assert m.group(1) == 'true', (
        'a compensação do driver está DESLIGADA no robô real, e '
        f'`zona_morta={valor(MOVIMENTACAO, "zona_morta")}` no '
        'movimentacao.yaml supõe ela ligada. Sem ela a zona morta real é '
        '0,25–0,50 m/s (MODELO_ROBO2 §2) e o piso tem de subir junto.')


# ---------------------------------------------------------------------------
# A LOCALIZAÇÃO CONTRA O MAPA (decisão 022)
#
# O AMCL e o `tf_map_odom` publicam a MESMA transformada (`map → odom`). Subir
# os dois deixa a TF disputada entre um publicador que diz "identidade" e outro
# que diz a verdade: a pose pisca a cada consulta, e o sintoma — robô em
# ziguezague no RViz, plano que salta — não aponta para TF nenhuma.
# ---------------------------------------------------------------------------

AMCL = os.path.join(RAIZ, 'ros2_packages', 'robot_motion', 'config',
                    'localizacao_amcl.yaml')
SCAN_2D = os.path.join(RAIZ, 'ros2_packages', 'robot_base', 'config',
                       'scan_2d.yaml')


def _no_da_pilha(nome):
    """O `Node(...)` da pilha cujo `name=` é `nome`, como nó de AST.

    Lê o alvo em vez de reconstruí-lo — a lição que este arquivo já aprendeu
    três vezes (017, 019, 021).
    """
    import ast
    arvore, _ = _launch_ast()
    for no in ast.walk(arvore):
        if not (isinstance(no, ast.Call)
                and getattr(no.func, 'id', None) == 'Node'):
            continue
        for kw in no.keywords:
            if (kw.arg == 'name' and isinstance(kw.value, ast.Constant)
                    and kw.value.value == nome):
                return no
    raise AssertionError(f'a pilha não declara um nó chamado {nome!r}')


def _condicao(no_ast):
    import ast
    for kw in no_ast.keywords:
        if kw.arg == 'condition':
            return ast.unparse(kw.value)
    return None


def test_o_AMCL_e_a_TF_FIXA_nunca_sobem_juntos():
    """Duas fontes para `map → odom` é pose piscando entre elas.

    A trava é de condição: o publicador fixo só sobe quando a localização NÃO
    é amcl, e o amcl só sobe quando ela é (e há mapa).
    """
    fixa = _condicao(_no_da_pilha('tf_map_odom'))
    amcl = _condicao(_no_da_pilha('amcl'))
    assert fixa is not None, (
        'o `tf_map_odom` sobe SEMPRE — com o AMCL de pé, os dois publicam '
        '`map → odom` e a pose pisca')
    assert 'tf_fixa' in fixa, f'condição inesperada no tf_map_odom: {fixa}'
    assert amcl is not None and 'amcl' in amcl, (
        f'o `amcl` precisa de condição própria; achei {amcl}')


def test_o_amcl_entra_na_lista_do_lifecycle():
    """Nó de ciclo de vida fora da lista NÃO ativa, e não avisa.

    Ele sobe, fica em `unconfigured`, não publica TF — e como o `tf_map_odom`
    também não subiu (exclusão mútua), a árvore fica partida e o Nav2 inteiro
    não ativa. Silêncio total, que é o modo de falha mais caro deste projeto.
    """
    _, texto = _launch_ast()
    m = re.search(r'servidores_com_amcl\s*=\s*(.+?)\n\n', texto, re.S)
    assert m, 'a pilha não monta uma lista de servidores com amcl'
    assert "'amcl'" in m.group(1), (
        'o `amcl` ficou fora da lista do lifecycle_manager: sobe e nunca ativa')
    assert "'map_server'" in m.group(1), (
        'sem `map_server` na lista o AMCL espera um mapa que nunca vem')


@pytest.mark.parametrize('chave_amcl,chave_scan', [
    ('laser_min_range', 'range_min'),
    ('laser_max_range', 'range_max'),
])
def test_o_alcance_do_amcl_bate_com_o_do_SCAN(chave_amcl, chave_scan):
    """O AMCL não pode esperar um alcance que a fatia 2D não entrega.

    Pedir mais longe que o scan corta faz ele tratar "não medi" como "medi
    longe" — feixe fantasma de 20 m atravessando parede. E os dois números
    moram em pacotes diferentes (`robot_motion` e `robot_base`), que é
    exatamente a distância em que números copiados envelhecem.
    """
    a, s = valor(AMCL, chave_amcl), valor(SCAN_2D, chave_scan)
    assert a == s, (
        f'{chave_amcl}={a} no AMCL contra {chave_scan}={s} na fatia 2D')


def test_o_amcl_sabe_que_o_robo_e_DIFERENCIAL():
    """Modelo omnidirecional espalha partícula para o lado — movimento que
    este robô (2 rodas + boba) não faz. Partícula gasta em pose impossível é
    partícula a menos onde ele de fato pode estar."""
    with open(AMCL) as f:
        texto = f.read()
    assert 'DifferentialMotionModel' in texto


def test_a_pose_inicial_do_amcl_vem_por_PARAMETRO():
    """🔴 O NUC NÃO TEM TELA.

    O jeito normal de dizer ao AMCL onde o robô está é clicar "2D Pose
    Estimate" no RViz — que no robô real não existe (desde a 019 o rviz nem
    sobe por padrão lá). Sem pose inicial o filtro nasce espalhado pelo mapa
    inteiro e converge para qualquer lugar, ou para lugar nenhum.
    """
    _, texto = _launch_ast()
    for arg in ('pose_x', 'pose_y', 'pose_yaw'):
        # `\s*` porque a declaração pode quebrar a linha — o teste não pode
        # depender da formatação do arquivo que ele julga.
        assert re.search(r"DeclareLaunchArgument\(\s*'" + arg + r"'", texto), (
            f'`{arg}` não é argumento da pilha — sem tela no NUC, a pose '
            'inicial só pode chegar por parâmetro')
    with open(AMCL) as f:
        assert 'set_initial_pose: true' in f.read()


@pytest.mark.parametrize('mapa,loc,vale', [
    ('/QUALQUER/mapa.yaml', 'amcl', True),
    ('/QUALQUER/mapa.yaml', 'fixa', True),
    ('nenhum', 'fixa', True),
    ('nenhum', 'amcl', False),      # AMCL sem mapa não é pilha degradada
    ('nenhum', 'AMCL', False),      # valor inexistente
])
def test_a_combinacao_sem_sentido_morre_na_SUBIDA(mapa, loc, vale):
    """`amcl` sem mapa não é uma pilha pior: é uma pilha que não funciona.

    Sem `map_server` o AMCL espera um mapa que nunca vem e não ativa; com o
    `tf_map_odom` fora (exclusão mútua), a árvore fica partida e o bringup
    aborta. Melhor morrer na subida, com a frase certa, do que trinta segundos
    depois com cara de bug de código.

    Esta função é Python puro — aqui ela roda DE VERDADE, não por leitura.
    """
    pytest.importorskip('launch')
    from launch import LaunchContext

    import ast
    _, texto = _launch_ast()
    arvore = ast.parse(texto)
    fonte = None
    for no in ast.walk(arvore):
        if (isinstance(no, ast.FunctionDef)
                and no.name == '_recusa_combinacao_sem_sentido'):
            fonte = ast.get_source_segment(texto, no)
    assert fonte, 'a pilha não tem a função de recusa'

    from launch.substitutions import LaunchConfiguration
    ambiente = {'LaunchConfiguration': LaunchConfiguration}
    exec(fonte, ambiente)                     # noqa: S102 - fonte do repo
    ctx = LaunchContext()
    ctx.launch_configurations.update({'mapa': mapa, 'localizacao': loc})

    if vale:
        assert ambiente['_recusa_combinacao_sem_sentido'](ctx) == []
    else:
        with pytest.raises(RuntimeError):
            ambiente['_recusa_combinacao_sem_sentido'](ctx)


def test_o_spawn_do_simulador_e_a_pose_do_amcl_sao_O_MESMO_argumento():
    """Nascer o robô num lugar e dizer ao AMCL que ele está em outro.

    Com dois números separados, o filtro "corrige" uma diferença que não existe
    no mundo e converge para a pose errada. O sintoma — mapa e nuvem
    desalinhados — é IDÊNTICO ao de uma fatia 2D mal ajustada (decisão 021).
    Dois defeitos com o mesmo rosto é o que faz perder o dia.
    """
    import ast
    arvore, texto = _launch_ast()
    for no in ast.walk(arvore):
        if not (isinstance(no, ast.Call)
                and getattr(no.func, 'id', None) == 'IncludeLaunchDescription'):
            continue
        trecho = ast.get_source_segment(texto, no) or ''
        if 'sim.launch.py' not in trecho:
            continue
        assert "'x': LaunchConfiguration('pose_x')" in trecho, (
            'o spawn do simulador não sai de `pose_x` — nascer o robô num '
            'lugar e informar outro ao AMCL dá pose convergindo errado, com '
            'cara de fatia 2D mal ajustada')
        assert "'y': LaunchConfiguration('pose_y')" in trecho
        return
    raise AssertionError('a pilha não inclui o sim.launch.py')


# ---------------------------------------------------------------------------
# O PIVÔ FORA DO CAMINHO — decisão 023 (12-08, 4ª leva)
#
# O `limiar_pivo` é o único portão entre o seguidor e a manobra bang-bang. Ele
# já valeu 15° e 45°, e as duas vezes o robô ficou girando no lugar sem sair.
# O que estes testes travam não é o número: é a AMARRA entre o número e o
# mecanismo. Enquanto a manobra não fechar contra a placa que retém, o portão
# tem de estar fechado — e no dia em que alguém consertar o mecanismo, é o
# teste do `lei_de_pivo` que abre, não este que se apaga.
# ---------------------------------------------------------------------------

MOVIMENTACAO_SIM = os.path.join(RAIZ, 'ros2_packages', 'robot_motion',
                                'config', 'movimentacao_sim.yaml')
CONTROLADOR = os.path.join(RAIZ, 'ros2_packages', 'robot_motion',
                           'robot_motion', 'heading_controller.py')


def _default_do_no(caminho, nome):
    """O default de um parâmetro declarado, lido por AST do arquivo do nó."""
    import ast

    with open(caminho) as f:
        arvore = ast.parse(f.read())
    for no in ast.walk(arvore):
        if not isinstance(no, ast.Tuple) or len(no.elts) != 2:
            continue
        chave, val = no.elts
        if (isinstance(chave, ast.Constant) and chave.value == nome
                and isinstance(val, ast.Constant)):
            return val.value
    raise AssertionError(f'`{nome}` não é parâmetro declarado em {caminho}')


def test_o_default_do_NO_mantem_o_pivo_desligado():
    """O perfil LIGA o pivô (decisão 036); o default do nó não.

    Regra da decisão 019: o default é o caso seguro. Quem sobe o
    `heading_controller` sem perfil — bancada, teste solto — não pode ganhar de
    brinde uma manobra que só é segura dentro de uma faixa de ângulo.
    """
    assert _default_do_no(CONTROLADOR, 'limiar_pivo') > math.pi


@pytest.mark.parametrize('arquivo', [MOVIMENTACAO_SIM, MOVIMENTACAO])
def test_o_pivo_so_dispara_para_erro_GRANDE(arquivo):
    """O gatilho tem de casar com o MENOR GOLPE que a manobra sabe dar.

    A regra tem duas formas, e a razão é o cabeçalho desta seção: *"no dia em
    que alguém consertar o mecanismo, é o teste do `lei_de_pivo` que abre"*.

    ANTES (decisão 023): a placa entrega um módulo só (2,204 rad/s) e segura a
    saída cheia 0,52 s depois do corte -> varredura de 93–101°, que NÃO depende
    do wz do corte. Um golpe de ~95° só é a manobra certa para erro da ordem de
    95°, daí a faixa de 70 a 100°.

    DEPOIS (14-08, 4ª leva): o `heading_controller` dava `return` dentro do
    pivô ANTES do bloco do freio de giro (037) — o pivô era o ÚNICO caminho da
    cadeia que girava sem freio, e por isso varreu 150–310° por pulso na
    corrida G. Com o `wz` do pivô caindo no freio, o golpe mínimo passa a sair
    do `pivo_a_dec` do perfil: `wz²/(2·a_dec)`.

    Perfil que declara `pivo_a_dec` está afirmando "meu pivô é freado" e cai na
    regra amarrada ao mecanismo; quem não declara continua na faixa velha.
    """
    WZ_MODULO = 2.204        # rad/s, o módulo único da placa (023)
    limiar = math.degrees(valor(arquivo, 'limiar_pivo'))
    try:
        a_dec = valor(arquivo, 'pivo_a_dec')
    except Exception:
        a_dec = None
    if not a_dec:
        assert 70.0 <= limiar <= 100.0, (
            f'limiar_pivo={limiar:.0f}° fora da faixa em que o quantum de ~95° '
            'é a manobra certa')
        return
    golpe_min = math.degrees(WZ_MODULO ** 2 / (2.0 * a_dec))
    assert limiar >= golpe_min, (
        f'limiar_pivo={limiar:.0f}° é MENOR que o golpe mínimo de '
        f'{golpe_min:.0f}° (pivo_a_dec={a_dec}): o pulso passa do alvo e '
        'ressuscita o ciclo-limite')
    assert limiar <= 4.0 * golpe_min, (
        f'limiar_pivo={limiar:.0f}° é {limiar / golpe_min:.1f}x o golpe mínimo '
        f'de {golpe_min:.0f}°: gatilho alto demais deixa o robô fazendo BALÃO '
        'onde o pivô já resolveria')


@pytest.mark.parametrize('arquivo', [MOVIMENTACAO_SIM, MOVIMENTACAO])
def test_o_pivo_da_UM_golpe_e_devolve(arquivo):
    """Mais de um pulso ressuscita o ciclo-limite de 12-08.

    Com `max_pulsos` alto a lei fica caçando o alvo com um martelo de 95°: as
    quatro corridas mediram giro total de 382° a 1321° para deslocamento de
    0,06 a 0,23 m, oscilando ±50–60° com período de ~5,5 s. Um pulso é "vira
    grosso e sai da frente"; o resto é do arco.
    """
    assert valor(arquivo, 'pivo_max_pulsos') == 1


def test_a_chegada_nao_espera_um_angulo_que_a_maquina_nao_fecha():
    """`aponta_no_fim` acompanha o pivô — ligá-lo sozinho pendura a chegada.

    A fase 2 pede `v=0` mais um ângulo. Sem o pivô quem atende é a lei
    contínua, e ela não arrasta (medido: `v`=0,000 de 2° a 150°) mas também
    não gira abaixo de 90° no perfil do simulador. Esperar ali é ficar parado
    em silêncio à espera de uma manobra que não vem — o BO-3 exato.
    """
    from test_configs_coerentes import _default_do_seguidor
    assert _default_do_seguidor('aponta_no_fim') is False, (
        'aponta_no_fim=True com o pivô fora do caminho pendura o seguidor na '
        'chegada. Os dois religam juntos — ver decisão 023.')


# ---------------------------------------------------------------------------
# A RECUPERAÇÃO ALCANÇÁVEL — decisão 024 (12-08, 4ª leva)
#
# O seguidor sempre teve uma recuperação (a ré da 009) e ela era inalcançável
# por CONSTRUÇÃO: a guarda de plano velho dava `return` antes da checagem de
# progresso, e o plano vence justamente quando o robô trava. Defeito de ORDEM,
# não de lógica — nenhum teste de valor pegaria. Estes leem a ordem.
# ---------------------------------------------------------------------------

def _passo_do_seguidor():
    """O corpo do `passo()` do seguidor, como AST."""
    import ast

    with open(SEGUIDOR) as f:
        arvore = ast.parse(f.read())
    for no in ast.walk(arvore):
        if isinstance(no, ast.FunctionDef) and no.name == 'passo':
            return no
    raise AssertionError('o seguidor não tem `passo()`')


def _linha_de(no_passo, agulha, arg=None):
    """Primeira linha em que `agulha(arg, ...)` é chamado dentro do `passo()`.

    ⚠️ `arg` não é luxo: `para()` é chamado três vezes no `passo()`, e a
    primeira é a da CHEGADA, lá no topo. Um teste de ordem que casasse com
    qualquer `para()` compararia contra a linha errada e passaria sempre — que
    é o defeito da 019 (teste que não testa o alvo) de novo.
    """
    import ast

    achadas = []
    for no in ast.walk(no_passo):
        if not isinstance(no, ast.Call):
            continue
        nome = getattr(no.func, 'attr', getattr(no.func, 'id', None))
        if nome != agulha:
            continue
        if arg is not None:
            if not no.args or not isinstance(no.args[0], ast.Constant):
                continue
            if arg not in str(no.args[0].value):
                continue
        achadas.append(no.lineno)
    if not achadas:
        alvo = agulha if arg is None else f'{agulha}({arg!r})'
        raise AssertionError(f'`{alvo}` não é chamado dentro do `passo()`')
    return min(achadas)


def _linha_do_despacho_da_re(no_passo):
    """Linha do `if self.estado == 're':` — o DESPACHO da máquina de estados.

    ⚠️ Não vale procurar por `passo_de_re`: ele é chamado em DOIS lugares (o
    despacho e o resgate dentro do ramo de plano velho), e casar com o
    primeiro deles fez a primeira versão deste teste sobreviver à mutação que
    ele existe para pegar. Testar a chamada errada é não testar.
    """
    import ast

    for no in ast.walk(no_passo):
        if not isinstance(no, ast.If):
            continue
        t = no.test
        if (isinstance(t, ast.Compare) and isinstance(t.ops[0], ast.Eq)
                and isinstance(t.comparators[0], ast.Constant)
                and t.comparators[0].value == 're'
                and getattr(t.left, 'attr', None) == 'estado'):
            return no.lineno
    raise AssertionError("o `passo()` não despacha o estado 're'")


def test_a_re_em_curso_sobrevive_ao_plano_velho():
    """Uma ré começada tem de continuar mesmo com o plano vencido.

    Ninguém replaneja para um robô emperrado, então o plano vence JUSTAMENTE
    enquanto ele recua. Com a guarda antes, ela abortava a manobra no primeiro
    ciclo — a recuperação durava 50 ms.
    """
    p = _passo_do_seguidor()
    assert _linha_do_despacho_da_re(p) < _linha_de(p, 'para', 'plano velho'), (
        "a guarda de plano velho vem antes do despacho de `estado == 're'` — "
        'uma ré em curso morre no primeiro ciclo dela. Ver decisão 024.')


def test_o_emperramento_e_avaliado_ANTES_de_desistir_por_plano_velho():
    """A checagem de progresso tem de alcançar o caso do plano vencido.

    Era aqui que a ré ficava inalcançável: `para('plano velho')` retornava
    antes de `progresso.atualiza`, e o robô parava para sempre. Medido na 4ª
    leva de 12-08: 87 s de CSV com a pose imóvel na mesma casa decimal, a
    2,49 m do objetivo.
    """
    p = _passo_do_seguidor()
    assert _linha_de(p, 'atualiza') < _linha_de(p, 'para', 'plano velho'), (
        'nenhuma checagem de progresso acontece antes de `para()` — a única '
        'recuperação do seguidor volta a ser inalcançável por construção. '
        'Ver decisão 024.')


def test_a_re_tem_teto_sem_plano_novo():
    """Recuar não ressuscita objetivo abortado — então a ré tem de parar.

    Sem teto, o robô atravessa a sala de ré em passos de 0,30 m, e isso PARECE
    recuperação. O teto existe para que o log diga a verdade: quem desistiu foi
    o objetivo, lá no `bt_navigator`.
    """
    teto = _default_do_seguidor('re_max_sem_plano')
    assert isinstance(teto, int) and 1 <= teto <= 3, (
        f're_max_sem_plano={teto} — sem teto pequeno a ré vira passeio de '
        'costas com cara de recuperação. Ver decisão 024.')
    assert _default_do_seguidor('re_habilitada') is True, (
        'a ré é a única recuperação que este robô tem desde que o pivô saiu '
        'do caminho (023). Desligá-la deixa o seguidor sem saída.')


# ---------------------------------------------------------------------------
# O FURO NO BLOQUEIO — decisão 025
#
# Um canal que passa por fora do reflexo é a coisa mais perigosa que existe
# nesta pilha. O que estes testes travam é o que o torna aceitável: ele perde
# para o humano, e quem publica nele mediu o vão antes.
# ---------------------------------------------------------------------------

def test_o_desencalhe_vence_a_autonomia_e_PERDE_para_o_humano():
    """A ordem inteira, e cada desigualdade tem motivo próprio.

    Acima da autonomia: senão o comando de desencalhe nunca chega à roda, e o
    canal não serve para nada. Abaixo do humano: um desencalhe automático que
    o operador não consegue interromper é pior que um robô parado — e este
    canal, por definição, dirige o robô SEM o freio de mão.
    """
    t = _mux()['topics']
    assert 'desencalhe' in t, (
        'sem canal de desencalhe o seguidor pede ré e o reflexo veta — '
        'medido em 12-08: 831 de 831 amostras zeradas. Ver decisão 025.')
    d = t['desencalhe']['priority']
    assert d > t['autonomia']['priority'], (
        'desencalhe abaixo da autonomia nunca chega à roda')
    for nome in ('teclado', 'web'):
        assert t[nome]['priority'] > d, (
            f'{nome} tem de vencer o desencalhe — quem manda é a pessoa')


def test_o_seguidor_publica_no_CANAL_que_o_mux_escuta():
    """O nome do tópico vive em dois arquivos. Divergir aqui é o furo virar
    um tópico que ninguém lê: o seguidor "recua", o log diz que recuou, e a
    roda não vê nada. Falha silenciosa — a classe de defeito mais cara deste
    projeto (BO-3)."""
    topico = _mux()['topics']['desencalhe']['topic']
    with open(SEGUIDOR) as f:
        fonte = f.read()
    assert f"'/{topico}'" in fonte, (
        f'o mux escuta `{topico}` e o seguidor não publica lá')


def test_a_re_NAO_recua_sem_medir_o_vao():
    """O furo é no bloqueio, nunca na percepção.

    `orcamento_de_re` sem `vao_traseiro` devolve o orçamento CEGO, e é ele que
    a ré usava antes da 025. Com o canal que fura o reflexo, recuar às cegas
    deixou de ser "aposta curta" e passou a ser recuar sem NENHUMA defesa —
    nem a do reflexo, que agora está por fora. Este teste exige que o seguidor
    chame a medida.
    """
    with open(SEGUIDOR) as f:
        fonte = f.read()
    assert 'vao_no_corredor_traseiro' in fonte, (
        'o seguidor não mede o vão traseiro — a ré voltou a ser cega, e agora '
        'ela fura o reflexo. Ver decisão 025.')
    assert 'vao_traseiro=None' not in fonte, (
        'ainda há chamada de `orcamento_de_re(vao_traseiro=None)`: é o '
        'orçamento CEGO, e ele não pode mais sair pelo canal que fura.')


def test_o_corredor_da_re_cobre_o_CORPO_e_nao_o_raio():
    """A largura do corredor tem de cobrir o corpo, não o raio do Nav2.

    O `robot_radius` (0,32) é RAIO e a bitola (0,270) é entre-eixos de roda:
    usar qualquer um dos dois como largura mediria um corredor mais estreito
    que o robô, e a quina passaria por fora da medida. É exatamente o modo de
    falhar do setor angular, com outra roupa.
    """
    largura = _default_do_seguidor('re_largura')
    raio_nav2 = valor(PRODUCAO, 'robot_radius')
    assert largura > raio_nav2, (
        f'corredor de {largura} m contra robot_radius {raio_nav2} — a largura '
        'do corredor não pode ser menor que o diâmetro efetivo do corpo')
    recuo = _default_do_seguidor('re_recuo_para_choque')
    assert 0.0 < recuo < largura, f'recuo do para-choque implausível: {recuo}'


def test_a_cegueira_do_scan_cabe_DENTRO_da_folga_da_re():
    """A invariante que torna a janela de `/scan` vencido aceitável.

    Recuando a `v_piso`, uma janela vencida inteira é percorrida às cegas. Ela
    só é segura porque a `folga` que o orçamento desconta do vão medido é
    maior que essa distância. Se alguém subir a janela (ou o piso de linear)
    sem mexer na folga, o robô passa a poder gastar margem que não existe —
    e o sintoma é uma batida traseira, não um aviso.

    Medido em 12-08: `/scan` a 7,7 Hz, p99 0,317 s, máx 0,513 s.
    """
    janela = _default_do_seguidor('re_scan_velho_s')
    v_piso = _default_do_seguidor('v_piso')
    folga = _default_do_seguidor('re_folga')
    cego = janela * v_piso
    assert cego < folga, (
        f'{janela} s de janela a {v_piso} m/s dão {cego:.3f} m às cegas, '
        f'contra folga de {folga} m. A cegueira tem de caber na margem.')
    assert janela > 0.513, (
        f'janela de {janela} s abaixo do pior intervalo de /scan MEDIDO '
        '(0,513 s): a ré abortaria por falso alarme, e o sintoma seria um '
        'robô que se recusa a se desencalhar.')


def test_o_relogio_do_emperramento_e_maior_que_a_PROPRIA_re():
    """A conta que impede a fuga de 12-08, e ela é de tempo.

    O relógio zera quando o robô bate a marca anterior, e depois de uma ré a
    marca é a distância PÓS-recuo — então bastaria ganhar `re_avanco_min`. O
    que consome o tempo não é a distância: é a MANOBRA mais a inércia da
    placa. O robô ainda derrapa para trás depois do zero (`atraso_desliga`
    0,52 s), precisa virar, e só então começa a ganhar.

    ⚠️ A primeira versão deste teste derivava `re_orcamento_cego / v_piso`
    (1,48 s) e por isso APROVAVA 1,5 s — o valor que produziu a fuga. Derivação
    bonita da grandeza errada aprova o defeito. O número que vale é o MEDIDO:
    as rés da corrida da porta duraram 1,6 a 2,8 s, e o rearme veio 1,6 s
    depois de cada fim.

    Medido: 9 rés seguidas levaram o robô de 2,50 m para 5,10 m do objetivo,
    de costas, até o vão traseiro cair de 3,17 m para 0,31 m.
    """
    RE_MAIS_LONGA_MEDIDA_S = 2.8
    parado_s = _default_do_seguidor('re_parado_s')
    assert parado_s > RE_MAIS_LONGA_MEDIDA_S, (
        f're_parado_s={parado_s} s contra manobras de até '
        f'{RE_MAIS_LONGA_MEDIDA_S} s medidas. A ré rearma antes de o robô ter '
        'chance física de aproveitar a anterior — é a fuga de 12-08.')


def test_a_re_para_de_insistir_quando_nao_esta_resolvendo():
    """O teto ESTRUTURAL, o que não depende de sintonia.

    Recuo que não aproxima não é recuperação. Sem este teto, qualquer erro de
    sintonia no relógio volta a produzir um robô andando de costas com cara de
    quem está se desencalhando — e agora ele faz isso pelo canal que fura o
    reflexo, o que torna o passeio bem menos engraçado.
    """
    teto = _default_do_seguidor('re_max_seguidas')
    assert isinstance(teto, int) and 1 <= teto <= 4, (
        f're_max_seguidas={teto} — sem teto pequeno a ré vira fuga.')
    with open(SEGUIDOR) as f:
        fonte = f.read()
    assert 'res_seguidas' in fonte and 'dist_antes_da_re' in fonte, (
        'o seguidor não contabiliza rés que não melhoraram nada — o teto '
        'existe no parâmetro e não na lógica.')


# ---------------------------------------------------------------------------
# O SUAVIZADOR — decisão 026
# ---------------------------------------------------------------------------

ARVORE = os.path.join(RAIZ, 'ros2_packages', 'robot_motion', 'behavior_trees',
                      'replanejamento_com_suavizacao.xml')


def _arvore():
    import xml.etree.ElementTree as ET
    return ET.parse(ARVORE).getroot()


def test_a_arvore_propria_chama_o_suavizador():
    """Se ninguém chama `SmoothPath`, o `smoother_server` sobe e não faz nada.

    Nenhuma das doze árvores de fábrica do Jazzy chama `SmoothPath` — foi por
    isso que esta existe. Servidor de pé sem ninguém chamando é a pior forma
    de falha deste projeto: log limpo, nó vivo, zero efeito (BO-3).
    """
    ids = [n.get('smoother_id') for n in _arvore().iter('SmoothPath')]
    assert ids, 'a árvore não chama SmoothPath — o smoother_server é inerte'
    plugins = _nav2()['smoother_server']['ros__parameters']['smoother_plugins']
    for i in ids:
        assert i in plugins, (
            f'a árvore pede o suavizador `{i}` e o nav2.yaml não o declara')


def test_a_arvore_NUNCA_DESISTE_e_nao_move_roda_para_recuperar():
    """Duas exigências opostas na mesma árvore (14-08, decisão 035).

    **Nunca desistir** é pedido explícito do dono: *"faça essa PORRA desse robô
    não desistir dos goals, leia como o robo do Controle_robo_web faz e faça
    igual"*. A peça que faz isso no robô 1 é UMA — um `RecoveryNode` com muitas
    repetições envolvendo a navegação inteira. Remendar só um ramo não basta, e
    isso foi medido: proteger só o `ComputePathToPose` deixou o `FollowPath`
    matar o objetivo do mesmo jeito.

    **Não mover roda para recuperar** continua valendo: `Spin` é pivô (a placa
    não entrega módulo, 023), `BackUp` é a ré que a 009 tirou do Nav2, e `Wait`
    exige o `behavior_server`, que esta pilha não sobe. Os três seriam no-op —
    o `cmd_vel` deles sai pelo tópico ignorado e a árvore acharia que recuperou
    sem nada ter acontecido (BO-3).
    """
    xml = open(ARVORE).read()
    corpo = xml[xml.index('<root'):]
    assert 'RecoveryNode' in corpo and 'number_of_retries="1000"' in corpo, (
        'sem RecoveryNode de muitas repetições em volta de tudo, um tropeço '
        'do planejador ou do controlador mata a missão')
    for proibido in ('<Spin', '<BackUp', '<Wait'):
        assert proibido not in corpo, (
            f'{proibido} é no-op nesta cadeia — a árvore acharia que recuperou '
            'sem nada ter acontecido')
    assert 'ClearEntireCostmap' in corpo, (
        'a recuperação que FUNCIONA aqui é limpar costmap: três corridas de '
        '14-08 morreram com "start or goal pose are occupied" por marca ao '
        'vivo do Livox, e limpar resolve sem mover o robô')

def test_o_suavizador_esta_na_lista_do_lifecycle():
    """Servidor que sobe fora da lista nunca é ativado; servidor na lista que
    não sobe **aborta o bringup inteiro** (o defeito de 06-08 que levou o
    collision_monitor junto). Os dois lados quebram, então é par travado."""
    with open(os.path.join(RAIZ, 'ros2_packages', 'robot_motion', 'launch',
                           'pilha.launch.py')) as f:
        launch = f.read()
    assert "'smoother_server'" in launch.split('servidores = ')[1][:200], (
        'smoother_server fora da lista do lifecycle_manager — ele sobe e '
        'nunca ativa')
    assert "'nav2_smoother', 'smoother_server'" in launch, (
        'o nó do smoother não é lançado, mas está na lista: isso ABORTA o '
        'bringup inteiro')


def test_a_launch_usa_a_arvore_PROPRIA_e_nao_a_de_fabrica():
    with open(os.path.join(RAIZ, 'ros2_packages', 'robot_motion', 'launch',
                           'pilha.launch.py')) as f:
        launch = f.read()
    assert 'replanejamento_com_suavizacao.xml' in launch
    assert 'navigate_w_replanning_time.xml' not in launch.split('bt_xml =')[1][:400]


@pytest.mark.parametrize('qual', ['local_costmap', 'global_costmap'])
def test_o_raio_do_planejador_fica_na_faixa_util(qual):
    """O raio que o Theta* obedece tem de caber entre o reflexo e o corpo.

    Os dois lados foram medidos com o robô travado, em 14-08:

        < 0,26   o planejador manda por onde o polígono do reflexo VETA, e o
                 robô fica entre os dois — foi o impasse que o dono previu:
                 *"um fala que passa e o outro não deixa"*;
        > 0,314  (o circunscrito da caixa) o Theta* proíbe 15,1% das células
                 livres da sala, inclusive aquela onde o robô está, e devolve
                 `Could not generate path`.
    """
    import yaml
    d = yaml.safe_load(open(PRODUCAO))[qual][qual]['ros__parameters']
    raio = d['robot_radius']
    assert 0.26 <= raio <= 0.314, (
        f'{qual}: raio {raio} fora da faixa útil [0,26 ; 0,314]')


@pytest.mark.parametrize('qual', ['local_costmap', 'global_costmap'])
def test_a_inflacao_TEM_de_passar_do_raio_senao_nao_HA_gradiente(qual):
    """Inflação <= raio deixa a paisagem de custo BINÁRIA, e é isso que faz o
    robô colar na parede.

    A `inflation_layer` marca 253 (proibido) tudo dentro do inscrito e gradua
    só o que está entre o inscrito e `inflation_radius`. Se a inflação não
    passar do raio, essa faixa é VAZIA: célula perto é proibida, célula longe é
    de graça, e nada no meio. O Theta*, que minimiza distância, raspa a borda
    do proibido — o dono viu na tela: *"tá querendo colar nos obstáculos e
    paredes ao invés de ir pelo meio"*.
    """
    import yaml
    d = yaml.safe_load(open(PRODUCAO))[qual][qual]['ros__parameters']
    raio = d['robot_radius']
    inflacao = d['inflation_layer']['inflation_radius']
    assert inflacao > raio + 0.05, (
        f'{qual}: inflação {inflacao} não abre faixa graduada sobre o raio '
        f'{raio} — sem gradiente o caminho cola na parede')
