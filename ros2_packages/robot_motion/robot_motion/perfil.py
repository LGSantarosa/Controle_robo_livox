"""O que cada nó da pilha recebe, por robô — etapa 4 (PLANO_ETAPA4_ROBO3.md §2).

Função pura, sem ROS, launch, ament_index nem xacro: recebe o número do robô, o
`share/` do `robot_motion` e — só o robô 3 — o `share/` do `robot_base` (onde
mora o artefato de geometria), e devolve caminhos e dicionários. A `pilha.launch.py` monta o robô 2 por aqui, e
é isso que faz desta função o consumidor real do perfil, e não código paralelo.

    nav2                        o YAML dos servidores do Nav2 e dos costmaps
    nav2_rewrites               {caminho: valor} a reescrever nesse YAML ({} = nenhuma)
    collision_monitor           o YAML do reflexo
    collision_monitor_rewrites  {caminho: valor} a reescrever nele ({} = nenhuma)
    path_follower               sobreposição dos defaults do nó ({} = nenhuma)

Reescrita é por CAMINHO COMPLETO (tupla de chaves), aplicada por
`aplica_reescritas` — os costmaps são nós dentro dos servidores do Nav2, e
dicionário passado ao `Node` do servidor não chega neles (plano §3).

O robô 2 NÃO tem arquivo de sobreposição: o perfil dele é a base de hoje, sem
camada nova para quebrar. O robô 3 (passo 4) é a MESMA base + reescritas:
geometria (b) lida de `geometria_robo3.yaml`, política (c) de
`config/perfil_robo3.yaml`, cada uma na sua chave.

⚠️ Perfil montável não é robô liberado: quem decide se a pilha SOBE para um
robô é a própria pilha (`_recusa_robo`), não esta função.
"""
import copy
import json
import os

import yaml


class RoboSemPerfil(ValueError):
    """Pediram o perfil de um robô que não tem (ou ainda não tem) perfil."""


def parametros(robo: int, share_motion: str, *, share_base: str = None) -> dict:
    # `type is int` e não `== 2`: `2.0` e `True` comparam igual a inteiro, e
    # perfil de robô não se escolhe por conversão.
    if type(robo) is int and robo == 2:
        return {
            'nav2': os.path.join(share_motion, 'config', 'nav2.yaml'),
            'nav2_rewrites': {},
            'collision_monitor': os.path.join(share_motion, 'config',
                                              'collision_monitor.yaml'),
            'collision_monitor_rewrites': {},
            'path_follower': {},
        }
    if type(robo) is int and robo == 3:
        if share_base is None:
            raise ValueError('o perfil do robô 3 precisa de share_base (o share/ '
                             'do robot_base, onde está geometria_robo3.yaml)')
        return _robo3(share_motion, share_base)
    raise RoboSemPerfil(f'robô {robo!r} não tem perfil (só os robôs 2 e 3)')


def _le(caminho):
    with open(caminho) as f:
        return yaml.safe_load(f)


def _retangulo(frente, tras, meia):
    """Vértices na ordem do robô 2 (frente-esq, frente-dir, trás-dir, trás-esq).

    Arredonda a 1 µm só para o texto não carregar lixo de float (0,0825 + 0,03
    = 0,11249999…); a conta é a mesma."""
    r = lambda v: round(v, 6)  # noqa: E731
    return [[r(frente), r(meia)], [r(frente), r(-meia)],
            [r(tras), r(-meia)], [r(tras), r(meia)]]


def _robo3(share_motion, share_base):
    perfil = _le(os.path.join(share_motion, 'config', 'perfil_robo3.yaml'))
    geo = _le(os.path.join(share_base, 'config', perfil['geometria']['arquivo']))

    # (b) — do artefato. Só retângulo alinhado e simétrico em y: é o que a 052
    # entrega, e as margens por face abaixo só fazem sentido nele.
    poligono = geo['footprint']['poligono']
    xs = [v[0] for v in poligono]
    ys = [v[1] for v in poligono]
    frente, tras, meia = max(xs), min(xs), max(ys)
    if (len(poligono) != 4 or min(ys) != -meia
            or sorted(map(tuple, poligono)) != sorted(map(tuple, _retangulo(
                frente, tras, meia)))):
        raise ValueError(f'footprint do robô 3 não é retângulo simétrico: {poligono}')
    raio_pivo = geo['raio_varrido_pivo']

    # (c) — do perfil, cada margem na sua chave.
    padding = perfil['footprint_padding']['valor']
    ap = perfil['approach_margem']
    st = perfil['stop_margem']

    nav2_rw = {}
    for c in ('global_costmap', 'local_costmap'):
        # O Nav2 exige o footprint como TEXTO; json dá a mesma lista de volta.
        nav2_rw[(c, c, 'ros__parameters', 'footprint')] = json.dumps(poligono)
        nav2_rw[(c, c, 'ros__parameters', 'footprint_padding')] = padding

    cm = ('collision_monitor', 'ros__parameters')
    cm_rw = {
        cm + ('PolygonApproach', 'points'): json.dumps(_retangulo(
            frente + ap['faces'], tras - ap['faces'], meia + ap['faces'])),
        cm + ('PolygonApproach', 'time_before_collision'): ap['time_before_collision'],
        cm + ('PolygonStop', 'points'): json.dumps(_retangulo(
            frente + st['frente'], tras - st['tras'], meia + st['lados'])),
    }

    seguidor = {
        'passagem_meia_largura': meia,
        'passagem_margem': perfil['passagem_margem']['valor'],
        # A regra do robô 2: o corredor da ré tem a largura do Stop.
        're_largura': 2 * (meia + st['lados']),
        're_recuo_para_choque': -tras,
        'avanco_para_choque': frente,
        'desencalhe_pivo_folga': raio_pivo + perfil['desencalhe_pivo_margem']['valor'],
    }
    return {
        'nav2': os.path.join(share_motion, 'config', 'nav2.yaml'),
        'nav2_rewrites': nav2_rw,
        'collision_monitor': os.path.join(share_motion, 'config', 'collision_monitor.yaml'),
        'collision_monitor_rewrites': cm_rw,
        # float explícito: parâmetro double do ROS recebendo int derruba o nó.
        'path_follower': {k: float(round(v, 6)) for k, v in seguidor.items()},
    }


def aplica_reescritas(dados: dict, reescritas: dict) -> dict:
    """Cópia de `dados` com cada FOLHA existente em `reescritas` trocada.

    Caminho = tupla não vazia de textos, do topo do YAML até a folha. Reprova
    (ValueError, sem efeito parcial — o original nunca muda) caminho que não
    existe, que para num ramo (trocaria um dicionário inteiro) ou que atravessa
    uma folha. Criar chave em silêncio seria o nó lendo parâmetro que ninguém
    declarou, e o valor pedido sumindo sem aviso.
    """
    novo = copy.deepcopy(dados)
    for caminho, valor in reescritas.items():
        if (not isinstance(caminho, tuple) or not caminho
                or not all(isinstance(k, str) for k in caminho)):
            raise ValueError(f'caminho de reescrita tem de ser tupla não vazia '
                             f'de textos: {caminho!r}')
        no = novo
        for i, chave in enumerate(caminho[:-1]):
            if not isinstance(no, dict) or chave not in no:
                raise ValueError(f'reescrita {caminho!r}: {chave!r} não existe '
                                 f'em {caminho[:i]!r}')
            no = no[chave]
        folha = caminho[-1]
        if not isinstance(no, dict):
            raise ValueError(f'reescrita {caminho!r}: {caminho[-2]!r} é folha, '
                             'não dá para descer nela')
        if folha not in no:
            raise ValueError(f'reescrita {caminho!r}: {folha!r} não existe')
        if isinstance(no[folha], dict):
            raise ValueError(f'reescrita {caminho!r}: {folha!r} é ramo, não folha')
        no[folha] = valor
    return novo
