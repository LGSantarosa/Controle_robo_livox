"""O que cada nó da pilha recebe, por robô — etapa 4 (PLANO_ETAPA4_ROBO3.md §2).

Função pura, sem ROS, launch, ament_index nem xacro: recebe o número do robô, o
`share/` do `robot_motion` e — só o robô 3 — o `share/` do `robot_base` (onde
mora o artefato de geometria), e devolve caminhos e dicionários. A `pilha.launch.py` monta o robô 2 por aqui, e
é isso que faz desta função o consumidor real do perfil, e não código paralelo.

    nav2                        o YAML dos servidores do Nav2 e dos costmaps
    nav2_rewrites               {caminho: valor} a reescrever nesse YAML ({} = nenhuma)
    collision_monitor           o YAML do reflexo
    collision_monitor_rewrites  {caminho: valor} a reescrever nele ({} = nenhuma)
    twist_mux                   o YAML do árbitro de comando (arquivo INTEIRO)
    path_follower               sobreposição dos defaults do nó ({} = nenhuma)

O `twist_mux` é arquivo inteiro e não reescrita porque o que muda entre os dois
robôs são as PRÓPRIAS FAIXAS — o robô 3 tem `dpad_vel` e não tem `key_vel` nem
`web_vel`. Reescrever folha por folha aqui seria descrever um arquivo dentro de
outro; e faixa que não existe não dá erro, dá silêncio (etapa 6, D3).

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
            # 🔴 O MUX DO ROBÔ 2 É O `twist_mux.yaml`, exatamente este, e o
            # caminho é o valor do parâmetro que o nó recebe: apontar para
            # outro arquivo aqui, ainda que de conteúdo igual, quebraria a
            # comparação byte a byte que protege o robô que funciona.
            'twist_mux': os.path.join(share_motion, 'config',
                                      'twist_mux.yaml'),
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
        # Mux PRÓPRIO, e de nome inequívoco (D3): o do robô 2 não tem
        # `dpad_vel`, e o do controle físico do robô 3
        # (`robot_nav/config/twist_mux_robo3.yaml`, que fica intocado) não tem
        # `auto_vel` nem `unstuck_vel`. Faltar faixa não dá erro: a autonomia e
        # o desencalhe publicariam para o vazio, com o Nav2 dizendo que está
        # navegando.
        'twist_mux': os.path.join(share_motion, 'config',
                                  'twist_mux_pilha_robo3.yaml'),
        # float explícito: parâmetro double do ROS recebendo int derruba o nó.
        'path_follower': {k: float(round(v, 6)) for k, v in seguidor.items()},
    }


# Como se chamam os YAMLs materializados dentro da pasta da corrida. O nome
# diz de quem é o arquivo, porque a pasta é evidência: dois YAMLs anônimos não
# contam história nenhuma depois (etapa 6, D2).
NOMES = {'nav2': 'perfil_nav2.yaml',
         'collision_monitor': 'perfil_collision_monitor.yaml'}


def destinos(perfil: dict, pasta: str) -> dict:
    """Qual arquivo cada consumidor vai ler, dado este perfil e esta corrida.

    Sem reescrita, é o próprio arquivo do `share/` — e isso NÃO é economia de
    disco: o caminho é o valor do parâmetro que o nó recebe, e materializar
    para o robô 2, mesmo com bytes idênticos, mudaria esse valor e quebraria a
    comparação byte a byte que protege o robô que funciona (plano §6).
    """
    return {c: (os.path.join(pasta, NOMES[c]) if perfil[f'{c}_rewrites']
                else perfil[c])
            for c in NOMES}


def materializa(perfil: dict, pasta: str) -> dict:
    """Escreve os YAMLs reescritos na pasta da corrida; devolve o que escreveu.

    Sem reescrita não escreve nada e **nem cria a pasta** — pasta de corrida
    vazia num robô que não materializa seria evidência de coisa nenhuma.

    A escrita é ATÔMICA: serializa num temporário ao lado e troca por
    `os.replace`. O motivo é o costmap: um YAML truncado não dá erro de
    leitura, dá configuração pela metade — e o `os.replace` só é atômico
    dentro do mesmo sistema de arquivos, por isso o temporário mora no mesmo
    diretório do destino, nunca em `/tmp`.
    """
    # 🔴 PREFLIGHT DOS DOIS, e é aqui que mora a diferença entre "nada nasceu"
    # e "metade nasceu". Validar-e-escrever um por vez publicaria o YAML do
    # Nav2 e só então descobriria que a reescrita do reflexo é inválida — e a
    # pasta da corrida ficaria com UM arquivo, que é pior do que nenhum:
    # parece evidência completa. Tudo é lido e reescrito EM MEMÓRIA antes de a
    # pasta existir.
    pronto = {}
    for chave, destino in destinos(perfil, pasta).items():
        if destino == perfil[chave]:
            continue
        pronto[chave] = (destino, aplica_reescritas(_le(perfil[chave]),
                                                    perfil[f'{chave}_rewrites']))
    if not pronto:
        return {}

    escritos = {}
    os.makedirs(pasta, exist_ok=True)
    for chave, (destino, dados) in pronto.items():
        parcial = destino + '.parcial'
        try:
            with open(parcial, 'w') as f:
                yaml.safe_dump(dados, f)
            os.replace(parcial, destino)
        except BaseException:
            if os.path.exists(parcial):
                os.remove(parcial)
            raise
        escritos[chave] = destino
    return escritos


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
