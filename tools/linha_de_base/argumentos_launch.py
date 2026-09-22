#!/usr/bin/env python3
"""Argumentos declarados dos launches do §8 da etapa 4 — a lista travada.

O §8 do plano exige que os comandos antigos continuem funcionando sem argumento
novo: nenhum argumento some e nenhum default muda (exceto `robo`, novo, padrão
2, na pilha). Este módulo extrai `{argumento: default}` de cada launch pela
mesma API do `ros2 launch --show-args`, **cortado na fronteira do projeto**:
desce recursivamente pelos launches dos nossos pacotes e PARA no primeiro
include de pacote de terceiro (`ros_gz_sim`, `fast_lio`, `livox_ros_driver2`).

Por que o corte (22-09): a versão anterior gravava também os argumentos
transitivos de terceiro, e com isso a linha de base passou a depender do que
está instalado no PC. No PC da captura o `fast_lio` não existia, a
`localizacao.launch.py` estourava ao ser carregada e o subárvore inteiro era
pulado calado — a trava gravou só `congela_parado` para a `base.launch.py` e
**nunca viu o `frame_da_pose`**, que é nosso e existe desde 17-09. Noutro PC,
com `fast_lio` instalado, os mesmos comandos davam 6 argumentos a mais e o
teste reprovava sem nada ter mudado no repo.

O preço, assumido: um upgrade do `ros_gz_sim` que mude `gz_args` não é mais
pego por esta trava. Ela passa a responder só por argumento NOSSO, que é o que
o §8 promete. Terceiro tem dono e versão próprios.

Para que o resultado não dependa da máquina, a extração roda com um
**esqueleto fixo** dos pacotes de terceiro no início do `AMENT_PREFIX_PATH`:
launches vazios com os nomes que os nossos procuram. Assim os nossos launches
carregam igual com ou sem o terceiro de verdade instalado, e o que vem de
dentro do esqueleto é descartado pela fronteira de qualquer jeito.

Os defaults que são caminho absoluto dependem da máquina; saem normalizados:
`.../install/<pkg>/share/<pkg>` → `<share:pkg>`, a raiz do repo → `<repo>` e o
home → `~`. Default de texto puro sai como o texto; o que é expressão
(PythonExpression) sai pelo `describe()`, que é determinístico.

    python3 tools/linha_de_base/argumentos_launch.py            # mostra
    python3 tools/linha_de_base/argumentos_launch.py --escreve <arquivo.yaml>

Exige o ROS e o `install/` do repo carregados (os launches chamam
`get_package_share_directory`).
"""
import argparse
import contextlib
import os
import re
import sys
import tempfile

import yaml

AQUI = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(AQUI))
TRAVA = os.path.join(AQUI, 'argumentos_launch.yaml')

# Os launches do §8, relativos à raiz do repo.
LAUNCHES = (
    'ros2_packages/robot_motion/launch/pilha.launch.py',
    'ros2_packages/robot_base/launch/base.launch.py',
    'ros2_packages/robot_motion/launch/joystick.launch.py',
    'ros2_packages/robot_base/launch/sim_robo3.launch.py',
    'ros2_packages/robot_nav/launch/controle_robo3.launch.py',
)

# Argumento novo só entra se estiver aqui, com o default exato (§8).
NOVOS_PERMITIDOS = {
    'ros2_packages/robot_motion/launch/pilha.launch.py': {'robo': '2'},
}

# Os pacotes ROS deste repo. Tudo o mais é terceiro, e a recursão para nele.
PACOTES_DO_PROJETO = ('robot_base', 'robot_motion', 'robot_nav', 'robot_planning')

# Esqueleto dos pacotes de terceiro: só os arquivos que os nossos launches
# procuram por caminho. Launch vazio não declara argumento nenhum, então o
# conteúdo do esqueleto nunca entra na linha de base — ele existe para os
# nossos launches CARREGAREM com ou sem o terceiro instalado de verdade.
ESQUELETO_TERCEIROS = {
    'ros_gz_sim': ('launch/gz_sim.launch.py',),
    'fast_lio': ('launch/mapping.launch.py', 'config/mid360.yaml'),
    'livox_ros_driver2': ('launch_ROS2/msg_MID360_launch.py',),
}

# O esqueleto declara UM argumento, de propósito: ele é o canário da fronteira.
# Launch vazio não provaria nada — sem argumento nenhum lá dentro, tirar o corte
# não mudaria a linha de base e o teste passaria com a fronteira quebrada
# (medido em 22-09, mutação M3). Com o canário, se ele aparecer na linha de
# base é porque a recursão atravessou um include de terceiro.
CANARIO_DE_TERCEIRO = 'canario_de_terceiro_nao_entra'

_LAUNCH_ESQUELETO = '''from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('%s', default_value='nao-deve-entrar'),
    ])
''' % CANARIO_DE_TERCEIRO


def normaliza_texto(t, repo=REPO, home=os.path.expanduser('~')):
    t = re.sub(r'[^\s\'"]*/install/([A-Za-z0-9_]+)/share/\1', r'<share:\1>', t)
    t = t.replace(repo, '<repo>')
    if home and home != '/':
        t = t.replace(home, '~')
    return t


def _texto(default):
    from launch.substitutions import TextSubstitution
    if not default:
        return None
    if all(isinstance(s, TextSubstitution) for s in default):
        return ''.join(s.text for s in default)
    return ''.join(s.describe() for s in default)


def e_do_projeto(caminho):
    """O launch é de um pacote NOSSO? (fonte em `ros2_packages/` ou share dele)

    Vale para o fonte do repo e para o `share/` instalado — é por lá que os
    nossos launches se incluem (`get_package_share_directory`). Terceiro é
    todo o resto, inclusive o `twist_mux` vendorizado, que tem dono upstream.
    """
    if not caminho:
        return False
    c = os.path.realpath(str(caminho)).replace(os.sep, '/')
    for pkg in PACOTES_DO_PROJETO:
        if f'/ros2_packages/{pkg}/' in c + '/':
            return True
        if re.search(rf'/install/{pkg}/share/{pkg}(/|$)', c):
            return True
    return False


def _caminho_do_include(inc):
    """Arquivo que um IncludeLaunchDescription aponta, ou None se indecifrável.

    O `.location` da fonte só devolve texto útil DEPOIS que o launch resolve o
    include com um contexto; antes disso ele é a `repr()` das substituições
    (`<...TextSubstitution object at 0x...>`) — foi o que fez a primeira versão
    deste corte jogar fora o `frame_da_pose`, que é nosso. O caminho de verdade
    está guardado como lista de substituições; nos nossos includes elas são
    `TextSubstitution` puro, que resolve com contexto vazio.

    Indecifrável devolve None, e None conta como terceiro: na dúvida a trava
    não promete.
    """
    from launch import LaunchContext
    from launch.utilities import perform_substitutions
    fonte = getattr(inc, 'launch_description_source', None)
    if fonte is None:
        return None
    loc = getattr(fonte, 'location', None)
    if isinstance(loc, str) and os.path.exists(loc):
        return loc
    subs = (getattr(fonte, '_LaunchDescriptionSource__expanded_location', None)
            or getattr(fonte, '_LaunchDescriptionSource__location', None))
    if isinstance(subs, str):
        return subs
    if not subs:
        return None
    try:
        return perform_substitutions(LaunchContext(), list(subs))
    except Exception:
        return None


@contextlib.contextmanager
def esqueleto_de_terceiros():
    """`AMENT_PREFIX_PATH` com o esqueleto fixo dos terceiros na frente.

    Os nossos launches resolvem `fast_lio`/`livox_ros_driver2`/`ros_gz_sim`
    para este esqueleto, esteja o terceiro de verdade instalado ou não — é o
    que faz a linha de base não depender da máquina.
    """
    antigo = os.environ.get('AMENT_PREFIX_PATH', '')
    with tempfile.TemporaryDirectory(prefix='trava-terceiros-') as raiz:
        prefixos = []
        for pkg, arquivos in ESQUELETO_TERCEIROS.items():
            prefixo = os.path.join(raiz, 'install', pkg)
            share = os.path.join(prefixo, 'share', pkg)
            indice = os.path.join(prefixo, 'share', 'ament_index',
                                  'resource_index', 'packages')
            os.makedirs(indice, exist_ok=True)
            open(os.path.join(indice, pkg), 'w').close()
            for rel in arquivos:
                destino = os.path.join(share, *rel.split('/'))
                os.makedirs(os.path.dirname(destino), exist_ok=True)
                with open(destino, 'w') as f:
                    f.write(_LAUNCH_ESQUELETO if rel.endswith('.py') else '{}\n')
            prefixos.append(prefixo)
        os.environ['AMENT_PREFIX_PATH'] = os.pathsep.join(
            prefixos + ([antigo] if antigo else []))
        try:
            yield raiz
        finally:
            if antigo:
                os.environ['AMENT_PREFIX_PATH'] = antigo
            else:
                os.environ.pop('AMENT_PREFIX_PATH', None)


def argumentos_com_fronteira(ld):
    """`DeclareLaunchArgument` alcançáveis sem atravessar include de terceiro.

    É a mesma caminhada do `get_launch_arguments()` do `launch` (mesmos
    `describe_sub_entities`/`describe_conditional_sub_entities`, mesma regra de
    nome repetido e de `ResetLaunchConfigurations`), com uma diferença: o
    include cuja fonte não é nossa não é percorrido.

    Não dá para usar o `get_launch_arguments_with_include_launch_description_actions`
    e filtrar pela lista que ele devolve: essa lista é UM objeto só,
    compartilhado entre os irmãos do mesmo nível e mutado depois de ter sido
    guardado, então ela mistura ancestrais com irmãos. Medido em 22-09: o
    `frame_da_pose`, declarado na nossa `localizacao.launch.py`, vinha com o
    `fast_lio` e o `livox_ros_driver2` na "cadeia" — que são includes IRMÃOS,
    declarados depois dele no mesmo arquivo.
    """
    from launch.actions import (
        DeclareLaunchArgument, IncludeLaunchDescription,
        ResetLaunchConfigurations)
    achados, vistos = [], set()

    def anda(entidades):
        for e in entidades:
            if isinstance(e, DeclareLaunchArgument):
                if e.name not in vistos:
                    vistos.add(e.name)
                    achados.append(e)
            if isinstance(e, ResetLaunchConfigurations):
                return
            if (isinstance(e, IncludeLaunchDescription)
                    and not e_do_projeto(_caminho_do_include(e))):
                continue  # A FRONTEIRA.
            anda(e.describe_sub_entities())
            for condicional in e.describe_conditional_sub_entities():
                anda(condicional[1])

    anda(ld.entities)
    return achados


def extrai(rel):
    from launch.launch_description_sources import (
        get_launch_description_from_python_launch_file)
    ld = get_launch_description_from_python_launch_file(os.path.join(REPO, rel))
    saida = {}
    for a in argumentos_com_fronteira(ld):
        t = _texto(a.default_value)
        saida[a.name] = normaliza_texto(t) if t is not None else None
    return dict(sorted(saida.items()))


def extrai_todos():
    with esqueleto_de_terceiros():
        return {rel: extrai(rel) for rel in LAUNCHES}


def confere(travado, atual, novos_permitidos=NOVOS_PERMITIDOS):
    """Lista de problemas (vazia = ok): sumiu, default mudou, novo não permitido."""
    prob = []
    for rel in sorted(set(travado) | set(atual)):
        if rel not in atual:
            prob.append(f'{rel}: launch sumiu da lista')
            continue
        if rel not in travado:
            prob.append(f'{rel}: launch fora da trava')
            continue
        t, a = travado[rel], atual[rel]
        for nome in sorted(set(t) | set(a)):
            if nome not in a:
                prob.append(f'{rel}: argumento {nome!r} sumiu')
            elif nome not in t:
                perm = novos_permitidos.get(rel, {})
                if nome not in perm:
                    prob.append(f'{rel}: argumento novo {nome!r} não permitido')
                elif perm[nome] != a[nome]:
                    prob.append(f'{rel}: {nome!r} novo com default {a[nome]!r}, '
                                f'permitido {perm[nome]!r}')
            elif t[nome] != a[nome]:
                prob.append(f'{rel}: default de {nome!r} mudou: {t[nome]!r} → {a[nome]!r}')
    return prob


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--escreve')
    a = ap.parse_args(argv)
    todos = extrai_todos()
    texto = yaml.safe_dump(todos, sort_keys=True, allow_unicode=True, width=1000)
    if a.escreve:
        with open(a.escreve, 'w') as f:
            f.write('# Argumentos declarados dos launches do §8 da etapa 4, antes da\n'
                    '# etapa (passo 0). Gerado por argumentos_launch.py --escreve;\n'
                    '# conferido por test_argumentos_launch.py. NÃO regerar para\n'
                    '# fazer teste passar: mudança aqui é mudança de comando antigo.\n'
                    '#\n'
                    '# Só argumento NOSSO: a extração para na fronteira dos pacotes\n'
                    '# de terceiro (ros_gz_sim, fast_lio, livox_ros_driver2), e por\n'
                    '# isso esta lista é a mesma com ou sem eles instalados. Regerada\n'
                    '# em 22-09 por causa disso — ver o docstring do extrator.\n')
            f.write(texto)
    else:
        print(texto)
    return 0


if __name__ == '__main__':
    sys.exit(main())
