#!/usr/bin/env python3
"""A árvore de nós que a pilha sobe para o robô 2 — gate do passo 8, item (c).

    lista_nos.py <pilha.launch.py> <sim: true|false>

`docs/PLANO_ETAPA6_ROBO3.md` §6(c): "as duas combinações do robô 2 percorridas
e a lista de parâmetros de cada nó comparada com a de `1f49981`, byte a byte".

Percorre a launch num `LaunchContext` real e NÃO executa nada: `Node` e
`ExecuteProcess` são só anotados. Includes, `TimerAction`, `GroupAction`,
`OpaqueFunction` e `RegisterEventHandler` são expandidos, e a condição de cada
entidade é avaliada — o que sai é o conjunto de processos que aquela
combinação subiria, na ordem da launch.

Cada commit tem de ser avaliado contra o PRÓPRIO build (o `share/` dele no
ambiente): os parâmetros vêm de arquivos do `share/`, e avaliar a launch antiga
contra o `install/` novo daria "igual" por construção. Por isso o arquivo de
parâmetros sai como caminho relativo ao `share/` MAIS o sha256 do conteúdo, e
qualquer caminho absoluto vira relativo ao prefixo que o contém — assim duas
árvores em lugares diferentes do disco comparam byte a byte.
"""
import contextlib
import hashlib
import os
import re
import sys

import yaml
from launch import LaunchContext, LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler,
                            TimerAction)
from launch.utilities import (normalize_to_list_of_substitutions,
                              perform_substitutions)
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
from launch.launch_description_sources import (
    get_launch_description_from_python_launch_file)

PREFIXOS = sorted({p for p in os.environ.get('AMENT_PREFIX_PATH', '').split(':')
                   if p}, key=len, reverse=True)
LONGO = 200     # texto maior que isto (URDF, por exemplo) sai como hash


RAIZ = None     # a raiz do repositório avaliado; preenchida no main
CARIMBO = re.compile(r'\d{4}-\d{2}-\d{2}_\d{6}')


def _relativo(texto):
    """Tira do texto o que depende de ONDE e QUANDO a árvore foi avaliada:
    prefixos de instalação, a raiz do repositório (mundo e mapa vêm dela) e
    carimbos de hora (a pasta de log leva a hora da avaliação). Troca TODAS as
    ocorrências — o URDF carrega caminho absoluto no meio do texto."""
    for p in PREFIXOS:
        texto = texto.replace(p + os.sep, '<prefixo>/')
    if RAIZ:
        texto = texto.replace(RAIZ + os.sep, '<raiz>/')
    return CARIMBO.sub('<carimbo>', texto)


def _sha(dados):
    return hashlib.sha256(dados).hexdigest()


def _valor(v):
    """Valor de parâmetro comparável: caminho vira relativo + conteúdo."""
    if isinstance(v, os.PathLike) or (isinstance(v, str) and os.path.isfile(v)):
        c = os.fspath(v)
        with open(c, 'rb') as f:
            return {'arquivo': _relativo(c), 'sha256': _sha(f.read())}
    if isinstance(v, str):
        v = _relativo(v)
        if len(v) > LONGO:
            return {'texto_longo': len(v), 'sha256': _sha(v.encode())}
        return v
    if isinstance(v, dict):
        return {str(k): _valor(x) for k, x in sorted(v.items(), key=lambda i: str(i[0]))}
    if isinstance(v, (list, tuple)):
        return [_valor(x) for x in v]
    return v


def _texto(ctx, subs):
    if subs is None:
        return None
    return _relativo(perform_substitutions(
        ctx, normalize_to_list_of_substitutions(subs)))


def _no(ctx, e):
    remaps = [[_texto(ctx, a), _texto(ctx, b)]
              for a, b in (e._Node__remappings or [])]
    args = [_texto(ctx, a) for a in (e._Node__arguments or [])]
    return {
        'tipo': 'Node',
        'pacote': _texto(ctx, e._Node__package),
        'executavel': _texto(ctx, e._Node__node_executable),
        'nome': _texto(ctx, e._Node__node_name),
        'namespace': _texto(ctx, e._Node__node_namespace),
        'parametros': _valor(evaluate_parameters(ctx, e._Node__parameters or [])),
        'remapeamentos': remaps,
        'argumentos': [_valor(a) for a in args],
    }


def _processo(ctx, e):
    return {'tipo': 'ExecuteProcess',
            'cmd': [_valor(_texto(ctx, c)) for c in e.cmd]}


def percorre(ctx, entidades, saida):
    for e in entidades:
        cond = getattr(e, 'condition', None)
        if cond is not None and not cond.evaluate(ctx):
            continue
        if isinstance(e, Node):
            saida.append(_no(ctx, e))
        elif isinstance(e, ExecuteProcess):
            saida.append(_processo(ctx, e))
        elif isinstance(e, TimerAction):
            percorre(ctx, e.actions, saida)
        elif isinstance(e, RegisterEventHandler):
            acoes = e.event_handler._OnActionEventBase__actions_on_event
            percorre(ctx, acoes if isinstance(acoes, list) else [], saida)
        elif isinstance(e, (LaunchDescription, IncludeLaunchDescription,
                            DeclareLaunchArgument)) or hasattr(e, 'visit'):
            filhos = e.visit(ctx) or []
            percorre(ctx, filhos, saida)


def main(argv):
    if len(argv) != 2 or argv[1] not in ('true', 'false'):
        print(__doc__, file=sys.stderr)
        return 2
    global RAIZ
    # <raiz>/ros2_packages/robot_motion/launch/pilha.launch.py
    RAIZ = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(argv[0])))))
    nos = []
    # A launch imprime aviso no stdout; o stdout desta ferramenta é só o YAML.
    with contextlib.redirect_stdout(sys.stderr):
        ld = get_launch_description_from_python_launch_file(argv[0])
        ctx = LaunchContext()
        ctx.launch_configurations.update({'robo': '2', 'sim': argv[1]})
        percorre(ctx, ld.entities, nos)
    yaml.safe_dump(nos, sys.stdout, sort_keys=True, allow_unicode=True,
                   default_flow_style=False)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
