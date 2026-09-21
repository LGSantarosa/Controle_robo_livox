#!/usr/bin/env python3
"""Argumentos declarados dos launches do §8 da etapa 4 — a lista travada.

O §8 do plano exige que os comandos antigos continuem funcionando sem argumento
novo: nenhum argumento some e nenhum default muda (exceto `robo`, novo, padrão
2, na pilha). Este módulo extrai `{argumento: default}` de cada launch pela
mesma API do `ros2 launch --show-args` (`get_launch_arguments()`, que inclui
os argumentos dos launches incluídos — inclusive os do `ros_gz_sim`, que o
usuário também consegue passar).

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
import os
import re
import sys

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


def extrai(rel):
    from launch.launch_description_sources import (
        get_launch_description_from_python_launch_file)
    ld = get_launch_description_from_python_launch_file(os.path.join(REPO, rel))
    saida = {}
    for a in ld.get_launch_arguments():
        t = _texto(a.default_value)
        saida[a.name] = normaliza_texto(t) if t is not None else None
    return dict(sorted(saida.items()))


def extrai_todos():
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
                    '# fazer teste passar: mudança aqui é mudança de comando antigo.\n')
            f.write(texto)
    else:
        print(texto)
    return 0


if __name__ == '__main__':
    sys.exit(main())
