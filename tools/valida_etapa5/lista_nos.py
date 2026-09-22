#!/usr/bin/env python3
"""A lista COMPLETA de nós contra a esperada, e o `uniq -d` — etapa 4, §10.6.

    lista_nos.py <esperados.yaml> <nos.txt> <saida.yaml>

`nos.txt` é a saída crua de `ros2 node list --no-daemon` (uma linha por nó,
com repetição se houver). Reusa o `avalia_grafo` do captura.py (mesma regra:
ocultos fora, `grafo_somente` exatos e voláteis com cardinalidade), e
acrescenta o `uniq -d` literal. Sai 1 se houver duplicado, faltando, sobrando
ou volátil fora da cardinalidade.
"""
import importlib.util
import os
import sys

import yaml

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


def _captura():
    spec = importlib.util.spec_from_file_location(
        'captura', os.path.join(RAIZ, 'tools/linha_de_base/captura.py'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def avalia(linhas, cfg, cap):
    vistos = [x.strip() for x in linhas if x.strip()]
    uniq_d = sorted({n for n in vistos if vistos.count(n) > 1})
    g = cap.avalia_grafo(vistos, cfg['nos'], cfg['grafo_somente'])
    ok = not (uniq_d or g['duplicados'] or g['faltando'] or g['sobrando']
              or g.get('volateis_invalidos'))
    return {'veredito': 'APROVADO' if ok else 'REPROVADO', 'uniq_d': uniq_d,
            'vistos': sorted(vistos), **{k: g[k] for k in sorted(g)}}


def main(esperados, nos_txt, saida):
    cap = _captura()
    with open(nos_txt) as f:
        r = avalia(f.readlines(), cap.le_configuracao(esperados), cap)
    with open(saida, 'w') as f:
        yaml.safe_dump(r, f, allow_unicode=True, sort_keys=False)
    print(f"{r['veredito']}: uniq -d {r['uniq_d'] or 'vazio'}, "
          f"faltando {r['faltando'] or 'nada'}, sobrando {r['sobrando'] or 'nada'}")
    return 0 if r['veredito'] == 'APROVADO' else 1


if __name__ == '__main__':
    sys.exit(main(*sys.argv[1:4]))
