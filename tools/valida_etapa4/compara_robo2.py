#!/usr/bin/env python3
"""Parâmetros do robô 2 no Gazebo contra as réguas — etapa 4, passo 7 (§10.2).

    compara_robo2.py <captura>/parametros_normalizados.yaml <saida.yaml>

Reusa o normalizador (`tools/linha_de_base/normaliza.py`) sem mudar nada nele.
Três conferências, todas obrigatórias:

  A. contra a baseline v2, SEM permissão: a v2 já contém as duas diferenças
     do §1 (padding explícito com o mesmo valor; `avanco_para_choque` 0,28),
     então qualquer diferença reprova;
  B. contra a baseline ORIGINAL, nos nós que ela tinha com dump (os 23 — o
     `collision_monitor` e o `controller_manager` saíram vazios nela, o furo
     de 21-09), com a permissão cadastrada do passo 2
     (`permitidas/passo2_robo2.yaml`). A permissão tem de ser USADA; sobra de
     permissão reprova aqui, mais estrito que o `compara` da régua;
  C. o §1 item 1: `footprint_padding` dos dois costmaps = 0.009999999776482582
     (o valor vivo do passo 0; entrou escrito no passo 1 sem mudar o dump,
     então não tem arquivo de permissão).

Nenhuma baseline é escrita: só lidas.
"""
import importlib.util
import os
import sys

import yaml

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
V2 = os.path.join(RAIZ, 'docs/dados/2026-09-21-baseline-v2-robo2/02-baseline-v2-aprovada/'
                  'parametros_normalizados.yaml')
ORIGINAL = os.path.join(RAIZ, 'docs/dados/2026-09-21-baseline-robo2/02-baseline-aprovada/'
                        'parametros_normalizados.yaml')
PERMITIDA_P2 = os.path.join(RAIZ, 'tools/linha_de_base/permitidas/passo2_robo2.yaml')
PADDING = 0.009999999776482582
COSTMAPS = ('/global_costmap/global_costmap', '/local_costmap/local_costmap')


def _nz():
    spec = importlib.util.spec_from_file_location(
        'normaliza', os.path.join(RAIZ, 'tools/linha_de_base/normaliza.py'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def confere(nova, v2, original, permitidas, nz):
    """Puro: as três conferências sobre dicionários normalizados."""
    a = nz.compara(v2, nova)
    com_dump = {n for n, p in original.items() if p}
    b = nz.compara({n: original[n] for n in com_dump},
                   {n: nova[n] for n in com_dump if n in nova}, permitidas)
    padding = {c: nova.get(c, {}).get('footprint_padding', 'AUSENTE') for c in COSTMAPS}
    itens = {
        'A_v2_sem_permissao': a.aprovado and not a.diferencas,
        'B_original_com_passo2': b.aprovado and not b.permissoes_sem_uso,
        'C_padding_dos_dois_costmaps': all(v == PADDING for v in padding.values()),
    }
    return {
        'veredito': 'APROVADO' if all(itens.values()) else 'REPROVADO',
        'itens': itens,
        'A_relatorio': nz.relatorio(a),
        'B_nos_comparados': len(com_dump),
        'B_relatorio': nz.relatorio(b),
        'C_padding': padding,
    }


def main(caminho_nova, saida):
    nz = _nz()
    r = confere(nz.le(caminho_nova), nz.le(V2), nz.le(ORIGINAL),
                nz.le_permitidas(PERMITIDA_P2), nz)
    with open(saida, 'w') as f:
        yaml.safe_dump(r, f, allow_unicode=True, sort_keys=False)
    print(f"{r['veredito']}: " + ', '.join(
        f"{k}={'ok' if v else 'FALHOU'}" for k, v in r['itens'].items()))
    return 0 if r['veredito'] == 'APROVADO' else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1], sys.argv[2]))
