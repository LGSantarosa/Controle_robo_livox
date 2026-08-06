#!/usr/bin/env python3
"""Mede o **S**: o robô oscila em torno do rumo, ou pende para um lado?

    python3 tools/banco/mede_o_s.py docs/dados/AAAA-MM-DD-.../\\*.csv

## Por que a curvatura sozinha não serve

`medir.py --resumo curvatura` responde "quanto ele saiu da reta por metro". Isso
NÃO distingue os dois defeitos que este robô tem:

    pende   ->  arca sempre para o mesmo lado e não volta
    oscila  ->  vai para um lado, volta, vai de novo  (o S)

Um robô que serpenteia ±10° e sempre volta pode dar curvatura média quase nula;
um que pende 10° e fica lá dá quase a mesma coisa. Foi por isso que a bancada de
05-08 mediu `+0,0417 1/m` com o compensador ligado e o dono, olhando, disse
*"de frente ele faz um pequeno S"* — o número não contradizia o olho, ele apenas
não falava sobre aquilo.

O que separa os dois é **o sinal da guinada trocar**, e quantas vezes.

## Colunas

    amp     amplitude do yaw em torno do rumo de referência [graus]
    invs    inversões do sentido de giro — o S propriamente dito
    T       período médio entre inversões [s], comparável aos 2,8 s de 27-07
    deriva  quanto o rumo saiu do inicial ao FIM da corrida [graus]
    dur     duração do trecho com comando [s]

Ler assim:

    invs 0 + deriva grande   -> PENDE (é o robô cru, sem compensador)
    invs >= 2 + deriva ~0    -> OSCILA (é o S)
    invs 1                   -> transiente: corrigiu uma vez e assentou

⚠️ **`invs` depende da duração.** Uma corrida de 4,5 s vê ~2 períodos; para
separar transiente de ciclo-limite é preciso corrida LONGA. Foi assim que se
descobriu, em 05-08, que o simulador não tem o S: 27 s e 8 m de percurso deram
**1 inversão** e deriva final de 0,0° — ele corrige uma vez e fica.

## Referência medida (bancada 05-08, robô real, `--v 0.25 --wz 0`)

    sem compensador     amp 68°       invs 0    deriva -68°
    com compensador     amp 9,4-18,5° invs 2    T 2,2-2,4 s   deriva 1-4°
"""
import csv
import math
import sys

# rad/s — abaixo disso é ruído da derivação de pose, não guinada de verdade.
# Com `--janela 0.5` no robô (obrigatória: /Odometry a 10 Hz), o piso de ruído
# medido em 05-08 foi ~0,3°/amostra; 0,06 rad/s fica com folga acima disso.
LIMIAR_WZ = 0.06


def norm(a):
    return math.atan2(math.sin(a), math.cos(a))


def le(p):
    with open(p) as f:
        return [{k: (float(v) if v not in ('', None) else None)
                 for k, v in ln.items() if k != 'fase'}
                for ln in csv.DictReader(f)]


def mede(caminho):
    r = [x for x in le(caminho) if x['wz_pose'] is not None]
    andando = [i for i in range(len(r)) if abs(r[i]['cmd_v']) > 1e-6]
    if not andando:
        return None
    i0, i1 = andando[0], andando[-1]

    # A referência é o rumo no instante em que o comando começou — o mesmo que
    # o compensador trava quando `segura_rumo` está ligado.
    ref = r[i0]['yaw']
    erro = [math.degrees(norm(r[i]['yaw'] - ref)) for i in range(i0, i1 + 1)]

    sinais = [1 if r[i]['wz_pose'] > LIMIAR_WZ else
              (-1 if r[i]['wz_pose'] < -LIMIAR_WZ else 0)
              for i in range(i0, i1 + 1)]
    invs, instantes, ultimo = 0, [], None
    for k, s in enumerate(sinais):
        if s == 0:
            continue
        if ultimo is not None and s != ultimo:
            invs += 1
            instantes.append(r[i0 + k]['t'])
        ultimo = s
    periodo = ((instantes[-1] - instantes[0]) / (len(instantes) - 1)
               if len(instantes) > 1 else float('nan'))

    return {'amp': max(erro) - min(erro), 'invs': invs, 'T': periodo,
            'deriva': erro[-1], 'dur': r[i1]['t'] - r[i0]['t']}


def main(caminhos):
    print(f"{'arquivo':<36} {'amp':>7} {'invs':>5} {'T[s]':>6} "
          f"{'deriva':>8} {'dur':>6}")
    for p in caminhos:
        d = mede(p)
        nome = p.split('/')[-1]
        if d is None:
            print(f'{nome:<36} (sem trecho com comando)')
            continue
        T = f"{d['T']:.2f}" if d['T'] == d['T'] else '  —'
        print(f"{nome:<36} {d['amp']:>6.1f}° {d['invs']:>5} {T:>6} "
              f"{d['deriva']:>7.1f}° {d['dur']:>5.1f}s")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        raise SystemExit(1)
    main(sys.argv[1:])
