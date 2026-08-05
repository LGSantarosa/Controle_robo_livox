#!/usr/bin/env python3
"""Giro TOTAL de um pivô: do comando ligar até o robô parar.

A mesma régua da varredura do simulador (04-08, fatia 3 da 011):
"giro total do ligar ate parar, do CSV cru". Yaw ACUMULADO amostra a
amostra — entre as pontas o atan2 enrola em ±180°, e o pivô passa disso.
"""
import csv
import math
import sys

LIMIAR_PARADO = 0.05   # rad/s — o mesmo do medir.py


def norm(a):
    return math.atan2(math.sin(a), math.cos(a))


def le(p):
    with open(p) as f:
        return [{k: (float(v) if v not in ('', None) else None)
                 for k, v in linha.items() if k not in ('fase',)}
                for linha in csv.DictReader(f)]


def pivo(p):
    r = le(p)
    liga = next((i for i in range(len(r)) if abs(r[i]['cmd_wz']) > 1e-6), None)
    if liga is None:
        return None, 'nenhum comando de giro no CSV'
    corte = next((i for i in range(liga, len(r))
                  if abs(r[i]['cmd_wz']) < 1e-6), len(r) - 1)

    pico = max(abs(x['wz_pose']) for x in r[liga:])

    # ⚠️ O robô só COMEÇA a girar ~0,27 s depois do comando (latência de liga,
    # 01-08). Com pulso curto o comando já acabou antes de ele sair — procurar
    # a parada a partir do CORTE acha "parado" instantaneamente, porque ele
    # ainda nem partiu, e o giro inteiro fica de fora. Achar primeiro onde o
    # movimento COMEÇOU é o que torna o pulso curto mensurável.
    comecou = next((i for i in range(liga, len(r))
                    if abs(r[i]['wz_pose']) >= LIMIAR_PARADO), None)
    if comecou is None:
        return {'giro': 0.0, 'sobra': 0.0, 'pico': pico, 'n': len(r),
                't_liga': r[corte]['t'] - r[liga]['t'], 't_parar': 0.0}, None

    # Parou = primeira amostra depois de ter COMEÇADO que fica abaixo do
    # limiar e continua abaixo (evita cortar na passagem por zero).
    parou = len(r) - 1
    for i in range(comecou, len(r)):
        if all(abs(x['wz_pose']) < LIMIAR_PARADO for x in r[i:i + 3]):
            parou = i
            break

    giro = sum(norm(r[i]['yaw'] - r[i - 1]['yaw'])
               for i in range(liga + 1, parou + 1))
    sobra = sum(norm(r[i]['yaw'] - r[i - 1]['yaw'])
                for i in range(corte + 1, parou + 1))
    return {
        'giro': math.degrees(giro),
        'sobra': math.degrees(sobra),
        'pico': pico,
        't_liga': r[corte]['t'] - r[liga]['t'],
        't_parar': r[parou]['t'] - r[liga]['t'],
        'n': len(r),
    }, None


if __name__ == '__main__':
    print(f"{'arquivo':<28} {'giro':>8} {'sobra':>8} {'pico':>7} "
          f"{'t_lig':>6} {'t_parar':>8}")
    for p in sys.argv[1:]:
        d, erro = pivo(p)
        nome = p.split('/')[-1]
        if erro:
            print(f'{nome:<28} {erro}')
            continue
        print(f"{nome:<28} {d['giro']:>7.1f}° {d['sobra']:>7.1f}° "
              f"{d['pico']:>7.2f} {d['t_liga']:>6.2f} {d['t_parar']:>8.2f}")
