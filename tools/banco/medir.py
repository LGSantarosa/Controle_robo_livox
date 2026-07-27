#!/usr/bin/env python3
"""Lê os CSVs do banco e cospe os NÚMEROS do robô.

    python3 medir.py zona_morta_linear zm_lin.csv
    python3 medir.py degrau_giro deg_10.csv
    python3 medir.py curva curva_v04_wz05.csv

Cada leitura devolve o parâmetro que vai para o YAML da movimentação, com a
evidência ao lado — não um veredito solto.
"""
import csv
import math
import sys

LIMIAR_PARADO = 0.02   # m/s e rad/s abaixo disso é considerado imóvel


def le(p):
    return [{k: (float(v) if v not in ('', None) else None)
             for k, v in l.items()} for l in csv.DictReader(open(p))]


def zona_morta(r, campo_cmd, campo_med, unidade):
    """Primeiro comando da rampa que produziu movimento sustentado."""
    for i, l in enumerate(r):
        if abs(l[campo_med]) > LIMIAR_PARADO:
            # exige que continue se movendo, para não pegar solavanco
            seg = r[i:i + 25]
            if len(seg) == 25 and all(abs(k[campo_med]) > LIMIAR_PARADO / 2 for k in seg):
                print(f'  ZONA MORTA = {abs(l[campo_cmd]):.3f} {unidade}')
                print(f'    (saiu do lugar em t={l["t"]:.2f} s, '
                      f'comando {l[campo_cmd]:.3f}, medido {l[campo_med]:.3f})')
                return abs(l[campo_cmd])
    print('  ROBÔ NÃO SAIU DO LUGAR EM NENHUM PONTO DA RAMPA.')
    print(f'    comando máximo tentado: {max(abs(l[campo_cmd]) for l in r):.3f} {unidade}')
    print('    -> refazer com --rampa-ate maior')
    return None


def a_dec(r):
    """Desaceleração angular: wz no instante do corte, e quanto ainda girou."""
    corte = None
    for i in range(1, len(r)):
        if abs(r[i - 1]['cmd_wz']) > 1e-6 and abs(r[i]['cmd_wz']) < 1e-6:
            corte = i
            break
    if corte is None:
        print('  não achei o corte do comando de giro no CSV')
        return None
    wz_corte = r[corte - 1]['wz_pose']
    yaw_corte = r[corte]['yaw']
    # rumo em que ele efetivamente parou de girar
    parou = None
    for l in r[corte:]:
        if abs(l['wz_pose']) < LIMIAR_PARADO:
            parou = l
            break
    if parou is None:
        print('  ele não parou de girar dentro do ensaio — aumentar --dur')
        return None
    dyaw = abs(math.atan2(math.sin(parou['yaw'] - yaw_corte),
                          math.cos(parou['yaw'] - yaw_corte)))
    if dyaw < 1e-3:
        print('  girou de menos depois do corte para medir')
        return None
    a = wz_corte ** 2 / (2.0 * dyaw)
    print(f'  a_dec = {a:.3f} rad/s²')
    print(f'    (cortou com wz={wz_corte:.3f} rad/s e ainda girou '
          f'{math.degrees(dyaw):.1f}° em {parou["t"] - r[corte]["t"]:.2f} s)')
    print(f'  -> SOBREPASSO do controlador velho seria {wz_corte**2/(2*a):.3f} rad '
          f'({math.degrees(wz_corte**2/(2*a)):.0f}°)')
    return a


def curva(r):
    """wz realizado contra comandado, e derrapada (roda contra pose)."""
    seg = [l for l in r if l['t'] >= 4.0 and abs(l['cmd_wz']) > 1e-6]
    if not seg:
        print('  sem trecho de curva sustentada no CSV')
        return
    cmd = sum(l['cmd_wz'] for l in seg) / len(seg)
    med = sum(l['wz_pose'] for l in seg) / len(seg)
    v = sum(l['v_pose'] for l in seg) / len(seg)
    print(f'  a v={v:.2f} m/s:  wz pedido {cmd:.3f}  ->  wz realizado {med:.3f} '
          f'rad/s  ({100*med/cmd:.0f}% do pedido)')
    print(f'    raio da curva: {v/med:.2f} m' if abs(med) > 1e-3 else '')
    if seg[0]['yaw_roda'] not in (None, ''):
        d0 = seg[0]['yaw'] - seg[0]['yaw_roda']
        d1 = seg[-1]['yaw'] - seg[-1]['yaw_roda']
        giro = abs(seg[-1]['yaw'] - seg[0]['yaw'])
        if giro > 0.1:
            print(f'    DERRAPADA: a roda acha que girou {math.degrees(abs(d1-d0)):.1f}° '
                  f'a mais que a pose, numa curva de {math.degrees(giro):.0f}°')


def aceleracao(r):
    subida = [l for l in r if 2.0 <= l['t'] < 6.0]
    if len(subida) < 10:
        print('  ensaio curto demais')
        return
    v_max = max(l['v_pose'] for l in subida)
    # tempo do 10% ao 90% da velocidade atingida
    t10 = next((l['t'] for l in subida if l['v_pose'] > 0.1 * v_max), None)
    t90 = next((l['t'] for l in subida if l['v_pose'] > 0.9 * v_max), None)
    print(f'  velocidade atingida: {v_max:.3f} m/s')
    if t10 and t90 and t90 > t10:
        print(f'  aceleração ≈ {0.8*v_max/(t90-t10):.3f} m/s² '
              f'(10%→90% em {t90-t10:.2f} s)')
    descida = [l for l in r if l['t'] >= 6.0]
    if descida:
        t_parou = next((l['t'] for l in descida
                        if abs(l['v_pose']) < LIMIAR_PARADO), None)
        if t_parou:
            print(f'  desaceleração ≈ {v_max/(t_parou-6.0):.3f} m/s² '
                  f'(parou {t_parou-6.0:.2f} s depois do corte)')


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        raise SystemExit(1)
    tipo, arq = sys.argv[1], sys.argv[2]
    r = le(arq)
    print(f'=== {tipo}  ({arq}, {len(r)} amostras)')
    if tipo == 'zona_morta_linear':
        zona_morta(r, 'cmd_v', 'v_pose', 'm/s')
    elif tipo == 'zona_morta_giro':
        zm = zona_morta(r, 'cmd_wz', 'wz_pose', 'rad/s')
        if zm:
            print(f'    -> em m/s de roda, com bitola L: zona_morta = {zm:.3f}·L/2')
    elif tipo == 'degrau_giro':
        a_dec(r)
    elif tipo == 'curva':
        curva(r)
    elif tipo == 'aceleracao_linear':
        aceleracao(r)
    else:
        print(f'tipo desconhecido: {tipo}')


if __name__ == '__main__':
    main()
