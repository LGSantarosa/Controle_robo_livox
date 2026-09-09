#!/usr/bin/env python3
"""Lê os perfis do robô 3 e produz números para calibrar o Gazebo.

    python3 tools/banco/mede_robo3.py corrida.csv
    python3 tools/banco/mede_robo3.py --pasta docs/dados/AAAA-MM-DD-bancada-robo3

O resumo não decide se o robô é bom. Ele mede a planta: latência, retenção,
velocidade, aceleração, curvatura, assimetria, derrapagem e dispersão entre
repetições. A decisão fica para depois dos CSV existirem.
"""

import argparse
import csv
import json
import math
import os
import re
import statistics
import sys


def norm_ang(a):
    return math.atan2(math.sin(a), math.cos(a))


def numero(v):
    if v in ('', None):
        return None
    try:
        return float(v)
    except (TypeError, ValueError):
        return v


def le(caminho):
    with open(caminho, newline='') as arq:
        return [{k: numero(v) for k, v in l.items()} for l in csv.DictReader(arq)]


def mediana(valores):
    vs = [v for v in valores if isinstance(v, (int, float)) and math.isfinite(v)]
    return statistics.median(vs) if vs else None


def acumulado(rows, campo, inicio=0, fim=None):
    fim = len(rows) if fim is None else fim
    total = 0.0
    for i in range(max(1, inicio + 1), fim):
        a, b = rows[i - 1].get(campo), rows[i].get(campo)
        if isinstance(a, (int, float)) and isinstance(b, (int, float)):
            total += norm_ang(b - a)
    return total


def primeiro_sustentado(rows, inicio, condicao, janela_s=0.30, fim=None):
    limite = len(rows) if fim is None else min(fim, len(rows))
    for i in range(inicio, limite):
        if not condicao(rows[i]):
            continue
        t0 = rows[i]['t']
        j = i
        while j < limite and condicao(rows[j]):
            if rows[j]['t'] - t0 >= janela_s:
                return i
            j += 1
    return None


def _indices_ativos(rows):
    return [i for i, l in enumerate(rows)
            if abs(l.get('cmd_v') or 0.0) > 1e-8 or
            abs(l.get('cmd_wz') or 0.0) > 1e-8]


def _segmentos(rows):
    """Um segmento por fase comandada; inversão devolve ida e volta."""
    ativos = _indices_ativos(rows)
    if not ativos:
        return []
    saida, ini = [], ativos[0]
    fase = rows[ini].get('fase')
    ant = ini
    for i in ativos[1:]:
        if i != ant + 1 or rows[i].get('fase') != fase:
            saida.append((ini, ant + 1, fase))
            ini, fase = i, rows[i].get('fase')
        ant = i
    saida.append((ini, ant + 1, fase))
    return saida


def _deslocamento(rows, i0, i1):
    a, b = rows[i0], rows[i1]
    dx, dy = b['x'] - a['x'], b['y'] - a['y']
    yaw0 = a['yaw']
    return (dx * math.cos(yaw0) + dy * math.sin(yaw0),
            -dx * math.sin(yaw0) + dy * math.cos(yaw0))


def mede(caminho, bitola=None, raio=None):
    rows = le(caminho)
    if len(rows) < 2:
        raise ValueError(f'{caminho}: menos de duas amostras')

    meta_path = os.path.splitext(caminho)[0] + '.json'
    meta = {}
    if os.path.exists(meta_path):
        with open(meta_path) as arq:
            meta = json.load(arq)
    bitola = bitola or meta.get('bitola_m') or 0.3225
    raio = raio or meta.get('raio_roda_m') or 0.0825

    taxa = (len(rows) - 1) / max(1e-9, rows[-1]['t'] - rows[0]['t'])
    dx, dy = _deslocamento(rows, 0, len(rows) - 1)
    resultado = {
        'arquivo': os.path.basename(caminho),
        'perfil': meta.get('perfil', '?'),
        'status': meta.get('status', '?'),
        'amostras': len(rows),
        'taxa_hz': taxa,
        'desloc_long_m': dx,
        'desloc_lateral_m': dy,
        'trajeto_m': (rows[-1].get('trajeto_pose') or 0.0) -
                     (rows[0].get('trajeto_pose') or 0.0),
        'giro_total_rad': acumulado(rows, 'yaw'),
    }

    tensoes = [l.get('tensao') for l in rows]
    tensoes = [v for v in tensoes if isinstance(v, (int, float))]
    if tensoes:
        resultado.update(tensao_inicio_v=tensoes[0], tensao_min_v=min(tensoes),
                          queda_tensao_v=tensoes[0] - min(tensoes))

    segmentos = _segmentos(rows)
    if not segmentos:
        resultado.update(
            deriva_m=math.hypot(rows[-1]['x'] - rows[0]['x'],
                                rows[-1]['y'] - rows[0]['y']),
            deriva_yaw_rad=acumulado(rows, 'yaw'),
            ruido_v_p95=percentil([abs(l.get('v_pose') or 0.0) for l in rows], 95),
            ruido_wz_p95=percentil([abs(l.get('wz_pose') or 0.0) for l in rows], 95),
        )
        resultado['segmentos'] = []
        return resultado

    medidos = []
    for indice, (i0, i1, fase) in enumerate(segmentos):
        proximo_comando = (segmentos[indice + 1][0]
                           if indice + 1 < len(segmentos) else len(rows))
        ativos = rows[i0:i1]
        cmd_v = mediana([l['cmd_v'] for l in ativos]) or 0.0
        cmd_wz = mediana([l['cmd_wz'] for l in ativos]) or 0.0
        # A metade final é a melhor aproximação de regime sem esconder que um
        # pulso curto talvez nunca tenha chegado a regime.
        meio = i0 + max(1, (i1 - i0) // 2)
        regime = rows[meio:i1]
        v_med = mediana([l.get('v_pose') for l in regime])
        wz_med = mediana([l.get('wz_pose') for l in regime])

        limiar_v = 0.02
        limiar_wz = 0.04
        mov = primeiro_sustentado(
            rows, i0,
            lambda l: ((abs(cmd_v) > 1e-8 and abs(l.get('v_pose') or 0.0) > limiar_v)
                       or (abs(cmd_wz) > 1e-8 and
                           abs(l.get('wz_pose') or 0.0) > limiar_wz)),
            0.20, fim=i1)
        latencia = rows[mov]['t'] - rows[i0]['t'] if mov is not None else None

        fim_cmd = i1
        pico_fim = min(proximo_comando,
                       fim_cmd + max(1, int(taxa * 1.5)))
        grandeza = 'wz_pose' if abs(cmd_wz) >= abs(cmd_v) else 'v_pose'
        if fim_cmd < len(rows) and pico_fim > fim_cmd:
            pico = max(range(fim_cmd, pico_fim),
                       key=lambda i: abs(rows[i].get(grandeza) or 0.0))
            retencao = rows[pico]['t'] - rows[fim_cmd]['t']
        else:
            pico, retencao = i1 - 1, None

        assentou = primeiro_sustentado(
            rows, fim_cmd,
            lambda l: (abs(l.get('v_pose') or 0.0) < 0.02 and
                       abs(l.get('wz_pose') or 0.0) < 0.04),
            0.40, fim=proximo_comando)
        tempo_parada = (rows[assentou]['t'] - rows[fim_cmd]['t']
                        if assentou is not None and fim_cmd < len(rows) else None)

        path0 = rows[i0].get('trajeto_pose') or 0.0
        path1 = rows[i1 - 1].get('trajeto_pose') or path0
        caminho = max(0.0, path1 - path0)
        giro = acumulado(rows, 'yaw', i0, i1)
        curvatura = giro / caminho if caminho > 0.03 else None

        ve = mediana([l.get('vel_esq') for l in regime])
        vd = mediana([l.get('vel_dir') for l in regime])
        assimetria = None
        if ve is not None and vd is not None and abs(cmd_wz) < 1e-8:
            den = (abs(ve) + abs(vd)) / 2
            if den > 1e-6:
                assimetria = (abs(ve) - abs(vd)) / den

        giro_roda = 0.0
        for i in range(i0 + 1, i1):
            e, d = rows[i].get('vel_esq'), rows[i].get('vel_dir')
            if e is None or d is None:
                continue
            dt = rows[i]['t'] - rows[i - 1]['t']
            giro_roda += ((d - e) * raio / bitola) * dt
        razao_giro = giro / giro_roda if abs(giro_roda) > 0.02 else None

        coast_end = (assentou if assentou is not None
                     else max(fim_cmd, proximo_comando - 1))
        coast_path = ((rows[coast_end].get('trajeto_pose') or path1) - path1
                      if fim_cmd < len(rows) else 0.0)
        coast_yaw = (acumulado(rows, 'yaw', max(i0, fim_cmd - 1), coast_end + 1)
                     if fim_cmd < len(rows) else 0.0)
        medidos.append({
            'fase': fase,
            'cmd_v': cmd_v, 'cmd_wz': cmd_wz,
            'duracao_cmd_s': rows[i1 - 1]['t'] - rows[i0]['t'],
            'v_regime_m_s': v_med, 'wz_regime_rad_s': wz_med,
            'v_pico_m_s': max(abs(l.get('v_pose') or 0.0) for l in rows[i0:pico_fim]),
            'wz_pico_rad_s': max(abs(l.get('wz_pose') or 0.0)
                                 for l in rows[i0:pico_fim]),
            'latencia_s': latencia,
            'retencao_ate_pico_s': retencao,
            'tempo_parada_s': tempo_parada,
            'caminho_ativo_m': caminho,
            'giro_ativo_rad': giro,
            'curvatura_1_m': curvatura,
            'coast_m': coast_path,
            'coast_yaw_rad': coast_yaw,
            'vel_esq_regime_rad_s': ve,
            'vel_dir_regime_rad_s': vd,
            'assimetria_rodas': assimetria,
            'giro_lio_sobre_roda': razao_giro,
            'corrente_esq_pico_a': maximo_abs(rows[i0:pico_fim], 'corrente_esq'),
            'corrente_dir_pico_a': maximo_abs(rows[i0:pico_fim], 'corrente_dir'),
        })

    resultado['segmentos'] = medidos
    # Corridas de pulso têm um segmento. Espelhar as métricas no topo facilita
    # a tabela agregada sem perder a estrutura da inversão.
    if len(medidos) == 1:
        resultado.update(medidos[0])
    return resultado


def maximo_abs(rows, campo):
    vs = [abs(l[campo]) for l in rows if isinstance(l.get(campo), (int, float))]
    return max(vs) if vs else None


def percentil(valores, p):
    vs = sorted(v for v in valores if isinstance(v, (int, float)))
    if not vs:
        return None
    return vs[round((len(vs) - 1) * p / 100)]


def condicao(nome):
    base = os.path.splitext(os.path.basename(nome))[0]
    return re.sub(r'-(?:[abc]|r\d+)$', '', base)


def agrega(resultados):
    grupos = {}
    for r in resultados:
        grupos.setdefault(condicao(r['arquivo']), []).append(r)
    saida = {}
    campos = [
        'v_regime_m_s', 'wz_regime_rad_s', 'v_pico_m_s', 'wz_pico_rad_s',
        'latencia_s', 'retencao_ate_pico_s', 'tempo_parada_s',
        'curvatura_1_m', 'coast_m', 'coast_yaw_rad', 'assimetria_rodas',
        'giro_lio_sobre_roda', 'queda_tensao_v',
    ]
    for nome, rs in grupos.items():
        g = {'n': len(rs), 'arquivos': [r['arquivo'] for r in rs]}
        for campo in campos:
            vs = [r.get(campo) for r in rs if isinstance(r.get(campo), (int, float))]
            if vs:
                g[campo + '_media'] = statistics.mean(vs)
                g[campo + '_min'] = min(vs)
                g[campo + '_max'] = max(vs)
                g[campo + '_dp'] = statistics.stdev(vs) if len(vs) > 1 else 0.0
        saida[nome] = g
    return saida


def imprime(r):
    print(f"\n=== {r['arquivo']}  perfil={r['perfil']}  status={r['status']}")
    print(f"  {r['amostras']} amostras a {r['taxa_hz']:.1f} Hz; "
          f"trajeto {r['trajeto_m']:.3f} m; giro "
          f"{math.degrees(r['giro_total_rad']):+.1f}°")
    if 'deriva_m' in r:
        print(f"  deriva: {1000*r['deriva_m']:.1f} mm, "
              f"{math.degrees(r['deriva_yaw_rad']):+.2f}°")
        print(f"  ruído p95: v={r['ruido_v_p95']:.3f} m/s, "
              f"wz={r['ruido_wz_p95']:.3f} rad/s")
        return
    for s in r['segmentos']:
        print(f"  [{s['fase']}] cmd=({s['cmd_v']:+.2f}, {s['cmd_wz']:+.2f})  "
              f"regime=({formata(s['v_regime_m_s'])} m/s, "
              f"{formata(s['wz_regime_rad_s'])} rad/s)")
        print(f"    latência {formata(s['latencia_s'])} s; retenção até pico "
              f"{formata(s['retencao_ate_pico_s'])} s; parada "
              f"{formata(s['tempo_parada_s'])} s")
        print(f"    curvatura {formata(s['curvatura_1_m'])} 1/m; cauda "
              f"{formata(s['coast_m'])} m / "
              f"{formata(math.degrees(s['coast_yaw_rad']))}°")
        print(f"    rodas esq/dir {formata(s['vel_esq_regime_rad_s'])} / "
              f"{formata(s['vel_dir_regime_rad_s'])} rad/s; assimetria "
              f"{formata_percent(s['assimetria_rodas'])}")


def formata(v):
    return '?' if v is None else f'{v:.3f}'


def formata_percent(v):
    return '?' if v is None else f'{100*v:+.1f}%'


def escreve_tabela(pasta, resultados):
    escalares = []
    for r in resultados:
        l = {k: v for k, v in r.items()
             if k != 'segmentos' and not isinstance(v, (dict, list))}
        escalares.append(l)
    campos = sorted({k for l in escalares for k in l})
    with open(os.path.join(pasta, 'resumo.csv'), 'w', newline='') as arq:
        w = csv.DictWriter(arq, fieldnames=campos)
        w.writeheader()
        w.writerows(escalares)
    pacote = {'corridas': resultados, 'grupos': agrega(resultados)}
    with open(os.path.join(pasta, 'resumo.json'), 'w') as arq:
        json.dump(pacote, arq, indent=2, ensure_ascii=False)
        arq.write('\n')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('csv', nargs='*')
    ap.add_argument('--pasta')
    ap.add_argument('--bitola', type=float)
    ap.add_argument('--raio', type=float)
    cfg = ap.parse_args(argv)
    caminhos = list(cfg.csv)
    if cfg.pasta:
        caminhos += sorted(
            os.path.join(cfg.pasta, p) for p in os.listdir(cfg.pasta)
            if p.endswith('.csv') and p != 'resumo.csv')
    if not caminhos:
        ap.error('passe um CSV ou --pasta')
    resultados = []
    for caminho in caminhos:
        try:
            r = mede(caminho, cfg.bitola, cfg.raio)
        except (OSError, ValueError) as exc:
            print(f'[ignorado] {exc}', file=sys.stderr)
            continue
        resultados.append(r)
        imprime(r)
    if cfg.pasta and resultados:
        escreve_tabela(cfg.pasta, resultados)
        print(f'\nResumo agregado -> {cfg.pasta}/resumo.csv e resumo.json')
    return 0 if resultados else 1


if __name__ == '__main__':
    sys.exit(main())
