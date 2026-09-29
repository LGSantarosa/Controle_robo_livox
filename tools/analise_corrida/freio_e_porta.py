#!/usr/bin/env python3
"""Julga o freio linear e a passagem da porta a partir do `freeze_capture.csv`.

Escrito para o A/B da decisão 062: ele produz os MESMOS números para a condição
com o freio ligado e para a desligada, e é ele quem define o que cada número
significa. Sem isso o A/B compararia duas leituras feitas com réguas diferentes.

## Por que as medidas saem dos DADOS e não do `launch.log`

O `compensador_rumo` anuncia cada atuação do freio com
`throttle_duration_sec=0.5`: um episódio contínuo de 1,0 s aparece DUAS vezes no
log, e um de 0,4 s aparece uma. Contar linhas contaria o throttle, não o freio.
Aqui um episódio é definido pelo sinal publicado, e o log só serve para localizar
no tempo o que os dados já mediram.

## As definições, explícitas de propósito

- **atuação do freio**: amostras consecutivas de `/cmd_vel_bruto.vx` negativas
  enquanto a ENTRADA (`/compensador_rumo/cmd_vel.vx`) está em zero. A condição
  sobre a entrada é o que separa "o freio mandou ré" de "alguém pediu ré";
  amostras separadas por mais de `GAP_EPISODIO` contam como episódios distintos.
- **parado**: `|vx| <= V_PARADO`. Escolher um limiar é obrigatório — a odometria
  do simulador não entrega zero exato — e ele fica aqui, num lugar só.
- **repouso depois da chegada**: o primeiro instante em que o robô fica parado
  por `T_REPOUSO` seguidos, depois do fim do objetivo. É ele, e não o instante
  do `SUCCEEDED`, que dá a pose final: no `SUCCEEDED` o robô ainda anda a
  ~0,30 m/s.
- **recuo**: distância entre a pose em que `vx` cruza o zero (indo para
  negativo) e a pose de repouso.

Uso::

    freio_e_porta.py <freeze_capture.csv> [--log <launch.log>]
"""
import argparse
import csv
import json
import math
import re
import sys

# ── as réguas, todas num lugar ───────────────────────────────────────────────
V_PARADO = 0.005      # [m/s] abaixo disto a odometria do sim já é ruído
T_REPOUSO = 0.30      # [s] parado por este tempo seguido = assentou
GAP_EPISODIO = 0.30   # [s] buraco maior que isto separa dois episódios
JANELA_FIM = 6.0      # [s] quanto olhar depois do fim de um objetivo
# ⚠️ Distância máxima no tempo para casar uma amostra de SAÍDA com a de
# ENTRADA. Sem ela, `min(entrada, key=...)` casa com a entrada mais próxima
# QUALQUER que seja a distância: uma entrada zerada dez segundos antes
# classificaria uma ré pedida como freio. Duas vezes o período nominal de
# 0,05 s dá folga para jitter sem casar coisa de outro instante.
CASAMENTO_MAX = 0.10  # [s]
TOLERANCIA_GOAL = 0.25  # [m] a tolerância viva do Nav2, para o veredito

CMD_SAIDA = '/cmd_vel_bruto'              # saída do compensador (no sim)
CMD_ENTRADA = '/compensador_rumo/cmd_vel'  # ⚠️ ENTRADA, não saída
ESCAPE = '/unstuck_vel'    # o escape do path_follower, quando gravado
COLISAO = 'collision_state'  # ⚠️ do próprio CSV: `AÇÃO:polígono` em `extra`


def carrega(caminho):
    """`{topico: [(t, vx, wz, px, py)]}`, cada lista ordenada no tempo."""
    por_topico = {}
    with open(caminho) as f:
        for r in csv.DictReader(f):
            try:
                t = float(r['t_wall'])
            except (ValueError, KeyError):
                continue

            def num(campo):
                try:
                    return float(r[campo])
                except (ValueError, KeyError, TypeError):
                    return None
            por_topico.setdefault(r['topic'], []).append(
                (t, num('vx'), num('wz'), num('px'), num('py'), r.get('extra')))
    for v in por_topico.values():
        v.sort(key=lambda x: x[0])
    return por_topico


def objetivos(dados):
    """[(t_inicio, t_fim)] de cada objetivo, pelo `goal_active`."""
    pares, inicio = [], None
    for t, *_, extra in dados.get('goal_active', []):
        if extra == '1':
            inicio = t
        elif extra == '0' and inicio is not None:
            pares.append((inicio, t))
            inicio = None
    return pares


def episodios_de_freio(dados):
    """Cada atuação do freio: (t_inicio, t_fim, vx_comandado, n_amostras)."""
    entrada = dados.get(CMD_ENTRADA, [])
    saida = dados.get(CMD_SAIDA, [])
    if not entrada or not saida:
        return []

    def entrada_em_zero(t):
        # A amostra de entrada mais próxima no tempo — mas só vale se estiver
        # DENTRO de `CASAMENTO_MAX`. Sem esse limite, uma entrada zerada muito
        # distante classificaria como freio uma ré que alguém pediu.
        melhor = min(entrada, key=lambda e: abs(e[0] - t))
        if abs(melhor[0] - t) > CASAMENTO_MAX:
            return False
        return melhor[1] is not None and abs(melhor[1]) <= 1e-9

    negativas = [(t, vx) for t, vx, *_ in saida
                 if vx is not None and vx < -1e-9 and entrada_em_zero(t)]
    eps, atual = [], []
    for amostra in negativas:
        if atual and amostra[0] - atual[-1][0] > GAP_EPISODIO:
            eps.append(atual)
            atual = []
        atual.append(amostra)
    if atual:
        eps.append(atual)
    return [(e[0][0], e[-1][0], min(v for _, v in e), len(e)) for e in eps]


def repouso(dados, desde):
    """Primeira pose em que o robô fica parado por `T_REPOUSO` seguidos."""
    od = [x for x in dados.get('odom', []) if x[0] >= desde and x[1] is not None]
    for i, (t, vx, _, px, py, _) in enumerate(od):
        if abs(vx) > V_PARADO:
            continue
        seguintes = [x for x in od[i:] if x[0] <= t + T_REPOUSO]
        if seguintes and all(abs(x[1]) <= V_PARADO for x in seguintes) \
                and seguintes[-1][0] - t >= T_REPOUSO * 0.9:
            return t, px, py
    return None


def cruza_zero(dados, desde, ate):
    """Onde `vx` passa de positivo para <= 0 — o início do recuo."""
    od = [x for x in dados.get('odom', [])
          if desde <= x[0] <= ate and x[1] is not None]
    for i in range(1, len(od)):
        if od[i - 1][1] > 0 and od[i][1] <= 0:
            return od[i][0], od[i][3], od[i][4]
    return None


def distancia(dados, a, b):
    od = [x for x in dados.get('odom', [])
          if a <= x[0] <= b and x[3] is not None]
    return sum(math.hypot(od[i][3] - od[i - 1][3], od[i][4] - od[i - 1][4])
               for i in range(1, len(od)))


def fases_do_stop(dados, t_stop, t_libera, eps):
    """As fases DENTRO de um `STOP:PolygonStop`, separadas.

    Medir a janela inteira do STOP mistura quatro coisas: a retenção da planta,
    a ré do freio, o escape deliberado do `path_follower` e uma eventual
    SEGUNDA atuação do freio quando o escape termina. O A/B precisa delas
    separadas — a duração total do STOP fica como resultado secundário.

    Devolve avanço residual (o que a planta ainda andou para a frente depois do
    corte), o recuo do freio, e os episódios de freio que caem nesta janela.
    """
    od = [x for x in dados.get('odom', [])
          if t_stop - 0.2 <= x[0] <= t_libera + 0.5 and x[1] is not None]
    if not od:
        return None
    p0 = next((x for x in od if x[0] >= t_stop), od[0])

    # fase 1: o que a planta AINDA andou para a frente depois do corte. Vai do
    # STOP até o cruzamento do zero — e não até o último `vx` positivo da
    # janela, que englobaria o escape deliberado lá na frente.
    cz = cruza_zero(dados, p0[0], t_libera)
    t_fim_avanco = cz[0] if cz else p0[0]
    avanco = distancia(dados, p0[0], t_fim_avanco)

    # fase 2: a ré do freio, do cruzamento do zero até o robô ASSENTAR. O fim é
    # ficar parado, não voltar a andar: o próximo movimento para a frente é o
    # escape do `path_follower`, que é outra fase e não pode entrar nesta.
    recuo, t_fim_re = 0.0, None
    if cz:
        # ⚠️ Não basta "a primeira amostra com |vx| <= V_PARADO": a odometria
        # cruza o zero ao inverter o sentido, e uma única amostra zerada no
        # MEIO da ré encerraria o recuo cedo demais. O fim é o repouso
        # SUSTENTADO por `T_REPOUSO`, a mesma régua da chegada.
        rep = repouso(dados, cz[0])
        depois = [x for x in od if x[0] > cz[0]]
        t_fim_re = rep[0] if rep else (depois[-1][0] if depois else cz[0])
        recuo = distancia(dados, cz[0], t_fim_re)

    return {
        't_stop': t_stop,
        'duracao_total_do_stop_s': round(t_libera - t_stop, 3),
        'avanco_residual_cm': round(avanco * 100, 1),
        't_fim_do_avanco_s': round(t_fim_avanco - t_stop, 3),
        'recuo_do_freio_cm': round(recuo * 100, 1),
        't_cruza_zero_s': round(cz[0] - t_stop, 3) if cz else None,
        't_fim_do_recuo_s': round(t_fim_re - t_stop, 3) if t_fim_re else None,
        'episodios_de_freio_nesta_janela': [
            round(e[0] - t_stop, 3) for e in eps
            if t_stop - 0.2 <= e[0] <= t_libera + 0.5],
    }


def escapes(dados):
    """Episódios de escape do `path_follower`, por `/unstuck_vel`.

    Devolve `None` quando o tópico não foi gravado — que é diferente de "não
    houve escape". A corrida de 29-09 caiu nesse caso: o `freeze_capture` não
    assina `/unstuck_vel`, e contar zero ali seria afirmar o que não se mediu.
    """
    amostras = [(t, vx) for t, vx, *_ in dados.get(ESCAPE, [])
                if vx is not None]
    if not amostras:
        return None
    ativos = [(t, vx) for t, vx in amostras if abs(vx) > 1e-9]
    eps, atual = [], []
    for a in ativos:
        if atual and a[0] - atual[-1][0] > GAP_EPISODIO:
            eps.append(atual)
            atual = []
        atual.append(a)
    if atual:
        eps.append(atual)
    return [{'t_inicio': e[0][0], 'duracao_s': round(e[-1][0] - e[0][0], 3),
             'vx_max': max(abs(v) for _, v in e)} for e in eps]


def transicoes_do_csv(dados):
    """Transições do reflexo pelo `collision_state` do PRÓPRIO CSV.

    ⚠️ Esta é a fonte preferida. O `freeze_capture` já grava `AÇÃO:polígono` na
    coluna `extra`, estruturado; ler o texto em inglês do `launch.log` para a
    mesma informação é depender de formatação de log, que muda com a versão do
    Nav2. O log fica como recurso para quando o CSV não tiver o tópico.
    """
    return [(t, extra) for t, *_, extra in dados.get(COLISAO, []) if extra]


def transicoes_do_monitor(caminho_log):
    """[(t, texto)] das transições do `collision_monitor`, para localizar."""
    if not caminho_log:
        return []
    padrao = re.compile(
        r'\[(1[0-9]{9}\.[0-9]+)\].*?(Robot to stop due to (\w+)|'
        r'Robot to continue normal operation)')
    achados = []
    with open(caminho_log, errors='replace') as f:
        for linha in f:
            m = padrao.search(linha)
            if m:
                achados.append((float(m.group(1)), m.group(2)[:70]))
    return achados


def relatorio(csv_path, log_path=None, goals=None):
    """`goals`: [(x, y)] na ordem dos objetivos, para julgar a tolerância."""
    dados = carrega(csv_path)
    eps = episodios_de_freio(dados)
    esc = escapes(dados)
    out = {'arquivo': csv_path, 'reguas': {
        'V_PARADO': V_PARADO, 'T_REPOUSO': T_REPOUSO,
        'GAP_EPISODIO': GAP_EPISODIO, 'JANELA_FIM': JANELA_FIM,
        'CASAMENTO_MAX': CASAMENTO_MAX, 'TOLERANCIA_GOAL': TOLERANCIA_GOAL,
        'cmd_saida': CMD_SAIDA, 'cmd_entrada': CMD_ENTRADA},
        'episodios_de_freio': [], 'corridas': [], 'monitor': [],
        'escapes': esc,
        'escapes_medidos': esc is not None}

    for t0, t1, vmin, n in eps:
        out['episodios_de_freio'].append({
            't_inicio': t0, 'duracao_s': round(t1 - t0, 3),
            'vx_comandado': vmin, 'amostras': n})

    for i, (a, b) in enumerate(objetivos(dados), 1):
        rep = repouso(dados, b)
        cz = cruza_zero(dados, b - 0.5, b + JANELA_FIM)
        od_a = [x for x in dados.get('odom', []) if x[0] <= a and x[3] is not None]
        od_b = [x for x in dados.get('odom', []) if x[0] <= b and x[3] is not None]
        item = {
            'corrida': i, 'duracao_goal_s': round(b - a, 2),
            'partiu_de': [round(od_a[-1][3], 3), round(od_a[-1][4], 3)] if od_a else None,
            'pose_no_succeeded': [round(od_b[-1][3], 3), round(od_b[-1][4], 3)] if od_b else None,
            'vx_no_succeeded': round(od_b[-1][1], 3) if od_b else None,
            'dist_ate_o_succeeded_m': round(distancia(dados, a, b), 3),
        }
        if rep:
            item['pose_de_repouso'] = [round(rep[1], 3), round(rep[2], 3)]
            item['t_repouso_apos_goal_s'] = round(rep[0] - b, 3)
            item['dist_ate_o_repouso_m'] = round(distancia(dados, a, rep[0]), 3)
            if cz:
                item['recuo_cm'] = round(
                    math.hypot(rep[1] - cz[1], rep[2] - cz[2]) * 100, 1)
                item['t_cruza_zero_apos_goal_s'] = round(cz[0] - b, 3)
        jan = [x for x in dados.get('odom', [])
               if b - 0.5 <= x[0] <= b + JANELA_FIM and x[1] is not None]
        if jan:
            item['pico_de_re_m_s'] = round(min(x[1] for x in jan), 3)

        # O veredito da chegada, quando o objetivo é conhecido. Sem `goals` as
        # chaves ficam de fora — nunca "passou" por omissão.
        alvo = goals[i - 1] if goals and len(goals) >= i else None
        if alvo and rep:
            erro = math.hypot(rep[1] - alvo[0], rep[2] - alvo[1])
            item['goal'] = [alvo[0], alvo[1]]
            item['erro_no_repouso_m'] = round(erro, 3)
            item['dentro_da_tolerancia'] = bool(erro <= TOLERANCIA_GOAL)
            if od_b:
                item['erro_no_succeeded_m'] = round(math.hypot(
                    od_b[-1][3] - alvo[0], od_b[-1][4] - alvo[1]), 3)
        item['freio_apos_o_succeeded'] = [
            round(e[0] - b, 3) for e in eps if b <= e[0] <= b + JANELA_FIM]
        out['corridas'].append(item)

    # O CSV é a fonte preferida; o log só entra se o tópico não foi gravado.
    trans = transicoes_do_csv(dados)
    out['fonte_das_transicoes'] = COLISAO
    if not trans:
        trans = transicoes_do_monitor(log_path)
        out['fonte_das_transicoes'] = 'launch.log'
    out['monitor'] = [{'t': t, 'evento': txt} for t, txt in trans]

    # Cada PolygonStop e a liberação que vem depois dele.
    out['paradas_de_protecao'] = []
    for i, (t, txt) in enumerate(trans):
        # No CSV a ação vem como `STOP:PolygonStop`; no log, como texto em
        # inglês. Reconhecer os dois deixa o juiz igual nas duas fontes.
        if 'PolygonStop' not in txt:
            continue
        libera = next((u for u, d in trans[i + 1:]
                       if d.startswith('DO_NOTHING') or 'continue normal' in d),
                      None)
        if libera:
            fases = fases_do_stop(dados, t, libera, eps)
            if fases:
                out['paradas_de_protecao'].append(fases)
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv')
    p.add_argument('--log', default=None)
    p.add_argument('--goal', action='append', default=None, metavar='X,Y',
                   help='objetivo desta corrida, na ordem; repita por corrida')
    a = p.parse_args()
    goals = None
    if a.goal:
        goals = [tuple(float(v) for v in g.split(',')) for g in a.goal]
    json.dump(relatorio(a.csv, a.log, goals), sys.stdout, indent=2,
              ensure_ascii=False)
    print()


if __name__ == '__main__':
    main()
