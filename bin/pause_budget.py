#!/usr/bin/env python3
"""pause_budget — orçamento do TEMPO PARADO com goal ativo (o vilão da fluidez).

Por que existe (2026-07-03, dono): "ele para bastante ainda... tem momentos que
ele erra à toa e perde tempo esperando à toa". A régua: NÃO baixar limiar de
certeza nenhum — só cortar espera onde a decisão JÁ foi tomada. Este script lê
o freeze_capture.csv (cadeia de velocidade + estados) e atribui cada segundo
parado-com-goal a UMA causa, por precedência (a mais a jusante que explica):

  guard_hold    motion_guard em blocked/slowing segurando comando que existia
  collision     auto_vel_raw comandava, auto_vel ~0 (collision_monitor cortou)
  wz_engolido   cmd_vel manda giro (|wz|>=1.0) e o odom não gira (zona-morta/
                física/rodas) — o robô DECIDIU girar e não acontece
  vx_zona_morta cmd_vel manda 0<vx<0.20 e o robô não anda (comando fraco)
  vx_sem_efeito cmd_vel manda vx>=0.20 e o robô não anda (encalhe físico)
  unstuck       unstuck_vel comandando (manobra em curso) com robô parado
  humano        joy/key/web comandando (o dono assumiu — não é defeito)
  compensador_gap  o mux repassou e o atuador não recebeu (robô 2)
  mux_gap       o elo anterior comandava e o seguinte saiu ~0 (mux não repassa)
  movimentacao_muda  NINGUÉM cortou: `auto_vel_raw` saiu ~0. A lei de rumo não
                converteu erro em comando (ou saiu abaixo da zona morta). É a
                categoria da PORTA — ver 20-08. O estado do reflexo no momento
                vai entre colchetes, para provar que ele estava inocente
  follower_off  follow_vel ~0 (o driver decidiu não comandar: replan/alvo/
                chegando) — inclui follow_state na quebra fina
  outro         parado sem nenhuma assinatura acima

Uso:  bin/pause_budget.py controle_web/logs/freeze_capture.csv
      (aceita o CSV antigo de 6 colunas; as categorias novas viram 'outro')

Read-only, sem ROS. Eu (assistente) rodo e leio — o dono só roda a rota.
"""
import csv
import math
import sys
from collections import defaultdict

# 🔴 LIMIARES DO ROBÔ 2 — os do robô 1 estavam AQUI e mentiam neste chassi.
# O `CLAUDE.md` avisa em letras grandes: "os knobs anti-skid (zona-morta 1.7,
# autoridade de giro 6.0) eram do atrito do skid-steer 4 rodas; o diferencial
# gira fácil, calibrar do zero". Herdados, eles classificavam errado no lado
# perigoso: com `CMD_WZ = 0.50` um pedido de giro de 0,3 rad/s — que neste robô
# é comando de verdade — contava como "ninguém pediu nada", e a culpa caía na
# camada errada.
#
#   zona morta da roda   0,0178 m/s  MEDIDA 31-07 (decisão 020)
#   piso da movimentação 0,068 m/s   = zona_morta + margem_piso (0,05)
#   tetos                v_max 0,50 m/s · wz_max 1,00 rad/s (movimentacao.yaml)
STOP_VX = 0.05      # |vx| odom abaixo disso = não translada
STOP_WZ = 0.15      # |wz| odom abaixo disso = não gira
CMD_VX = 0.02       # comando linear "existe" (acima da zona morta medida)
CMD_WZ = 0.10       # comando de giro "existe" (wz_max daqui é 1,0, não 6,0)
PISO_VX = 0.068     # abaixo disto a movimentação não deveria nem mandar
WZ_STRONG = 0.50    # metade do wz_max: giro que TEM de mexer o robô
STALE = 0.6         # s sem msg num tópico -> valor considerado zerado


# 🔴 20-08 — O ROBÔ 2 TEM OUTRA CADEIA, e este script nasceu lendo a do robô 1.
# Sem esta tabela ele lia o CSV novo inteiro e classificava tudo como `outro`,
# que é pior do que não rodar: parece resposta. Os nomes à esquerda são os que
# o `freeze_capture` grava neste robô; à direita, o papel que a classificação
# já conhecia. O que não aparece aqui passa com o próprio nome.
#
#   robô 1                        robô 2
#   follow_vel                    (não existe: o seguidor fala Float64)
#   auto_vel_pre (mux autonomia)  (não existe: mux único)
#   auto_vel_raw (pós-guard)      auto_vel_raw (o heading_controller pediu)
#   auto_vel                      auto_vel
#   cmd_vel (pós-mux final)       compensador_rumo/cmd_vel
#   -                             hoverboard_base_controller/cmd_vel = ATUADOR
ALIAS = {
    'Odometry': 'odom',
    'compensador_rumo/cmd_vel': 'mux_out',
    'hoverboard_base_controller/cmd_vel': 'cmd_vel',   # o ATUADOR, nos dois
    'cmd_vel_bruto': 'placa_in',         # só no sim: entrada da placa fingida
}
HUMANO = ('joy_vel', 'key_vel', 'web_vel')


def _norm_topic(t):
    return ALIAS.get(t.lstrip('/'), t.lstrip('/'))


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return math.nan


class Track:
    """último (vx,wz,t) de um tópico; zera se ficar velho (nó calou)."""

    def __init__(self):
        self.vx = self.wz = 0.0
        self.t = -math.inf

    def set(self, t, vx, wz):
        self.t, self.vx, self.wz = t, vx, wz

    def at(self, t):
        if t - self.t > STALE:
            return 0.0, 0.0
        return self.vx, self.wz


def classify(tr, states, t):
    """causa do instante parado t (precedência: mais a jusante primeiro)."""
    fv_x, fv_w = tr['follow_vel'].at(t)
    pre_x, pre_w = tr['auto_vel_pre'].at(t)
    raw_x, raw_w = tr['auto_vel_raw'].at(t)
    av_x, av_w = tr['auto_vel'].at(t)
    cv_x, cv_w = tr['cmd_vel'].at(t)
    us_x, us_w = tr['unstuck_vel'].at(t)

    if abs(cv_w) >= WZ_STRONG:
        return 'wz_engolido'
    if CMD_VX < abs(cv_x) < PISO_VX:
        return 'vx_zona_morta'
    if abs(cv_x) >= PISO_VX:
        return 'vx_sem_efeito'     # comando cheio e o robô não anda: encalhe
                                   # físico/rodas (não é decisão de ninguém)
    if abs(us_x) > 0.03 or abs(us_w) > 0.3:
        return 'unstuck'
    # Humano no comando é explicação suficiente e vem cedo: robô parado com
    # alguém segurando o controle não é defeito da autonomia.
    for h in HUMANO:
        hx, hw = tr[h].at(t)
        if abs(hx) > CMD_VX or abs(hw) > CMD_WZ:
            return 'humano'
    # Robô 2: entre o mux e o atuador há o compensador de curvatura. Se o mux
    # repassou e o atuador não recebeu, o elo perdido é ELE — e isso nenhuma
    # categoria do robô 1 nomeava.
    mo_x, mo_w = tr['mux_out'].at(t)
    # ⚠️ No SIMULADOR há um elo a mais: o compensador entrega em
    # `/cmd_vel_bruto` e a placa fingida é que publica no controlador. Mapear
    # os dois para o mesmo nome apagaria exatamente o degrau que a placa
    # introduz (patamar de borda, latência, assimetria) — que é a razão de ela
    # existir no sim. No robô `placa_in` não existe e fica zerado, sem efeito.
    pi_x, pi_w = tr['placa_in'].at(t)
    if (abs(pi_x) > CMD_VX or abs(pi_w) > CMD_WZ) and \
            abs(cv_x) <= CMD_VX and abs(cv_w) <= CMD_WZ:
        return 'placa_engoliu'
    if (abs(mo_x) > CMD_VX or abs(mo_w) > CMD_WZ) and \
            abs(cv_x) <= CMD_VX and abs(cv_w) <= CMD_WZ and \
            abs(pi_x) <= CMD_VX and abs(pi_w) <= CMD_WZ:
        return 'compensador_gap'
    if states.get('guard_state') in ('blocked', 'slowing') and \
            (abs(pre_x) > CMD_VX or abs(pre_w) > CMD_WZ) and \
            abs(raw_x) <= CMD_VX and abs(raw_w) <= CMD_WZ:
        return 'guard_hold'
    if (abs(raw_x) > CMD_VX or abs(raw_w) > CMD_WZ) and \
            abs(av_x) <= CMD_VX and abs(av_w) <= CMD_WZ:
        return 'collision'
    if (abs(fv_x) > CMD_VX or abs(fv_w) > CMD_WZ) and \
            abs(pre_x) <= CMD_VX and abs(pre_w) <= CMD_WZ:
        return 'mux_gap'
    if (abs(av_x) > CMD_VX or abs(av_w) > CMD_WZ) and \
            abs(mo_x) <= CMD_VX and abs(mo_w) <= CMD_WZ:
        return 'mux_gap'
    # 🔴 A categoria que responde a pergunta de 20-08. Chegar aqui com o robô
    # parado significa: NINGUÉM cortou nada — a movimentação simplesmente não
    # pediu. É o caso da porta, onde a lei de rumo tem 50° de erro na mão e o
    # `|wz|` real é 0,00: ou a lei não converteu o erro em comando, ou o
    # comando saiu abaixo da zona-morta do atuador. Os dois se separam olhando
    # `auto_vel_raw` no CSV: zerado é o primeiro, pequeno é o segundo.
    if abs(raw_x) <= CMD_VX and abs(raw_w) <= CMD_WZ:
        if states.get('follow_state') is not None:
            return 'follower_off[%s]' % states.get('follow_state', '?')
        return 'movimentacao_muda[%s]' % states.get('collision_state', '-')
    if abs(fv_x) <= CMD_VX and abs(fv_w) <= CMD_WZ:
        return 'follower_off[%s]' % states.get('follow_state', '?')
    return 'outro'


def main(path):
    tr = defaultdict(Track)
    states = {}
    goal = True          # CSV antigo não grava goal_active -> conta tudo
    goal_seen = False
    stopped_since = None
    budget = defaultdict(float)
    episodes = []          # (t_ini, dur, {causa: s})
    cur_ep = None
    last_t = None
    t0 = None
    total_goal = 0.0

    with open(path, newline='') as f:
        for row in csv.reader(f):
            if not row or row[0] == 't_wall':
                continue
            t = _f(row[0])
            if math.isnan(t):
                continue
            if t0 is None:
                t0 = t
            topic = _norm_topic(row[1])
            extra = row[6] if len(row) > 6 else ''
            if topic in ('follow_state', 'guard_state', 'collision_state'):
                states[topic] = extra
                continue
            if topic == 'goal_active':
                goal = (extra == '1')
                goal_seen = True
                continue
            vx, wz = _f(row[2]), _f(row[3])
            if topic == 'odom':
                stopped = abs(vx) < STOP_VX and abs(wz) < STOP_WZ
                if last_t is not None and goal:
                    dt = min(t - last_t, STALE)
                    total_goal += dt
                    if stopped and stopped_since is not None:
                        cause = classify(tr, states, t)
                        budget[cause] += dt
                        if cur_ep is None:
                            cur_ep = [stopped_since, defaultdict(float)]
                        cur_ep[1][cause] += dt
                if stopped:
                    if stopped_since is None:
                        stopped_since = t
                else:
                    if cur_ep is not None:
                        dur = sum(cur_ep[1].values())
                        if dur >= 1.0:
                            episodes.append((cur_ep[0], dur, dict(cur_ep[1])))
                        cur_ep = None
                    stopped_since = None
                last_t = t
            elif not math.isnan(vx):
                tr[topic].set(t, vx, wz)

    if cur_ep is not None:
        dur = sum(cur_ep[1].values())
        if dur >= 1.0:
            episodes.append((cur_ep[0], dur, dict(cur_ep[1])))

    tot_stop = sum(budget.values())
    print('janela total do CSV : %.0fs' % ((last_t or 0) - (t0 or 0)))
    print('tempo com goal ativo: %.0fs' % total_goal)
    print('PARADO com goal     : %.0fs (%.0f%% do tempo de missão)'
          % (tot_stop, 100 * tot_stop / total_goal if total_goal else 0))
    print('\n== ORÇAMENTO (quem segura o robô) ==')
    for cause, s in sorted(budget.items(), key=lambda kv: -kv[1]):
        print('  %-34s %6.1fs  (%4.1f%%)'
              % (cause, s, 100 * s / tot_stop if tot_stop else 0))
    print('\n== EPISÓDIOS >= 3s (os vilões) ==')
    big = [e for e in sorted(episodes, key=lambda e: -e[1]) if e[1] >= 3.0]
    for t_ini, dur, causes in big[:15]:
        main_c = max(causes, key=causes.get)
        mix = ' '.join('%s=%.1f' % (c, s)
                       for c, s in sorted(causes.items(), key=lambda kv: -kv[1]))
        print('  t+%5.0fs  %5.1fs  %-20s (%s)'
              % (t_ini - t0, dur, main_c, mix))
    if not big:
        print('  (nenhum)')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
