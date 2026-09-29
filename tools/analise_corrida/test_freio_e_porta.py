"""Prova do instrumento que vai julgar o A/B da 062.

Este arquivo existe porque o instrumento ERROU. Na primeira versão, escrita em
29-09, duas fases saíram furadas contra a corrida real:

  - o `avanco_residual` ia até o último `vx` positivo da janela do STOP, o que
    engolia o escape frontal do `path_follower` lá adiante e devolvia 58,4 cm
    onde a resposta é 14,1;
  - o `recuo_do_freio` terminava no primeiro movimento para a frente, que é
    justamente o escape — devolvia 5,6 s onde a resposta é 1,6 s.

Os dois erros passaram despercebidos porque o número saía plausível. Um
instrumento que julga A/B não pode ser conferido "no olho": aqui cada cenário
é sintético, com a resposta conhecida por construção.
"""
import csv
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import freio_e_porta as fp  # noqa: E402

T0 = 1000.0
DT = 0.02


def escreve(tmp_path, linhas):
    caminho = tmp_path / 'freeze_capture.csv'
    with open(caminho, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_wall', 'topic', 'vx', 'wz', 'px', 'py', 'extra'])
        w.writerows(linhas)
    return str(caminho)


def corrida(perfil_vx, saida=None, entrada=None, goal=(0.5, None)):
    """Monta linhas a partir de um perfil de `vx` amostrado a `DT`.

    `perfil_vx` é a velocidade real (vira `odom`, com `px` integrado);
    `saida`/`entrada` são funções de `t` para os dois tópicos de comando.
    """
    linhas, px = [], 0.0
    t_goal_on, t_goal_off = goal
    linhas.append([f'{T0 + t_goal_on:.6f}', 'goal_active', '', '', '', '', '1'])
    for i, vx in enumerate(perfil_vx):
        t = T0 + i * DT
        px += vx * DT
        linhas.append([f'{t:.6f}', 'odom', f'{vx:.6f}', '0', f'{px:.6f}',
                       '0.000000', ''])
        if saida is not None:
            linhas.append([f'{t:.6f}', fp.CMD_SAIDA, f'{saida(t - T0):.6f}',
                           '0', '', '', ''])
        if entrada is not None:
            linhas.append([f'{t:.6f}', fp.CMD_ENTRADA,
                           f'{entrada(t - T0):.6f}', '0', '', '', ''])
    if t_goal_off is not None:
        linhas.append([f'{T0 + t_goal_off:.6f}', 'goal_active', '', '', '', '',
                       '0'])
    return linhas


# ─── episódios de freio ──────────────────────────────────────────────────────

def test_um_episodio_continuo_conta_uma_vez(tmp_path):
    """Duas linhas de log por throttle não podem virar dois episódios."""
    n = 100
    # saída negativa de t=0,5 a t=1,5 (1,0 s) — no log isso sairia 2 ou 3 vezes
    caminho = escreve(tmp_path, corrida(
        [0.3] * n,
        saida=lambda t: -0.5 if 0.5 <= t <= 1.5 else 0.0,
        entrada=lambda t: 0.0))
    eps = fp.episodios_de_freio(fp.carrega(caminho))
    assert len(eps) == 1, eps
    assert eps[0][1] - eps[0][0] == pytest.approx(1.0, abs=0.05)


def test_dois_episodios_separados_contam_duas_vezes(tmp_path):
    caminho = escreve(tmp_path, corrida(
        [0.3] * 200,
        saida=lambda t: -0.5 if (0.2 <= t <= 0.5 or 2.0 <= t <= 2.4) else 0.0,
        entrada=lambda t: 0.0))
    assert len(fp.episodios_de_freio(fp.carrega(caminho))) == 2


def test_re_pedida_pelo_seguidor_nao_e_freio(tmp_path):
    """A condição sobre a ENTRADA é o que separa freio de ré comandada.

    Sem ela, um `path_follower` pedindo ré viraria "o freio atuou" — e o A/B
    reprovaria a condição errada.
    """
    caminho = escreve(tmp_path, corrida(
        [-0.2] * 100,
        saida=lambda t: -0.5,
        entrada=lambda t: -0.5))     # alguém PEDIU a ré
    assert fp.episodios_de_freio(fp.carrega(caminho)) == []


# ─── as fases do STOP: os dois erros de 29-09 ────────────────────────────────

def _cenario_da_porta(tmp_path):
    """Reproduz a forma da porta real: avanço, ré, pausa, escape, pausa.

    Por construção, a `DT` de 0,02 s:
      - 0,00–0,40 s  vx +0,30 …  0   -> avanço residual
      - 0,40–1,00 s  vx  0 … −0,20 … 0 -> a ré do freio
      - 1,00–3,00 s  parado
      - 3,00–3,60 s  vx +0,25        -> o ESCAPE (não é avanço residual!)
      - 3,60–4,00 s  parado
    """
    perfil = []
    t = 0.0
    while t < 4.0:
        if t < 0.4:
            vx = 0.30 * (1 - t / 0.4)
        elif t < 0.7:
            vx = -0.20 * ((t - 0.4) / 0.3)
        elif t < 1.0:
            vx = -0.20 * (1 - (t - 0.7) / 0.3)
        elif t < 3.0:
            vx = 0.0
        elif t < 3.6:
            vx = 0.25
        else:
            vx = 0.0
        perfil.append(vx)
        t += DT
    return escreve(tmp_path, corrida(
        perfil,
        saida=lambda u: -0.5 if u < 0.3 else 0.0,
        entrada=lambda u: 0.0))


def test_o_avanco_residual_nao_engole_o_escape(tmp_path):
    """🔴 O erro nº 1 de 29-09: devolvia 58,4 cm onde a resposta era 14,1."""
    dados = fp.carrega(_cenario_da_porta(tmp_path))
    eps = fp.episodios_de_freio(dados)
    f = fp.fases_do_stop(dados, T0, T0 + 4.0, eps)
    # o avanço é só a área sob a rampa que desce de 0,30 a 0 em 0,4 s ~ 6 cm
    assert f['avanco_residual_cm'] < 10, f
    assert f['t_fim_do_avanco_s'] < 0.6, f
    # e o escape (0,25 m/s por 0,6 s = 15 cm) fica FORA dele
    assert f['avanco_residual_cm'] < 15


def test_o_recuo_termina_ao_assentar_e_nao_no_escape(tmp_path):
    """🔴 O erro nº 2 de 29-09: terminava o recuo no escape, 4 s depois."""
    dados = fp.carrega(_cenario_da_porta(tmp_path))
    eps = fp.episodios_de_freio(dados)
    f = fp.fases_do_stop(dados, T0, T0 + 4.0, eps)
    assert f['t_fim_do_recuo_s'] < 1.5, f
    assert f['recuo_do_freio_cm'] == pytest.approx(4.0, abs=2.0), f


def test_sem_freio_nao_ha_recuo(tmp_path):
    """O lado B do A/B: comando sempre zero, robô só desacelera."""
    perfil = [max(0.0, 0.30 * (1 - t * DT / 0.5)) for t in range(200)]
    caminho = escreve(tmp_path, corrida(
        perfil, saida=lambda u: 0.0, entrada=lambda u: 0.0))
    dados = fp.carrega(caminho)
    assert fp.episodios_de_freio(dados) == []
    f = fp.fases_do_stop(dados, T0, T0 + 3.0, [])
    assert f['recuo_do_freio_cm'] == 0.0, f


# ─── repouso e chegada ───────────────────────────────────────────────────────

def test_repouso_nao_e_o_instante_do_succeeded(tmp_path):
    """No `SUCCEEDED` o robô ainda anda: a pose final é a de REPOUSO."""
    perfil = [max(0.0, 0.30 - 0.30 * (i * DT) / 1.0) for i in range(200)]
    caminho = escreve(tmp_path, corrida(
        perfil, saida=lambda u: 0.0, entrada=lambda u: 0.0,
        goal=(0.1, 0.2)))
    dados = fp.carrega(caminho)
    rep = fp.repouso(dados, T0 + 0.2)
    assert rep is not None
    # o SUCCEEDED foi em t=0,2 s, com o robô ainda a ~0,24 m/s; o repouso vem
    # bem depois, e a pose é OUTRA
    assert rep[0] - T0 > 0.9, rep
    od_succ = [x for x in dados['odom'] if x[0] <= T0 + 0.2][-1]
    assert rep[1] > od_succ[3] + 0.05, (rep, od_succ)


def test_um_pisca_de_parado_nao_conta_como_repouso(tmp_path):
    """`T_REPOUSO` existe para isso: zero instantâneo não é ter parado."""
    perfil = []
    for i in range(300):
        t = i * DT
        perfil.append(0.0 if 0.5 <= t < 0.54 else 0.25)
    caminho = escreve(tmp_path, corrida(
        perfil, saida=lambda u: 0.0, entrada=lambda u: 0.0))
    rep = fp.repouso(fp.carrega(caminho), T0)
    assert rep is None or rep[0] - T0 > 1.0, rep
