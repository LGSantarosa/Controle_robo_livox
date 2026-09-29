#!/usr/bin/env python3
"""Esperas e conferências de uma tentativa do A/B — a lógica, sem ROS.

O executor (`bin/ab-freio`) colhe as leituras e chama estas funções. Elas
ficam aqui, puras, porque foi decidindo "já pode?" no olho que as sessões
anteriores se perderam: `ros2 action list` respondendo AUSENTE por ter sido
consultado cedo demais, pilha declarada pronta com a árvore de TF quebrada,
corrida começando com o robô ainda assentando no chão do Gazebo.

Cada função responde uma pergunta com um critério escrito, e o executor grava
a resposta. Nenhuma delas decide sozinha abortar — quem aborta é o executor,
mas o MOTIVO vem daqui, em texto, para o registro da tentativa.
"""
import math

import provas


def pose_na_largada(observada, pedida,
                    tol_m=provas.TOL_POSE_INICIAL_M,
                    tol_yaw=provas.TOL_YAW_INICIAL_RAD):
    """A tentativa pode começar? `(ok, motivo)`.

    Comparar duas metades do A/B que partiram de lugares diferentes é o erro
    que este desenho existe para evitar, então a conferência é ANTES do goal,
    não uma nota de rodapé depois.
    """
    if observada is None:
        return False, 'nenhuma leitura de pose na largada'
    d = math.hypot(observada['x'] - pedida['x'], observada['y'] - pedida['y'])
    dy = abs(_normaliza(observada.get('yaw', pedida['yaw']) - pedida['yaw']))
    if d > tol_m:
        return False, (f'pose na largada a {d:.3f} m da pedida '
                       f'(tolerância {tol_m:.3f} m): '
                       f"({observada['x']:.3f}; {observada['y']:.3f}) em vez de "
                       f"({pedida['x']:.3f}; {pedida['y']:.3f})")
    if dy > tol_yaw:
        return False, (f'yaw na largada a {dy:.3f} rad do pedido '
                       f'(tolerância {tol_yaw:.3f} rad)')
    return True, f'pose na largada a {d:.3f} m e {dy:.3f} rad do pedido'


def _normaliza(a):
    """Ângulo em (−π, π] — sem isto, −3,10 e +3,18 pareceriam 6,28 de erro."""
    return math.atan2(math.sin(a), math.cos(a))


def repouso_sustentado(amostras, v_parado=0.005, t_repouso=0.30):
    """O robô está parado de verdade? `(ok, motivo)`.

    `amostras` é `[(t, vx)]`. As mesmas réguas do juiz
    (`tools/analise_corrida/freio_e_porta.py`) de propósito: se o executor
    usasse um critério e o juiz outro, "o robô parou" significaria duas coisas
    na mesma sessão.
    """
    if not amostras:
        return False, 'nenhuma amostra de velocidade'
    fim = amostras[-1][0]
    janela = [v for t, v in amostras if t >= fim - t_repouso]
    if not janela:
        return False, 'janela de repouso vazia'
    if amostras[-1][0] - amostras[0][0] < t_repouso:
        return False, (f'só {amostras[-1][0] - amostras[0][0]:.2f} s de '
                       f'amostras, precisa de {t_repouso:.2f} s')
    pico = max(abs(v) for v in janela)
    if pico > v_parado:
        return False, (f'ainda se movendo: |vx| chegou a {pico:.4f} m/s nos '
                       f'últimos {t_repouso:.2f} s')
    return True, f'parado há {t_repouso:.2f} s (|vx| <= {pico:.4f} m/s)'


def freio_vivo_confere(lido, esperado):
    """O `freio_linear` do nó vivo é o que a linha de comando pediu?

    Default é justamente o que está sendo testado: ele não serve de testemunha
    de si mesmo. `lido` é o texto cru do `ros2 param get`.
    """
    if lido is None:
        return False, 'não consegui ler `freio_linear` do compensador_rumo'
    texto = str(lido).strip().lower()
    if 'true' in texto:
        valor = True
    elif 'false' in texto:
        valor = False
    else:
        return False, f'resposta não reconhecida do param get: {lido!r}'
    if valor is not esperado:
        return False, (f'freio_linear vivo é {valor}, e esta tentativa pediu '
                       f'{esperado} — a pilha não está na condição do teste')
    return True, f'freio_linear vivo = {valor}, como pedido'


def tolerancia_confere(lida, esperada=provas.TOLERANCIA_GOAL_ESPERADA):
    """A tolerância viva do `goal_checker` bate com a que o juiz usa?

    Se o perfil subir com outra, os números não são comparáveis com a linha de
    base de 29-09 — e um "chegou" medido contra régua diferente não é o mesmo
    "chegou".
    """
    if lida is None:
        return False, 'não consegui ler a tolerância do goal_checker'
    try:
        v = float(lida)
    except (TypeError, ValueError):
        return False, f'tolerância ilegível: {lida!r}'
    if abs(v - esperada) > 1e-9:
        return False, (f'tolerância viva é {v} e o juiz usa {esperada}: os '
                       f'números não seriam comparáveis com a linha de base')
    return True, f'tolerância do goal_checker = {v} m'


def bag_confere(metadata_existe, topicos_gravados, exigidos):
    """O bag fechou e trouxe os tópicos que o veredito precisa?

    Zero mensagem em `/unstuck_vel` só pode virar "zero escapes" se o
    `metadata.yaml` provar que o tópico foi GRAVADO. Sem essa prova continua
    "não medido", e o critério 6 da 062 §7 fica sem resposta.
    """
    if not metadata_existe:
        return False, 'metadata.yaml ausente: o gravador não fechou direito'
    faltando = [t for t in exigidos if t not in set(topicos_gravados or [])]
    if faltando:
        return False, f'tópicos fora do bag: {", ".join(faltando)}'
    return True, f'{len(topicos_gravados)} tópicos no bag, os exigidos inclusive'
