#!/usr/bin/env python3
"""O julgamento do objetivo curto (etapa 6, passo 7) — função PURA.

    avalia(evidencia) -> {item: (ok, detalhe)}

Sem ROS, sem disco, sem Gazebo: entra um dicionário de evidência, sai um
veredito por item, no molde do `confere.py`/`topicos.py` da etapa 6. Quem colhe
a evidência é outro programa; aqui só se JULGA, e é por isso que o contrato
inteiro cabe em teste sem subir nada (`test_valida_etapa7.py`, 39 casos).

O contrato está em `docs/PLANO_ETAPA6_ROBO3.md` §4.6.1 e na decisão 056 §4.1,
fechado em `6718c57` **antes** deste arquivo existir. Três critérios
simultâneos:

  1. a ação devolve `SUCCEEDED`;
  2. a pose final cai dentro do `xy_goal_tolerance` VIVO, e a pose INICIAL
     prova que o objetivo não nasceu dentro dela;
  3. um comando **acima do patamar vivo** chega ao consumidor final,
     `/hoverboard_base_controller/cmd_vel`, depois do modelo de atuador.

🔴 DUAS REGRAS QUE ATRAVESSAM O ARQUIVO INTEIRO:

  · **nenhum número vivo mora aqui.** Tolerância, `deadband_speed`,
    `escala_real`, `raio` e `bitola` saem todos da evidência. A `placa_simulada`
    tem default para todos eles, então um julgador que "completasse" o que
    faltasse devolveria veredito com número que não é o da corrida — e o
    resultado teria a cara certa. Por isso:
  · **dado ausente REPROVA, com o nome do que faltou.** Nunca vira default e
    nunca vira aprovação. Ausência de evidência não é evidência.

⚠️ LIMITE HONESTO: isto julga a PLACA SIMULADA herdada do robô 2. Não mede a
zona morta real do robô 3, e passar aqui não diz nada sobre o atuador físico.
"""
import math

TOPICO_FINAL = '/hoverboard_base_controller/cmd_vel'
PUBLICADOR_NOMINAL = '/placa_simulada'
CONSUMIDOR_NOMINAL = '/hoverboard_base_controller'

# A folga da comparação do patamar, e ela NÃO é cosmética: no modelo `medido` a
# placa multiplica os dois lados por `k = deadband_speed/mx`, então o comando
# que sobrevive sai EXATAMENTE no patamar. Sem folga, a corrida boa fica na
# dependência do último bit do float.
FOLGA = 1e-3

SUCESSO = ' 7.1 a ação devolveu SUCCEEDED'
TOLERANCIA = ' 7.2 pose final dentro do xy_goal_tolerance vivo'
NASCEU_FORA = ' 7.2 o objetivo não nasceu dentro da tolerância'
PATAMAR = ' 7.3 comando acima do patamar vivo no consumidor final'
TOPOLOGIA = ' 7.3 topologia nominal do tópico observado'
MODELO = ' 7.3 a placa está no modelo medido'
JANELA = ' 7.3 a amostra está dentro da janela do objetivo'

CAMPOS_DO_PATAMAR = ('deadband_speed', 'escala_real', 'raio', 'bitola')


class _Falta(Exception):
    """Dado que não veio. Vira REPROVADO com o nome do que faltou — nunca um
    valor assumido."""


def _exige(d, chave, onde):
    if not isinstance(d, dict) or chave not in d or d[chave] is None:
        raise _Falta(f'faltou {chave} em {onde}')
    return d[chave]


def _ponto(evidencia, chave):
    p = _exige(evidencia, chave, 'evidência')
    return float(_exige(p, 'x', chave)), float(_exige(p, 'y', chave))


def _distancia_xy(a, b):
    """Distância EUCLIDIANA, e a escolha tem consequência medida em teste.

    A tolerância do Nav2 é um RAIO, não uma caixa nem um eixo:

      · julgar só `|Δx|` deixa passar 0,30 m de desvio lateral como erro ZERO;
      · julgar `|Δx| < tol and |Δy| < tol` (a caixa) aprova 0,20/0,20, que está
        a 0,2828 — fora;
      · somar (Manhattan) reprova 0,15/0,15, que está a 0,2121 — dentro.

    Os três erros têm caso próprio no `test_valida_etapa7.py`.
    """
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _patamar(placa):
    """`deadband_speed · escala_real · raio` — em m/s de borda de roda.

    É a velocidade que a placa entrega quando o comando cai dentro do patamar.
    Abaixo disto, no robô, é zona morta: o comando existe e a roda não anda.
    """
    for campo in ('deadband_speed', 'escala_real', 'raio'):
        _exige(placa, campo, '/placa_simulada')
    return (float(placa['deadband_speed']) * float(placa['escala_real'])
            * float(placa['raio']))


def _comando_efetivo(v, wz, bitola):
    """A velocidade da roda MAIS RÁPIDA, em m/s de borda.

    `v` sozinho não serve: giro puro (`v` zero, `wz` alto) é comando efetivo —
    as rodas andam em sentidos opostos —, e julgar por `v` o chamaria de nulo.
    """
    meia = float(bitola) / 2.0
    return max(abs(v - wz * meia), abs(v + wz * meia))


def _na_janela(t, janela):
    """Janela FECHADA: `objetivo_aceito <= t <= resultado`.

    O instante do aceite e o do resultado pertencem à corrida. Depois do
    resultado é teardown — estritamente, sem folga temporal: a quinta corrida
    da etapa 6 mostrou o que o teardown escreve (decisão 057), e folga aqui só
    serviria para salvar corrida ruim com comando que veio depois do fim.
    """
    inicio = float(_exige(janela, 'objetivo_aceito', 'janela'))
    fim = float(_exige(janela, 'resultado', 'janela'))
    return inicio <= float(t) <= fim


def _amostras_do_consumidor_final(evidencia):
    """Só o consumidor final entra na conta.

    🔴 O `/cmd_vel_bruto` é DIAGNÓSTICO e não satisfaz o contrato: o que passa
    por ele ainda pode ser engolido pela placa — abaixo de uma unidade a saída é
    zero, e a latência zera o resto. Foi por isso que o critério antigo ("não
    nulo", observado antes do modelo) não servia: ele aprovaria o robô parado.
    """
    return [a for a in evidencia.get('amostras') or []
            if a.get('topico') == TOPICO_FINAL]


# ── os itens ────────────────────────────────────────────────────────────────

def _julga_sucesso(evidencia):
    r = _exige(evidencia, 'resultado_acao', 'evidência')
    return r == 'SUCCEEDED', str(r)


def _julga_tolerancia(evidencia):
    tol = float(_exige(evidencia, 'xy_goal_tolerance', 'evidência'))
    erro = _distancia_xy(_ponto(evidencia, 'pose_final'),
                         _ponto(evidencia, 'goal'))
    return erro <= tol, f'erro {erro:.4f} m, tolerância viva {tol:.4f} m'


def _julga_nasceu_fora(evidencia):
    """A pose inicial é evidência OBRIGATÓRIA, não enfeite: sem ela, "chegou"
    pode ser "já estava lá" — o modo de falha mais provável com alvo de 1 m."""
    tol = float(_exige(evidencia, 'xy_goal_tolerance', 'evidência'))
    partida = _distancia_xy(_ponto(evidencia, 'pose_inicial'),
                            _ponto(evidencia, 'goal'))
    return partida > tol, f'partida a {partida:.4f} m, tolerância {tol:.4f} m'


def _julga_modelo(evidencia):
    """`modelo: ideal` transforma a placa em fio e `cru` troca o modelo de zona
    morta. Em qualquer dos dois o critério do patamar não prova o que diz."""
    placa = _exige(evidencia, 'placa', 'evidência')
    m = _exige(placa, 'modelo', '/placa_simulada')
    return m == 'medido', str(m)


def _julga_topologia(evidencia):
    grafo = _exige(evidencia, 'grafo', 'evidência')
    topico = _exige(grafo, TOPICO_FINAL, 'grafo')
    pubs = list(_exige(topico, 'publicadores', TOPICO_FINAL))
    assinantes = list(_exige(topico, 'assinantes', TOPICO_FINAL))
    # Publicador ÚNICO e nominal: com dois, não se sabe de quem é a amostra que
    # aprovou — e se não for a placa, o comando não atravessou o modelo de
    # atuador, que é a única coisa que este critério existe para provar.
    if pubs != [PUBLICADOR_NOMINAL]:
        return False, f'publicadores {pubs}, esperado [{PUBLICADOR_NOMINAL!r}]'
    # Publicar para ninguém não é "chegar ao controlador".
    if CONSUMIDOR_NOMINAL not in assinantes:
        return False, f'assinantes {assinantes}, sem {CONSUMIDOR_NOMINAL}'
    return True, f'{PUBLICADOR_NOMINAL} → {CONSUMIDOR_NOMINAL}'


def _julga_janela(evidencia):
    janela = _exige(evidencia, 'janela', 'evidência')
    dentro = [a for a in _amostras_do_consumidor_final(evidencia)
              if _na_janela(_exige(a, 't', 'amostra'), janela)]
    if not dentro:
        return False, (f'nenhuma amostra de {TOPICO_FINAL} entre '
                       f"{janela['objetivo_aceito']} e {janela['resultado']}")
    return True, f'{len(dentro)} amostra(s) na janela'


def _julga_patamar(evidencia):
    placa = _exige(evidencia, 'placa', 'evidência')
    patamar = _patamar(placa)
    bitola = _exige(placa, 'bitola', '/placa_simulada')
    janela = _exige(evidencia, 'janela', 'evidência')

    melhor = 0.0
    for a in _amostras_do_consumidor_final(evidencia):
        if not _na_janela(_exige(a, 't', 'amostra'), janela):
            continue
        melhor = max(melhor, _comando_efetivo(
            float(_exige(a, 'v', 'amostra')),
            float(_exige(a, 'wz', 'amostra')), bitola))

    detalhe = (f'maior comando efetivo {melhor:.4f} m/s, '
               f'patamar vivo {patamar:.4f} m/s (folga {FOLGA})')
    return melhor >= patamar - FOLGA, detalhe


_ITENS = (
    (SUCESSO, _julga_sucesso),
    (TOLERANCIA, _julga_tolerancia),
    (NASCEU_FORA, _julga_nasceu_fora),
    (TOPOLOGIA, _julga_topologia),
    (MODELO, _julga_modelo),
    (JANELA, _julga_janela),
    (PATAMAR, _julga_patamar),
)


def avalia(evidencia):
    """Julga a evidência do objetivo curto. Não levanta: falta de dado vira
    REPROVADO com o nome do que faltou, porque o resultado tem de DIZER o que
    houve — o programa que chama isto escreve CSV, e CSV que estoura no meio
    perde a corrida inteira."""
    itens = {}
    for nome, julga in _ITENS:
        try:
            itens[nome] = julga(evidencia)
        except _Falta as e:
            itens[nome] = (False, str(e))
        except (TypeError, ValueError) as e:
            itens[nome] = (False, f'evidência ilegível: {e}')
    return itens
