#!/usr/bin/env python3
"""A evidência do objetivo curto, montada a partir dos brutos da corrida
(decisão 060) — função PURA.

    monta(brutos) -> (evidencia, falhas)

Sem ROS e sem bag: quem grava, consulta e extrai é o `bin/valida-etapa7` e o
`le_bag.py`; aqui só se MONTA o dicionário que o `julga.avalia` recebe. As
chaves de `brutos` estão descritas no `test_monta_etapa7.py`.

`falhas` é a lista de falhas de COLETA, `(fonte, detalhe)`, com fonte `acao`,
`tolerancia`, `grafo` ou `placa`; cada uma vira item próprio no
`resultado.csv`. Duas regras, e é nelas que mora o valor disto:

  · fonte que falhou deixa o campo AUSENTE — nunca preenchido por outra fonte.
    O juiz então reprova pelo que faltou, e essa segunda reprovação é
    consequência deliberada, não duplicidade;
  · conferência cruzada que diverge (a linha final do `objetivo.log`, o
    `perfil_nav2.yaml` materializado) reprova a coleta, mas NÃO apaga o valor
    canônico: o juiz continua julgando o status estruturado e a tolerância viva.

Tempos chegam em nanossegundos inteiros e só viram segundos aqui, uma vez, na
montagem final — a janela é fechada, e a igualdade nas bordas tem de
sobreviver à conversão.
"""
import re

import yaml

TOPICO_FINAL = '/hoverboard_base_controller/cmd_vel'
CAMPOS_PLACA = ('modelo', 'deadband_speed', 'escala_real', 'raio', 'bitola')

# action_msgs/msg/GoalStatus
TERMINAIS = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}

ACAO = 'acao'
TOL = 'tolerancia'
GRAFO = 'grafo'
PLACA = 'placa'

_S = 1_000_000_000


class _Falha(Exception):
    def __init__(self, fonte, detalhe):
        super().__init__(detalhe)
        self.fonte = fonte
        self.detalhe = detalhe


def _segundos(ns):
    return ns / _S


def _ns_valido(ns):
    """Tempo ROS: inteiro de nanossegundos, não negativo. `bool` é `int` em
    Python e passaria calado."""
    return isinstance(ns, int) and not isinstance(ns, bool) and ns >= 0


def _plano(d, prefixo=''):
    """O dump vivo sai achatado com pontos; o materializado, aninhado. Achatar
    os dois deixa uma única forma de perguntar `<id>.xy_goal_tolerance`."""
    saida = {}
    for k, v in d.items():
        chave = f'{prefixo}{k}'
        if isinstance(v, dict):
            saida.update(_plano(v, f'{chave}.'))
        else:
            saida[chave] = v
    return saida


def _parametros(texto, no, fonte, onde):
    """`ros__parameters` de UM nó num YAML de parâmetros, achatado."""
    try:
        dados = yaml.safe_load(texto) if isinstance(texto, str) else None
    except yaml.YAMLError as e:
        raise _Falha(fonte, f'{onde} ilegível: {e}') from None
    if not isinstance(dados, dict) or not isinstance(dados.get(no), dict) \
            or not isinstance(dados[no].get('ros__parameters'), dict):
        raise _Falha(fonte, f'{onde} ilegível: sem {no}/ros__parameters')
    return _plano(dados[no]['ros__parameters'])


# ── a ação e a janela ───────────────────────────────────────────────────────

def _uuid_do_log(log):
    if not isinstance(log, str):
        raise _Falha(ACAO, 'objetivo.log ilegível')
    ids = re.findall(r'^Goal accepted with ID: ([0-9a-fA-F]{32})\s*$', log,
                     re.MULTILINE)
    if not ids:
        raise _Falha(ACAO, 'uuid ausente no objetivo.log '
                           '("Goal accepted with ID:")')
    if len(ids) > 1:
        raise _Falha(ACAO, f'uuid duplicado no objetivo.log: {ids}')
    return ids[0].lower()


def _acao(log, status):
    """Devolve `(resultado, aceite_ns, terminal_ns)` do status estruturado,
    correlacionado pelo UUID do log. Qualquer ambiguidade reprova — nunca uma
    escolha silenciosa."""
    uuid = _uuid_do_log(log)
    if not isinstance(status, list):
        raise _Falha(ACAO, 'status da ação ilegível ou não gravado')
    try:
        linhas = sorted(status, key=lambda linha: linha['t_ns'])
        vistos = {g['uuid'].lower() for linha in linhas for g in linha['goals']}
    except (KeyError, TypeError, AttributeError) as e:
        raise _Falha(ACAO, f'status da ação ilegível: {e!r}') from None
    if uuid not in vistos:
        raise _Falha(ACAO, f'uuid {uuid} do objetivo.log não aparece no '
                           'status gravado')
    outros = sorted(vistos - {uuid})
    if outros:
        raise _Falha(ACAO, f'goal concorrente no status: {outros} além de '
                           f'{uuid}')

    stamp = terminal = t_terminal = None
    for linha in linhas:
        for g in linha['goals']:
            if stamp is None:
                stamp = g['stamp_ns']
            elif g['stamp_ns'] != stamp:
                raise _Falha(ACAO, f'goal_info.stamp mudou para o uuid {uuid}: '
                                   f'{stamp} → {g["stamp_ns"]} ns')
            codigo = g['status']
            if terminal is None:
                if codigo in TERMINAIS:
                    terminal, t_terminal = codigo, linha['t_ns']
            elif codigo != terminal:
                raise _Falha(ACAO, 'transição inconsistente depois do terminal '
                                   f'{TERMINAIS[terminal]}: status {codigo!r} '
                                   f'gravado em {linha["t_ns"]} ns')
    if terminal is None:
        raise _Falha(ACAO, f'nenhum status terminal gravado para o uuid {uuid}')
    if not (_ns_valido(stamp) and _ns_valido(t_terminal)):
        raise _Falha(ACAO, f'cronologia inválida: tempo ROS inválido '
                           f'(aceite {stamp!r}, terminal {t_terminal!r})')
    if stamp > t_terminal:
        raise _Falha(ACAO, f'cronologia inválida: aceite {stamp} ns depois do '
                           f'primeiro terminal {t_terminal} ns')
    return TERMINAIS[terminal], stamp, t_terminal


def _confere_log(log, resultado):
    finais = re.findall(r'^Goal finished with status: (\w+)', log,
                        re.MULTILINE)
    if finais != [resultado]:
        raise _Falha(ACAO, f'objetivo.log diz {finais}, o status gravado diz '
                           f'{resultado}')


# ── tolerância, placa e grafo ───────────────────────────────────────────────

def _tolerancia(dump_controller):
    p = _parametros(dump_controller, '/controller_server', TOL,
                    'dump do /controller_server')
    ids = p.get('goal_checker_plugins')
    if not isinstance(ids, list) or len(ids) != 1:
        raise _Falha(TOL, 'goal_checker_plugins precisa de exatamente um id; '
                          f'veio {ids!r}')
    chave = f'{ids[0]}.xy_goal_tolerance'
    if chave not in p:
        raise _Falha(TOL, f'faltou {chave} no dump do /controller_server')
    return chave, p[chave]


def _confere_materializado(texto, chave, vivo):
    m = _parametros(texto, 'controller_server', TOL,
                    'perfil_nav2.yaml materializado')
    if chave not in m:
        raise _Falha(TOL, f'faltou {chave} no perfil_nav2.yaml materializado')
    if m[chave] != vivo:
        raise _Falha(TOL, f'{chave} vivo {vivo!r} diverge do materializado '
                          f'{m[chave]!r}')


def _placa(dump_placa):
    """Devolve a placa PARCIAL que houver; o que faltou vai na falha e o juiz
    diz, ele mesmo, qual campo faltou."""
    p = _parametros(dump_placa, '/placa_simulada', PLACA,
                    'dump da /placa_simulada')
    placa = {c: p[c] for c in CAMPOS_PLACA if c in p}
    faltando = [c for c in CAMPOS_PLACA if c not in p]
    return placa, faltando


def _grafo(texto, onde):
    """Publicadores e assinantes de um `ros2 topic info -v`, como MULTICONJUNTO
    ordenado: a ordem em que o rclpy lista endpoints não é topologia."""
    if not isinstance(texto, str):
        raise _Falha(GRAFO, f'{onde} ilegível')
    contagens = dict(re.findall(r'^(Publisher|Subscription) count: (\d+)\s*$',
                                texto, re.MULTILINE))
    if set(contagens) != {'Publisher', 'Subscription'}:
        raise _Falha(GRAFO, f'{onde} ilegível: sem as contagens de '
                            'publicadores e assinantes')
    pubs, subs = [], []
    nome = espaco = None
    for linha in texto.splitlines():
        if linha.startswith('Node name: '):
            nome = linha[len('Node name: '):].strip()
        elif linha.startswith('Node namespace: '):
            espaco = linha[len('Node namespace: '):].strip()
        elif linha.startswith('Endpoint type: '):
            if nome is None or espaco is None:
                raise _Falha(GRAFO, f'{onde} ilegível: endpoint sem nó')
            completo = f'{espaco.rstrip("/")}/{nome}'
            tipo = linha[len('Endpoint type: '):].strip()
            {'PUBLISHER': pubs, 'SUBSCRIPTION': subs}.get(tipo, []).append(
                completo)
            nome = espaco = None
    if len(pubs) != int(contagens['Publisher']) \
            or len(subs) != int(contagens['Subscription']):
        raise _Falha(GRAFO, f'{onde} ilegível: contagens não batem com os '
                            'endpoints listados')
    return {'publicadores': sorted(pubs), 'assinantes': sorted(subs)}


def _amostra(a):
    """`t` é o instante de GRAVAÇÃO; o `header_ns` é herdado do comando de
    entrada (`placa_simulada.py:431`) e não entra. Tempo inválido deixa `t`
    ausente, e o juiz diz qual amostra."""
    if not isinstance(a, dict):
        return a
    saida = {k: a[k] for k in ('topico', 'v', 'wz') if k in a}
    if _ns_valido(a.get('t_ns')):
        saida['t'] = _segundos(a['t_ns'])
    return saida


# ── a montagem ──────────────────────────────────────────────────────────────

def monta(brutos):
    evidencia, falhas = {}, []

    def tenta(passo):
        try:
            passo()
        except _Falha as f:
            falhas.append((f.fonte, f.detalhe))

    for chave in ('goal', 'pose_inicial', 'pose_final'):
        if chave in brutos:
            evidencia[chave] = brutos[chave]
    evidencia['amostras'] = [_amostra(a) for a in brutos.get('amostras') or []]

    def acao():
        resultado, aceite, terminal = _acao(brutos.get('objetivo_log'),
                                            brutos.get('status'))
        evidencia['resultado_acao'] = resultado
        evidencia['janela'] = {'objetivo_aceito': _segundos(aceite),
                               'resultado': _segundos(terminal)}
        _confere_log(brutos['objetivo_log'], resultado)

    def tolerancia():
        chave, vivo = _tolerancia(brutos.get('dump_controller'))
        evidencia['xy_goal_tolerance'] = vivo
        _confere_materializado(brutos.get('nav2_materializado'), chave, vivo)

    def placa():
        parcial, faltando = _placa(brutos.get('dump_placa'))
        evidencia['placa'] = parcial
        if faltando:
            raise _Falha(PLACA, f'faltou {", ".join(faltando)} no dump da '
                                '/placa_simulada')

    def grafo():
        antes = _grafo(brutos.get('grafo_antes'), 'grafo antes do goal')
        depois = _grafo(brutos.get('grafo_depois'), 'grafo depois do resultado')
        if antes != depois:
            raise _Falha(GRAFO, f'as duas capturas do grafo diferem: antes '
                                f'{antes}, depois {depois}')
        evidencia['grafo'] = {TOPICO_FINAL: antes}

    for passo in (acao, tolerancia, placa, grafo):
        tenta(passo)
    return evidencia, falhas
