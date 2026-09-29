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
import re

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


def repouso_sustentado(amostras, v_parado=0.005, t_repouso=0.30,
                       wz_parado=0.02, buraco_max=0.15, min_amostras=4):
    """O robô está parado de verdade? `(ok, motivo)`.

    `amostras` é `[(t, vx)]` ou `[(t, vx, wz)]`. As mesmas réguas do juiz
    (`tools/analise_corrida/freio_e_porta.py`) de propósito: se o executor
    usasse um critério e o juiz outro, "o robô parou" significaria duas coisas
    na mesma sessão.

    🔴 **Cobertura, não só duração.** A primeira versão aceitava DUAS amostras
    isoladas, uma em t=0 e outra em t=0,31: a janela tinha 0,31 s e as duas
    estavam zeradas, então "parado". Entre elas o robô podia ter andado um
    metro. Agora a janela precisa de `min_amostras` e nenhum buraco maior que
    `buraco_max`.

    ⚠️ E confere `wz` quando ele vem: robô girando no lugar tem `vx` zero e não
    está parado — na porta, girar muda a pose de entrada tanto quanto andar.
    """
    if not amostras:
        return False, 'nenhuma amostra de velocidade'
    norm = [(a[0], a[1], (a[2] if len(a) > 2 else None)) for a in amostras]
    norm.sort(key=lambda a: a[0])
    fim = norm[-1][0]
    janela = [a for a in norm if a[0] >= fim - t_repouso]
    if norm[-1][0] - norm[0][0] < t_repouso:
        return False, (f'só {norm[-1][0] - norm[0][0]:.2f} s de amostras, '
                       f'precisa de {t_repouso:.2f} s')
    if len(janela) < min_amostras:
        return False, (f'só {len(janela)} amostra(s) nos últimos '
                       f'{t_repouso:.2f} s, precisa de {min_amostras}')
    cobertura = janela[-1][0] - janela[0][0]
    if cobertura < t_repouso * 0.9:
        return False, (f'a janela cobre só {cobertura:.2f} s dos '
                       f'{t_repouso:.2f} s exigidos')
    buracos = [janela[i][0] - janela[i - 1][0] for i in range(1, len(janela))]
    if buracos and max(buracos) > buraco_max:
        return False, (f'buraco de {max(buracos):.2f} s entre amostras '
                       f'(máximo {buraco_max:.2f} s): a janela não é contínua')
    pico = max(abs(a[1]) for a in janela)
    if pico > v_parado:
        return False, (f'ainda se movendo: |vx| chegou a {pico:.4f} m/s nos '
                       f'últimos {t_repouso:.2f} s')
    giros = [abs(a[2]) for a in janela if a[2] is not None]
    if giros and max(giros) > wz_parado:
        return False, (f'ainda girando: |wz| chegou a {max(giros):.4f} rad/s')
    return True, (f'parado há {cobertura:.2f} s, {len(janela)} amostras '
                  f'(|vx| <= {pico:.4f} m/s)')


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


def pose_de_yaml(texto):
    """A pose de um `ros2 topic echo --once --field pose.pose /Odometry`.

    🔴 Nada de regex por `[xyzw]` solto. A primeira versão pegava as quatro
    primeiras ocorrências e misturava `position` com `orientation`: contra um
    `/Odometry` real em (2,0; 5,0) ela devolvia x = 0,0, porque o `x` de
    `orientation` sobrescrevia o de `position`. Aqui o YAML é lido como YAML e
    os dois blocos ficam separados.
    """
    if not texto or not texto.strip():
        return None
    try:
        import yaml
    except ImportError:
        return None
    # o `echo` pode emitir várias mensagens separadas por `---`
    for bloco in [b for b in texto.split('---') if b.strip()]:
        try:
            d = yaml.safe_load(bloco)
        except yaml.YAMLError:
            continue
        if not isinstance(d, dict):
            continue
        # com `--field pose.pose` o dicionário JÁ é a pose; sem ele, desce
        for caminho in ([], ['pose'], ['pose', 'pose']):
            alvo = d
            for chave in caminho:
                alvo = alvo.get(chave) if isinstance(alvo, dict) else None
            if isinstance(alvo, dict) and 'position' in alvo:
                p = alvo['position']
                o = alvo.get('orientation') or {}
                return {'x': float(p['x']), 'y': float(p['y']),
                        'yaw': 2 * math.atan2(float(o.get('z', 0.0)),
                                              float(o.get('w', 1.0)))}
    return None


def twist_de_yaml(texto):
    """`[(vx, wz)]` de um echo de `/Odometry` com `--field twist.twist`."""
    if not texto or not texto.strip():
        return []
    try:
        import yaml
    except ImportError:
        return []
    saida = []
    for bloco in [b for b in texto.split('---') if b.strip()]:
        try:
            d = yaml.safe_load(bloco)
        except yaml.YAMLError:
            continue
        if not isinstance(d, dict):
            continue
        for caminho in ([], ['twist'], ['twist', 'twist']):
            alvo = d
            for chave in caminho:
                alvo = alvo.get(chave) if isinstance(alvo, dict) else None
            if isinstance(alvo, dict) and 'linear' in alvo:
                saida.append((float(alvo['linear'].get('x', 0.0)),
                              float((alvo.get('angular') or {}).get('z', 0.0))))
                break
    return saida


TERMINAIS = ('SUCCEEDED', 'ABORTED', 'CANCELED', 'REJECTED', 'UNKNOWN')


def terminal_da_acao(texto):
    """Aceite, UUID e estado terminal de um `ros2 action send_goal -f`.

    Devolve `(aceito, uuid, estado)`. O Jazzy escreve `Goal accepted with ID:`
    — a primeira versão procurava `goal_id=`, que não existe nessa saída, e o
    UUID nunca era gravado.

    ⚠️ Os três resultados são separados de propósito: coleta válida, terminal
    real e resultado científico não são a mesma coisa. Um `ABORTED` genuíno
    pode ser o RESULTADO da condição testada — apagar ou repetir em silêncio
    seria descartar justamente o que o A/B foi medir.
    """
    if not texto:
        return False, None, 'UNKNOWN'
    m = re.search(r'Goal accepted with ID:\s*([0-9a-fA-F.]+)', texto)
    uuid = m.group(1) if m else None
    aceito = bool(m) or 'Goal accepted' in texto
    if re.search(r'Goal was rejected|goal was rejected', texto):
        return False, uuid, 'REJECTED'
    m2 = re.search(r'Goal finished with status:\s*([A-Z_]+)', texto)
    if m2:
        estado = m2.group(1).replace('STATUS_', '')
        return aceito, uuid, estado if estado in TERMINAIS else 'UNKNOWN'
    return aceito, uuid, 'UNKNOWN'


def contagens_do_bag(metadata_texto, exigidos, exigem_mensagem=()):
    """Tópicos e CONTAGENS do `metadata.yaml`. `(ok, motivo, contagens)`.

    Conferir só nomes deixa passar tópico assinado que não recebeu nada. E a
    distinção que interessa ao critério 6 da 062: `/unstuck_vel` **gravado com
    zero mensagens** é "zero escapes"; `/unstuck_vel` **ausente** é "não
    medido". Por isso ele entra em `exigidos` mas não em `exigem_mensagem`.
    """
    if not metadata_texto:
        return False, 'metadata.yaml ausente ou vazio', {}
    try:
        import yaml
        d = yaml.safe_load(metadata_texto) or {}
    except Exception:  # noqa: BLE001 - metadata corrompido é o próprio defeito
        return False, 'metadata.yaml ilegível', {}
    info = (d.get('rosbag2_bagfile_information') or {})
    contagens = {}
    for t in info.get('topics_with_message_count') or []:
        meta = (t.get('topic_metadata') or {})
        contagens[meta.get('name')] = int(t.get('message_count') or 0)
    faltando = [t for t in exigidos if t not in contagens]
    if faltando:
        return False, f'tópicos fora do bag: {", ".join(faltando)}', contagens
    vazios = [t for t in exigem_mensagem if contagens.get(t, 0) == 0]
    if vazios:
        return False, f'tópicos gravados sem mensagem: {", ".join(vazios)}', \
            contagens
    return True, (f'{len(contagens)} tópicos no bag, os exigidos inclusive'), \
        contagens
