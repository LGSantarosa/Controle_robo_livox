#!/usr/bin/env python3
"""A leitura de uma corrida de navegação — sem ROS, para ter teste.

O `corrida_nav.py` grava; este módulo julga. A separação é a mesma do
`nuvem.py`/`nuvem_pontos.py`: o que decide veredito tem de rodar sem robô e sem
rclpy, senão não tem prova.

Uma corrida de navegação tem quatro perguntas, e elas são independentes — o
robô pode chegar ao ponto e mesmo assim a corrida não valer:

    chegou?          distância final ao alvo, contra o raio de chegada
    quem dirigiu?    houve comando humano em /key_vel? -> a corrida é MISTA
    o reflexo agiu?  a autonomia pediu e a saída não saiu -> ele freou
    a pose aguentou? /Odometry a 10 Hz COM o robô andando (é a pergunta do
                     nuvem_pontos, que come CPU convertendo a nuvem)

A última existe porque em 11-08 a ponte da nuvem foi morta durante as corridas
para proteger o FAST-LIO, e a conferência de taxa foi feita com o robô PARADO.
Navegação precisa das duas coisas vivas ao mesmo tempo, e ninguém mediu isso.
"""
import math

# m/s e rad/s — abaixo disso o comando é zero. Mesmo limiar do `homem_morto.py`:
# fica acima do ruído de quantização do encoder e bem abaixo do patamar.
PARADO = 0.02


def distancia(x, y, alvo):
    return math.hypot(alvo[0] - x, alvo[1] - y)


def caminho_andado(linhas):
    """Comprimento do rastro [m]. Salto de pose não é caminho.

    O LIO relocaliza e pula; somar o salto inflaria a tortuosidade e faria uma
    corrida reta parecer errante. 0,5 m entre amostras a 20 Hz seriam 10 m/s.
    """
    total = 0.0
    ant = None
    for l in linhas:
        p = (l['x'], l['y'])
        if ant is not None:
            d = math.hypot(p[0] - ant[0], p[1] - ant[1])
            if d < 0.5:
                total += d
        ant = p
    return total


def intervalos_de_pose(ts):
    """(mediana [s], pior [s]) entre chegadas de `/Odometry`."""
    ds = sorted(b - a for a, b in zip(ts, ts[1:]) if b > a)
    if not ds:
        return None, None
    meio = len(ds) // 2
    mediana = ds[meio] if len(ds) % 2 else (ds[meio - 1] + ds[meio]) / 2.0
    return mediana, ds[-1]


def freadas_do_reflexo(linhas):
    """Trechos em que a autonomia PEDIU e a saída do reflexo não saiu.

    `/auto_vel_raw` é o que o nosso seguidor quer; `/auto_vel` é o que o
    `collision_monitor` deixa passar. Pedido não-nulo com saída nula é o
    reflexo agindo — e é a ÚNICA evidência de que ele agiu, porque parar por
    reflexo e parar por ter chegado se parecem de fora.

    Devolve [(t_inicio, duracao), ...].
    """
    trechos = []
    inicio = None
    ant = None
    for l in linhas:
        freando = abs(l['raw_v']) > PARADO and abs(l['saida_v']) <= PARADO
        if freando and inicio is None:
            inicio = l['t']
        elif not freando and inicio is not None:
            trechos.append((inicio, ant - inicio))
            inicio = None
        ant = l['t']
    if inicio is not None and ant is not None:
        trechos.append((inicio, ant - inicio))
    return trechos


def humano_interveio(linhas):
    """Amostras com comando humano. Corrida com humano no meio é MISTA.

    Não é reprovação — o humano tem prioridade de propósito, e tirar o robô da
    parede é o uso legítimo. Mas uma corrida em que alguém deu um empurrãozinho
    não prova autonomia, e essa contaminação não pode passar despercebida no
    CSV.
    """
    return sum(1 for l in linhas if abs(l.get('key_v', 0.0)) > PARADO)


def replanejamentos(linhas):
    """Quantas vezes o plano MUDOU de tamanho.

    Contagem grosseira de propósito: o Nav2 replaneja a 1 Hz e um plano
    reaproveitado tem o mesmo tamanho. O que interessa aqui não é o número
    exato, é distinguir "planejou uma vez e seguiu" de "ficou replanejando sem
    sair do lugar", que é a assinatura do robô preso.
    """
    n = 0
    ant = None
    for l in linhas:
        if l['plano_n'] and l['plano_n'] != ant:
            n += 1
        ant = l['plano_n']
    return n


def metricas(linhas, alvo, raio_chegada, ts_pose=()):
    if not linhas:
        return None
    fim = linhas[-1]
    reta = distancia(linhas[0]['x'], linhas[0]['y'], alvo)
    andado = caminho_andado(linhas)
    mediana, pior = intervalos_de_pose(list(ts_pose))
    return {
        'duracao': fim['t'] - linhas[0]['t'],
        'dist_final': distancia(fim['x'], fim['y'], alvo),
        'chegou': distancia(fim['x'], fim['y'], alvo) <= raio_chegada,
        'reta': reta,
        'andado': andado,
        'tortuosidade': (andado / reta) if reta > 1e-6 else float('inf'),
        'freadas': freadas_do_reflexo(linhas),
        'amostras_humano': humano_interveio(linhas),
        'replanejamentos': replanejamentos(linhas),
        'pose_mediana': mediana,
        'pose_pior': pior,
        'amostras': len(linhas),
    }


def veredito(m, raio_chegada):
    """As linhas que vão para a tela, cada uma com o número que a produziu."""
    if m is None:
        return ['nada gravado — a pilha estava de pé? o objetivo foi aceito?']
    L = []
    marca = '🟢' if m['chegou'] else '🔴'
    L.append(f"{marca} chegada        {m['dist_final']:.3f} m do alvo "
             f"(raio {raio_chegada:.2f} m) em {m['duracao']:.1f} s")
    L.append(f"   caminho        {m['andado']:.2f} m andados para "
             f"{m['reta']:.2f} m de reta  (tortuosidade {m['tortuosidade']:.2f})")
    L.append(f"   replanejou     {m['replanejamentos']}x")

    if m['freadas']:
        total = sum(d for _, d in m['freadas'])
        L.append(f"🛡️ reflexo        agiu {len(m['freadas'])}x, {total:.1f} s "
                 f"no total (1ª vez em t={m['freadas'][0][0]:.1f} s)")
    else:
        L.append('   reflexo        não agiu — a autonomia passou inteira')

    if m['amostras_humano']:
        L.append(f"⚠️ HUMANO         {m['amostras_humano']} amostras com comando "
                 'em /key_vel — esta corrida é MISTA e não prova autonomia')

    if m['pose_mediana'] is None:
        L.append('🔴 pose           NENHUMA amostra de /Odometry — sem pose não '
                 'há navegação; o FAST-LIO estava de pé?')
    else:
        hz = 1.0 / m['pose_mediana'] if m['pose_mediana'] else 0.0
        ok = m['pose_pior'] <= 0.30
        L.append(f"{'   ' if ok else '🔴 '}pose           {hz:.1f} Hz mediana, "
                 f"pior intervalo {m['pose_pior']:.3f} s"
                 + ('' if ok else '  <- o LIO ENGASGOU com o robô andando; '
                                  'suspeito nº 1 é o nuvem_pontos comendo CPU'))
    return L
