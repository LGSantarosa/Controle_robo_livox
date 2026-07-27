"""A lei de movimentação do robô 2 — funções puras, sem ROS.

Está separada do nó de propósito: é ela que carrega a decisão técnica
(`docs/decisoes/005-lei-de-frenagem-de-rumo.md`), e é ela que precisa ser
testável sem subir simulador nenhum.

Nada aqui tem ganho ajustável. Todo parâmetro é uma grandeza física com
unidade do SI, medida no robô pelo banco de `tools/banco/`.
"""
import math


def norm_ang(a):
    """Traz um ângulo para (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def wz_de_frenagem(erro, a_dec, wz_max):
    """Giro que respeita a própria frenagem.

        wz = sinal(e) · min( wz_max , √(2·a_dec·|e|) )

    Nunca pede mais giro do que se consegue frear dentro do erro que ainda
    falta. O termo da raiz é a inversão de `Δrumo = wz²/(2·a_dec)` — a mesma
    conta que explica o S, usada como limite em vez de sofrida como defeito.

    Consequência medida: o sobrepasso não depende do tamanho do erro (0,1° a
    0,8° em erros de 45°, 90° e 180°).
    """
    if a_dec <= 0.0:
        raise ValueError('a_dec tem que ser positivo — é desaceleração física')
    if erro == 0.0:
        return 0.0
    return math.copysign(min(wz_max, math.sqrt(2.0 * a_dec * abs(erro))), erro)


def piso_de_linear(zona_morta, wz, bitola, margem):
    """Linear mínima para as DUAS rodas ficarem acima da zona morta.

    A roda interna anda a `v - |wz|·bitola/2`. Para ela sair do lugar,

        v >= zona_morta + |wz|·bitola/2 + margem

    Sem esse piso, um erro de rumo acima de 90° zera a linear (ver
    `linear_de_avanco`), as duas rodas caem juntas na zona morta e o robô fica
    **parado encarando o erro**, sem erro nenhum no log. Medido no simulador:
    com zona morta de 0,15 m/s, 22 s imóvel com o controlador pedindo 1,0
    rad/s.

    Usa o `wz` do instante, não o teto: só exige avanço na medida da curva que
    está sendo pedida. No pior caso (wz no teto) recai na fórmula da decisão
    005.
    """
    return zona_morta + abs(wz) * bitola / 2.0 + margem


def linear_de_avanco(erro, v_max):
    """Avanço que cede com o desalinhamento: `v = v_max·cos(e)`, sem ré.

    Erro acima de 90° zera a linear — o robô prefere se orientar a avançar na
    direção errada.
    """
    return v_max * max(0.0, math.cos(erro))


def wz_possivel_a(v, zona_morta, bitola, margem):
    """Maior giro que ainda deixa a roda interna acima da zona morta, a esta `v`.

    Inverte o piso: `v >= zona_morta + |wz|·bitola/2 + margem`. Devolve 0
    quando nem parado o robô consegue girar àquela velocidade — o que é uma
    resposta legítima, e não um erro a esconder.
    """
    folga = v - zona_morta - margem
    return max(0.0, 2.0 * folga / bitola)


def comando(erro, v_max, a_dec, wz_max, zona_morta, bitola, margem_piso,
            tolerancia):
    """(v, wz) em SI para um erro de rumo. É o laço de controle inteiro.

    Ordem das defesas, e o porquê de ser esta:

    1. o giro sai da lei de frenagem;
    2. a linear cede com o desalinhamento, mas **sobe até o piso** que tira as
       rodas da zona morta;
    3. o piso nunca ultrapassa a velocidade pedida — andar mais rápido do que
       mandaram é pior do que curvar devagar;
    4. e então o giro é **reduzido ao que aquela velocidade sustenta**. Sem
       este último passo, uma velocidade pedida abaixo do piso derrubaria a
       roda interna de volta na zona morta e o robô travaria (BO-3) — agora
       ele apenas curva mais devagar. Se quem chamou quiser curva mais fechada,
       que peça mais velocidade: a decisão é de quem conhece a rota.

    Reduzir o giro é sempre seguro perante a lei de frenagem: menos giro do que
    se pode frear continua sendo menos giro do que se pode frear.

    Dentro da tolerância o giro é zerado — sem isso o robô fica caçando ruído
    de pose para sempre, e o piso nunca desliga.
    """
    erro = norm_ang(erro)
    if abs(erro) <= tolerancia:
        return v_max, 0.0

    wz = wz_de_frenagem(erro, a_dec, wz_max)
    v = linear_de_avanco(erro, v_max)

    piso = piso_de_linear(zona_morta, wz, bitola, margem_piso)
    v = min(max(v, piso), v_max)

    teto = wz_possivel_a(v, zona_morta, bitola, margem_piso)
    return v, math.copysign(min(abs(wz), teto), wz)


def wz_minimo_parado(zona_morta, bitola):
    """Menor giro possível com o robô PARADO.

    Girando no lugar as rodas andam a ±wz·bitola/2. Abaixo de
    `2·zona_morta/bitola` as duas ficam na zona morta e o robô não gira: não
    existe giro parado devagar. É limite físico, não ajuste de ganho — quem
    precisar apontar o robô no lugar (navegação) tem que conviver com isso.
    """
    return 2.0 * zona_morta / bitola
