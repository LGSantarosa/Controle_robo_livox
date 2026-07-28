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
    """Linear que tira a roda interna da banda morta **por cima**.

        v >= zona_morta + |wz|·bitola/2 + margem

    É uma das duas saídas possíveis; a outra é `teto_de_pivo` (por baixo).
    Ver `ajusta_para_zona_morta`.
    """
    return zona_morta + abs(wz) * bitola / 2.0 + margem


def teto_de_pivo(zona_morta, wz, bitola, margem):
    """Linear que tira a roda interna da banda morta **por baixo** — o pivô.

        v <= |wz|·bitola/2 - (zona_morta + margem)

    Girando, a roda interna anda a `v - |wz|·bitola/2`. Ela sai da banda morta
    quando esse valor é grande em MÓDULO — ou seja, andando bastante para
    frente (piso) **ou** bastante para trás (aqui). A roda de dentro girando ao
    contrário é exatamente o que faz o robô virar no lugar.

    Devolve negativo quando o giro pedido é pequeno demais para tirar a roda da
    banda sozinho; nesse caso o pivô não existe e só resta a saída por cima.
    """
    return abs(wz) * bitola / 2.0 - (zona_morta + margem)


def ajusta_para_zona_morta(v_desejada, wz, zona_morta, bitola, margem, v_teto):
    """Empurra (v, wz) para fora da banda morta da roda interna.

    A banda proibida é `|v - |wz|·bitola/2| < zona_morta + margem`. Existem
    **duas** saídas, e considerar só uma delas foi um defeito real desta
    camada: o robô ficava proibido de pivotar por aritmética, não por física,
    e por isso descrevia arcos enormes — chegando a orbitar pontos próximos
    sem nunca alcançá-los.

    Escolhe a saída mais perto da velocidade desejada. Isso faz a coisa certa
    sozinho: com erro de rumo grande a desejada já é baixa (cos) e ele pivota;
    com erro pequeno o giro é pequeno, o pivô nem existe, e ele acelera.
    """
    limiar = zona_morta + margem
    meia = abs(wz) * bitola / 2.0
    if abs(v_desejada - meia) >= limiar:
        return v_desejada, wz               # já está fora da banda

    por_baixo = teto_de_pivo(zona_morta, wz, bitola, margem)
    por_cima = piso_de_linear(zona_morta, wz, bitola, margem)

    pode_pivotar = por_baixo >= 0.0
    cabe_acelerar = por_cima <= v_teto

    if pode_pivotar and (not cabe_acelerar
                         or (v_desejada - por_baixo) <= (por_cima - v_desejada)):
        return por_baixo, wz
    if cabe_acelerar:
        return por_cima, wz

    # Nem pivota nem acelera até o piso: quem cede é o giro.
    #
    # Vai na velocidade MÁXIMA permitida, não na mínima: é ela que deixa mais
    # espaço para a roda interna girar sem cair na banda, e portanto permite o
    # maior giro. Escolher a mínima aqui deixava o robô sem poder curvar nada
    # em certas combinações de parâmetro.
    v = v_teto
    teto_wz = max(0.0, 2.0 * (v - limiar) / bitola)
    return v, math.copysign(min(abs(wz), teto_wz), wz)


def linear_de_avanco(erro, v_max):
    """Avanço que cede com o desalinhamento: `v = v_max·cos(e)`, sem ré.

    Erro acima de 90° zera a linear — o robô prefere se orientar a avançar na
    direção errada.
    """
    return v_max * max(0.0, math.cos(erro))


def comando(erro, v_max, a_dec, wz_max, zona_morta, bitola, margem_piso,
            tolerancia, erro_do_movimento=None):
    """(v, wz) em SI para um erro de rumo. É o laço de controle inteiro.

    1. o giro sai da lei de frenagem, sobre o erro do BICO — é o rumo que ele
       controla;
    2. a linear cede com o desalinhamento do MOVIMENTO, não do bico;
    3. o par é empurrado para fora da banda morta da roda interna, pela saída
       mais próxima — acelerando ou **pivotando**.

    O passo 2 é o que faz o robô parar para virar quando precisa. Usar o erro
    do bico ali foi um defeito medido: este robô **escorrega**. Em órbita
    fechada o bico ficava a 50° do alvo (`cos` = 0,64, segue a 64% da
    velocidade) enquanto o movimento estava a 87° — perpendicular, sem
    aproximação nenhuma. Ele se recusava a parar para virar porque, pelo
    nariz, o rumo não parecia tão errado assim.

    `erro_do_movimento` é opcional: sem estimativa de direção confiável (robô
    quase parado), recai no erro do bico, que é o comportamento antigo.
    """
    erro = norm_ang(erro)
    if abs(erro) <= tolerancia:
        return v_max, 0.0

    wz = wz_de_frenagem(erro, a_dec, wz_max)

    # O mais desalinhado dos dois manda na linear: é o conservador, e evita
    # que uma estimativa ruim de direção acelere o robô.
    erro_linear = erro
    if erro_do_movimento is not None:
        erro_linear = max(abs(erro), abs(norm_ang(erro_do_movimento)))
    v = linear_de_avanco(erro_linear, v_max)

    return ajusta_para_zona_morta(v, wz, zona_morta, bitola, margem_piso, v_max)


def comando_de_re(v_pedida, zona_morta, margem, v_max):
    """Ré RETA: giro zero, módulo acima da zona morta, sinal negativo.

    A ré é um modo à parte, não um sinal que a lei de rumo descobre no meio do
    cálculo. `linear_de_avanco` continua sem ré (`max(0, cos e)`): quem decide
    recuar é a navegação, que conhece a geometria do alvo — esta função só
    executa, e executa reto.

    Reta por decisão, e a decisão protege o que não sabemos: andando para trás
    a boba deixa de ser arrastada e passa a ser empurrada, que é a
    configuração instável do carrinho de supermercado. Curvar nessa condição é
    a manobra sobre a qual não existe medida nenhuma — o simulador não pode
    dar essa resposta, porque a boba dele não chega a virar (ver BO-4). Reta
    também mantém a zona morta simétrica: as duas rodas na mesma velocidade,
    longe da banda proibida.
    """
    if v_pedida >= 0.0:
        raise ValueError('comando_de_re só aceita velocidade negativa — '
                         'a ré é modo explícito, não sinal descoberto')
    modulo = min(max(abs(v_pedida), zona_morta + margem), v_max)
    return -modulo, 0.0


def pivo_disponivel(zona_morta, bitola, margem, wz_max):
    """O robô consegue girar no próprio eixo, com estes números?

    Girando parado a roda interna anda a `-|wz|·bitola/2`; ela só sai da banda
    morta se esse valor superar `zona_morta + margem` em módulo. Logo o pivô
    existe apenas se

        wz_max · bitola/2  >=  zona_morta + margem

    **A bitola decide.** Medido no simulador: com bitola 0,20 e zona morta
    0,10, o pivô exigiria 1,3 rad/s contra um teto de 1,0 — impossível, e o
    robô volta a orbitar pontos próximos. Com a bitola real (0,32) o mesmo
    número dá 0,81 rad/s e o pivô existe. Por isso medir a bitola com trena é
    mais urgente do que medir a zona morta.
    """
    return wz_max * bitola / 2.0 >= zona_morta + margem


def wz_minimo_parado(zona_morta, bitola):
    """Menor giro possível com o robô PARADO.

    Girando no lugar as rodas andam a ±wz·bitola/2. Abaixo de
    `2·zona_morta/bitola` as duas ficam na zona morta e o robô não gira: não
    existe giro parado devagar. É limite físico, não ajuste de ganho — quem
    precisar apontar o robô no lugar (navegação) tem que conviver com isso.
    """
    return 2.0 * zona_morta / bitola
