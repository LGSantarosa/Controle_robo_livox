"""Freio de giro por CONTRA-TORQUE — lógica pura, sem ROS.

## Por que existe

Esta máquina **não tem freio**. Zerar o comando não para nada: a placa segura a
saída cheia por `atraso_desliga` (0,52 s, medido no robô em 04-08) e o robô
continua girando. Medido em 14-08, comando de 0,6 s e depois zero:

    fase comandada   3,5°
    SOBRA           60,3°     <- 94% do giro acontece com o comando em zero
    total           63,8°
    pico de wz      ~1,1 rad/s, e ele chega 0,65 s DEPOIS do corte
    para em         ~1,5 s

E isto reproduz a máquina de verdade (06-08, `liga 0,30`): fase comandada ~3°,
sobra 56–68°, parada em 1,86–2,16 s.

O dono, descrevendo o mesmo fenômeno antes de qualquer medida: *"ele chega no
90, mas chega rápido, aí solta o motor, mas a inércia joga ele mais 90 graus até
parar de verdade"*.

## A ideia, e por que ela é a ÚNICA disponível

A placa entrega **um módulo de giro só** (2,204 rad/s) e não sabe desacelerar —
pedir menos não dá menos (decisão 020: o comando escolhe o RAIO, a placa escolhe
o módulo). Então a única forma de tirar energia é **torque contrário**: comandar
o giro oposto. Ela sabe fazer isso, porque o módulo vale nos dois sentidos.

## O número que manda: QUANDO SOLTAR

A retenção de 0,52 s vale para o contra-comando **também**. Soltar o freio com o
robô já parado significa 0,52 s de torque reverso sobrando — e ele gira para o
outro lado. Por isso o freio solta **cedo**, com o robô ainda girando no sentido
de origem. Varredura de 14-08 (Gazebo, `tools/banco/freio_de_giro.py`):

    solta em    giro total    veredito
     1,10 rad/s    27,3°      ok
     1,00          27,7°      ok
     0,90          13,8°      o melhor visto
     0,80          27,8°      ok
     0,60         −50,7°      INVERTEU
     0,40         −60,0°      INVERTEU

⚠️ **O espalho é real e está registrado**: duas corridas no mesmo 0,90 deram
13,8° e 27,0°. Mesma assinatura bimodal que o robô real mostrou com pulso curto
em 06-08 (2,2° contra 29,7°), cuja causa lá foi a quantização do laço de 10 Hz.
O número honesto é "solta entre 0,9 e 1,1 e o giro cai para um terço", não um
ótimo fino.

## O que esta lei NÃO faz

Não escolhe para onde virar, não decide quando parar de girar — isso é de quem
chama. Ela só responde: *"dado que você quer parar de girar e o robô está a esta
taxa, o que mando agora?"*
"""
import math

FREANDO = 'freando'
SOLTO = 'solto'


class FreioDeGiro:
    """Contra-torque enquanto o giro no sentido de origem passar do limiar.

    Uso, uma vez por ciclo de controle::

        wz_saida = freio.passo(wz_pedido, wz_medido, dt)

    `wz_pedido` é o que a lei de rumo queria comandar. Enquanto ela pede giro, o
    freio fica fora do caminho. Quando ela **para de pedir** (|wz_pedido| abaixo
    de `parado`), o freio assume se ainda houver inércia.
    """

    def __init__(self, wz_comando=1.0, solta_em=1.0, pico_min=0.6,
                 teto_s=1.5, parado=0.05):
        if solta_em <= 0.0:
            raise ValueError('solta_em tem de ser positivo — é uma taxa de giro')
        # ⚠️ `wz_comando` é irrelevante em MÓDULO (a placa escolhe o dela) e o
        # que importa é o SINAL. Fica como parâmetro porque quem consome o
        # comando lá embaixo continua sendo a lei de zona morta.
        self.wz_comando = wz_comando
        self.solta_em = solta_em
        # A inércia aparece DEPOIS do corte (pico em +0,65 s). Sem esperar o
        # pico o freio solta na subida e não freia nada — foi o defeito das
        # duas primeiras varreduras de 14-08.
        self.pico_min = pico_min
        self.teto_s = teto_s
        self.parado = parado
        self.reset()

    def reset(self):
        self.estado = SOLTO
        self.sentido = 0.0
        self.maior = 0.0
        self.t = 0.0
        self.motivo = ''

    def passo(self, wz_pedido, wz_medido, dt):
        """Devolve o `wz` a comandar neste ciclo."""
        # Quem quer girar manda; freio não disputa com a lei de rumo.
        if abs(wz_pedido) > self.parado:
            if self.estado == FREANDO:
                self.reset()
                self.motivo = 'a lei voltou a pedir giro'
            return wz_pedido

        if self.estado == SOLTO:
            if abs(wz_medido) <= self.solta_em:
                return 0.0            # não há inércia que justifique frear
            self.estado = FREANDO
            self.sentido = math.copysign(1.0, wz_medido)
            self.maior = abs(wz_medido)
            self.t = 0.0

        self.t += dt
        # ⚠️ COMPONENTE NO SENTIDO DE ORIGEM, nunca o módulo. Testar `|wz|`
        # nunca solta: depois de o freio inverter o giro o módulo volta a subir
        # e o contra-torque fica preso até o teto. Foi medido: −300° de giro.
        no_sentido = wz_medido * self.sentido
        self.maior = max(self.maior, no_sentido)

        if self.t >= self.teto_s:
            self.reset()
            self.motivo = 'teto de tempo'
            return 0.0
        if self.maior >= self.pico_min and no_sentido <= self.solta_em:
            self.reset()
            self.motivo = 'giro caiu abaixo do limiar'
            return 0.0

        # O contra-torque segue o sentido de ORIGEM e não o do instante: perto
        # do zero o sinal medido oscila e o freio ficaria batendo palma.
        return -self.sentido * self.wz_comando
