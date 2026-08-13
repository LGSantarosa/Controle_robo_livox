"""Freio LINEAR por contra-torque — lógica pura, sem ROS.

## Por que existe, e por que ele é o eixo que estava batendo

A 037 mediu e implementou o freio de GIRO. Este é o mesmo mecanismo no eixo que
ficou de fora — e era o eixo da colisão. Medido em 13-08, na corrida da porta
gravada (`docs/dados/2026-08-13-re-na-porta/volta-pra-sala.csv`):

    t=21,85 s   o reflexo ZERA a saída       folga 0,30 m
    t=22,31 s   o robô para                  folga 0,20 m
    -------------------------------------------------------
    +0,10 m andados com o comando em ZERO, e 0,20 < 0,2275 (meia-largura)

O reflexo não falhou: cortou o comando a 0,30 m da parede. O que não existia era
o freio. A mesma retenção de `atraso_desliga` (0,52 s, medida no robô em 04-08)
que segura o giro segura a marcha.

## O que a bancada mediu (13-08, `tools/banco/freio_linear.py`, Gazebo)

    limiar  freou por   SOBRA
     0.35     0.33 s   -0.006 m
     0.25     0.44 s   -0.010 m
     0.25     0.04 s   +0.018 m
     0.20     0.00 s   +0.107 m   <- não engatou: é a LINHA DE BASE
     0.20     0.00 s   +0.127 m   <- idem
     0.15     0.54 s   -0.020 m
     0.15     0.50 s   -0.084 m   <- passou do ponto e voltou 8 cm

As duas linhas que não engataram são a melhor testemunha do ensaio: **sem freio
a sobra é +0,107 e +0,127 m**, em cima dos +0,100 m medidos na batida. Com o
freio engatado ela fica entre −0,084 e +0,018 m.

⚠️ **A varredura NÃO resolve o limiar ótimo** e está escrito assim no diário: o
portão `pico_min` da bancada domina o instante de soltar, então o que ficou
medido é "engatou / não engatou", não "0,25 é melhor que 0,15". `solta_em=0.25`
é o valor cujas duas corridas ficaram mais apertadas (−0,010 e +0,018), não um
ótimo demonstrado. Para a segurança tanto faz: qualquer engate corta a invasão
em ~10x, e o preço do exagero é recuar alguns centímetros PARA LONGE da parede.

## Por que o freio mora na ÚLTIMA camada (`compensador_rumo`)

Porque quem corta o comando é o reflexo, e o reflexo está ACIMA. Um freio antes
do `collision_monitor` teria o contra-torque zerado pelo próprio reflexo — ele
morreria exatamente no instante em que é preciso. O argumento já estava escrito
no `twist_mux.yaml` para o arco: *"o arco do corpo é do robô, não da fonte"*. A
inércia da placa também é do robô. Freio que alguém pode vetar não é freio.

## O que esta lei NÃO faz

Não decide parar — isso é do reflexo, do humano ou da autonomia. Ela responde:
*"você parou de pedir marcha e o robô ainda anda a esta velocidade; o que mando
agora?"*. E não freia curva: o eixo de giro é da `FreioDeGiro` (037), que roda
no `heading_controller`. Os dois são independentes e podem atuar juntos.
"""
import math

FREANDO = 'freando'
SOLTO = 'solto'


class FreioLinear:
    """Contra-torque enquanto a marcha no sentido de origem passar do limiar.

    Uso, uma vez por ciclo de controle::

        v_saida = freio.passo(v_pedido, v_medido, dt)

    `v_medido` tem de ser a velocidade **com sinal** (projeção do passo da pose
    no rumo do robô), nunca o módulo: sem o sinal o freio não distingue "ainda
    indo para a parede" de "já voltando", e fica preso ao contra-torque até o
    teto. É o defeito que a bancada do giro registrou com −300° em 14-08.
    """

    def __init__(self, v_comando=0.5, solta_em=0.25, pico_min=0.12,
                 teto_s=1.0, parado=0.02):
        if solta_em <= 0.0:
            raise ValueError('solta_em tem de ser positivo — é uma velocidade')
        # ⚠️ `v_comando` é irrelevante em MÓDULO: entre 0,008 e 0,838 m/s a
        # placa entrega SEMPRE 0,298 (decisão 020, medido em 31-07). O que
        # importa aqui é o SINAL. Fica como parâmetro porque quem consome o
        # comando lá embaixo continua sendo a zona morta do atuador.
        self.v_comando = v_comando
        self.solta_em = solta_em
        # A inércia aparece DEPOIS do corte. Sem esperar o pico, o freio solta
        # na subida e não freia nada — o defeito das duas primeiras varreduras
        # do giro em 14-08, repetido aqui de propósito porque a planta é a mesma.
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

    def passo(self, v_pedido, v_medido, dt):
        """Devolve o `v` a comandar neste ciclo."""
        # Quem quer andar manda; freio não disputa com quem está dirigindo.
        if abs(v_pedido) > self.parado:
            if self.estado == FREANDO:
                self.reset()
                self.motivo = 'voltaram a pedir marcha'
            return v_pedido

        if self.estado == SOLTO:
            if abs(v_medido) <= self.solta_em:
                return 0.0            # não há inércia que justifique frear
            self.estado = FREANDO
            self.sentido = math.copysign(1.0, v_medido)
            self.maior = abs(v_medido)
            self.t = 0.0

        self.t += dt
        # COMPONENTE NO SENTIDO DE ORIGEM, nunca o módulo — ver a docstring da
        # classe: com o módulo o freio nunca solta depois de inverter a marcha.
        no_sentido = v_medido * self.sentido
        self.maior = max(self.maior, no_sentido)

        if self.t >= self.teto_s:
            self.reset()
            self.motivo = 'teto de tempo'
            return 0.0
        if self.maior >= self.pico_min and no_sentido <= self.solta_em:
            self.reset()
            self.motivo = 'marcha caiu abaixo do limiar'
            return 0.0

        # O contra-torque segue o sentido de ORIGEM e não o do instante: perto
        # do zero o sinal medido oscila e o freio ficaria batendo palma.
        return -self.sentido * self.v_comando
