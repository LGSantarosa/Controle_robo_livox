"""Pivô por corte previsto — lógica pura, sem ROS.

Fatia 3 da decisão 011. Terceira lei do robô 2, no padrão da casa
(`lei_de_rumo`, `lei_de_reta`): testável sem subir simulador nenhum.

## Por que não é PID

A compensação de zona morta do driver entrega **um único** `wz` (~2,2 rad/s,
medido no robô em 04-08 e confirmado no simulador). Não há magnitude para
modular: pedir 0,1 ou 1,0 rad/s dá a mesma coisa na placa. **A única alavanca
é DECIDIR QUANDO CORTAR** — depois do corte o robô ainda varre ~113°.

## A decisão 005 não morreu; mudou de emprego

A lei de frenagem da 005 (`wz = √(2·a_dec·|e|)`) caiu em 01-08 como
*comando*, porque a placa não obedece magnitude. Mas invertida ela vira um
**critério de corte**, e nessa forma o atuador não atrapalha:

    corta quando   |erro| ≤ wz_atual² / (2·a_dec)

É a mesma física de 27-07 ("distância de frenagem de rumo"), usada como
gatilho em vez de como setpoint. O que a placa destruiu foi o setpoint.

## Por que aproximação sucessiva, e não um corte só

A curva tempo→ângulo depende da taxa de subida do `wz`, e ela tem origens
diferentes no simulador (limite de config, exato) e no robô (inércia, 25% de
espalho — 04-08). Um corte único calibrado aqui erraria lá.

A saída é **errar de propósito para baixo**: `a_dec` conservador (BAIXO) faz
a sobra prevista ficar GRANDE, o corte vir CEDO e o robô parar ANTES do alvo.
Sobrou erro? Outro pulso, menor. Cada pulso é monotônico e o erro só encolhe.
É o achado de 27-07 aplicado de novo: *errar `a_dec` para baixo é de graça,
para cima traz o S de volta.*

## O piso de resolução, e o delator do BO-3

Abaixo de ~0,4 s ligado o robô **não sai do lugar** (zona morta de tempo,
medida em 04-08). Logo existe um pivô mínimo de ~4°, e pedir menos que isso
é pedir o impossível: o comando sai, nada acontece, e sem defesa a lei
ficaria presa para sempre num erro que não sabe corrigir — o BO-3 exato.

Por isso: `tolerancia` nunca abaixo do piso, e um cronômetro que **desiste e
delata** se houver comando sem movimento. Nunca parar em silêncio.
"""
import math

PRONTO = 'pronto'          # dentro da tolerância, nada a fazer
GIRANDO = 'girando'        # comando ligado, esperando a hora de cortar
ASSENTANDO = 'assentando'  # cortado, o robô ainda varre — esperando parar
DESISTIU = 'desistiu'      # orçamento estourou, ou comando sem efeito


def norm_ang(a):
    """Traz um ângulo para (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class PivoPorCorte:
    """Uma instância por manobra: ela carrega estado e orçamento.

    Parâmetros, todos com unidade e todos medidos ou derivados de medida:
      a_dec        [rad/s²] desaceleração suposta para prever a sobra.
                   CONSERVADOR = baixo. Errar para baixo custa tempo; para
                   cima custa sobrepasso, que é o defeito que se quer evitar.
      tolerancia   [rad]    erro aceito. Não pode ficar abaixo do pivô mínimo
                   (~4°, medido): pedir menos é pedir o impossível.
      wz_comando   [rad/s]  o que se pede. A placa entrega o que ela quer;
                   isto existe para o SINAL e para o dia em que houver
                   modulação de verdade.
      limiar_parado[rad/s]  abaixo disso o robô é considerado imóvel.
      max_pulsos   quantas mordidas antes de desistir.
      teto_tempo   [s]      orçamento total da manobra.
      tempo_sem_efeito [s]  comando ligado por mais que isto sem o robô se
                   mexer = delatar (BO-3), não insistir calado.
    """

    # ⚠️ A CONDIÇÃO DE SEGURANÇA DA LEI, e ela é uma desigualdade:
    #
    #       a_dec (o que a lei supõe)  ≤  a_dec real da máquina
    #
    # Só assim a sobra prevista fica ≥ a sobra real, o corte vem cedo e o robô
    # chega POR BAIXO. Violando-a ele passa do alvo, tem de voltar, passa de
    # novo — o S de 27-07 ressuscitado. O padrão 0,6 fica abaixo de tudo que
    # foi medido: 0,67 no simulador (wz baixo) e 0,83 no robô (a_dec da cauda,
    # 04-08). Quem baixar este número paga em pulsos; quem subir paga em
    # sobrepasso, e sobrepasso é o defeito que esta lei existe para não ter.
    def __init__(self, a_dec=0.6, tolerancia=math.radians(6.0),
                 wz_comando=1.0, limiar_parado=0.05, max_pulsos=6,
                 teto_tempo=20.0, tempo_sem_efeito=1.2):
        if a_dec <= 0.0:
            raise ValueError('a_dec tem de ser positivo — é desaceleração')
        if tolerancia < math.radians(4.0):
            # O piso NÃO é opinião: abaixo de ~0,4 s ligado o robô não sai do
            # lugar (04-08). Aceitar tolerância menor é programar um laço que
            # nunca fecha.
            raise ValueError('tolerância abaixo do pivô mínimo medido (~4°)')
        self.a_dec = a_dec
        self.tolerancia = tolerancia
        self.wz_comando = wz_comando
        self.limiar_parado = limiar_parado
        self.max_pulsos = max_pulsos
        self.teto_tempo = teto_tempo
        self.tempo_sem_efeito = tempo_sem_efeito
        self.reset()

    def reset(self):
        self.estado = GIRANDO
        self.pulsos = 1
        self.t = 0.0
        self.t_parado_comandando = 0.0
        self.motivo = None

    def sobra_prevista(self, wz):
        """Quanto o robô ainda vira se o comando for cortado agora [rad]."""
        return wz * wz / (2.0 * self.a_dec)

    def passo(self, erro, wz_real, dt):
        """Um ciclo. Devolve (wz_comandado, estado).

        `erro` é o quanto falta girar [rad], já normalizado; `wz_real` é o giro
        medido pela pose (LIO no robô, verdade no Gazebo).
        """
        self.t += dt

        if self.estado in (PRONTO, DESISTIU):
            return 0.0, self.estado

        if self.t > self.teto_tempo:
            self.motivo = (f'orçamento de {self.teto_tempo:.0f} s estourou com '
                           f'{math.degrees(abs(erro)):.0f}° ainda por girar')
            self.estado = DESISTIU
            return 0.0, self.estado

        if self.estado == ASSENTANDO:
            if abs(wz_real) >= self.limiar_parado:
                return 0.0, ASSENTANDO          # ainda varrendo, deixa parar
            # Parou. Fechou?
            if abs(erro) <= self.tolerancia:
                self.estado = PRONTO
                return 0.0, PRONTO
            if self.pulsos >= self.max_pulsos:
                self.motivo = (f'{self.pulsos} pulsos e ainda faltam '
                               f'{math.degrees(abs(erro)):.0f}° — o resíduo é '
                               f'menor que o pivô mínimo desta máquina?')
                self.estado = DESISTIU
                return 0.0, DESISTIU
            self.pulsos += 1
            self.estado = GIRANDO
            self.t_parado_comandando = 0.0

        # --- GIRANDO ---
        if abs(erro) <= self.tolerancia:
            # Chegou girando: corta e deixa assentar (a sobra ainda vem).
            self.estado = ASSENTANDO
            return 0.0, ASSENTANDO

        # Comando ligado e robô imóvel: ou é a latência de liga (~0,27 s), ou
        # é o BO-3 — pedido abaixo do que a máquina faz. Delatar, não insistir.
        if abs(wz_real) < self.limiar_parado:
            self.t_parado_comandando += dt
            if self.t_parado_comandando > self.tempo_sem_efeito:
                self.motivo = (
                    f'comando de giro há {self.t_parado_comandando:.1f} s e o '
                    f'robô não se mexeu — pedido abaixo do que a máquina faz '
                    f'(BO-3), faltando {math.degrees(abs(erro)):.0f}°')
                self.estado = DESISTIU
                return 0.0, DESISTIU
        else:
            self.t_parado_comandando = 0.0

        if abs(erro) <= self.sobra_prevista(wz_real):
            # A partir daqui a inércia sozinha entrega o que falta.
            self.estado = ASSENTANDO
            return 0.0, ASSENTANDO

        return math.copysign(self.wz_comando, erro), GIRANDO
