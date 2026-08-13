#!/usr/bin/env python3
"""Compensador de rumo — a camada que faz "reto" significar reto.

Este robô comandado a ir reto descreve um círculo de 1,22 m de raio
(−0,817 1/m de frente; de ré −0,098, medido em 04-08). Este nó fica entre
quem comanda e o atuador: entra o `cmd_vel` desejado, sai o `cmd_vel`
corrigido, com o yaw do `/Odometry` fechando a malha. A lei (feedforward
medido + PI) vive em `lei_de_reta.py`, pura e testada sem ROS; o racional
com os números é a decisão 011.

Serve QUALQUER comandante — bancada, teleop, seguidor, Nav2 — porque
corrigir fidelidade de comando é problema de todos eles e não deve ser
resolvido dentro de nenhum.

Tópicos:
    entra  ~/cmd_vel     geometry_msgs/TwistStamped   o comando desejado
           /Odometry     nav_msgs/Odometry            pose (LIO ou Gazebo)
    sai    /hoverboard_base_controller/cmd_vel        o comando corrigido
           (no simulador, remapear a saída para /cmd_vel_bruto, como o
            heading_controller já faz no navegacao.launch.py)

⚠️ Sem pose fresca o nó NÃO corrige: passa o comando adiante INTOCADO e
avisa alto. A alternativa (segurar o comando) violaria a prioridade do
humano; corrigir sem sensor é impossível; e fazer qualquer um dos dois em
silêncio é o defeito do BO-3. O robô volta a arcar — mas dizendo por quê.
"""
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from robot_motion.heading_controller import yaw_de
from robot_motion.lei_de_freio_linear import FreioLinear
from robot_motion.lei_de_reta import MalhaDeReta, herdado_ff


class CompensadorRumo(Node):
    def __init__(self):
        super().__init__('compensador_rumo')

        p = self.declare_parameters('', [
            # A curvatura MEDIDA do robô comandado reto (bancada 04-08) —
            # com sinal, na convenção da bancada (Δyaw/caminho).
            ('curv_frente', -0.817),
            ('curv_re', -0.098),
            # ⚠️ OS DOIS ACIMA SÃO O ÚLTIMO VALOR VISTO, NÃO UMA CONSTANTE DO
            # ROBÔ — decisão 013, caminho 3 (escolhido pelo dono em 07-08).
            #
            # A curvatura crua muda de um dia para o outro mais do que muda
            # dentro do dia:
            #
            #     04-08   média −0,8031   faixa −0,8282 a −0,7832   disp. 3%
            #     05-08   média −0,9116   faixa −0,9310 a −0,8958   disp. 2%
            #
            # 13,5% entre dias, com as faixas NÃO se tocando. Não é ruído de
            # medida: é a planta em dois estados. Nenhum valor fixo serve para
            # os dois, e o limiar de aceitação da 011 (|curvatura| < 0,05) cai
            # dentro da faixa em que ela se move sozinha.
            #
            # O protocolo é medir três corridas SEM compensador no começo da
            # sessão e subir a pilha com o valor do dia:
            #
            #   ros2 launch robot_motion pilha.launch.py mapa:=nenhum \
            #       curv_frente:=-0.9116 curv_medido_em:=2026-08-05
            #
            # `curv_medido_em` não entra na conta — entra no LOG. Sem ele, o
            # nó sobe anunciando um número que parece medido e é herdado, e a
            # bancada mede um robô que não existe (o defeito da bitola, 29-07).
            # A régua que produz o valor e a linha pronta para colar é
            # `medir.py --resumo curvatura`.
            ('curv_medido_em', 'HERDADO'),
            # ⚠️ GANHOS REDUZIDOS 4,1x EM 05-08, e o número saiu do robô.
            #
            # Com kp=1,0 e ki=0,5 o robô OSCILA, e a oscilação CRESCE — não é
            # transiente. Medido nas três corridas da bancada de 05-08
            # (`docs/dados/2026-08-05-bancada-robo/A2-com-comp-frente-*.csv`):
            #
            #     1ª excursão  −2,97 / −6,47 / −5,43 graus
            #     2ª excursão  +6,46 / +12,04 / +11,84
            #     cresce 2,07x por meio-período, meio-período 2,46 s
            #
            # É o S que o dono viu a olho ("de frente ele faz um pequeno S para
            # tentar compensar o erro"). A causa é TEMPO MORTO, e o número
            # fecha por duas rotas independentes:
            #
            #   · a frequência da oscilação (1,277 rad/s) com este PI exige
            #     0,94 s de atraso puro no laço;
            #   · a placa MEDIDA tem 0,27 s de atraso de liga (01-08) mais
            #     0,52 s de desliga (04-08) = 0,79 s, mais pose a 10 Hz e a
            #     janela de 0,2 s da velocidade -> ~0,94 s.
            #
            # Crescer 2,07x por meio-período põe o ganho de laço em ~2,07 na
            # travessia de fase; ele precisa ficar abaixo de 1. Reduzindo os
            # dois JUNTOS por 4,1x (a razão ki/kp não muda — é ganho a menos,
            # não controlador diferente) o laço vai para |L| ~ 0,5, ou seja
            # margem de ganho 2x.
            #
            # PREÇO, calculado: o feedforward erra ~0,095 1/m (planta de 05-08
            # em −0,9116 contra ff −0,817), o que pede 0,028 rad/s de correção
            # contínua a 0,30 m/s. Com kp=0,25 isso vira ~6,5° de erro de rumo
            # enquanto o integrador não o come, contra 1,6° antes. Trocar ±12°
            # CRESCENDO por 6,5° assentando é o negócio.
            #
            # ⚠️ NÃO foi possível verificar isto no simulador, e a razão está
            # em `docs/dados/2026-08-05-ki-no-simulador/`: lá o feedforward
            # cancela o arco por construção (a placa fingida e este nó usam o
            # MESMO −0,817), e a boba é um patim (BO-4). O simulador não
            # oscila em ganho nenhum — varrer ki de 1,0 a 0,0 não mudou uma
            # única inversão. O projeto veio da planta medida, que é o que
            # sobra quando o simulador não pode arbitrar.
            #
            # ⏳ A CONFERIR NO ROBÔ (`docs/PLANO_SINTONIA_RUMO.md`): a mesma
            # corrida tem de mostrar a amplitude DECAINDO em vez de crescer.
            # Se ela continuar crescendo com estes ganhos, o S não é do laço e
            # o caminho passa a ser o BO-4.
            ('kp', 0.25),           # [1/s]  wz por rad de erro de rumo
            ('ki', 0.12),           # [1/s²] wz por rad·s acumulado
            ('wz_max', 0.6),        # [rad/s] grampo da correção
            # ⚠️ FICA EM 0,6, e isto foi DECIDIDO com medida em 05-08, não
            # herdado. Ao baixar `ki` 4,1x eu subi este teto para 2,5 achando
            # que o que precisava ser preservado era a AUTORIDADE do
            # integrador (`ki · int_max`). O teste de fechamento derrubou:
            #
            #   int_max   ff certo: 2ª excursão | curvatura    ff 25% errado: curv
            #     0,6          0,4°  | −0,0012                      0,0007
            #     1,0          7,1°  | +0,0061                     −0,0026
            #     2,5          9,0°  | +0,0076                     +0,0053
            #
            # Com tempo morto, `int_max` não é só teto de autoridade: é a
            # proteção contra WINDUP, que é justamente o que produz sobrepasso.
            # Subir para 2,5 trouxe o sino de volta (9° de segunda excursão) —
            # desfazendo a mudança que veio matá-lo.
            #
            # Em 0,6 a curvatura fica em 0,0007 e −0,0012 nos dois casos, três
            # ordens de grandeza abaixo do critério de 0,05 da 011. O preço é
            # que, com o ff 25% errado, o rumo assenta ~6,8° fora da referência
            # capturada (o integrador satura e o P sustenta o resto com erro).
            # ➡️ O conserto disso é o FEEDFORWARD estar certo, não o integrador
            # brigar — ver `curv_frente` acima.
            ('int_max', 0.6),       # [rad·s] anti-windup
            ('limiar_curva', 0.05),  # [rad/s] acima disso é curva: passa
            # False quando há controlador de rumo ACIMA (a pilha): aí este nó
            # só cancela o arco e não disputa a direção. Ver `lei_de_reta`.
            ('segura_rumo', True),
            # Pose mais velha que isto = sem sensor: passa reto e grita.
            ('validade_pose', 0.5),  # [s]

            # --- preditor de Smith, OPT-IN (racional em `lei_de_reta.py`) ---
            # Desligado por padrão de propósito: o caminho de produção é o de
            # ganho baixo, que não depende de modelo nenhum. O preditor troca
            # "ganho baixo" por "descontar o atraso", e com isso permite subir
            # os ganhos de volta — mas paga com dependência do modelo.
            #
            # Para comparar os dois no robô, lado a lado (`PLANO_SINTONIA_RUMO`):
            #   ros2 run robot_motion compensador_rumo --ros-args \
            #       -p preditor:=true -p kp:=1.0 -p ki:=0.5
            ('preditor', False),
            ('preditor_atraso', 0.94),   # [s] o atraso MEDIDO do laço
            # 1,0 SUBESTIMA (o realizado é ~1,19x o comandado na faixa reta).
            # Subestimar degrada em direção ao caso sem preditor — lado seguro.
            ('preditor_ganho', 1.0),
            ('preditor_max', 0.35),      # [rad] grampo da previsão (~20°)

            # --- estimador do ff, OPT-IN (decisão 013 caminho 2) ---
            # O racional inteiro, com os números de 10-08, está em
            # `lei_de_reta.py`, ao lado dos atributos `adapta_*`. Resumo:
            # a curvatura crua deriva +19% em 5,4 minutos, então o `curv_frente`
            # medido no começo da sessão envelhece dentro dela; e o integrador
            # não cobre a diferença porque é rápido demais (tempo morto de
            # 0,94 s) e some a cada parada. O estimador drena para a curvatura,
            # devagar e sem solavanco, o que o integrador segura em regime.
            #
            #   ros2 run robot_motion compensador_rumo --ros-args \
            #       -p adapta:=true -p curv_frente:=<a média do dia>
            #
            # ⚠️ Régua de aceitação: 2,5 m (~10 s). Corrida de 1,2 m NÃO julga
            # rumo — em 10-08 a mesma corrida deu −0,0162 (passa) cortada em
            # 1,2 m e +0,0882 (reprova) medida inteira.
            ('adapta', False),
            ('adapta_t', 8.0),           # [s] constante de tempo da drenagem
            ('adapta_desvio_max', 0.5),  # [1/m] quanto pode fugir da semente
            ('adapta_v_min', 0.05),      # [m/s] abaixo disso não estima

            # --- ganho da cadeia de giro (decisão 038) ---
            # A FRAÇÃO do `wz` pedido que o robô entrega. Medido em 13-08 no
            # Gazebo: 0,45 (varredura em `ganho_de_giro.py`). O racional inteiro
            # está em `lei_de_reta.py`, ao lado do atributo.
            #
            # ⚠️ 1,0 aqui é "não sei": o perfil do simulador traz o 0,45 medido
            # e o robô real fica em 1,0 até alguém medir LÁ. Mesma regra do
            # `curv_medido_em` (013) — knob de planta não se herda entre
            # máquinas, e este projeto já pagou por isso.
            ('ganho_wz', 1.0),
            ('wz_cmd_max', 2.2),   # [rad/s] o módulo da placa (020)

            # --- freio LINEAR (decisão 038), LIGADO por padrão ---
            # Ele mora aqui, na ÚLTIMA camada, porque quem corta o comando é o
            # reflexo, que está ACIMA: um freio antes do `collision_monitor`
            # teria o próprio contra-torque zerado por ele, morrendo no
            # instante em que é preciso. Mesmo argumento do arco, escrito no
            # `twist_mux.yaml`: "o arco do corpo é do robô, não da fonte" — a
            # inércia da placa também é. Freio vetável não é freio.
            #
            # Ligado por padrão porque a alternativa é o estado de 13-08: o
            # reflexo corta a 0,30 m da parede e o robô come 0,10 m dela.
            ('freio_linear', True),
            ('freio_solta_em', 0.25),    # [m/s] solta com o robô AINDA andando
            ('freio_pico_min', 0.12),    # [m/s] "a inércia já apareceu"
            ('freio_teto_s', 1.0),       # [s] teto duro do contra-torque
            ('freio_v_comando', 0.5),    # o módulo é da placa; aqui vale o SINAL
        ])
        par = {x.name: x.value for x in p}

        self.malha = MalhaDeReta(
            curv_frente=par['curv_frente'], curv_re=par['curv_re'],
            kp=par['kp'], ki=par['ki'], wz_max=par['wz_max'],
            int_max=par['int_max'], limiar_curva=par['limiar_curva'],
            segura_rumo=par['segura_rumo'], preditor=par['preditor'],
            preditor_atraso=par['preditor_atraso'],
            preditor_ganho=par['preditor_ganho'],
            preditor_max=par['preditor_max'],
            adapta=par['adapta'], adapta_t=par['adapta_t'],
            adapta_desvio_max=par['adapta_desvio_max'],
            adapta_v_min=par['adapta_v_min'],
            ganho_wz=par['ganho_wz'], wz_cmd_max=par['wz_cmd_max'])
        self.validade_pose = par['validade_pose']
        self.freio = FreioLinear(
            v_comando=par['freio_v_comando'], solta_em=par['freio_solta_em'],
            pico_min=par['freio_pico_min'], teto_s=par['freio_teto_s']
        ) if par['freio_linear'] else None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(
            TwistStamped, '/hoverboard_base_controller/cmd_vel', qos)
        self.create_subscription(TwistStamped, '~/cmd_vel', self.cb_cmd, qos)
        self.create_subscription(Odometry, '/Odometry', self.cb_odom, qos)

        self.yaw = None
        self.t_pose = None
        self.t_passo = None
        self.v_real = 0.0
        self.v_real_sinal = 0.0
        self.hist = []      # (t, x, y) para medir a velocidade de verdade

        self.get_logger().warn(
            f"compensador de rumo (decisão 011): ff {par['curv_frente']:+.3f} "
            f"1/m frente, {par['curv_re']:+.3f} ré; kp={par['kp']:.2f} "
            f"ki={par['ki']:.2f}, correção limitada a ±{par['wz_max']:.2f} "
            f"rad/s. Curva pedida (|wz|≥{par['limiar_curva']:.2f}) passa "
            f"intocada.")
        if par['ganho_wz'] < 1.0:
            self.get_logger().warn(
                f"GANHO DA CADEIA {par['ganho_wz']:.2f} (decisão 038): o que a "
                f"lei quer de giro sai dividido por ele, porque o robô entrega "
                f"só essa fração do que se pede. Com 1,00 o nó volta ao "
                f"comportamento de antes — e 1,00 é o que o ROBÔ REAL usa até "
                f"alguém medir o ganho lá.")
        else:
            self.get_logger().warn(
                "GANHO DA CADEIA 1,00 (não medido nesta máquina): o "
                "cancelamento do arco sai ATENUADO pelo que o atuador deixar "
                "de entregar. No Gazebo isso valeu −0,12 rad/s de arco "
                "sobrando, e era ele que comia a curva suave. Medir com "
                "`tools/banco/ganho_de_giro.py`.")
        if self.freio is not None:
            self.get_logger().warn(
                f"FREIO LINEAR LIGADO (decisão 038): comando cortado com o "
                f"robô andando vira contra-torque, soltando em "
                f"{par['freio_solta_em']:.2f} m/s, teto "
                f"{par['freio_teto_s']:.1f} s. Sem ele a placa segura 0,52 s e "
                f"o robô anda +0,10 m depois do corte — foi a batida de 13-08, "
                f"com o reflexo tendo cortado a 0,30 m da parede.")
        else:
            self.get_logger().error(
                "FREIO LINEAR DESLIGADO — o corte de comando não para este "
                "robô: ele desliza +0,10 m depois de zerar. Só rode assim numa "
                "bancada que precise do deslizamento cru.")
        # De onde veio o feedforward — a linha que separa "medido hoje" de
        # "herdado". Ela é WARN nos dois casos de propósito: o `rosout` é como
        # eu leio a bancada por ssh (o dono só roda), e um INFO se perde no
        # meio do bringup do Nav2.
        if herdado_ff(par['curv_medido_em']):
            self.get_logger().warn(
                f"⚠️ ff HERDADO: curv_frente {par['curv_frente']:+.4f} NÃO foi "
                f"medido nesta sessão. A planta muda ~13,5% entre dias "
                f"(04-08: −0,8031 · 05-08: −0,9116, faixas que não se tocam), "
                f"e o resíduo disso é maior que o critério de aceitação da 011. "
                f"Protocolo (decisão 013): três corridas SEM compensador, "
                f"`medir.py --resumo curvatura`, e subir a pilha com "
                f"`curv_frente:=<média> curv_medido_em:=<data>`.")
        else:
            self.get_logger().warn(
                f"ff MEDIDO em {par['curv_medido_em']}: curv_frente "
                f"{par['curv_frente']:+.4f} 1/m. Se esta data não for a de "
                f"hoje, o valor é de outra sessão e vale como herdado.")
        if par['adapta']:
            self.get_logger().warn(
                f"ESTIMADOR DO ff LIGADO (decisão 013, caminho 2): a curvatura "
                f"parte de {par['curv_frente']:+.4f} e passa a ser aprendida, "
                f"com constante de tempo {par['adapta_t']:.1f} s e desvio "
                f"máximo {par['adapta_desvio_max']:.2f} 1/m da semente. "
                f"Acompanhe por `ros2 topic echo /rosout --field msg | grep "
                f"'^curv_hat'` — se ele encostar no grampo, a semente está "
                f"errada ou o rumo de referência é ruim, não é deriva.")
        if par['preditor']:
            self.get_logger().warn(
                f"PREDITOR DE SMITH LIGADO: descontando {par['preditor_atraso']:.2f} s "
                f"de atraso (ganho {par['preditor_ganho']:.2f}, grampo "
                f"{par['preditor_max']:.2f} rad). Ele DEPENDE do modelo — se o "
                f"atraso real divergir, a correção sai errada. O caminho sem "
                f"modelo é desligá-lo e usar os ganhos baixos.")

    def agora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cb_odom(self, msg):
        self.yaw = yaw_de(msg.pose.pose.orientation)
        self.t_pose = self.agora()
        # Velocidade MEDIDA, da pose. O feedforward precisa dela e não do
        # comando: o arco é curvatura × distância percorrida, e quem decide a
        # distância é o patamar da placa. Janela de 0,2 s para não derivar
        # ruído; fonte é a pose, nunca o campo `twist` (`ensaio.py`).
        p = msg.pose.pose.position
        self.hist.append((self.t_pose, p.x, p.y))
        while len(self.hist) > 2 and self.t_pose - self.hist[0][0] > 0.2:
            self.hist.pop(0)
        if len(self.hist) >= 2:
            (t0, x0, y0), (t1, x1, y1) = self.hist[0], self.hist[-1]
            if t1 - t0 > 1e-4:
                self.v_real = math.hypot(x1 - x0, y1 - y0) / (t1 - t0)
                # COM SINAL, para o freio: o módulo não distingue "ainda indo
                # para a parede" de "já voltando", e um freio cego para isso
                # fica preso ao contra-torque até o teto (medido na bancada do
                # giro em 14-08, −300° de giro). Projeção no rumo atual.
                self.v_real_sinal = ((x1 - x0) * math.cos(self.yaw)
                                     + (y1 - y0) * math.sin(self.yaw)) / (t1 - t0)

    def cb_cmd(self, msg):
        v = msg.twist.linear.x
        wz = msg.twist.angular.z
        t = self.agora()

        sem_pose = (self.yaw is None
                    or t - self.t_pose > self.validade_pose)
        if sem_pose:
            # Nunca em silêncio (BO-3): o robô vai arcar, e o log diz.
            if abs(v) > 1e-9 or abs(wz) > 1e-9:
                self.get_logger().error(
                    'SEM POSE FRESCA no /Odometry — comando passa SEM '
                    'correção de rumo, o robô vai arcar como sempre arcou',
                    throttle_duration_sec=1.0)
            self.malha._descarta()
            # Sem pose não há `v_medido`, e freio sem velocidade medida é
            # chute: ele solta, como a correção de rumo solta. O aviso acima
            # já cobre os dois — o robô volta a arcar E a deslizar no corte.
            if self.freio is not None:
                self.freio.reset()
            return self.publica(msg, v, wz)

        dt = 0.0 if self.t_passo is None else t - self.t_passo
        self.t_passo = t
        saida = self.malha.passo(v, wz, self.yaw, dt, self.v_real)

        # --- FREIO LINEAR (038), depois da malha e antes do atuador ---
        # Zerar o comando não para nada: a placa segura a saída 0,52 s e o robô
        # anda +0,10 m — foi assim que ele comeu a porta em 13-08 com o reflexo
        # tendo cortado a 0,30 m. Aqui o corte de QUALQUER fonte (reflexo,
        # humano, autonomia) vira contra-torque.
        if self.freio is not None:
            v_freio = self.freio.passo(v, self.v_real_sinal, dt)
            if v_freio != v:
                # Contra-torque é PURO: wz vai a zero enquanto ele age. A
                # bancada mediu a sobra assim (só `linear.x`), e a correção de
                # rumo calculada para a marcha da frente teria o sinal errado
                # num comando de ré — meio segundo de esterço ao contrário.
                self.get_logger().warn(
                    f'FREIO LINEAR: cortaram o comando com o robô a '
                    f'{self.v_real_sinal:+.2f} m/s — contra-torque '
                    f'{v_freio:+.2f} ({self.freio.motivo or "freando"})',
                    throttle_duration_sec=0.5)
                return self.publica(msg, v_freio, 0.0)
        # O estimador tem de ser LEGÍVEL na bancada: o dono só roda, e a
        # diferença entre "aprendeu" e "encostou no grampo" não aparece no
        # comportamento — o robô anda torto dos dois jeitos.
        if self.malha.adapta and abs(v) > 1e-9:
            self.get_logger().warn(
                f'curv_hat {self.malha.curv_frente:+.4f} 1/m (frente, semente '
                f'{self.malha.curv_frente_semente:+.4f}) · integrador '
                f'{self.malha.integral:+.3f} rad·s',
                throttle_duration_sec=2.0)
        self.publica(msg, v, saida)

    def publica(self, msg, v, wz):
        fora = TwistStamped()
        fora.header = msg.header
        fora.twist.linear.x = float(v)
        fora.twist.angular.z = float(wz)
        self.pub.publish(fora)


def main():
    rclpy.init()
    no = CompensadorRumo()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        no.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
