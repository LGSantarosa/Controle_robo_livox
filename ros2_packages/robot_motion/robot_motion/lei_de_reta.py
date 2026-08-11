"""A malha fechada de rumo em reta — lógica pura, sem ROS.

Está separada do nó (`compensador_rumo.py`) pelo padrão da casa: é ela que
carrega a decisão técnica (`docs/decisoes/011-malha-fechada-de-rumo-em-reta.md`)
e é ela que precisa ser testável sem subir simulador nenhum.

O que ela corrige: este robô comandado a ir reto descreve um círculo
(−0,817 1/m de frente, −0,098 de ré — medido em 04-08). O que a torna
possível: a compensação de zona morta do driver preserva a RAZÃO entre as
rodas, então a curvatura comandada sobrevive ao patamar — há autoridade
contínua sobre o rumo, mesmo sem autoridade nenhuma sobre a velocidade.

    wz_saida = wz_ff + Kp·e + Ki·∫e          e = norm(rumo_ref − yaw)
    wz_ff    = −curvatura_medida(sentido) · |v_cmd|

PI, não PID: o viés é constante (caso de livro do integrador) e um D
derivaria pose de 10 Hz contra um atuador com ~0,27 s de latência — só
amplificaria ruído.
"""
import math


def norm_ang(a):
    """Traz um ângulo para (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


# O que conta como "não foi medido nesta sessão". Mora aqui, e não no nó,
# porque é a mesma pergunta que a lei faz sobre o `curv_frente` que recebe: o
# feedforward vale enquanto o número for do dia (decisão 013, caminho 3).
HERDADO = 'HERDADO'


def herdado_ff(medido_em):
    """O `curv_frente` que está rodando foi medido na sessão, ou é herança?

    Vale como herdado o default (`HERDADO`), o vazio e qualquer texto que
    comece por "herdado" — porque a forma de escrever isso à mão varia
    ("herdado 04-08", "herdado?") e a dúvida tem de cair para o lado que
    AVISA. Data que não se reconhece conta como medida: quem digitou uma data
    afirmou tê-la medido, e cabe ao log mostrar qual é para o dono desmentir.
    """
    t = (medido_em or '').strip()
    return not t or t.upper().startswith(HERDADO)


class MalhaDeReta:
    """Segura o rumo capturado enquanto o comando pedir reta.

    Uma instância por robô: ela carrega estado (referência de rumo e
    integrador), e o estado tem regras próprias de descarte — ver `passo`.

    Parâmetros são grandezas com unidade, não ganhos soltos:
      curv_frente, curv_re  [1/m]   curvatura que o robô descreve comandado
                                    reto — a MEDIDA da bancada, com sinal.
      kp                    [1/s]   quanto de wz por rad de erro de rumo.
      ki                    [1/s²]  quanto de wz por rad·s acumulado.
      wz_max                [rad/s] grampo da CORREÇÃO (não do comando).
      int_max               [rad·s] grampo do integrador (anti-windup).
      limiar_curva          [rad/s] |wz_cmd| a partir do qual o comando é
                                    curva de verdade e passa intocado.
    """

    # ⚠️ `kp` e `ki` reduzidos 4,1x em 05-08 (eram 1,0 e 0,5), porque com os
    # valores antigos o robô OSCILA e a oscilação CRESCE 2,07x por meio-período.
    # É tempo morto: 0,94 s de atraso efetivo no laço, confirmado por duas
    # rotas independentes (a frequência da oscilação medida, e a soma dos
    # atrasos de liga e desliga da placa). O racional completo, com os números
    # e o preço, está no `compensador_rumo.py`, ao lado do `declare_parameters`.
    #
    # ⚠️ Estes defaults têm de acompanhar os do nó — há um teste que compara os
    # dois, porque default duplicado é default que deriva.
    def __init__(self, curv_frente=-0.817, curv_re=-0.098, kp=0.25, ki=0.12,
                 wz_max=0.6, int_max=0.6, limiar_curva=0.05,
                 segura_rumo=True, preditor=False, preditor_atraso=0.94,
                 preditor_ganho=1.0, preditor_max=0.35,
                 adapta=False, adapta_t=8.0, adapta_desvio_max=0.5,
                 adapta_v_min=0.05):
        if kp < 0.0 or ki < 0.0:
            raise ValueError('kp e ki não podem ser negativos')
        # `segura_rumo=False` deixa só o FEEDFORWARD: cancela o arco do corpo
        # e não escolhe rumo nenhum. É o modo para quando há um controlador de
        # rumo ACIMA (o `heading_controller`, na pilha). Com ele ligado os dois
        # disputam: este nó segura o rumo que CAPTUROU, que não é o rumo que o
        # caminho quer, e o de cima tem de pivotar para desfazer — foi o
        # sintoma que o dono viu em 05-08 ("parece que ele está sem o PID, tá
        # pendendo pra direita, aí o pivô para tendo que arrumar isso").
        self.segura_rumo = segura_rumo
        self.curv_frente = curv_frente
        self.curv_re = curv_re
        self.kp = kp
        self.ki = ki
        self.wz_max = wz_max
        self.int_max = int_max
        self.limiar_curva = limiar_curva
        self.rumo_ref = None      # capturado ao entrar em reta
        self.integral = 0.0       # [rad·s]
        self.sentido = 0          # +1 frente, -1 ré, 0 parado

        # --- estimador do ff (opt-in, decisão 013 caminho 2) ---
        #
        # 🔴 O PROBLEMA QUE ELE RESOLVE, medido no robô em 10-08: a curvatura
        # crua da planta DERIVA DENTRO DA SESSÃO. Seis retas idênticas, mesmo
        # ponto, mesmo rumo, mesmo chão:
        #
        #     +0,0 min 0,8395   +1,5 min 0,9358   +4,8 min 0,9313
        #     +0,9 min 0,8154   +2,5 min 0,9175   +5,4 min 0,9687
        #                  +0,022 1/m por minuto (r=0,80), +19% em 5,4 min
        #
        # Os 13,5% ENTRE DIAS que motivaram a decisão 013 acontecem em cinco
        # minutos. Um `curv_frente` medido no começo da bancada envelhece
        # dentro da própria bancada — o caminho 3 é piso, não solução.
        #
        # E deixar o integrador cobrir a diferença NÃO funciona, embora a conta
        # dissesse que caberia (ki·int_max = 0,072 rad/s de autoridade contra
        # 0,038 rad/s necessários). Medido no mesmo dia, corridas de 2,5 m:
        #
        #     ff VELHO −0,8275   −13,5° → +22,3°   envoltória CRESCE 1,65x
        #     ff HOJE  −0,9383     0,0° → +13,0°   sobrecorrige, não assenta
        #
        # A razão é de ESCALA DE TEMPO: o integrador é o único que enxerga o
        # erro, mas vive dentro de um laço com 0,94 s de tempo morto (por isso
        # os ganhos caíram 4,1x em 06-08) e é ZERADO a cada parada pelo
        # `_descarta`. Cada corrida recomeça do zero e passa os 10 s inteiros
        # reaprendendo o que a anterior já sabia.
        #
        # 🟢 A SAÍDA: separar as duas escalas. O integrador continua com o
        # rápido (rad·s de rumo, descartado na parada); o que ele segura em
        # REGIME é drenado devagar para a `curv_*`, que é uma curvatura [1/m] e
        # SOBREVIVE à parada. A corrida seguinte já começa corrigida.
        #
        #     transf = integral · dt / adapta_t          [rad·s]
        #     Δcurv  = −(ki · transf) / v_real           [1/m]
        #     integral −= transf
        #
        # ⚠️ A transferência é SEM SOLAVANCO por construção, e isso não é
        # detalhe: no instante em que ela acontece o `wz` de saída não muda.
        # O feedforward cresce de `−Δcurv·v = ki·transf` e o termo integral
        # encolhe de exatamente `ki·transf`. Sem isso, cada transferência seria
        # um degrau no comando — e degrau num laço com 0,94 s de tempo morto é
        # como se fabrica a oscilação que este estimador veio matar.
        #
        # ⚠️ `adapta_t` PRECISA ser lento perto do laço (8 s contra ~9,6 s de
        # assentamento é pouca margem; é o valor de partida, e quem arbitra é o
        # robô). Dois integradores em série com escalas parecidas oscilam
        # juntos: a adaptação passa a perseguir o transiente em vez do viés.
        #
        # ⚠️ DESLIGADO POR PADRÃO, como o preditor entrou. A condição de
        # controle da próxima bancada é o comportamento de hoje; ligar por
        # `-p adapta:=true`. Régua de aceitação: corrida de 2,5 m (~10 s), NÃO
        # de 1,2 m — em 10-08 a mesma corrida mediu −0,0162 (passa) cortada em
        # 1,2 m e +0,0882 (reprova) medida inteira, porque o corte curto cai no
        # cruzamento de zero do S.
        self.adapta = adapta
        self.adapta_t = adapta_t
        self.adapta_desvio_max = adapta_desvio_max
        self.adapta_v_min = adapta_v_min
        # A semente é o que o launch passou (medido ou herdado). O grampo é
        # RELATIVO a ela: o estimador corrige deriva de planta, não inventa um
        # robô novo. Sem isso, um `/Odometry` travado ou uma referência de rumo
        # ruim empurrariam a curvatura para o grampo absoluto e ela ficaria lá.
        self.curv_frente_semente = curv_frente
        self.curv_re_semente = curv_re

        # --- preditor de Smith (opcional, DESLIGADO por padrão) ---
        self.preditor = preditor
        self.preditor_atraso = preditor_atraso
        self.preditor_ganho = preditor_ganho
        self.preditor_max = preditor_max
        self.em_transito = []     # [(dt, correcao)] ainda a caminho da roda

    def passo(self, v_cmd, wz_cmd, yaw, dt, v_real=None):
        """Um ciclo: devolve o wz corrigido para (v_cmd, wz_cmd) dados.

        `yaw` é o rumo atual [rad] (LIO no robô, pose verdadeira no Gazebo);
        `dt` é o tempo desde o último passo [s]. `v_real` é a velocidade
        MEDIDA — ver abaixo. `v_cmd` sai como entrou: esta lei não toca na
        velocidade (não teria autoridade: patamar).

        ⚠️ O feedforward escala com a velocidade **REAL**, não com a pedida.
        O arco é curvatura × distância percorrida, e quem decide a distância é
        o patamar da placa, não o comando. Medido em 05-08 dentro da pilha: o
        seguidor pedia 0,500 m/s, o robô andava 0,299, e o ff saía +0,408
        rad/s onde bastavam +0,244 — 67% a mais. Sem `v_real` a lei cai no
        comando, que é o certo só quando os dois coincidem.
        """
        v_ff = abs(v_real) if v_real is not None else abs(v_cmd)

        # Parado não há arco: ele nasce do movimento. E não há rumo a segurar
        # — referência velha é pior que nenhuma, o robô pode ter sido girado
        # no chão enquanto esperava.
        if abs(v_cmd) < 1e-9:
            self._descarta()
            return wz_cmd

        sentido = 1 if v_cmd > 0.0 else -1
        curv = self.curv_frente if sentido > 0 else self.curv_re
        ff = -curv * v_ff

        # O arco existe girando também: ele é do CORPO, não do comando. Por
        # isso o ff entra sempre, inclusive na curva pedida. O que a curva
        # dispensa é a malha de rumo — essa sim é assunto do comandante.
        if abs(wz_cmd) >= self.limiar_curva or not self.segura_rumo:
            if abs(wz_cmd) >= self.limiar_curva:
                self._descarta()
            return wz_cmd + ff
        if sentido != self.sentido:
            # O viés da frente (−0,82) não é o da ré (−0,10): integrador
            # carregado do sentido errado viraria chicote na troca.
            self._descarta()
            self.sentido = sentido
        if self.rumo_ref is None:
            self.rumo_ref = yaw

        e = norm_ang(self.rumo_ref - self.yaw_efetivo(yaw))

        # dt não-positivo (relógio andou para trás, primeira amostra) não
        # pode envenenar o integrador; o termo P segue valendo.
        if dt > 0.0:
            self.integral = max(-self.int_max,
                                min(self.int_max, self.integral + e * dt))

        correcao = self.kp * e + self.ki * self.integral
        self._registra_em_transito(correcao, dt)
        wz = ff + correcao
        # A drenagem entra DEPOIS de o comando deste ciclo estar formado: ela
        # muda o `curv_*` do ciclo seguinte, nunca este. É o que mantém a
        # transferência sem solavanco também no tempo.
        if dt > 0.0:
            self._drena_para_o_ff(sentido, v_ff, dt)
        return max(-self.wz_max, min(self.wz_max, wz))

    def _drena_para_o_ff(self, sentido, v_ff, dt):
        """Passa devagar, do integrador para a curvatura, o que é viés de planta.

        O integrador é rápido e some na parada; a curvatura é lenta e fica.
        Só o que ele segura em REGIME é viés — por isso a constante de tempo.

        Não adapta parado nem devagar: `Δcurv` divide por `v_ff`, e velocidade
        perto de zero transformaria qualquer resíduo do integrador em curvatura
        enorme. É a mesma razão pela qual `passo` descarta com `v_cmd` nulo.
        """
        if not self.adapta or self.ki <= 0.0 or v_ff < self.adapta_v_min:
            return
        transf = self.integral * dt / self.adapta_t          # [rad·s]
        dcurv = -(self.ki * transf) / v_ff                   # [1/m]
        atual = self.curv_frente if sentido > 0 else self.curv_re
        semente = (self.curv_frente_semente if sentido > 0
                   else self.curv_re_semente)
        novo = max(semente - self.adapta_desvio_max,
                   min(semente + self.adapta_desvio_max, atual + dcurv))
        aplicado = novo - atual
        if aplicado == 0.0:
            return
        # Tira do integrador EXATAMENTE o que virou feedforward — inclusive
        # quando o grampo cortou a transferência pela metade. Devolver mais do
        # que entrou deixaria o comando com um degrau para baixo.
        self.integral -= -(aplicado * v_ff) / self.ki
        if sentido > 0:
            self.curv_frente = novo
        else:
            self.curv_re = novo

    # ------------------------------------------------- preditor de Smith
    #
    # O problema que ele resolve: entre o comando sair e a roda responder
    # passam ~0,94 s (0,27 s de liga + 0,52 s de desliga da placa, mais a pose
    # a 10 Hz e a janela de 0,2 s da velocidade — número confirmado também
    # pela frequência em que o robô oscilou em 05-08). Nesse intervalo a malha
    # vê um yaw VELHO e corrige de novo o que já mandou corrigir. É isso que
    # produz o S, e é por isso que os ganhos tiveram de cair 4,1x.
    #
    # A saída clássica: em vez de baixar o ganho, **descontar o que já está a
    # caminho**. A malha passa a enxergar
    #
    #     yaw_efetivo = yaw_medido + (o giro que os comandos em trânsito ainda
    #                                 vão produzir)
    #
    # e com o atraso fora de dentro da malha o ganho pode voltar a subir.
    #
    # ⚠️ SÓ A CORREÇÃO ENTRA NA PREVISÃO, não o feedforward. O efeito futuro do
    # ff é, por construção, cancelado pelo arco futuro do corpo — prever um sem
    # prever o outro criaria um viés do tamanho do próprio ff, que é a maior
    # parcela da saída. Este é o detalhe que faz o preditor ajudar em vez de
    # atrapalhar.
    #
    # ⚠️ ELE DEPENDE DO MODELO. Se `preditor_atraso` ou `preditor_ganho` errarem
    # muito, ele prevê errado e pode piorar. Três defesas:
    #   1. desligado por padrão — o caminho de produção segue o de ganho baixo,
    #      que não depende de modelo nenhum;
    #   2. `preditor_ganho` em 1,0, que SUBESTIMA (o realizado é ~1,19x o
    #      comandado na faixa reta). Subestimar degrada em direção ao caso sem
    #      preditor, que é o lado seguro do erro — mesma lógica do `a_dec` em
    #      27-07 ("errar para baixo é de graça");
    #   3. `preditor_max` grampeia a previsão: por mais que a fila cresça, ela
    #      não desloca o yaw mais que isso.
    #
    # 🔴 **E O VEREDITO MEDIDO (06-08): ele PERDE para simplesmente baixar o
    # ganho.** Na planta de brinquedo com o atraso medido de 0,94 s:
    #
    #     configuração                 excursões              assenta   rumo
    #     ANTIGOS 1,0/0,5 sem pred.    15,2 15,3 12,2 10,0     39,9 s   +0,94°
    #     ANTIGOS 1,0/0,5 COM pred.    15,3  2,3  3,7  3,6     nunca    +3,65°
    #     NOVOS 0,25/0,12 sem pred.    16,5  0,4  0,2  0,1      9,6 s   +0,06°
    #     NOVOS 0,25/0,12 COM pred.    16,7  3,4  3,7  3,6     nunca    +3,63°
    #
    # Ele faz o que promete — mata a DIVERGÊNCIA dos ganhos antigos (12° viram
    # 3,6°) — mas deixa uma ondulação SUSTENTADA de ~3,6° e um viés de rumo, e
    # a redução de ganho assenta abaixo de 0,1°. Conferido que não é o grampo
    # (mesmo resultado de 0,35 a 2,0 rad) nem o `preditor_ganho` (subir de 1,0
    # para 1,4 piora monotonicamente).
    #
    # ⚠️ RESSALVA DA COMPARAÇÃO, que ela não resolve: na planta de brinquedo o
    # ARCO age imediatamente enquanto o wz comandado chega atrasado. No robô os
    # dois nascem do mesmo movimento e chegam juntos. Essa assimetria pode
    # penalizar o preditor injustamente — é por isso que ele fica no código, e
    # não é por isso que ele fica ligado. Quem arbitra é o robô
    # (`docs/PLANO_SINTONIA_RUMO.md`).

    def yaw_efetivo(self, yaw):
        """O yaw que a malha deve enxergar: o medido mais o que está a caminho."""
        if not self.preditor:
            return yaw
        pendente = sum(w * d for d, w in self.em_transito)
        pendente = max(-self.preditor_max,
                       min(self.preditor_max, self.preditor_ganho * pendente))
        return yaw + pendente

    def _registra_em_transito(self, correcao, dt):
        """Guarda a correção emitida e esquece o que já chegou na roda."""
        if not self.preditor or dt <= 0.0:
            return
        self.em_transito.append((dt, correcao))
        idade = sum(d for d, _ in self.em_transito)
        while self.em_transito and idade > self.preditor_atraso:
            idade -= self.em_transito.pop(0)[0]

    def _descarta(self):
        self.rumo_ref = None
        self.integral = 0.0
        self.sentido = 0
        # A fila do preditor também morre: ela descreve correções emitidas
        # contra uma referência que não existe mais. Carregá-la para a próxima
        # reta faria a malha descontar um giro que ninguém pediu — o mesmo
        # motivo pelo qual o integrador é zerado aqui.
        self.em_transito.clear()
