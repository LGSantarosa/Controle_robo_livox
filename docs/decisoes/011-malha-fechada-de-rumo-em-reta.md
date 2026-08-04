# 011 — Malha fechada de rumo em reta (o "PID" da reta)

**Data**: 2026-08-04 · **Status**: aceita pelo dono (fatia 1 aprovada em sessão)

## Contexto

O robô não anda reto: comandado `v=0,25, wz=0`, descreve um círculo de
**1,22 m de raio para a direita** (curvatura −0,817 1/m, n=2 matched,
04-08). De ré desvia 8,3× menos (−0,098 1/m, raio 10,2 m) — mas **não** é
reto: em 10 m ainda dá ~70–90° de desvio. As causas físicas (boba + resíduo
que sobrevive à ré) **ficam onde estão** — decisão de escopo do dono, e é o
assunto do artigo (BO-2): controlar esta geometria, não consertá-la.

O simulador reproduz as três manobras (arco de frente, arco de ré, pivô)
dentro da dispersão do robô — validado por número (aceitação de 04-08) e
pelo olho do dono. É a primeira vez que dá para desenvolver um controlador
contra o defeito de verdade sem gastar robô.

## O que decide

Uma **camada de correção de rumo entre quem comanda e o atuador**:
entra o `cmd_vel` desejado, sai o `cmd_vel` corrigido. Vive no
`robot_motion`, como nó próprio (`compensador_rumo`), com a lei pura em
`lei_de_reta.py` (testável sem ROS, padrão da casa desde a 005).

    comandante ──cmd_vel──▶ compensador_rumo ──cmd_vel──▶ atuador
                                   ▲
                               /Odometry (yaw do LIO; no Gazebo, pose verdadeira)

### A lei (por que funciona neste atuador)

A compensação de zona morta do driver **preserva a razão entre as rodas e
destrói a magnitude** (medido em 31-07/01-08). Consequência que esta decisão
explora: andando, a **curvatura comandada sobrevive** (`wz/v` define a razão,
e a razão passa) — há autoridade contínua sobre o rumo mesmo com o patamar.
O que não há é autoridade sobre velocidade, e esta lei não toca em `v`.

    wz_saida = wz_ff + Kp·e + Ki·∫e         e = norm(rumo_ref − yaw)
    wz_ff    = −curv_medida(sentido) · |v_cmd|

- **Feedforward pela curvatura MEDIDA** (−0,817 frente / −0,098 ré — os
  mesmos números do modelo da placa, por construção): cancela ~80–90 % do
  arco antes de qualquer erro aparecer. Errar o ff para baixo é seguro — o
  integrador come o resto.
- **PI, não PID**: o D derivaria o yaw do LIO a 10 Hz com latência de
  atuador de ~0,27 s — amplificaria ruído para corrigir um viés que é
  **constante**. Viés constante é o caso de livro do integrador.
- **Referência de rumo capturada na hora**: quando o comando entra em
  "reta" (`|wz_cmd| < limiar`), o yaw daquele instante vira a referência.
  Comando de curva (`|wz_cmd| ≥ limiar`) passa **intocado** e rearma a
  captura — curva é assunto do comandante (e do pivô, fatia 3).
- **Troca de sentido** zera o integrador: o viés da frente (−0,82) não é o
  da ré (−0,10), e um integrador carregado do sentido errado viraria chicote.
- **Anti-windup** por grampo no integrador; `wz` de saída grampeado em
  `wz_max` próprio (a correção de regime é ~0,25 rad/s; o grampo existe
  para o transitório, não para operar saturado).

### Critério de aceitação (a régua já existe)

`medir.py curvatura`, mesma manobra da bancada de 04-08, n=3 por sentido:

    sem compensador:  frente −0,82 1/m,  ré −0,10 1/m   (hoje)
    com compensador:  |curvatura| < 0,05 1/m nos dois sentidos

0,05 1/m = raio > 20 m = menos de 3° por metro andado. Fecha no Gazebo
primeiro (fatias 1–2), no robô real depois (fatia 4), pelo mesmo protocolo.

## Alternativas descartadas

1. **Plugin de controller dentro do Nav2** — acoplaria a correção ao Nav2:
   teleop e bancada não ganhariam nada, e a decisão 008/009 já fixou "Nav2
   planeja, nós seguimos". Reabrir isso custaria mais que o plugin renderia.
2. **Dentro do `heading_controller` (005)** — ele responde por *rumo alvo*
   vindo da navegação; esta camada responde por *fidelidade de comando* e
   tem de servir qualquer comandante (inclusive o próprio heading_controller
   um dia). Misturar os dois deixaria a 005 — já ferida pelo patamar —
   carregando mais uma responsabilidade.
3. **Consertar a causa física (boba)** — fora de escopo por decisão do dono
   (04-08), e mesmo consertada sobra raio de ~10 m que exige esta malha do
   mesmo jeito. A malha é necessária COM ou SEM conserto.
4. **Mapa estático de correção (offset fixo em wz, sem sensor)** — é o
   feedforward sozinho. A dispersão medida entre corridas idênticas é 21 %
   na curvatura: sem integrador, sobraria arco de até ~0,17 1/m (raio 6 m),
   acima do critério. O ff entra, mas não basta.

## O que esta decisão NÃO cobre (e onde isso está registrado)

- **Pivô** (girar parado): não há modulação possível — a placa entrega um
  wz só. É corte previsto, não PID, e **só se sintoniza depois de modelar o
  atraso de desliga** no simulador (hoje: 99° de sobrepasso lá contra 49°
  no robô — sintonizar contra isso erraria 2×). Fatias 2–3.
- **Velocidade**: o patamar continua mandando. Nada aqui toca em `v`.
- **Delator do BO-3** (comando sem movimento → gritar): pertence a esta
  camada, entra quando ela virar permanente (fatia 5).

## Referências

- Números do robô: `docs/dados/2026-08-04-bancada-robo/` e `MODELO_ROBO2.md`.
- Aceitação do simulador: `docs/dados/2026-08-04-aceitacao-simulador/`.
- Autoridade sobre curvatura sob compensação: `placa_simulada.py` (cabeçalho)
  e DIARIO 01-08.
- Padrão lei-pura + nó: decisão 005, `lei_de_rumo.py`.
