# 051 — URDF do robô 3 girado 180°: motrizes na frente

**Data**: 2026-09-18 (PC de dev, robô desligado)
**Status**: aplicada no modelo e nos testes; **só simulação**. Nada vai ao robô.
**Toca**: `robot_base/description/robo3.urdf.xacro`, `robot_base/test/test_urdf_robo3.py`
(e comentários do `robo3.gazebo.xacro` e do `hoverboard_controllers_sim_robo3.yaml`).
**Vem de**: decisão do dono em 16-09 (frente = motrizes, traseira = bobas) e
etapa 3 do `docs/PLANO_NAV2_ROBO3.md`, §3, itens 1 a 4.

---

## 1. O que muda

O `base_link` continua no eixo das motrizes, no chão (C8). O que gira é o
**corpo em relação a ele**:

| grandeza | antes (até 17-09) | agora |
|---|---|---|
| centro da caixa `caixa_cx` | +0,093 | **−0,093** |
| pivô das bobas `boba_x` | +0,2485 | **−0,2485** |
| rodinha em relação ao pivô (junta) | −0,020 | **+0,020** (lado do eixo) |
| garfo: origem visual e inercial | −0,010 | **+0,010** |
| Livox `livox_x` | +0,093 (centro da caixa) | **−0,093** (centro da caixa) |
| Livox yaw | ausente (0) | **0 explícito** — ver §3 |

Intocados: `linear_sign`/`frente:=-1.0`, as posições y das juntas (convenção do
ROS: `left_wheel_joint` em +y), e todos os parâmetros do controlador.

## 2. Por que é uma mudança só (e não metade primeiro)

A rotação acopla quatro coisas: posição em x, direção do trail, qual roda física
é a esquerda e o yaw do Livox. **Inverter x sem o resto não é rotação**, é um
modelo semanticamente intermediário. E o smoke do Gazebo não pega meia rotação:
ele não comanda movimento, então não vê esquerda/direita trocadas nem sinal de
yaw errado. Proposta minha de fazer só os itens 1 e 2 primeiro, **recusada pelo
dono** com esse argumento. Ele estava certo.

Os testes vieram antes (4 vermelhos, 21 verdes) e entram no mesmo commit.

## 3. As três escolhas que a rotação NÃO decide sozinha

1. **Trail em repouso aponta para o eixo.** É a posição física girada junto com
   o corpo e preserva "as pontas com as pontas" (pivô rente à caixa). O teste
   cobre os **três** lugares: a junta da rodinha, a origem visual do garfo e a
   origem inercial do garfo. Sem os três, o contato girava e o garfo continuava
   desenhado e pesando do lado antigo, com a suíte verde (apontado pelo dono).
2. **Esquerda/direita física não mora no URDF.** No robô 3 ela vive no
   `linear_sign: -1.0`: a decisão 049 provou que negar só a linear equivale ao
   giro de 180° com a troca de lado. O teste
   `test_urdf_girado_exige_frente_negativa_no_controle` trava a **coerência**
   (URDF girado ⇒ `frente` −1.0 por padrão). Ele **não observa roda física** —
   o nome foi trocado para não prometer isso.
3. **Yaw 0 do Livox é convenção nova de montagem**, decidida pelo dono em 18-09:
   *"o x do sensor será alinhado à frente nova"*. **Não** decorre do giro — girar
   rigidamente o chute antigo daria yaw π. Vale porque o sensor não está
   montado; é **provisória até a etapa 7** medir a pose 6D.

## 4. Alternativas

| alternativa | por que não |
|---|---|
| mover o `base_link` para a frente nova sem girar o corpo | quebra C8 (o centro de rotação deixa de ser a origem) |
| trocar os nomes `left`/`right` das juntas | viola a convenção do ROS (+y = esquerda) e não troca nada físico |
| só itens 1 e 2 agora, 3 e 4 depois | modelo intermediário; ver §2 |
| Livox com yaw π | gira um chute sobre uma montagem que não existe |

## 5. O que isto NÃO valida

- **Envolvente varrida e footprints** (§3, item 5): andando para a frente as
  bobas giram e saem 20 mm da caixa. Fica separado, com teste geométrico
  próprio. O smoke **não** prova isso (nem sobe o Nav2).
- O comprimento e a porta passaram a ser calculados **por extremos**
  (`max x − min x`): verdes antes e depois, porque o giro não muda a envolvente
  rígida. Antes, a conta do comprimento só fazia sentido para um dos lados.
- Nada de dinâmica, bitola, curvatura ou zona morta — ver §8-B do plano.
