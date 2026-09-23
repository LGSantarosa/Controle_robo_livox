# 056 — A pilha escolhe o robô, e a reescrita do perfil vira arquivo

**Data**: 2026-09-23 (PC de dev, robô desligado)
**Status**: 🟡 **PROPOSTA** — aguardando revisão do dono. Passa a "aplicada"
só depois da implementação e das provas do `docs/PLANO_ETAPA6_ROBO3.md` §4–§7.
**Toca (proposto)**: `ros2_packages/robot_motion/launch/pilha.launch.py`,
`robot_motion/perfil.py`, `config/twist_mux_robo3.yaml`, `config/perfil_robo3.yaml`.
**Não toca**: nada do robô 2 (`nav2.yaml`, `collision_monitor.yaml`,
`twist_mux.yaml`, `movimentacao*.yaml`, `robot_base/`, `robot_nav/`).
**Vem de**: etapa 6 — a pilha com `robo:=3` no Gazebo.

---

## 1. Contexto

A etapa 4 montou o perfil do robô 3 e **provou o YAML reescrito**; a pilha
continuou subindo o robô 2. A etapa 6 quer a pilha subindo o robô 3 no Gazebo,
e a leitura do código mostrou que a barreira não é a que se supunha.

`_recusa_robo` é uma porta, e trocá-la é uma linha. O que **de fato** impede o
robô 3 são duas coisas estruturais:

1. **A pilha monta o perfil fixo** (`perfil.parametros(2, pkg)`) e **aborta de
   propósito** se o perfil pedir reescrita, porque ela não sabe aplicá-las:
   *"reescrita pedida e não aplicada seria costmap (ou reflexo) lendo o arquivo
   sem ela, em silêncio"*. O perfil do robô 3 pede sete.
2. **O momento está errado**: `generate_launch_description()` roda **sem
   contexto**, e `robo` só existe dentro de um `LaunchContext`. O perfil é
   escolhido antes de existir a informação que o escolhe.

E há uma razão pela qual dicionário não resolve: os costmaps são **nós dentro**
dos servidores do Nav2; parâmetro passado ao `Node` do servidor **não desce**
até eles. Quem chega lá é o **arquivo**.

## 2. Decisão proposta

**(a) A reescrita vira arquivo.** A pilha aplica `perfil.aplica_reescritas`
(que já existe, já é testada, e reprova caminho inexistente sem efeito
parcial) e **escreve o YAML resultante em disco**. Os servidores do Nav2 e o
`collision_monitor` passam a receber esse arquivo. Para o robô 2, cujo conjunto
de reescritas é vazio, o conteúdo é idêntico à base.

**(b) O arquivo mora na pasta do carimbo da corrida**, ao lado do log, com o
caminho impresso na subida — não em `/tmp`. O YAML que os costmaps realmente
leram é **evidência**, e num PIBIT evidência não pode evaporar no reboot.

**(c) O robô é lido do `argv`, e o contexto é quem confere.** O perfil é
escolhido em `generate_launch_description()` lendo o `argv` — mecanismo que
esta launch já usa (`_passou`) — e o `_recusa_robo`, que roda **no contexto**,
compara o valor do `argv` com o do contexto e **mata se divergirem**, antes de
qualquer ação operacional.

**(d) `robo:=3 sim:=false` continua recusado**, e a mensagem nomeia o que
falta: a fronteira do atuador real fora do grafo e a localização que ainda não
existe (LIO é a etapa 7). Recusa como trava, não como pendência.

**(e) O mux vira chave do perfil**, e o `twist_mux_robo3.yaml` ganha a faixa
`auto_vel` **abaixo das humanas**. Hoje nenhum dos dois muxes serve inteiro ao
robô 3: o da pilha não tem `dpad_vel`, o do robô 3 não tem `auto_vel`.

## 3. Alternativas descartadas

| alternativa | por que não |
|---|---|
| **`OpaqueFunction` montando a pilha inteira** no contexto (o jeito "certo" do launch) | é reescrever uma launch de 835 linhas que hoje sobe o robô 2 **funcionando**. A etapa 6 é sobre o robô 3; pagar com risco de regressão no robô 2 é trocar o ativo pelo experimento. Fica registrado como o desenho para quando a pilha for reescrita por outro motivo |
| **materializar os dois perfis e escolher o arquivo por substituição** | não resolve a sobreposição do `path_follower` (é dicionário, não arquivo) e tornaria o `robot_base` **dependência obrigatória para subir o robô 2** — um artefato de geometria do robô 3 faltando derrubaria o robô que funciona |
| **ler o `argv` sem a trava de coerência** | é o modo de falha caro: `argv` e contexto discordando (include programático, `ros2 launch` chamado de outra launch) dá **footprint do robô errado sem uma linha de aviso**. Mesma família da zona morta — diferença real, sintoma nenhum. A trava transforma isso em morte imediata |
| **passar as reescritas como dicionário ao `Node`** do servidor | não chega aos costmaps: eles são nós dentro do servidor. É o defeito que o §3 do plano da etapa 4 já tinha identificado |
| **YAML materializado em `/tmp`** | some, e some justamente quando se quer reabrir a corrida que deu errado. Um arquivo por corrida é barato |
| **um `twist_mux.yaml` único com as cinco faixas** para os dois robôs | mexe no mux do robô 2, que o gate de não regressão da etapa proíbe. E a prioridade do direcional (110, acima do analógico) é decisão do robô 3, da etapa 5 |
| **deixar o robô 3 com o mux do robô 2** | o direcional some sem aviso; a faixa que a etapa 5 acabou de criar morreria na primeira integração |

## 4. O que se pretende provar, e o que não

**A provar (plano §4, cada item com teste):** a matriz das quatro combinações;
um mux só; a cadeia terminando no `hoverboard_base_controller` e nenhum nó de
fronteira de hardware no grafo; `use_sim_time` vivo por lista nominal
versionada; `footprint` e `footprint_padding` **vivos** nos dois costmaps
contra o que o perfil devolve; `/Odometry`, `/scan` e TF; objetivo curto com
critério de três partes; encerramento sem órfão; e o gate de não regressão do
robô 2 (§6).

**Fora de alcance, e não se pretende:** 🔴 Livox, rede, IP, FAST-LIO; 🔴 escala,
massa e dinâmica do chassi 3 (provisórias por declaração do próprio
`sim_robo3`); 🔴 derrapada, zona morta e a placa (modelo herdado do robô 2);
🔴 comparação entre os chassis; 🔴 a MEGA e o `frente:=-1.0`.

## 5. Referências

- `docs/PLANO_ETAPA6_ROBO3.md` — o plano desta etapa (achados A–F no §0).
- `docs/PLANO_NAV2_ROBO3.md` §8, linha 6 da tabela de etapas; §1, linha 1 (por
  que a "etapa 3" da v1 caiu).
- `docs/PLANO_ETAPA4_ROBO3.md` §3 — por que a reescrita é por caminho completo.
- Decisões 052 (geometria canônica), 053 (perfil), 054 (fronteira do contrato).
- `ros2_packages/robot_motion/robot_motion/perfil.py` (`aplica_reescritas`),
  `launch/pilha.launch.py` (`_recusa_robo`, `_passou`, a guarda das reescritas).
