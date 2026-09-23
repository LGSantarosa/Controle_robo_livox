# 056 — A pilha escolhe o robô, e a reescrita do perfil vira arquivo

**Data**: 2026-09-23 (PC de dev, robô desligado)
**Status**: 🟡 **PROPOSTA — desenho APROVADO pelo dono em 23-09**, com as seis
correções obrigatórias já incorporadas (`PLANO_ETAPA6_ROBO3.md` §9). Passa a
"aplicada" só depois da implementação e das provas do plano §4–§7.
**Toca (proposto)**: `ros2_packages/robot_motion/launch/pilha.launch.py`,
`robot_motion/perfil.py`, `robot_motion/config/twist_mux_pilha_robo3.yaml`
(**novo**), `robot_motion/config/perfil_robo3.yaml`.
**Não toca**: nada do robô 2 —
`robot_motion/config/{nav2,collision_monitor,twist_mux,movimentacao*}.yaml`,
`robot_base/` e `robot_nav/` inteiros (inclusive
`robot_nav/config/twist_mux.yaml`, o mux do controle físico).
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

**(a) A reescrita vira arquivo — e só a do robô 3.** A pilha aplica
`perfil.aplica_reescritas` (que já existe, já é testada, e reprova caminho
inexistente sem efeito parcial) e **escreve os dois YAMLs resultantes em
disco**: o do Nav2 e o do `collision_monitor`. 🔴 **O robô 2 não materializa
nada** e segue recebendo os caminhos do `share/`. Materializar para ele, ainda
que com bytes idênticos, mudaria o **valor do parâmetro** (o caminho) e
tornaria impossível o gate "lista de parâmetros idêntica byte a byte" — a
própria trava que protege o robô que funciona.

**(b) Os dois arquivos moram na pasta do carimbo da corrida**, que é a **raiz
de evidências**, com os **caminhos absolutos impressos** na subida e entrada no
`SHA256SUMS` — não em `/tmp`. A escrita é **atômica** (temporário no mesmo
diretório + `os.replace`): meio arquivo lido por um costmap é pior que nenhum.
O YAML que os costmaps realmente leram é **evidência**, e num PIBIT evidência
não pode evaporar no reboot. ⚠️ O rosbag vai num **subdiretório ainda
inexistente** (`<carimbo>/bag/`), porque `ros2 bag record -o` exige diretório
que não existe e abortaria contra a raiz já criada.

**(c) O robô é lido do `argv`, e o contexto é quem confere.** O perfil é
escolhido em `generate_launch_description()` lendo o `argv` — mecanismo que
esta launch já usa (`_passou`) — e o `_recusa_robo`, que roda **no contexto**,
confere. É uma **ponte deliberada, não uma API geral**, e a trava tem três
bocas, todas matando **antes de qualquer processo ou escrita em disco**:
`robo:=` repetido no `argv`; `argv` e contexto divergindo; e include
**programático** com `robo:=3` que não aparece no `argv` — que cai na segunda
boca, porque o `argv` entrega o default `2`.

**(c′) A escrita acontece depois da validação.** O **caminho** dos dois
arquivos é calculado na descrição (o `Node` precisa dele), mas os **bytes** são
escritos por uma ação posterior ao `_recusa_robo`. Combinação recusada não pode
deixar **falsa evidência** de uma corrida que não houve.

**(d) `robo:=3 sim:=false` continua recusado**, e a mensagem nomeia o que
falta: a fronteira do atuador real fora do grafo e a localização que ainda não
existe (LIO é a etapa 7). Recusa como trava, não como pendência.

**(e) O mux vira chave do perfil**, num arquivo novo e de nome inequívoco,
`robot_motion/config/twist_mux_pilha_robo3.yaml`, com **quatro** faixas:
`dpad_vel` 110, `joy_vel` 100, `unstuck_vel` 30, `auto_vel` 10. Hoje nenhum dos
dois muxes serve inteiro ao robô 3: o da pilha não tem `dpad_vel`; o do robô 3
não tem `auto_vel` **nem `unstuck_vel`** — e sem este último o desencalhe que o
próprio perfil do robô 3 configura (`perfil.py:108–115`) publicaria para o
vazio, porque quem publica em `/unstuck_vel` é o `path_follower`
(`path_follower.py:741`). `key_vel` e `web_vel` **ficam de fora**: canal humano
novo no robô 3 exige decisão própria, e não entra por herança do mux do robô 2.

## 3. Alternativas descartadas

| alternativa | por que não |
|---|---|
| **`OpaqueFunction` montando a pilha inteira** no contexto (o jeito "certo" do launch) | é reescrever uma launch de 835 linhas que hoje sobe o robô 2 **funcionando**. A etapa 6 é sobre o robô 3; pagar com risco de regressão no robô 2 é trocar o ativo pelo experimento. Fica registrado como o desenho para quando a pilha for reescrita por outro motivo |
| **materializar os dois perfis e escolher o arquivo por substituição** | não resolve a sobreposição do `path_follower` (é dicionário, não arquivo) e tornaria o `robot_base` **dependência obrigatória para subir o robô 2** — um artefato de geometria do robô 3 faltando derrubaria o robô que funciona |
| **ler o `argv` sem a trava de coerência** | é o modo de falha caro: `argv` e contexto discordando (include programático, `ros2 launch` chamado de outra launch) dá **footprint do robô errado sem uma linha de aviso**. Mesma família da zona morta — diferença real, sintoma nenhum. A trava transforma isso em morte imediata |
| **passar as reescritas como dicionário ao `Node`** do servidor | não chega aos costmaps: eles são nós dentro do servidor. É o defeito que o §3 do plano da etapa 4 já tinha identificado |
| **YAML materializado em `/tmp`** | some, e some justamente quando se quer reabrir a corrida que deu errado. Dois arquivos por corrida é barato |
| **materializar também para o robô 2**, por uniformidade | uniformidade custaria o gate: o caminho é o valor do parâmetro, e ele mudaria. Perder a comparação byte a byte contra `1f49981` para deixar o código simétrico é trocar a proteção do robô que funciona por estética |
| **escrever os arquivos antes de validar `robo` e `sim`** | subida recusada deixaria pasta com YAML dentro — **falsa evidência** de uma corrida que não houve. Num projeto em que a pasta da corrida é a prova, isso é pior do que não ter prova |
| **só acrescentar `auto_vel` ao mux do robô 3** | deixaria o **desencalhe** publicando para o vazio, com o perfil do robô 3 configurando ré e pivô que nunca chegariam ao atuador. Zona morta de novo: configurado, silencioso, inoperante |
| **reaproveitar o nome `twist_mux_robo3.yaml`** para o mux da pilha | passariam a existir dois arquivos com o mesmo nome curto e papéis diferentes (controle físico × pilha), e a próxima pessoa a abrir um deles não teria como saber qual é o canônico |
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
