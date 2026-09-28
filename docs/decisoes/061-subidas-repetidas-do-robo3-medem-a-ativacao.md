# 061 — Subidas repetidas do robô 3 medem a ativação dos controladores

**Data**: 2026-09-28 (PC de dev; robô e lidar desligados)
**Status**: proposta; escrita antes dos testes vermelhos e do código
**Toca**: um wrapper novo `bin/subidas-robo3`, `tools/subidas_robo3/` (medidas
puras e testes) e o `testpaths` do `pytest.ini`
**Não toca**: pilha, launch, controladores, Nav2, `tools/valida_etapa6/`
(usado como está), os wrappers congelados

---

## 1. Contexto

Na corrida `~/etapa7/20260928_114620` (decisão 060 §6.6) o
`joint_state_broadcaster` não ativou: o `controller_manager` registrou
`Switch controller timed out after 5 seconds!`, o spawner morreu com código 1 e
`/joint_states` ficou com 0 mensagens no bag. Foi **1 em 8** corridas do robô 3,
logo depois de um rebuild do `robot_base`; as outras sete ativaram. Os três
critérios do passo 7 não dependem dele, e a corrida seguinte, nominal, fechou o
passo. A falha ficou como dívida separada de inicialização.

Hipótese — **não** causa: o `switch` só se completa quando o laço de controle
do Gazebo roda; simulação lenta na subida (carga de CPU) estouraria os 5 s.

Antes da bateria de navegação (reta longa, giro, percurso combinado,
repetibilidade), a subida precisa ser medida: navegar em cima de uma subida que
às vezes não é nominal mistura duas perguntas.

## 2. Decisão

### 2.1 O instrumento

`bash bin/subidas-robo3 [N]` (N = 20 por padrão) sobe e derruba a pilha do
robô 3 headless N vezes, **uma subida por vez, cada uma limpa**, com os
argumentos da corrida do passo 7:
`robo:=3 sim:=true gui:=false rviz:=false localizacao:=fixa bag:=false`.

- Domínio **50** (47 é da etapa 6, 48 do explora, 49 da etapa 7), só
  localhost; `GZ_PARTITION` própria **por subida**.
- **Um** `colcon build` no início, registrado; nenhum entre subidas (rebuild é
  variável da hipótese, e misturá-lo esconderia o efeito).
- Árvore suja barra; ROS/Gazebo já de pé barra; processo com qualquer
  `VALIDA_ETAPA4_MARCA` barra — as pré-condições do `valida-etapa7`.
- Cada subida tem subpasta `subida_NN/`, que é também o **valor da marca**
  dela: a limpeza da subida só alcança o que ela lançou.
- A limpeza é a `limpa()` de `tools/valida_etapa6/lib.sh`, **como está**
  (grupos registrados, INT, 20 s, KILL, marcados, domínio vazio).

### 2.2 O que se mede, sem perturbar a subida

A sondagem **durante** a subida é só a espera do `valida-etapa7` (ciclo de vida
dos três servidores do Nav2, ação listada, TF `map→base_link`), igual para
manter as condições da corrida em que a falha apareceu. As medidas dos
controladores e de `/joint_states` são feitas **uma vez, depois** de a pilha
estar pronta, com `timeout`:

| coluna do CSV | fonte |
|---|---|
| `nav2_tf_pronto`, `t_nav2_tf_s` | a espera do `valida-etapa7`, prazo de 300 s de parede |
| `jsb_estado`, `base_estado`, `controladores_ok` | **uma** chamada ao serviço `/controller_manager/list_controllers` (timeout 15 s) — o `ros2 control` (`ros2controlcli`) **não está instalado** neste PC (conferido em 28-09); instalar seria mudar o sistema |
| `joint_states_msgs`, `joint_states_janela_s` | **uma** janela de `ros2 topic echo /joint_states` (timeout) |
| `switch_timeout`, `falha_ativar`, `died_antes_do_fim` | contagem no `launch.log` bruto, antes do primeiro `signal_handler(SIGINT` |
| `t_ativacao_jsb_s` | no `launch.log`: `Activating controllers: [ joint_state_broadcaster ]` → `Configured and activated joint_state_broadcaster`; vazio se não ativou |
| `load1_max`, `psi_cpu_some_avg10_max`, `psi_cpu_some_us` | amostrador de `/proc/loadavg` e `/proc/pressure/cpu` a cada 1 s, **durante toda a janela** da subida (do lançamento ao fim das medidas); o `total` do PSI vira o delta da janela, em µs |
| `limpeza_ok`, `residuo` | a `limpa()` da subida e uma varredura independente antes da próxima |

O `launch.log` bruto e a série do amostrador ficam na subpasta.

### 2.3 O que é falha, e o que interrompe

- **Subida nominal**: Nav2/TF prontos, os dois controladores `active`,
  `/joint_states` com mensagens na janela, e zero `switch_timeout`.
- **Falha de subida** é registrada, a subida é limpa, e a bateria **continua**
  se a limpeza fechar.
- **Resíduo interrompe tudo**: limpeza reprovada, ou a varredura antes da
  próxima subida achar processo ROS/Gazebo, processo marcado, segmento DDS em
  `/dev/shm` ou nó no domínio 50. A bateria sai com RC 1 e fica
  **INCOMPLETA** — continuar subiria a próxima sobre sujeira.

### 2.3.1 O teardown — regra do dono, 2026-09-28 (14h)

A bateria de 14h10 parou depois da subida 1 (nominal) com 34 segmentos Fast
DDS órfãos em `/dev/shm`. A única diferença para o ensaio limpo foi o
`collision_monitor` morrer com **SIGSEGV (-11) no teardown** (dívida 057). A
causa provável — um participante Fast DDS morto por segmentação não libera os
segmentos que abriu, inclusive as portas dos outros — é **hipótese**. A dívida
057 deixa de ser cosmética: o SIGSEGV impede a próxima subida limpa.

Regra, depois da limpeza de processos de cada subida:

- **Tolerado e registrado**: SIGSEGV do `collision_monitor` **iniciado depois**
  do primeiro `signal_handler(SIGINT`. A bateria continua.
- **Interrompe na hora**: SIGSEGV antes do teardown; sinal em qualquer outro
  processo; outro sinal no `collision_monitor`; limpeza de processos reprovada
  (processo, marca ou nó vivo).
- **Segmento órfão**: inventário **antes** de qualquer limpeza (segmentos com
  horário, `fuser` de cada um, os processos mortos no `launch.log`); só então,
  e **somente** se não houver processo, nó ou marca viva e **todos** os
  segmentos estiverem sem dono, `fastdds shm clean` — nunca remoção manual. A
  seguir, recontagem: sobrou segmento, interrompe.
- Segmento removido assim é **limpeza recuperada**, não limpeza nominal: o CSV
  separa `subida_nominal` (a subida), `sigsegv_teardown`, `sinais_fora_da_regra`,
  `shm_orfaos` e `limpeza_recuperada`, e o veredito escreve o teardown à parte.

A bateria que segue esta regra **mede a taxa do SIGSEGV** do teardown; ela
**não** libera, por si, o teste no hardware.

### 2.3.2 A remoção manual controlada — regra do dono, 2026-09-28 (14h30)

Medido às 14h28: o `fastdds shm clean` só reconhece como zumbi o segmento que
ainda tem a trava `_el`. Dos 34 órfãos da bateria `141047` removeu **5** (os com
`_el`); **29** ficaram — 15 portas `fastrtps_port<N>` e 14
`sem.fastrtps_port<N>_mutex`, sem `_el` e sem dono. Hipótese (não medida): os
donos saíram limpos e apagaram as próprias travas, mas o `collision_monitor`
morto por SIGSEGV deixou as portas deles referenciadas.

Autorizada **só como recuperação controlada da bateria em simulação**
(`tools/subidas_robo3/shm_recupera.py`), depois do `fastdds shm clean`:

- candidatos = diferença entre o inventário **antes** (tirado no início da
  subida) e o **depois** (depois da limpeza de processos, antes do `fastdds`);
- só `fastrtps_port<N>` e `sem.fastrtps_port<N>_mutex`, **sem** o `_el` irmão,
  arquivo regular (nunca symlink — `lstat`, nunca seguido), do usuário atual,
  direto no diretório de SHM;
- tamanho e mtime iguais aos do inventário; `fuser` vazio; e o "nada vivo"
  (ROS, Gazebo, Fast DDS, marca, nó) **reconferido** imediatamente antes;
- **tudo** validado antes de remover **qualquer** um: um arquivo inválido,
  inesperado ou que mudou, e **nenhum** sai; remoção por nome, um a um, a
  partir da lista — nunca com padrão expandido;
- recontagem: sobrou, interrompe.

Cada ocorrência fica `teardown_anomalo = 1` e `limpeza_manual_recuperada = 1` —
nunca execução inteiramente nominal. A opção de desligar o transporte SHM do
Fast DDS na simulação fica **descartada**: mudaria o sistema sob teste.

A decisão **057** passa a **bloqueadora para hardware** até o SIGSEGV do
`collision_monitor` ser entendido.

### 2.4 O veredito da bateria

- **ESTÁVEL** somente com **N/N** subidas nominais e nenhuma interrupção. O
  rótulo é **da subida**; o teardown (SIGSEGV tolerados, limpezas recuperadas)
  vem escrito no mesmo veredito, e limpeza recuperada nunca conta como nominal.
- Com 20/20, a leitura honesta é de limite: o limite superior unilateral de
  95% para a taxa de falha fica em ~**13,9%** (1 − 0,05^(1/20)). Se a taxa
  real fosse 1/8, a chance de ver ao menos uma falha em 20 seria 93,1%.
- Qualquer falha: **INSTÁVEL**, com a contagem e as linhas no CSV.

A pasta leva o manifesto no padrão da etapa 7: escopo declarado, sem
`console.txt`, `sha256sum -c`, RC 1 se falhar.

## 3. Alternativas descartadas

| alternativa | por que não |
|---|---|
| rodar o `bin/valida-etapa7` N vezes | ~3 min cada e com objetivo: a navegação é variável a mais, e a pergunta é só a subida |
| o `bin/valida-etapa6` | evidência congelada de passo fechado, e sobe coisas que esta pergunta não pede |
| sondar `/joint_states` ou os controladores **durante** a subida | a sondagem pesa exatamente na janela suspeita |
| rebuild antes de cada subida | mistura a variável "build" com a repetição; se a bateria limpa for estável, o build vira a próxima pergunta, isolada |
| aumentar o timeout do `switch` ou mexer na ordem dos spawners já | seria consertar antes de reproduzir e medir |

## 4. Sequência, uma mudança por vez

1. **Esta decisão.**
2. **Testes vermelhos** da parte pura (`tools/subidas_robo3/mede.py`: leitura
   do `list_controllers`, análise do log, resumo do amostrador, a linha do CSV
   e o veredito) e de um **teste offline do wrapper com comandos calçados**,
   que prova: falha de subida registrada e bateria continuando; resíduo
   interrompendo; manifesto fechando. Os calços ficam num domínio que não é o
   50 e o teste confere que nenhum `gz`/`ros2` real subiu.
3. O código, até verde.
4. O pedido de "pode" para as 20 subidas.

## 5. O que esta decisão não prova

A **causa**. Se a falha aparecer, o CSV mostra em que condições (carga,
pressão, tempo de ativação); se não aparecer, mostra um limite para a taxa.
Nada de hardware, nada de navegação, e nada sobre o robô 2.

## 6. Registro das execuções

| pasta (`~/subidas-robo3/`) | o que foi | conta como subida? |
|---|---|---|
| `20260928_140215` | **falha do instrumento, N=0**: a varredura de resíduo inicial rodava **antes** do `source /opt/ros/jazzy/setup.bash`; sem o ambiente (apagado pela reexecução limpa), o `ros2 node list` quebrou (`PackageNotFoundError: ros2cli`) e a varredura, que trata consulta com erro como "não prova vazio", acusou resíduo e recusou subir. Nada foi lançado. O teste offline não pegou porque o calço do `ros2` não precisava do ambiente. Corrigido com vermelho antes (`35aaf86`): o calço passou a exigir o ambiente, o teste percorre a reexecução real com prefixo sujo injetado, e o `source` tem o código conferido | **não** |
| `20260928_141047` | **INCOMPLETA**: subida 1 **nominal**; no teardown, SIGSEGV do `collision_monitor` e 34 segmentos Fast DDS órfãos; a varredura antes da subida 2 interrompeu. Dois defeitos do instrumento apareceram nela: a limpeza do `trap` escreveu na `subida_02` (que não subiu) **depois** do manifesto, que por isso não fecha; e o escopo exigia `subida_02/launch.log`. Corrigidos com vermelho (`bee4e5a`). A pasta fica **intocada**, manifesto incluído | 1 subida medida; **não** é bateria válida |
| `20260928_144822` | **ensaio N=1** depois do reinício do PC (queda de luz; `/dev/shm` zerado): subida 1 **nominal** (Nav2 + TF em 38,2 s), teardown **limpo** (0 SIGSEGV, 0 órfão), veredito ESTÁVEL 1/1, manifesto confere. Os cinco `exit code 1` da 057 aparecem também aqui | ensaio; **não** é bateria |
| `20260928_145005` | **INCOMPLETA por interrupção manual.** Subida 1 **nominal** (23,6 s); no teardown, **SIGSEGV do `collision_monitor`** e 36 órfãos (8 pelo `shm clean`, 28 pela remoção manual; limpeza APROVADA). Pela ordem do dono (SIGSEGV = parar), SIGINT no **grupo do script** com a subida 2 já lançada; a limpeza dela aprovou, mas o `tee` morreu junto — `console.txt` corta, **sem `veredito.txt`**, e o amostrador ficou órfão (encerrado por PID). A subida 2 **não** ganhou linha. Evidência e nota: `docs/dados/2026-09-28-subidas-robo3/bateria_20260928_145005{,.NOTA.md,.SHA256SUMS}` | só a subida 1; **não** é bateria válida |

**Lição do instrumento (145005):** parar a bateria com sinal no grupo do
script perde o console, porque o `tee` está no mesmo grupo. Ainda não há
parada pedida limpa (ex.: parar ao fim da subida corrente); até haver, um
SIGSEGV observado só é interrompido com esse custo, que fica registrado.

**Estado depois de 145005:** a bateria de 20 **não foi feita**. Pelo dono,
**nenhuma outra bateria** até a correção do SIGSEGV (057, adendo de 28-09
tarde) passar por ensaio e bateria limpa.
