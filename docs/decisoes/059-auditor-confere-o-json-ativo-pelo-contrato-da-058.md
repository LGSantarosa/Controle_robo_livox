# 059 — O auditor confere o JSON ativo pelo contrato da 058

**Data**: 2026-09-25 (PC de dev; lidar e robô desligados)
**Status**: aplicada na `etapa6-pilha-robo3`; escrita antes do teste vermelho
e do código; validação offline, sem hardware
**Toca**: `bin/audita-livox` (checagens 7 e 8), um conferidor em
`tools/audita_livox/` e os testes do auditor
**Não toca**: `setup_livox.sh`, `tools/prepara_config_livox.py`, template,
perfis, launch, navegação ou atuador

---

## 1. Contexto e causa

O `bin/audita-livox` nasceu na etapa 6 (decisões 055 e seguintes) quando havia
um `robot_base/config/MID360_config.json` universal. A checagem 7 comparava
esse arquivo versionado com o JSON de runtime por `cmp`, e a checagem 8 lia o
IP do host **do versionado**.

A decisão 058 (`279f408`, trazida por cherry-pick como `6dc13f1`) removeu o
arquivo universal: ficam só o `MID360_config.template.json`, com
`__HOST_IP__`/`__LIDAR_IP__`, e o `livox_host_profiles.json`. As duas linhas
estavam corretas sozinhas; juntas, o auditor passa a dizer **REPROVADO "sem o
versionado …"** em qualquer máquina, inclusive numa montada exatamente como a
058 manda. Os 23 testes do auditor não pegam isso porque o fixture escreve o
próprio `MID360_config.json` universal num workspace fingido — ou seja, testam
a árvore que não existe mais.

Não é só um caminho a trocar. Sob a 058 não existe mais "o JSON certo" para
comparar byte a byte: o certo depende da máquina (perfil) e da unidade (IP
escolhido no setup). O auditor precisa de outro critério de "certo".

## 2. Decisão

### 2.1 Checagem 7 — o JSON ativo é uma materialização válida do template

O auditor lê o JSON de runtime (o que o launch consome, como antes) e exige,
nesta ordem, reprovando no primeiro que falhar:

0. `MID360.host_net_info` e `lidar_configs[0]` são objetos — estrutura
   errada é defeito do JSON (REPROVADO), nunca queda do conferidor
   (que o bash trataria como INCONCLUSIVO);
1. os quatro campos de host (`cmd_data_ip`, `push_msg_ip`, `point_data_ip`,
   `imu_data_ip`) contêm o **mesmo IPv4 concreto**, sem placeholder;
2. `lidar_configs` tem exatamente uma entrada, com IP IPv4 concreto;
3. o host corresponde a **exatamente um** perfil de `livox_host_profiles.json`;
4. o sensor pertence à rede desse perfil e não é o host, o endereço de rede nem
   o broadcast (a mesma `valida_ip_lidar` que o setup usa);
5. o JSON de runtime é **igual**, como estrutura, ao template materializado com
   esses próprios dois IPs (a mesma `renderiza_config` do setup). Isso pega
   porta, tipo de nuvem ou extrínseca editados à mão.

Template ou perfis ausentes no workspace continuam REPROVADO: o repositório
está incompleto. O detalhe do APROVADO diz perfil, host e sensor, e diz que o
sensor **não foi consultado**.

### 2.2 Checagem 8 — o host vem do runtime, não de arquivo versionado

O IP do host é o do JSON de runtime, e a máscara é a do perfil que casou. Os
vereditos mantêm a gradação que já existia, e ganham os dois casos que o
setup da 058 também recusa:

| situação na máquina | veredito |
|---|---|
| host/prefixo em exatamente uma interface | APROVADO |
| host/prefixo em mais de uma interface | REPROVADO (rota ambígua) |
| host presente com outra máscara | REPROVADO |
| outro endereço dentro da rede do perfil | REPROVADO (máquina errada para este JSON) |
| nada na rede do perfil | INCONCLUSIVO (cabo fora hoje não é máquina errada) |
| host ilegível ou sem perfil | INCONCLUSIVO (a checagem 7 já reprovou) |

### 2.3 Onde mora a lógica

Num conferidor Python pequeno em `tools/audita_livox/`, que **importa** as
funções do `tools/prepara_config_livox.py` (`carrega_perfil`,
`valida_ip_lidar`, `renderiza_config`) em vez de reescrevê-las. O bash segue
dono do relatório, dos três vereditos e da distinção link × cópia real. A
entrada de interfaces continua sendo `ip -brief addr`, o mesmo formato que o
auditor já lia.

### 2.4 O que continua fora do alcance

Offline e sem sudo, o auditor **não** consulta o sensor: nada de ping, MAC/OUI,
nuvem ou `/Odometry`. Um sensor válido para o perfil pode estar desligado ou
ser outra unidade. O relatório continua dizendo isso em toda execução.

## 3. Alternativas descartadas

| alternativa | por que não |
|---|---|
| só trocar o caminho para o template | o template tem placeholder; o `cmp` reprovaria sempre, por outro motivo |
| checar apenas "há dois IPs no JSON" | aceitaria host de outra máquina, sensor fora da rede e porta editada à mão |
| exigir um IP de sensor fixo | reintroduz exatamente o par universal que a 058 removeu |
| reescrever a validação em bash | duplicaria a regra do setup e as duas derivariam; importar garante a mesma regra |
| consultar o sensor por ping/MAC | o auditor é offline e sem efeito colateral por contrato; isso é do setup |

## 4. Prova

- **Vermelho antes do código:** com o fixture na árvore da 058 (template e
  perfis versionados, **nenhum** `robot_base/config/MID360_config.json`, JSON
  ativo válido no runtime), a máquina certa reprovava com
  `[REPROVADO] config de rede  sem o versionado …/MID360_config.json` e
  `[INCONCLUSIVO] IP do host  sem o JSON versionado para ler` — 21 dos 37 casos
  vermelhos, todos por esse motivo ou por dependerem da máquina certa aprovar.
- **Segundo vermelho, achado na revisão antes do commit:** `host_net_info`
  como lista e `lidar_configs[0]` como texto derrubavam o conferidor com
  `AttributeError`, e o bash registrava `[INCONCLUSIVO] … o conferidor falhou`
  — JSON estruturalmente inválido passava por "não deu para provar". Dois
  casos novos exigindo REPROVADO, vermelhos antes da checagem de tipo.
- **Verde depois:** `tools/audita_livox` **39 passaram** (22 dos 23 anteriores,
  adaptados à árvore nova, mais 17 do contrato: as duas estruturas inválidas, bancada notebook `.5/.169`
  de 24-09 aprovando, template e perfis ausentes, placeholder, hosts
  divergentes, host sem perfil, sensor fora da rede / igual ao host / rede /
  broadcast, campo editado à mão, máscara errada e host em duas interfaces).
- O teste que dizia "`.169` no runtime reprova" **saiu de propósito**: sob a
  058, `.169` é um sensor válido para o perfil `notebook`; o auditor offline
  não tem como saber qual unidade está no cabo.
- **Suíte da raiz: zero regressões; 77 falhas deliberadas e preexistentes em
  `tools/valida_etapa7`** (os vermelhos do juiz estacionado em `f2f8f3e`, lá
  registrados como "77 failed"); 1434 passaram. A diferença para os 1403 de
  `f2f8f3e` é exatamente 15 (`test_config_livox`, 058) + 16 líquidos (esta
  decisão).
- **Execução real neste PC de dev** (só leitura): checagem 7 APROVADO,
  `perfil nuc: host 192.168.1.2/24, sensor 192.168.1.158 (não consultado)`;
  checagem 8 INCONCLUSIVO, "cabo fora?" — correto, este PC não tem interface
  na rede do lidar.
- `bash -n bin/audita-livox`: limpo.

O lidar e o robô continuam desligados. Nada disto prova rede real.
