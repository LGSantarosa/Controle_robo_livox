# 058 — Máquina e unidade Livox são eixos separados

**Data**: 2026-09-24 (PC de dev; lidar desligado)
**Status**: aplicada nesta branch; validação offline, sem hardware
**Toca**: `setup_livox.sh`, gerador/testes e configuração/documentação de rede
**Não toca**: lista `--packages-select`, etapa 6, launch, navegação ou atuador

---

## 1. Contexto e causa

Até hoje havia um único `robot_base/config/MID360_config.json`, com dois dados
que não têm a mesma identidade:

- o IP da interface da **máquina** que recebe os pacotes;
- o IP da **unidade física** do Mid-360 que os envia.

O `setup_livox.sh` copiava esse par sem validar a máquina local. Em 24-09 ele
teria reinstalado `.2/.158` no notebook cuja interface era `.5`, apagando a
cópia local `.5/.169` que entregou nuvem. O driver consegue parecer inicializar
com o host errado; os dados só não voltam, e o defeito aparece longe dali como
FAST-LIO sem `/Odometry`.

A causa não é “config do robô 3 ausente”. É o modelo: um perfil `robo2` ou
`robo3` continuaria amarrando computador e sensor e passaria a mentir assim
que a unidade migrasse — cenário normal deste projeto.

## 2. Uma inconsistência que não será escondida no nome do perfil

Os registros têm dois pares IP/MAC:

| observação | IP | MAC |
|---|---|---|
| 15-09, NUC / bancada do robô 2 | `192.168.1.158` | `e4:7a:2c:95:df:da` |
| 24-09, notebook / robô 3 | `192.168.1.169` | `e4:7a:2c:90:1d:f1` |

Ao mesmo tempo, o estado e o plano afirmavam que havia “só um Mid-360”,
emprestado entre os robôs. MACs diferentes sugerem duas unidades, mas a gente
não tem ainda a reconciliação física por etiqueta/número de série nem uma
varredura conjunta. Pode haver um segundo sensor ou uma leitura/atribuição
anterior errada.

**Decisão de inventário:** a quantidade de unidades fica explicitamente
**aberta**. `.158` e `.169` são observações datadas, não “sensor do robô 2” e
“sensor do robô 3”. A frase “só há um” deixa de sustentar decisões novas até a
próxima conferência física.

## 3. Decisão

### 3.1 Perfil identifica somente a máquina

`livox_host_profiles.json` contém:

| perfil | endereço local exigido |
|---|---|
| `nuc` | `192.168.1.2/24` |
| `notebook` | `192.168.1.5/24` |

O argumento continua se chamando `--perfil`, mas seu domínio é **máquina**, não
robô. Hostname e nome da interface não entram no contrato: a interface já foi
`enp2s0` e `enp1s0`; o endereço é que o driver usa.

### 3.2 O sensor é uma entrada de cada execução

Sem `--lidar-ip`, o setup varre a sub-rede, considera somente respondentes cujo
MAC tenha o OUI Livox `e4:7a:2c` e exige exatamente uma unidade. Zero ou duas
unidades reprovam em vez de escolher por memória.

Com `--lidar-ip`, o operador faz uma escolha visível e reproduzível. Se o IP
responder, o MAC precisa ter OUI Livox; se não responder, a geração offline é
permitida, mas sai com aviso **`NÃO confirmado`**. Portanto o valor pode ser
sobrescrito sem virar default silencioso, e preparar arquivo não se confunde
com provar hardware.

### 3.3 O host local é pré-condição, não recomendação

Antes do passo 0/5 — portanto antes de dependência, clone, `/usr/local` e build
— o setup exige que IP e máscara do perfil existam em exatamente uma interface
local. O caso real `.2` no perfil × `.5` na placa reprova imediatamente.

### 3.4 O JSON ativo é sempre gerado

O par fixo `.2/.158` sai do repositório. Ficam versionados:

- os perfis de máquina;
- um `MID360_config.template.json` sem IP real;
- o gerador testável `tools/prepara_config_livox.py`.

O resultado temporário combina host validado e sensor descoberto/explicitado,
é copiado para o caminho exigido pelo driver e conferido com `cmp`. O arquivo
ativo segue no clone ignorado e descartável, mas agora é reconstruível.

## 4. Alternativas descartadas

| alternativa | por que não |
|---|---|
| perfis `robo2` e `robo3`, cada um com host + lidar | mistura máquina, robô e unidade; quebra semanticamente no primeiro empréstimo/troca de sensor |
| manter `.2/.158` como default universal | repete exatamente o falso sucesso de 24-09 no notebook `.5` |
| escolher máquina por hostname ou `enp1s0`/`enp2s0` | hostname não prova a rota e o nome da interface já mudou; o contrato real do driver é o endereço |
| sempre exigir o sensor ligado | impediria preparar/reconstruir uma máquina offline; `--lidar-ip` deliberado mantém esse caminho sem fingir medição |
| aceitar qualquer IP que responda | outro equipamento da sub-rede poderia virar “lidar”; resposta viva precisa casar com o OUI Livox |
| gravar `.158` e `.169` num inventário ativo | a divergência física ainda não foi resolvida; registrar evidência é diferente de promovê-la a configuração |

## 5. Prova desta mudança

- `bash -n setup_livox.sh`: passou;
- `python3 -m py_compile tools/prepara_config_livox.py`: passou;
- testes dirigidos: **15 passaram** — perfis sem sensor/robô, template sem IP,
  host/máscara/interface obrigatórios, IP de sensor válido, zero/uma/duas
  unidades na varredura, override offline marcado e MAC não-Livox recusado;
- suíte inteira de `robot_base`: **127 passaram**;
- prova negativa neste PC: `./setup_livox.sh --perfil notebook --lidar-ip
  192.168.1.169` saiu com código 1 porque `.5/24` não existe localmente; não
  criou clone de driver nem fonte do SDK.

O lidar permaneceu desligado. Não foi provado auto-descobrimento contra uma
unidade real, cópia dentro do clone real, stream, FAST-LIO ou `/Odometry`.
A suíte da raiz também não é reivindicada aqui: esta worktree não tem
`install/`; a tentativa chegou a 570 passes e então falhou com
`PackageNotFoundError: robot_motion`, enquanto o `AMENT_PREFIX_PATH` apontava
para outro workspace. O rebuild/suíte com overlay correto é o item 3 separado.

## 6. Dívidas que continuam separadas

- confirmar fisicamente se `.158` e `.169` são duas unidades e anotar
  etiqueta/número de série;
- corrigir o `--packages-select` sem `hoverboard_driver` (item 2); esta decisão
  para antes de chegar nesse defeito e não o mascara;
- rodar a suíte completa/rebuild do item 3 fora desta worktree;
- integrar a pilha física do robô 3 na branch própria da etapa 6.
