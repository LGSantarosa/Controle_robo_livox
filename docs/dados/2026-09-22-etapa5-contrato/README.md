# Etapa 5 — contrato único `TwistStamped` na cadeia do robô 3, 22-09, PC de dev

Treze pastas copiadas **inteiras** de `~/validacao_etapa5/` (`SHA256SUMS`
cobre os 472 arquivos: `sha256sum -c SHA256SUMS`). Robô físico desligado,
MEGA fingida em pty, sem Gazebo. Plano: `docs/PLANO_ETAPA5_ROBO3.md`;
bancada: `bin/valida-etapa5`, `tools/valida_etapa5/`.

| pasta | commit | papel | veredito |
|---|---|---|---|
| `141628_contrato_caracterizacao` | `6640a0f` | 1ª caracterização da cadeia `Twist` | 🔴 25/26 — **régua do timeout estreita demais** |
| `142601_contrato_caracterizacao` | `8747c92` | **baseline oficial da cadeia `Twist`** | ✅ 26/26 |
| `154816_contrato_stamped` | `9c99c20` | 1ª rodada `TwistStamped` | 🔴 **falso negativo** do verificador da placa |
| `155627_diag-mega` | `9c99c20` | diagnóstico que falhou por erro meu (`fases.yaml` ausente ao lado da cópia) | 🔴 |
| `155931_diag-mega` | `9c99c20` | MEGA fingida + `mega_bridge` sozinhos, 3 partidas frias | ✅ causa demonstrada |
| `161116/161227/161323_diag-bridge` | `c914c1c` | tentativas minhas de provar bridge→MEGA, com erros de script | 🔴 |
| `161558_diag-bridge-minimo` | `c914c1c` | prova bridge→MEGA, arranjo mínimo | ✅ |
| `161606_contrato_stamped` | `c914c1c` | recusou: processo meu vazado (ver abaixo) | 🔴 pré-condição |
| `161631_contrato_stamped` | `c914c1c` | recusou: registro do sobe-robo3 obsoleto | 🔴 pré-condição |
| `161648_registro-obsoleto-2` | — | descarte do registro, **sem sinal** | ✅ |
| `161655_contrato_stamped` | `c914c1c` | **cadeia `TwistStamped`** | ✅ **26/26** |

## O que a etapa 5 provou

`joy_vel`, `dpad_vel` e `cmd_vel` são **`geometry_msgs/msg/TwistStamped`** no
grafo (`*/tipos.yaml`), e a conversão acontece só na fronteira do atuador
(`cmd_vel_to_wheels` com `use_stamped`; default cru, o `robot.launch.py` do
caminho legado intocado). Com o contrato novo, **os números não mudaram**:

| prova | `Twist` (142601) | `TwistStamped` (161655) |
|---|---|---|
| `reproducao_14_09` (`sinal:=-1.0 frente:=1.0 bitola:=0.3225`) | frente −120, esquerda **97** | idem, byte a byte |
| `padrao_hoje` | frente −120, esquerda **−256** | idem |
| `frente_mais_1` (só `frente` muda) | frente **+120**, esquerda −256 | idem — isola o `linear_sign` |
| prioridade (direcional × analógico) | o direcional manda | idem |
| timeout do mux | analógico volta em 0,35 s | **0,35 s** |
| homem-morto | zero ao soltar, depois silêncio | idem |
| perda do controle | frames cessam na hora; **último frame não-zero** | idem (último 9 ms antes) |
| TF `base_link → livox_frame` | yaw 0, diferença 0 do URDF | idem |

**Perda do controle (D4, ainda aberta):** ninguém na cadeia zera. O mux não
repete nem publica zero, o `cmd_vel_to_wheels` não guarda estado e o
`mega_bridge` só manda frame quando chega setpoint. Quem para o robô é o
**watchdog do firmware**, que esta bancada **não** alcança.

## Os dois defeitos que a etapa achou (e que não eram do contrato)

1. **Régua do timeout estreita demais** (`141628`): um frame do direcional
   ainda em trânsito chegou 2 ms depois da troca de fase, antes do zero. A
   régua passou a tolerar, até `t0 + 0,1 s` (dois períodos do `/joy` a 20 Hz),
   **só** o frame da fase anterior — e o zero tem de vir nesse prazo
   (`8747c92`). Os frames reais da pasta reprovada passam pela régua nova.
2. **Aviso de descoberta lido como placa desligada** (`154816`): com a
   descoberta incompleta, o `ros2 topic echo` imprime "does not appear to be
   published yet" **no stdout**; a saída fica não-vazia e sem `present:`, e o
   `sobe-robo3` acusava "placa desligada". Reproduzido isolado em `155931`
   (1 de 3 partidas frias) com a bateria publicando a 4,9 Hz o tempo todo, e
   `/hoverboard/wheel_velocities` a ~50 Hz. Conserto em `c914c1c`: tipo
   explícito, sem daemon, repetição até ter mensagem com `present:` (teto de
   10 s, cada tentativa em `battery_check.log`), e quatro vereditos distintos
   (descobrindo / sem leitura válida / `present:false` / `present:true`).

## Tropeços meus, registrados

- `155627_diag-mega`: a cópia instrumentada da MEGA fingida procura o
  `fases.yaml` ao lado dela, e eu não copiei.
- `161116` e `161227/161323_diag-bridge`: `--no-daemon` não existe no
  `ros2 topic pub`, e `--once` publica antes de a assinatura casar — a
  mensagem se perde. A prova saiu no arranjo mínimo (`161558`).
- `161606`: o arranjo mínimo deixou um `mega_bridge` vivo (matei o `ros2 run`,
  não o nó). A bancada recusou subir por cima — foi assim que eu soube.
- `161631`: o registro do `sobe-robo3` ficou da rodada `154816`, que voltou
  cedo sem chegar ao `--mata`. Descartado em `161648`, sem sinal.

## O que isto NÃO prova

Nada de hardware: nem o watchdog do firmware, nem a placa, nem o
`frente:=-1.0` no chão. Não prova a `pilha` com `robo:=3` (etapa 6) nem
qualquer coisa de navegação.
