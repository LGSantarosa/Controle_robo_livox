# Roteiro — o controle Xbox no robô (decisão 043)

> **Para a tarde de 18-08.** É o roteiro de UMA entrega: pôr o Xbox para
> dirigir o robô 2. O roteiro geral (método, deploy, armadilhas de acesso)
> é o `docs/PROXIMA_SESSAO_NO_ROBO.md` e continua valendo inteiro.
>
> **Leia antes**: `docs/decisoes/043-o-freio-de-mao-ganha-um-controle.md`.
>
> Nada disto rodou no robô. O que está no repo passou por 645 testes no dev e
> por zero segundos de hardware.

---

## 0. A ideia deste roteiro, e é o que ele tem de diferente

**A FASE A INTEIRA RODA COM A BATERIA DAS RODAS DESLIGADA.**

Parear, medir os botões e provar o homem-morto **não exigem roda nenhuma** — o
`joystick.launch.py` sobe sozinho, sem a base, e eu leio pelo ssh se o
`/joy_vel` só publica com o LB apertado. Isso é a validação de segurança
inteira feita com o robô fisicamente incapaz de se mover.

Só depois disso a bateria das rodas entra.

```
FASE A   bateria das RODAS DESLIGADA   parear · medir botões · homem-morto
FASE B   bateria das RODAS LIGADA      pilha completa · dirigir
```

⚠️ **São duas baterias independentes.** A do NUC + Livox alimenta a Fase A
inteira e **não tem telemetria** — olhe nela, ninguém avisa. A das rodas é a
que o `/hoverboard/battery_voltage` mede, e ela fica **desligada** até a Fase B.

---

## 1. Acesso — confira os DOIS lados antes de qualquer coisa

🔴 Já custou tempo duas vezes. O robô "some" e a causa é o SSID.

```bash
nmcli -t -f active,ssid dev wifi | grep '^yes'   # tem de ser "Trafico de banana"
ip -4 addr show wlo1 | grep inet                 # você fica em 10.244.3.x
ssh -o ServerAliveInterval=15 bara@<ip-do-nuc>   # o IP MUDA a cada sessão
```

Não respondeu? Varra a /24 (a chave de host do NUC já está no `known_hosts`):

```bash
seq 1 254 | sed 's|^|10.244.3.|' | xargs -P 254 -I{} sh -c \
  'timeout 3 bash -c "</dev/tcp/{}/22" 2>/dev/null && echo {}'
```

---

## 2. Deploy — robô sem se mover, rodas ainda DESLIGADAS

O NUC não tem chave do GitHub (igual 30-07 e 14-08), então vai de bundle:

```bash
# no DEV
git bundle create /tmp/repo.bundle --all
scp /tmp/repo.bundle bara@<ip>:/tmp/

# no NUC
cd ~/Controle_robo_livox
git fetch /tmp/repo.bundle main && git reset --hard FETCH_HEAD
git log --oneline -1        # tem de dar: 8f97636 o robo ganha um controle xbox
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros2_packages --symlink-install \
    --packages-select hoverboard_driver robot_base robot_motion
source install/setup.bash
```

⚠️ **`colcon build` SEMPRE** — o `reset --hard` troca o fonte, não o `install/`.
O `teleop_xbox.yaml` e a `joystick.launch.py` só existem para o ROS depois dele.

Confira que os dois chegaram ao `share`:

```bash
ls install/robot_motion/share/robot_motion/config/teleop_xbox.yaml \
   install/robot_motion/share/robot_motion/launch/joystick.launch.py
```

---

# FASE A — rodas DESLIGADAS

## 3. Parear o controle

Modo pareamento: ligue no botão **Xbox** (aceso fixo), depois segure o botão
**PAIR** (o pequeno na aresta de cima, ao lado do USB-C) por 3–5 s até o botão
Xbox piscar **RÁPIDO**.

```bash
# no NUC
cd ~/Controle_robo_livox && ./pair-xbox.sh
```

Ele cuida do BlueZ (ControllerMode=dual, agent NoInputNoOutput, FIFO, módulos)
e **só declara sucesso quando o `/dev/input/jsN` aparece de verdade** — não
quando o BlueZ diz "Connected". Se ele parar antes, o texto do erro já traz o
diagnóstico; me mande a saída.

> 🔁 **Se o pareamento não sair em ~10 min, NÃO gaste a tarde nele.** Ligue o
> controle **por cabo USB**: o `xpad` é nativo do kernel, o `/dev/input/js0`
> aparece sozinho e **todo o resto deste roteiro funciona igual** — a
> auto-detecção casa pelo nome, não pelo transporte. Perde-se andar junto do
> robô, não a sessão. O pareamento vira assunto de outro dia.

## 4. 🔴 MEDIR OS BOTÕES — este passo não é opcional

O `teleop_xbox.yaml` traz `LB=6` e `RB=7`. Esses números foram medidos **no
robô 1**, não neste controle. Se divergirem, o homem-morto cai num botão que
ninguém aperta e o robô simplesmente não anda.

```bash
./scripts/js_mapping.py /dev/input/js0     # use o jsN que o pair-xbox.sh disse
```

Aperte **LB** e depois **RB**, e mande os números.

**Se derem 6 e 7**: segue. **Se derem outros**, é uma linha cada, no NUC:

```bash
# ros2_packages/robot_motion/config/teleop_xbox.yaml
enable_button: <o do LB>
enable_turbo_button: <o do RB>
```

e então, ainda no NUC, o teste que trava a coerência tem de continuar verde:

```bash
python3 -m pytest ros2_packages/robot_motion/test/test_joystick_coerente.py -q
colcon build --base-paths ros2_packages --symlink-install --packages-select robot_motion
```

⚠️ Se mudar, **avise** — a correção volta pro dev e vira commit; não pode
ficar só no NUC.

## 5. O homem-morto, provado sem roda nenhuma

Sobe só o joystick, sem base, sem Nav2:

```bash
export ROS_DOMAIN_ID=42
mkdir -p ~/logs_sessao        # a Fase A roda antes do sobe-robo, que o criaria
cd ~/Controle_robo_livox && source install/setup.bash
setsid nohup ros2 launch robot_motion joystick.launch.py \
    > ~/logs_sessao/joystick.log 2>&1 < /dev/null &
sleep 5
grep "usando js" ~/logs_sessao/joystick.log
```

Agora as **três medidas** que fecham a segurança. Rode cada uma e me mande a
taxa:

```bash
# 5a) o controle está falando?  (esperado: ~20 Hz, mesmo sem tocar em nada)
timeout 8 ros2 topic hz /joy

# 5b) LB SOLTO, mexendo o analógico  -> tem de dar NADA / "no new messages"
timeout 8 ros2 topic hz /joy_vel

# 5c) LB APERTADO, mexendo o analógico -> tem de publicar (~20 Hz)
timeout 8 ros2 topic hz /joy_vel
```

```
5a com taxa · 5b MUDO · 5c com taxa   ->  homem-morto OK, pode ir pra Fase B
5b PUBLICANDO                         ->  🔴 PARE. O dead-man está no botão
                                           errado — volte ao passo 4.
5a mudo                               ->  o controle não está conectado
5c mudo mas 5a com taxa               ->  o config não carregou; me mande
                                           ~/logs_sessao/joystick.log
```

E o conteúdo, pra eu conferir que o eixo não está invertido — empurre o
analógico **pra frente** com o LB apertado:

```bash
timeout 5 ros2 topic echo /joy_vel --once
```

> `twist.linear.x` tem de ser **positivo** indo pra frente, e
> `twist.angular.z` **positivo** virando pra **esquerda**.

Mate este teste antes da Fase B (a pilha sobe o joystick sozinha):

🔴 **Por PID, nunca `pkill -f`** — em 12-08 o `pkill -f` matou a sessão ssh e o
`scan_2d` junto. É o motivo de o `sobe-robo` ter um aviso escrito sobre isso.

```bash
ps -eo pid,args | grep -E "joystick.launch.py|joy_node|teleop_twist_joy" | grep -v grep
# confira a lista, então mate SÓ esses PIDs:
kill -9 <pid> <pid> <pid>
# e confirme que não sobrou órfão:
ps -eo pid,args | grep -E "joy_node|teleop_twist_joy" | grep -v grep   # tem de sair vazio
```

---

# FASE B — bateria das RODAS LIGADA

⚠️ **Daqui em diante o robô pode se mover.** Espaço livre à frente, e ninguém
entre ele e a parede.

## 6. Subir a pilha

```bash
bash bin/sobe-robo
```

O passo **4/5** é novo e tem de imprimir duas coisas:

```
== 4/5 controle Xbox (o freio de mão) ==
   [joystick] usando js0 — /dev/input/js0 anuncia 'Xbox Wireless Controller'
   /joy: average rate: 20.0
```

E o pré-voo do fim continua mandando: **só mande objetivo se as DUAS derem
`Translation`** (`map -> base_link` e `odom -> base_link`).

## 7. O homem-morto, agora com as rodas

Primeiro comando da fase, e é o de não-movimento:

**Com o LB SOLTO, mexa o analógico esquerdo em todas as direções.**

```
rodas PARADAS      ->  ✅ pode dirigir
rodas se mexendo   ->  🔴 SOLTE TUDO e desligue a bateria das rodas.
                        Não dirija. Me chame.
```

Este é o único passo que eu não consigo julgar por ssh — só você vê a roda.

## 8. Dirigir

Segure o **LB** e mexa o analógico esquerdo. **RB** = turbo. Soltou o LB, ele
para.

⚠️ **O que você VAI sentir, e não é defeito do ajuste:** girar **parado** dá um
tranco de ~95°. É o módulo único da placa (2,204 rad/s, decisão 023) — o mesmo
"pivô forte demais" de 14-08, agora no seu dedo. **Andando**, o giro é
proporcional e se comporta. Baixar o ganho não conserta isso e piora o giro
bom; se incomodar, vira assunto de outro dia, com medida.

Ordem sugerida, do manso pro bravo:

```
1. reto pra frente, ~3 m, sem turbo
2. reto de ré, curto
3. curva andando (frente + lateral juntos)  <- é aqui que ele fica bom
4. giro parado                              <- é aqui que vem o tranco
5. turbo (RB) numa reta, com espaço
```

## 9. A medida que fecha a conversa do `segura_rumo`

Na corrida 1 (reto ~3 m, sem turbo), o CSV grava sozinho em `~/logs_robo2`.
Eu leio o desvio lateral por ssh e isso responde, **sem código novo**, se o
feedforward sozinho basta ou se a malha PI faz falta no teleop (043 §5).

Não precisa fazer nada além de dirigir reto e me avisar.

---

## 10. Derrubar tudo

```bash
bash bin/sobe-robo --mata
```

O `vivos()` agora enxerga o `joy_node` e o `teleop_twist_joy` (antes não
enxergava — eles rodam de `/opt/ros`). Ele só diz "🟢 limpo" com **zero**
processos restantes e a porta 5000 livre. Se disser 🔴, **não suba por cima**.

---

## 11. O que me mandar

```
passo 4    os números do LB e do RB
passo 5    as três taxas (5a, 5b, 5c) e o echo do /joy_vel
passo 6    o bloco 4/5 do sobe-robo
passo 7    "parou" ou "mexeu"
passo 8    o que você sentiu em cada uma das 5 manobras
sempre     ~/logs_sessao/joystick.log se algo não bater
```

Eu puxo CSV e `rosout` sozinho por ssh — não relate console.

---

## 12. Critério de sucesso da tarde

```
✅ o controle pareia e o /dev/input/jsN aparece
✅ LB e RB confirmados (ou o config corrigido e o teste verde)
✅ 5b MUDO com o LB solto            <- a segurança
✅ rodas paradas com o LB solto      <- a segurança, agora física
✅ ele anda reto e faz curva na sua mão
```

O tranco do giro parado **não** reprova a tarde — ele é conhecido, medido e
documentado desde a decisão 023.
