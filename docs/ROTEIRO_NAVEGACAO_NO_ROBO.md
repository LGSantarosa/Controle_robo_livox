# Roteiro — a primeira navegação autônoma no robô

> **Para o assistente que chegar frio nesta sessão.** Escrito em 12-08, no dev.
> É o roteiro de UMA sessão; o geral (método, acesso, armadilhas de ssh) é o
> `docs/PROXIMA_SESSAO_NO_ROBO.md` e continua valendo — leia o §1 e o §2 dele
> antes, principalmente a parte de achar o NUC na rede.
>
> **Leia antes**: decisões **019** (a pilha sobe sem argumento) e **020** (a zona
> morta medida), e a entrada 11-08 (3ª leva) do `DIARIO` — foi a última ida.

---

## 0. O que esta sessão responde, e o que ela NÃO responde

**O robô nunca recebeu um objetivo e foi até ele.** Cada elo da corrente já foi
visto de pé separado; a corrente inteira, com o robô andando, nunca rodou.

```
/goal_pose -> bt_navigator -> planner -> /plan -> path_follower
   -> heading_controller -> /auto_vel_raw -> collision_monitor -> /auto_vel
   -> twist_mux (<- /key_vel, o humano) -> compensador_rumo -> rodas
```

⚠️ **Não é sessão de sintonia.** Se o rumo estiver ruim, anote e siga — mexer em
ganho com a bateria correndo é como se perde uma tarde. O que se mede aqui é
*a corrente*, não a qualidade do seguimento.

---

## 1. O método, em cinco linhas

1. **O DONO SÓ RODA.** Você lê CSV e `rosout` por ssh; ele posiciona, liga, olha
   e aperta tecla. Nunca peça para ele relatar console.
2. **UMA CORRIDA POR VEZ**: diga o que o robô vai fazer, quanto espaço precisa,
   e **espere o "pode"**.
3. **RESPOSTA CURTA** com o robô ligado: número e veredito, 2–4 linhas.
4. **AVISE ligado × desligado**, e avise quando já pode desligar.
5. **NÃO EXPANDA ESCOPO** com a bateria correndo.

🔴 **A ordem dos experimentos é por risco crescente, e não é negociável**: o
robô só anda sozinho depois que o freio de mão do humano estiver PROVADO.

---

## 2. Deploy (robô LIGADO, nada se move)

```bash
# no dev:
git bundle create /tmp/repo.bundle --all
scp /tmp/repo.bundle bara@<ip>:/tmp/
# no NUC:
cd ~/Controle_robo_livox
git fetch /tmp/repo.bundle main && git reset --hard FETCH_HEAD
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros2_packages --symlink-install \
    --packages-select hoverboard_driver robot_base robot_motion
source install/setup.bash
mkdir -p /tmp/logs
```

⚠️ **`colcon build` SEMPRE** — `reset --hard` troca o fonte, não o `install/`.

🔴 **ANTES DE QUALQUER `git clean`: os 8 CSV de 2,5 m de 11-08 ainda estão no
NUC.** O `scp` deles não aconteceu (o robô desligou antes). Puxe primeiro:

```bash
scp bara@<ip>:~/Controle_robo_livox/docs/dados/2026-08-11-*/*.csv /tmp/resgate/
```

⚠️ **Nó novo com `nohup ... &` por ssh não fecha o canal** — use
`setsid ... < /dev/null > log 2>&1 &`.

---

## 3. Subir a base, e conferir o que a 019 e a 020 prometeram

```bash
setsid ros2 launch robot_base base.launch.py > /tmp/logs/base.log 2>&1 < /dev/null &
python3 tools/banco/sessao.py --checar      # wheel_separation TEM de dar 0.2700
ros2 run tf2_ros tf2_echo odom base_link    # TEM de responder SEM ninguém ajudar
```

🟢 **O `tf2_echo` responder sozinho é a confirmação da 019.** Em 06-08, 10-08 e
11-08 alguém teve de matar o `tf_odom` e subir na mão com
`-p frame_da_pose:=livox_frame`. Se precisar de novo, a 019 não chegou ao NUC —
confira o commit deployado antes de seguir.

⚠️ **O `nuvem_pontos` sobe junto e FICA VIVO** (decisão do dono, 12-08). Ele
come ~91% de um core convertendo a nuvem em Python. Não mate: o `corrida_nav.py`
mede a taxa do `/Odometry` **durante a corrida** e é essa medida que decide se o
custo dele é um problema. Em 11-08 a conferência foi com o robô parado, e isso
não responde.

---

## 4. Subir a pilha — e a confirmação de UMA LINHA da decisão 020

```bash
setsid ros2 launch robot_motion pilha.launch.py > /tmp/logs/pilha.log 2>&1 < /dev/null &
```

🟢 **SEM ARGUMENTO NENHUM.** Desde a 019 o default segue o `sim`: no robô real
já sobe com `mapa:=nenhum` e sem `rviz`. Se alguém digitar `mapa:=...` está
pedindo o mapa da pista SIMULADA.

```bash
grep -i "pivô" /tmp/logs/pilha.log
```

🎯 **TEM de dizer `pivô DISPONÍVEL acima de 0,13 rad/s`.** Se disser
`INDISPONÍVEL`, a decisão 020 não chegou (o `movimentacao.yaml` no NUC ainda
tem `zona_morta: 0.15`) — e aí o robô vai abrir todas as curvas. **Pare e
confira o deploy**; é barato agora e caro depois.

```bash
python3 tools/banco/checa_pilha.py --csv docs/dados/2026-08-XX-nav/preflight.csv
```

Espere **20/20**. O ❌ do "uma pilha só" pode ser o falso positivo do `bash -c`
do ssh contando como segunda pilha — confira com `ps`, nunca com `pgrep -c`.

---

## 5. EXPERIMENTO 1 — o homem-morto (robô LIGADO, **nas mãos do dono**)

🔴 **Este é o experimento que autoriza todos os outros.** É o único da lista
cuja falha é *pior que não ter a função*: você acha que tem freio de mão e não
tem. Em 06-08 o `/key_vel` não publicou **nada** — 44 s de gravação, zero
amostras.

**Robô no colo ou com as rodas fora do chão.** Num terminal ssh separado:

```bash
bin/robot-key                    # WASD; `q` sai
```

Em outro:

```bash
python3 tools/banco/homem_morto.py --csv docs/dados/2026-08-XX-nav/homem-morto-a.csv
```

O dono aperta `w`, **segura ~2 s** e **solta**. Ctrl-C fecha o arquivo.

O `bin/robot-key` já foi consertado em 11-08 (o `set -u` matava o script antes
do teleop existir). Se ainda não sair nada, o teleop agora relata a cada 2 s:

```bash
ros2 topic echo /rosout --field msg | grep -i "publicadas="
# publicadas=N  teclas_lidas=N  ultima='w'  ouvintes_de_/key_vel=N
```

Os três contadores separam as três hipóteses:

| sintoma | onde a corrente parte |
|---|---|
| `teclas_lidas=0` | o `select` não vê o stdin — terminal sem tty (ssh sem `-t`) |
| lê mas `publicadas=0` | o laço não chega ao `publica()` |
| publica mas `ouvintes=0` | o mux não assinou — nome do tópico ou QoS |

**Esperado**: [A] ≤ 0,45 s da solta até o `/key_vel` zerar; total até a roda
parar ~1,0 s (0,4 do homem-morto + ~0,5 do desliga da placa).

🛑 **Se a roda não parar, a sessão ACABA aqui.** Sem freio de mão o robô não
anda sozinho.

---

## 6. EXPERIMENTO 2 — o ff do dia (robô LIGADO, **2,5 m livres**)

A curvatura crua deriva 19% em 5 minutos (10-08), então ela se mede no começo de
cada sessão. **Três corridas SEM compensador:**

```bash
D=docs/dados/2026-08-XX-nav
for n in a b c; do
  python3 tools/banco/ensaio.py --ensaio reta --v 0.25 --wz 0 \
      --dur 16 --espaco 2.5 --janela 0.5 --csv $D/cru-$n.csv
done
python3 tools/banco/medir.py --resumo curvatura $D/cru-*.csv
```

⚠️ **Sem `--topico` aqui, de propósito**: estas três são CRUAS, o `ensaio.py`
fala direto com o atuador e o compensador não entra. É o oposto das corridas de
sintonia, onde `--topico /compensador_rumo/cmd_vel` é obrigatório.

⚠️ **`--espaco 2.5`, nunca 1,2** — a régua curta corta no cruzamento de zero do
S e aprova robô oscilando (medido em 10-08: a MESMA corrida dá −0,0162 em 1,2 m
e +0,0882 em 2,5 m).

Ele **se recusa** com n<3 ou dispersão acima de 5% — recusa é resultado, não
erro. Saindo a linha, derrube a pilha e suba de novo com ela:

```bash
ros2 launch robot_motion pilha.launch.py curv_frente:=<medido> \
    curv_medido_em:=2026-08-XX
ros2 topic echo /rosout --field msg | grep -i "^ff "   # TEM de dizer "ff MEDIDO"
```

🔵 **De quebra, esta é a primeira medida crua depois da 020.** Compare a
velocidade realizada com os 0,29–0,31 m/s de 11-08: se mudou, a zona morta mexeu
em algo que eu não previ.

---

## 7. EXPERIMENTO 3 — o primeiro objetivo (robô LIGADO, **3 m livres à frente**)

🟢 **Espaço VAZIO.** O reflexo ainda não foi provado; quem protege aqui é o
teclado do experimento 1 e a ausência de obstáculo.

⚠️ **O alvo é relativo a ONDE O ROBÔ ESTAVA quando a base subiu** — desde a 018
o `odom` é a pose do `base_link` na largada, e o `map` está colado nele. Alvo
`(2.0 · 0.0)` = 2 m à frente do ponto de partida.

**Com o `bin/robot-key` rodando em outro terminal**, e o dono com a mão nele:

```bash
python3 tools/banco/corrida_nav.py --alvo 2.0 0.0 --teto-s 40 \
    --csv docs/dados/2026-08-XX-nav/objetivo-a.csv
```

O instrumento manda o objetivo pela ação (cancela ao sair), grava a corrente
inteira a 20 Hz e imprime o veredito. Repetir **3 vezes**.

**O que cada linha do veredito responde:**

```
chegada        chegou dentro de 0,25 m? em quanto tempo?
caminho        tortuosidade ~1,0 = foi reto; 1,5+ = passeou
replanejou     muitos replanejamentos SEM andar = robô preso
reflexo        "não agiu" é o esperado AQUI (não há obstáculo)
HUMANO         se aparecer, a corrida é mista e não conta
pose           10 Hz mediana e pior intervalo ≤ 0,30 s
```

🎯 **A pergunta do `nuvem_pontos` se responde nesta linha `pose`.** Pior
intervalo acima de 0,30 s com o robô andando = o LIO engasgou, e o suspeito nº 1
é a ponte em Python. Se acontecer, é dado novo e muda o plano — não mate o nó
no meio da sessão sem me falar.

---

## 8. EXPERIMENTO 4 — o reflexo (robô LIGADO, **caixa no caminho**)

Mesma corrida, com uma **caixa de ~50 cm de altura** a ~1,5 m à frente, no meio
do caminho.

⚠️ **50 cm, não 40** — o Mid-360 está a 42 cm do chão (trena, 05-08) e a 0,5 m
de distância ele só vê acima de 36 cm. Caixa baixa é invisível **de perto**, e
isso não é defeito do reflexo, é geometria.

```bash
python3 tools/banco/corrida_nav.py --alvo 3.0 0.0 --teto-s 60 \
    --csv docs/dados/2026-08-XX-nav/reflexo-a.csv
```

**Dois resultados são bons, e eles são diferentes:**

- `reflexo agiu Nx` + robô parado antes da caixa → o freio funciona;
- `reflexo não agiu` + tortuosidade > 1,2 + chegou → **melhor ainda**: o costmap
  viu e o planejador contornou, sem precisar do freio.

🔴 **O ruim é chegar com `reflexo não agiu` e tortuosidade ~1,0**: significa que
passou por cima da caixa no plano e o freio não pegou. Aí a percepção está
cega e a sessão para — `tools/banco/percepcao.py` diz se o costmap marcou.

---

## 9. Fechamento (antes de desligar)

```bash
python3 tools/banco/checa_pilha.py --csv docs/dados/2026-08-XX-nav/preflight2.csv
scp -r bara@<ip>:~/Controle_robo_livox/docs/dados/2026-08-XX-nav /tmp/
```

E o `ambiente.txt` da pasta, com: **bateria no início e no fim**, piso, commit
deployado, e o que foi subido à mão (idealmente nada).

🔴 **Puxe os CSV ANTES de desligar.** O NUC cai junto com o robô — não tem
alimentação separada, e `/tmp/logs` não sobrevive ao reboot.

---

## Apêndice — armadilhas que já morderam

- **`pkill -f <nome>` mata a própria sessão ssh** se a linha do ssh contiver o
  nome. Mordeu duas vezes em 11-08. Use `ps` e mate por PID.
- **`ros2 launch` não morre com os nós.** Matar filhos por PID e deixar o launch
  vivo empilhou 3 pilhas simultâneas em 07-08, e o sintoma foi bringup abortando
  com cara de bug de código.
- **`ros2 node list` mostra fantasma** do daemon mesmo com zero processos
  (`ros2 daemon stop && start` limpa). Para saber o que está vivo, `ps`.
- **A placa desligada com a base de pé não dá sintoma**: o odom de roda é
  `open_loop` e mente. Quem prova é `/hoverboard/connected`.
- **Erros de TF `NaN` nas juntas das rodas são cosméticos** — o driver publica
  velocidade, não posição.
