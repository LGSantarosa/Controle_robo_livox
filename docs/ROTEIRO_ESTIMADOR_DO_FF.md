# Roteiro — o estimador do ff no robô (decisão 016)

> **Para o assistente que chegar frio nesta tarde.** Escrito em 11-08, de manhã,
> logo depois de implementar o estimador. É um roteiro de UM ensaio, não da
> sessão inteira: o roteiro geral (método, deploy, armadilhas) é o
> `docs/PROXIMA_SESSAO_NO_ROBO.md`, e ele continua valendo.
>
> **Leia antes**: `docs/decisoes/016-estimador-do-ff-por-drenagem-do-integrador.md`
> e a entrada 10-08 do `docs/DIARIO.md` (a sessão que motivou tudo isto).

---

## 0. O método, em cinco linhas

1. **O DONO SÓ RODA.** Você lê CSV e `rosout` por ssh; ele posiciona, liga,
   olha e aperta tecla. Nunca peça para ele relatar console.
2. **UMA CORRIDA POR VEZ**: diga o que o robô vai fazer, quanto espaço precisa,
   e **espere o "pode"**.
3. **RESPOSTA CURTA** com o robô ligado: o número e o veredito, 2–4 linhas.
   Tabela e ressalva vão para o commit e o diário.
4. **AVISE ligado × desligado**, e avise quando já pode desligar.
5. **NÃO EXPANDA ESCOPO** com a bateria correndo.

---

## 1. Acesso — o que custou meia hora em 10-08

🔴 **PRIMEIRO confira em que wifi o SEU PC está.** Em 10-08 o robô "sumiu" e eu
varri a rede errada inteira, casei chaves de host e cheguei a mandar o dono
rodar `ssh-copy-id` numa máquina de terceiro. A causa era o SSID.

```bash
nmcli -t -f active,ssid dev wifi | grep '^yes'   # tem de ser "Trafico de banana"
ip -4 addr show wlo1 | grep inet                 # você fica em 10.244.3.x
ping -c2 10.244.3.205                            # o NUC em 10-08; o IP MUDA
ssh -o ServerAliveInterval=15 bara@10.244.3.205  # a chave já está autorizada
```

Se não responder, varra a /24 e confirme pela **chave de host** antes de tentar
senha (o `known_hosts` já tem a do NUC):

```bash
seq 1 254 | sed 's|^|10.244.3.|' | xargs -P 254 -I{} sh -c \
  'timeout 3 bash -c "</dev/tcp/{}/22" 2>/dev/null && echo {}'
```

---

## 2. Deploy (robô LIGADO, nada se move)

```bash
git bundle create /tmp/repo.bundle --all           # no dev
scp /tmp/repo.bundle bara@<ip>:/tmp/
# no NUC:
cd ~/Controle_robo_livox
git fetch /tmp/repo.bundle main && git reset --hard FETCH_HEAD
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros2_packages --symlink-install \
    --packages-select hoverboard_driver robot_base robot_motion
source install/setup.bash
mkdir -p /tmp/logs            # NÃO sobrevive ao reboot, e o NUC cai com o robô
```

⚠️ **`colcon build` SEMPRE** — o `reset --hard` troca o fonte, não o `install/`.

⚠️ **Ao subir nó com `nohup ... &` por ssh o canal não fecha** e o seu comando
morre no timeout com o processo vivo. Use `setsid ... < /dev/null > log 2>&1 &`.

---

## 3. Subir a base (robô LIGADO)

```bash
setsid ros2 launch robot_base base.launch.py > /tmp/logs/base.log 2>&1 < /dev/null &
python3 tools/banco/sessao.py --checar        # TEM de dar wheel_separation = 0.2700
ps -eo pid,args | grep -E "[f]astlio_mapping|[l]ivox_ros_driver2_node"   # UM de cada
```

⚠️ **`ps`, nunca `pgrep -c`** — ele casa com a própria linha do ssh. E o
`checa_pilha.py` conta processo lendo `ps` justamente por isso; ainda assim ele
conta o SEU `bash -c` se a string estiver na linha de comando (falso positivo
visto em 10-08).

🟢 **Este ensaio NÃO precisa do Nav2.** Só base + compensador solto. Se alguém
quiser subir a pilha para outra coisa, o `tf_odom` precisa de
`-p frame_da_pose:=livox_frame` (ver o roteiro geral, §4).

⚪ **Erros de TF `NaN` nas juntas das rodas são cosméticos** — o driver não
publica posição, só velocidade. Não afetam `/Odometry` nem o ensaio.

⚠️ **Sobe um nó novo junto (11-08): o `nuvem_pontos`**, que converte a nuvem do
Livox para `PointCloud2` (decisões 017/018). **Este ensaio não precisa dele** —
se o NUC estiver apertado de CPU (ele converte ~20 mil pontos por quadro, a
10 Hz, em Python), mate por PID e siga. O que NÃO pode faltar é o `/Odometry` a
10 Hz: confira antes e depois de matar.

📋 **ANOTE A TENSÃO DA BATERIA agora e no fim.** Em 10-08 ela foi pedida duas
vezes e a sessão andou sem ela — é a única condição do dia que ficou sem
registro, e a deriva de planta é exatamente o que ela ajudaria a explicar.

---

## 4. EXPERIMENTO 1 — a planta de hoje (3 corridas, 1,2 m, SEM compensador)

**Para quê**: dá o número contra o qual o `curv_hat` vai ser julgado no fim.
Sem ele, "o estimador convergiu" não tem alvo.

🛑 **Nenhum compensador de pé.** Se estiver, mate por PID.

```bash
D=docs/dados/$(date +%Y-%m-%d)-estimador && mkdir -p $D
python3 tools/banco/ensaio.py --ensaio reta --v 0.25 --wz 0 \
    --dur 12 --espaco 1.2 --janela 0.5 --csv $D/cru-a.csv
# repetir -b e -c, do MESMO ponto e MESMO rumo
python3 tools/banco/medir.py --resumo curvatura $D/cru-*.csv
```

⚠️ **`--janela 0.5` sempre no robô** (o padrão de 0,2 s sobre pose a 10 Hz
inventa ruído). **Espaço em TODAS as direções**: sem compensador ele descreve
círculo de ~1,2 m de raio.

**Em 10-08 deu 0,82 no começo e 0,97 vinte minutos depois** — se hoje der algo
entre 0,80 e 1,00, é a mesma planta.

---

## 5. EXPERIMENTO 2 — CONTROLE: compensador SEM estimador (3 × 2,5 m)

A semente é **deliberadamente velha**: `−0.8275`, a média das duas primeiras
corridas de 10-08. É o que uma medida "do começo da sessão" teria dado, e é
contra ela que o estimador tem de mostrar serviço.

```bash
setsid ros2 run robot_motion compensador_rumo --ros-args \
    -p curv_frente:=-0.8275 -p curv_medido_em:=2026-08-10-VELHO \
    > /tmp/logs/comp-controle.log 2>&1 < /dev/null &
grep "ff " /tmp/logs/comp-controle.log        # confira a linha que ele anuncia
```

```bash
for n in a b c; do   # UMA POR VEZ, esperando o "pode" entre elas
python3 tools/banco/ensaio.py --ensaio reta --v 0.25 --wz 0 \
    --dur 16 --espaco 2.5 --janela 0.5 \
    --topico /compensador_rumo/cmd_vel --csv $D/controle-$n.csv
done
```

⚠️ **`--topico /compensador_rumo/cmd_vel`** é o que faz a corrida passar pelo
compensador. Sem isso o `ensaio.py` fala direto com o atuador e o compensador
não participa (é assim que se roda a condição "sem compensador").

**Esperado**: as três praticamente iguais. O erro não melhora sozinho.

---

## 6. EXPERIMENTO 3 — TESTE: o mesmo, com `adapta:=true` (3 × 2,5 m)

⚠️ **MATE E SUBA o nó** — `ros2 param set` não pega (armadilha conhecida).

```bash
# mate o compensador anterior pelo PID (e confira que sobrou UM)
setsid ros2 run robot_motion compensador_rumo --ros-args \
    -p curv_frente:=-0.8275 -p curv_medido_em:=2026-08-10-VELHO \
    -p adapta:=true > /tmp/logs/comp-adapta.log 2>&1 < /dev/null &
ps -eo pid,args | grep "[c]ompensador_rumo"     # duas linhas: ros2 run + o nó
```

Acompanhe o aprendizado ao vivo (é WARN de propósito, a cada 2 s):

```bash
ros2 topic echo /rosout --field msg | grep -i "^curv_hat"
```

Mesmas três corridas, mesmo ponto, mesmo rumo, uma por vez.

**A PREVISÃO FALSIFICÁVEL:**

```
                corrida a     corrida b        corrida c
controle        igual         igual            igual
COM estimador   igual à a     MENOR que a a    menor ainda
                              e o curv_hat andando para a planta de hoje
```

- se **b empatar com a**, o mecanismo não está agindo — confira no `rosout` se
  o `curv_hat` se moveu. Se não se moveu: o nó subiu sem `adapta:=true`, ou a
  velocidade ficou abaixo de 0,05 m/s, ou `ki` foi zerado;
- se o `curv_hat` **encostar no grampo** (±0,5 da semente, ou seja −1,3275),
  a semente está errada ou a referência de rumo é ruim — **não é deriva**.

---

## 7. EXPERIMENTO 4 — devagar, para ver se ASSENTA (2 × 2,5 m a 0,15 m/s)

Esta é a pergunta que a sala nunca deixou responder: o rumo **volta e fica**, ou
oscila para sempre? Em 10-08 a melhor corrida ainda subia no corte, aos 10 s.

**A 0,15 m/s os mesmos 2,5 m viram ~17 s de laço em vez de 10** — mais tempo de
malha por metro de chão, que é exatamente o que falta na sala.

```bash
python3 tools/banco/ensaio.py --ensaio reta --v 0.15 --wz 0 \
    --dur 25 --espaco 2.5 --janela 0.5 \
    --topico /compensador_rumo/cmd_vel --csv $D/devagar-a.csv
```

⚠️ **0,15 m/s ainda está dentro do patamar da placa** (todo comando entre ~0,008
e ~0,838 m/s vira a mesma coisa), então o robô pode andar **mais rápido que o
pedido**. O `ensaio.py` grava a velocidade real; confira antes de comparar com
as corridas de 0,25.

---

## 8. Como julgar — a régua, e ela mudou em 10-08

🔴 **Corrida de 1,2 m NÃO julga rumo.** A mesma corrida de 10-08 mediu −0,0162
(passa no critério da 011) cortada em 1,2 m e +0,0882 (reprova) medida inteira:
o corte curto cai no cruzamento de zero do S. **Aceitação é 2,5 m (~10 s)**, ao
menos um período.

Duas réguas, e a segunda é a que importa aqui:

```bash
python3 tools/banco/medir.py --resumo curvatura $D/controle-*.csv
python3 tools/banco/mede_o_s.py $D/controle-*.csv $D/adapta-*.csv
```

O `mede_o_s` é o instrumento certo porque **curvatura média não distingue
pender de oscilar**. Ele imprime, por corrida:

```
arquivo                     amp  invs   T[s]   deriva    dur
comp-longa-a.csv          35.8°     2   5.50    13.1°  10.1s     <- ff velho, 10-08
comp-longa-fresco-a.csv   13.0°     1      —    10.3°   9.9s     <- ff do dia, 10-08
```

**`amp` é o número do dia**: 35,8° com ff velho, 13,0° com ff do dia. O
estimador tem de levar as corridas b e c do experimento 3 para baixo de 13°,
partindo de uma semente velha.

---

## 9. Armadilhas que já morderam (todas medidas)

- **`ros2 param set` não chega no nó** — para trocar parâmetro, matar e subir;
- **`ros2 launch` NÃO morre com os nós**: mate o grupo de processos
  (`kill -TERM -<pgid>`), senão empilham pilhas paralelas (3 simultâneas em
  07-08) e o sintoma parece bug de código;
- **`ros2 node list` mostra fantasma** do daemon mesmo sem processo
  (`ros2 daemon stop && start` limpa). Para saber o que está vivo, `ps`;
- **o NUC cai junto com o robô** — salve em levas para o `origin`;
- **`/auto_vel` calado com o robô parado é o certo** (o `collision_monitor` não
  republica comando nulo);
- **o compensador só age no tópico dele**: corrida sem `--topico` é corrida sem
  compensador, mesmo com o nó de pé.

---

## 10. No fim da sessão

1. `ambiente.txt` na pasta de dados: piso, **bateria no início e no fim**,
   espaço, commit, o que cada corrida era e a observação do dono a olho;
2. puxe os CSV para o dev (`scp`), commite os dados, escreva a entrada do
   diário e atualize o `ESTADO_PROJETO.md`;
3. **atualize a decisão 016** com o veredito: o estimador ganhou, empatou ou
   perdeu — e, se ganhou, se o default deve deixar de ser opt-in;
4. avise o dono que pode desligar o robô assim que a última corrida gravar.
