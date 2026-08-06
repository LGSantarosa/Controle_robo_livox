# Próxima sessão no robô — roteiro de operação

> **Para quem é este documento: o assistente que chegar frio.** A memória dele
> não cruza sessões nem máquinas — é por isso que este arquivo existe, e é por
> isso que ele começa pelo método e não pelos comandos.
>
> Escrito em **2026-08-06**, depois de dois dias de trabalho (uma sessão de
> bancada no robô em 05-08 e três de dev em 05/06-08). Tudo o que está aqui tem
> origem citada.

---

## 0. O MÉTODO — leia antes dos comandos, é o que fez a sessão de ontem funcionar

O dono elogiou explicitamente o modo de conduzir de 05-08 e pediu que se repita:

> *"direto aos testes, sem se enrolar, só fazendo o que foi pedido, avisando
> pra desligar o robô, pedindo 'pode' para soltar o comando"*

Na prática:

1. **O DONO SÓ RODA.** Nunca peça para ele relatar console. Tudo grava CSV, que
   volta por `scp` e é lido no dev. Ele executa o físico: posicionar, ligar,
   olhar, apertar tecla.
2. **UMA CORRIDA POR VEZ.** Antes de cada comando que move a máquina: dizer o
   que o robô vai fazer, quanto espaço precisa, e **esperar o "pode"**.
3. **NÚMERO NA HORA.** Rodar o ensaio e já devolver a leitura, com o robô
   ligado. Não acumular CSV para ler depois — o dono decide o próximo passo com
   o número na mão.
4. **AVISAR LIGADO vs DESLIGADO**, e avisar quando já pode desligar.
5. **NÃO EXPANDIR ESCOPO.** Bloqueio que aparecer: diagnosticar, registrar,
   seguir. Não sair consertando com o robô ligado e a bateria correndo.
6. **RETIRAR HIPÓTESE ASSIM QUE O DADO A DERRUBAR**, em voz alta. Isso aconteceu
   três vezes em 05-08 e é o que manteve os números confiáveis.

⚠️ **Bateria é o recurso escasso.** Cada minuto de conversa é minuto de bateria.

---

## 1. Contexto mínimo — 5 minutos de leitura antes de tocar em qualquer coisa

Na ordem:

| ler | por quê |
|---|---|
| `ESTADO_PROJETO.md`, o topo até "A PRÓXIMA IDA AO ROBÔ" | o estado vivo, e a lista dos 6 itens em ordem de valor |
| `docs/PLANO_SINTONIA_RUMO.md` | o experimento nº 1 desta sessão, com critérios |
| `docs/PLANO_TESTE_ROBO.md` §1 | os experimentos 2, 3 e 4 |
| `tools/banco/CHECKLIST_ROBO.md` | a folha de campo, com as armadilhas antigas |

**O que mudou desde a última vez que o robô rodou (05-08), e nunca foi visto
por ele:**

- os **ganhos do compensador caíram 4,1×** (`kp` 1,00→0,25, `ki` 0,50→0,12);
- o robô passou a carregar **outra descrição** (`robo2.urdf.xacro` em vez do
  exemplo de demonstração do `ros2_control`) — ganhou geometria medida e o
  `livox_frame`;
- o **Mid-360 entrou a 42 cm** (medido com trena), não os 27 supostos;
- existe um **preditor de Smith** no compensador, **desligado** por padrão;
- o `twist_mux` passou a vir **do fonte** (`./setup_twist_mux.sh`).

---

## 2. Acesso e deploy

```bash
# 1) achar o robô. Em 05-08 foi 10.244.3.205, mas o IP MUDA.
#    O dono sabe o final; o resto vem da sua própria sub-rede:
ip -4 addr show | grep "inet "         # veja em que /24 você está
ping -c2 <ip-do-nuc>
ssh -o ServerAliveInterval=15 bara@<ip-do-nuc>
```

⚠️ **Se não pingar, o problema quase sempre é topologia, não senha.** Em 30-07
o dev estava em `10.150.13.54/19` e o NUC em `10.244.3.205` — sub-redes
diferentes, gateway não encaminhava. O dono resolve pondo o dev na mesma rede
do robô. A chave ssh já está autorizada; não deve pedir senha.

```bash
# 2) deploy — o NUC NÃO tem auth no GitHub, então vai por bundle
#    (no dev)
git bundle create /tmp/repo.bundle --all
scp /tmp/repo.bundle bara@<ip>:/tmp/
#    (no NUC)
cd ~/Controle_robo_livox
git fetch /tmp/repo.bundle main && git reset --hard FETCH_HEAD
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros2_packages --symlink-install \
    --packages-select hoverboard_driver robot_base robot_motion
source install/setup.bash
```

⚠️ **`colcon build` SEMPRE.** `git reset --hard` troca o fonte e **não** troca o
`install/` — e é do `install/` que o `tracao.launch.py` lê a descrição.

⚠️ Se houver CSVs não commitados no NUC, o `reset --hard` reclama de arquivos
untracked. Em 05-08 eles eram idênticos aos do repo (conferido por `md5sum`
antes de deixar o git passar por cima). **Confira antes de apagar dado de
bancada.**

Só para os testes C e D, uma vez por máquina:

```bash
./setup_twist_mux.sh
```

---

## 3. 🔴 PASSO BLOQUEANTE — a conferência que só existe nesta sessão

**Robô LIGADO.** Terminal 1, e fica vivo a sessão inteira:

```bash
ros2 launch robot_base base.launch.py
```

Terminal 2:

```bash
python3 tools/banco/sessao.py --checar
```

**TEM de imprimir:**

```
[ok] /Odometry a ~10 Hz
[ok] wheel_separation = 0.2700   (bate com a trena)
[ok] wheel_radius = 0.0800
```

🛑 **Se a bitola não der 0,2700, PARE A SESSÃO.** O robô trocou de descrição em
05-08 e isso nunca rodou na máquina. Os blocos de hardware foram comparados
renderizando os dois xacro e saem idênticos, mas comparação no dev não é
verificação no robô. Bitola errada = todo número da sessão sai enviesado **sem
sintoma nenhum**.

E a conferência de sempre — exatamente UMA pilha:

```bash
pgrep -a -f "[f]astlio_mapping"          # tem de listar UM processo
pgrep -a -f "[l]ivox_ros_driver2_node"   # idem
```

⚠️ **Use `pgrep -a`, nunca `pgrep -c`.** O `-c` mentiu em 05-08 acusando 2
pilhas: a própria linha de comando do ssh continha a palavra e o bash casou
consigo mesmo. O colchete protege contra o `pgrep`, não contra o resto do
comando.

⚠️ **`Received a non-finite error value` na subida não é o defeito de 24-07** se
parar em algumas dezenas de linhas. O do registro (quadro de 18 vs 26 bytes)
inunda o log para sempre e as rodas nunca giram.

---

## 4. Pedir ao dono, antes da primeira corrida

Três coisas, e a primeira já foi esquecida duas sessões seguidas:

```
piso    : <sala e superfície>
bateria : <cheia? tensão? e conferir de novo no fim>
espaço  : 1,2 m de raio livre EM TODAS AS DIREÇÕES
```

🔴 **Piso e bateria não são burocracia nesta sessão.** A varredura de ganhos
compara condições ao longo de uma tarde; a linha de base repetida no fim de
05-08 já sugeriu ~5% de queda de planta na sessão, que vale 0,046 1/m — do
tamanho do resíduo inteiro depois de compensar. Sem esses dois campos, a
varredura não é comparável consigo mesma.

**Ponto 0 marcado com fita, e o RUMO também.** Todas as corridas de uma
condição saem do mesmo ponto e do mesmo rumo.

---

## 5. EXPERIMENTO 1 — a sintonia do rumo (o principal)

Detalhes e racional: `docs/PLANO_SINTONIA_RUMO.md`. Aqui é a operação.

**A pergunta, e ela é falsificável:** com os ganhos novos, a amplitude da
oscilação **DECAI** em vez de crescer?

### Terminal 3 — o compensador, REINICIADO a cada condição

```bash
# condição A — os ganhos NOVOS (o projeto). É o default, não precisa passar nada:
ros2 run robot_motion compensador_rumo

# condição B — os ganhos VELHOS (o CONTROLE do dia):
ros2 run robot_motion compensador_rumo --ros-args -p kp:=1.0 -p ki:=0.5
```

🛑 **`ros2 param set` NÃO funciona neste nó** — ele copia os parâmetros no
construtor e não tem callback. O `param get` responde o valor novo e o nó segue
usando o velho. **Matar e subir de novo a cada condição**, e conferir na
primeira linha do log:

```
compensador de rumo (decisão 011): ff -0.817 ... kp=0.25 ki=0.12
```

### Terminal 4 — as corridas, três por condição

```bash
D=docs/dados/$(date +%Y-%m-%d)-sintonia-rumo
mkdir -p $D
python3 tools/banco/ensaio.py --ensaio reta --v 0.25 --wz 0 \
    --dur 12 --espaco 1.2 --janela 0.5 \
    --topico /compensador_rumo/cmd_vel \
    --csv $D/novos-frente-a.csv
```

Repetir `-b` e `-c`, reposicionando no ponto 0. Depois trocar o compensador
para os ganhos velhos e repetir com `velhos-frente-{a,b,c}.csv`.

⚠️ **`--janela 0.5` sempre no robô.** O padrão de 0,2 s sobre pose a 10 Hz
inventa ruído — foi o que gerou a falsa acusação contra o LIO em 31-07.

### A leitura, na hora

```bash
python3 tools/banco/mede_o_s.py $D/novos-frente-*.csv
python3 tools/banco/medir.py --resumo curvatura $D/novos-frente-*.csv
```

**Como julgar** (a referência de 05-08, com os ganhos velhos):

```
                      amp        invs   período   deriva
sem compensador      68,0°         0       —       −68°     <- PENDE
com comp. (velhos)  9,4–18,5°      2    2,2–2,4s   1–4°     <- o S
```

E a envoltória, que é o que decide — comparar a **1ª excursão com a 2ª**:

```
✅ PASSOU    a 2ª excursão é MENOR que a 1ª (decai), e |curvatura| < 0,05
             em TODA corrida, não só na média
❌ FALHOU    a 2ª continua maior que a 1ª (cresce, como em 05-08: 2,07x)
```

🔴 **Se falhar, NÃO é sessão perdida — é a resposta.** Significa que o S não é
do laço de controle, e o suspeito passa a ser mecânico: a boba (BO-4). Nesse
caso, o que a sessão tem de trazer é:

- **o vídeo da traseira durante uma corrida de frente**, com o S acontecendo
  (critério (b) do BO-4, aberto desde 28-07 — filmado em 04-08 e nunca trazido
  para o repo);
- **empurrar o robô com a mão, desligado**, e ver se a boba oscila sozinha.

---

## 6. Os outros experimentos, se a bateria permitir

Em ordem de valor. Nenhum deles justifica cortar o experimento 1.

### 2. Pivô `liga 0,10` e `liga 0,30`, n=3 cada — a outra previsão falsificável

```bash
P=docs/dados/$(date +%Y-%m-%d)-pivo && mkdir -p $P
python3 tools/banco/ensaio.py --ensaio degrau_giro --v 0 --wz 0.6 \
    --liga 0.10 --dur 10 --espaco 1.0 --janela 0.5 \
    --csv $P/liga0.10-a.csv
```

O ensaio publica **direto no atuador** (sem `--topico`), então o compensador
fica fora do caminho — e com `--v 0` ele seria passa-direto de qualquer jeito.

Leitura: `docs/dados/2026-08-05-aceitacao-atraso-desliga/pivo.py`.

**A previsão:** se a quantização do laço de 10 Hz estiver certa, `liga 0,10`
(1,0 ciclo, a fase decide se pega 0 ou 1) tem de sair **bimodal** — ora ~0°,
ora ~16°. Hoje só existe uma amostra (2,7°), compatível com ter pego zero
ciclos. São os dois pontos onde o simulador mais discorda do robô.

### 3. Teste C, item 1 — o homem-morto

Precisa do `./setup_twist_mux.sh` rodado. Os itens 2 e 3 do teste C **já estão
provados sem robô** (`tools/banco/prova_mux.py`). Falta só:

> `w` faz andar; **solte o teclado e ele para sozinho em 0,4 s.**

⚠️ **Se isso falhar, PARE TUDO.** Teleop que não para sozinho é pior que teleop
nenhum.

### 4. Teste D — o reflexo de colisão

**Caixa de 50 cm**, não 40: o Mid-360 está a 42 cm e a 0,5 m só vê acima de
36 cm. Ver `PLANO_TESTE_ROBO.md` §1.

### 5. O preditor de Smith — 3 corridas desempatam

```bash
ros2 run robot_motion compensador_rumo --ros-args \
    -p preditor:=true -p kp:=1.0 -p ki:=0.5
```

Na bancada matemática ele **perde** para o detune (mata a divergência mas deixa
ondulação sustentada de ~3,6°). A ressalva que só o robô resolve está no
`PLANO_SINTONIA_RUMO.md` §4. Se perder aqui também, fica desligado para sempre.

### 6. O vídeo da traseira

Independente de tudo: é o item mais barato que falta no projeto inteiro.

---

## 7. Como o dado volta

```bash
scp 'bara@<ip>:~/Controle_robo_livox/docs/dados/AAAA-MM-DD-*/*.csv' \
    docs/dados/AAAA-MM-DD-sintonia-rumo/
git add docs/dados/ && git commit -m "..." && git push
```

**Salve durante a sessão, não no fim** — bateria e rede caem. Em 05-08 os dados
foram commitados em quatro levas.

**Escreva o `ambiente.txt`** com piso, bateria, commit e a calibração viva que o
`--checar` imprimiu. Sem ele um limiar medido vira número sem unidade.

---

## 8. Ao terminar

- avisar que **o robô já pode ser desligado**;
- `docs/DIARIO.md`: entrada da sessão, **incluindo o que falhou**;
- `ESTADO_PROJETO.md`: atualizar o topo e a lista da próxima ida;
- se alguma previsão foi falsificada, **dizer isso em voz alta e cedo** — é o
  resultado mais valioso que uma sessão pode produzir.

---

## Apêndice — armadilhas que já custaram tempo neste projeto

| armadilha | como evitar |
|---|---|
| `pgrep -c` contando a si mesmo | usar `pgrep -a` e LER quem casou |
| `ros2 param set` que não chega no nó | matar e subir o nó; conferir a primeira linha do log |
| `git reset --hard` sem `colcon build` | build sempre; a descrição vem do `install/` |
| trava `--espaco` é RADIAL e usa a fonte mais alarmista | dimensionar pelo **disco varrido**, `--espaco` 1,0–1,2 |
| `--janela 0.2` no robô | sempre `0.5`; 10 Hz de pose com janela curta inventa ruído |
| comparar n=1 com n=1 num processo bimodal | n=3, e comparar **faixa** contra faixa |
| processos órfãos republicando `/Odometry` | matar por PID e conferir **1 publicador** antes de medir |
| dispersão em % sobre número quase nulo | ler o espalho **absoluto** também |
