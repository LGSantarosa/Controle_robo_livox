# Folha de campo — sessão de bancada no robô

Uma página, para levar ao laboratório. O protocolo e o *porquê* de cada ensaio
estão no `README.md` ao lado; aqui é só a ordem de chegar e rodar.

**Robô LIGADO. Área livre de 5 × 3 m. Alguém de olho, com a mão no disjuntor.**

---

## 1. Subir a base

```bash
cd ~/Controle_robo_livox
git fetch && git reset --hard origin/main
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch robot_base base.launch.py
```

Deixe esse terminal vivo. Tudo abaixo roda num **segundo terminal**, com as duas
linhas de `source` repetidas.

## 2. Conferir antes de medir — e antes de andar

```bash
python3 tools/banco/sessao.py --checar
```

Ele diz `Conferência ok` ou **reprova e para**. Reprovou, o mais provável é o
lidar: sem nuvem não há `/Odometry`, e sem `/Odometry` não há medida nenhuma.

```bash
# IP do lidar — a varredura é a fonte da verdade, não o número commitado
for i in $(seq 1 254); do ping -c1 -W1 192.168.1.$i >/dev/null 2>&1 & done; wait
ip neigh | grep -i 'e4:7a:2c'
```

Mudou? Editar `ros2_packages/robot_base/config/MID360_config.json` (**não** o
arquivo de dentro do driver, que se perde no próximo setup) e rodar
`./setup_livox.sh` de novo.

## 3. Cutucão de sanidade — com o robô LIVRE

```bash
python3 tools/banco/sessao.py --checar --mexer
```

Ele anda 2 s e gira 2 s, e confere o **sinal** do que aconteceu. Serve para pegar
roda trocada na fiação: um robô assim anda certo e gira ao contrário, e nenhum
dos seis ensaios acusa isso — eles medem tamanho, não sentido. Se acusar
inversão, **parar aqui**: a sessão inteira sairia espelhada.

## 4. A sessão

```bash
python3 tools/banco/sessao.py
```

Ele pergunta piso e bateria (entram no `ambiente.txt` — sem isso a medida não se
compara com a próxima sessão), e então roda os seis passos, pausando antes de
cada corrida para você reposicionar o robô. Cada corrida grava o CSV **e imprime
a leitura na hora**, então dá para repetir um ensaio esquisito ainda com o robô
ligado.

Ordem, tempo e espaço:

| passo | o que mede | corridas | espaço |
|---|---|---|---|
| 1 | zona morta linear | 1 × 20 s | 3 m reto |
| 2 | **zona morta de giro** ← item nº 1 | 1 × 20 s | raio de 1 m |
| 3 | `a_dec` (degrau de giro) | 3 × 12 s | 4 m |
| 4 | curva por velocidade | 3 × 12 s | círculo de 1,2 m de raio |
| 5 | aceleração linear | 1 × 10 s | 4 m |
| 6 | reta com cutucão, ida × ré | 3 × 16 s | 4 m nos dois sentidos |

Uns 15 min de robô andando, mais o reposicionamento.

Parou no meio (bateria, alguém entrou na sala)? Retoma sem perder o que já foi:

```bash
python3 tools/banco/sessao.py --de 4     # do passo 4 em diante
python3 tools/banco/sessao.py --so 2     # só o passo 2
```

## 5. Dois ensaios que pedem atenção humana

**Passo 2 — o robô pode ficar parado com o comando subindo.** Não é travamento e
não é para interromper: é a zona morta acontecendo, e é exatamente o número que
viemos buscar. Deixar rodar os 20 s.

**Passo 6 — filmar a traseira**, nas duas primeiras corridas (frente e ré). O que
se procura é a **boba dando meia-volta** quando o robô anda de ré, e o quanto ele
se desvia enquanto ela decide. É o único jeito de fechar o BO-4: o simulador não
tem boba de verdade, então esse vídeo é a medida.

## 6. Mandar os dados

Tudo caiu em `docs/dados/AAAA-MM-DD-bancada-robo/` — CSVs, `ambiente.txt` e
`leituras.txt` (o que a tela respondeu, salvo).

```bash
git add docs/dados/ && git commit -m "bancada: dados crus da sessão no robô" && git push
```

O vídeo do passo 6 vai por fora (é grande demais para o repo).

---

## Se der errado

| sintoma | quase sempre é |
|---|---|
| `/Odometry` não publica | IP do lidar — varredura acima |
| ninguém escuta `cmd_vel` | `base.launch.py` caiu; olhar o primeiro terminal |
| robô não sai do lugar no passo 1 | pode ser o resultado. Repetir com `--rampa-ate 0.6` |
| gira ao contrário no cutucão | rodas trocadas — **parar e avisar** |
| ensaio corta cedo | trava de distância (`--espaco`); o robô saiu da área |

Nada disso precisa de conserto no laboratório. **Mande os CSVs mesmo assim** —
ensaio que falhou também é dado, e o `leituras.txt` diz por quê.
