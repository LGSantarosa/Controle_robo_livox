# Folha de campo — sessão de bancada no robô

Uma página, para levar ao laboratório. O protocolo e o *porquê* de cada ensaio
estão no `README.md` ao lado; aqui é só a ordem de chegar e rodar.

**Robô LIGADO. Área livre de 5 × 3 m. Alguém de olho, com a mão no disjuntor.**

---

## 1. Subir a base

⚠️ **O NUC não tem autenticação no GitHub** (dívida de infra, 07-30): `git fetch`
e `git push` não funcionam lá. Código vai do dev por **bundle**, e os commits
saem **do dev**:

```bash
# no PC de dev, com o robô na mesma rede (ver a entrada 07-30 do diário)
git bundle create /tmp/repo.bundle --all
scp /tmp/repo.bundle bara@<ip-do-nuc>:/tmp/
# no NUC
cd ~/Controle_robo_livox && git pull /tmp/repo.bundle main
```

```bash
cd ~/Controle_robo_livox
source /opt/ros/jazzy/setup.bash
colcon build --packages-select hoverboard_driver     # ver o porquê abaixo
source install/setup.bash
ros2 launch robot_base base.launch.py
```

**Por que o build, sempre.** A bitola (0,270), o raio (0,080) e os nomes de roda
esquerda/direita moram no `hoverboard_controllers.yaml` e no xacro do
`hoverboard_driver`, e o `tracao.launch.py` lê os dois de `FindPackageShare` —
ou seja, da cópia **instalada**, não do fonte. Trocar o fonte não troca o
`install/`. Sem o build, a correção do giro espelhado **não existe para o robô**,
e todos os limiares saem enviesados sem sinal nenhum de que algo está errado.
É barato e idempotente.

Conferir em vez de confiar:

```bash
grep -E 'wheel_(separation|radius)|wheel_names' \
  install/hoverboard_driver/share/hoverboard_driver/config/hoverboard_controllers.yaml
# separation 0.270, radius 0.080, e o swap esquerda/direita aplicado
```

---

## 0. ANTES DE TUDO: o giro está espelhado (bloqueio de 07-30)

O cutucão de 30-07 pegou: comando de **+0,6 rad/s girou −78,5°**. Esquerda e
direita estão trocadas. A reta sai certa (as duas rodas no mesmo sentido) e só o
giro espelha — e **nenhum dos seis ensaios acusaria isso**, porque medem
magnitude. Medir assim é medir errado, e a sessão inteira sairia lixo.

O conserto, aprovado pelo dono e ainda **não aplicado** (a validação exige o robô
andando, com alguém de olho), em
`ros2_packages/hoverboard_driver/bringup/config/hoverboard_controllers.yaml`:

```yaml
left_wheel_names:  ["right_wheel_joint"]   # eram left/right, nesta ordem
right_wheel_names: ["left_wheel_joint"]
```

Depois: **rebuild** (acima), subir a base, e

```bash
python3 tools/banco/sessao.py --checar --mexer
```

O giro tem de sair **positivo** (anti-horário). Saiu? Seguir. Não saiu? **Parar
e avisar** — não medir. E só commitar o swap depois que o cutucão validar.

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

**Mas em 07-30 o IP estava CERTO e o lidar mesmo assim não streamava.** Ele
pingava e ACKava todo comando, com `/livox/lidar` mudo e RX de ~6 pacotes/3 s.
Causa provável: o Mid-360 **tranca a sessão de dado** quando o driver morre no
meio do handshake. Cura: **power-cycle do lidar**. Prevenção: derrubar o driver
com Ctrl-C e esperar — **nunca `kill -9`** no meio da subida.

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

Ordem, tempo e espaço — **27 corridas**:

| passo | o que mede | corridas | espaço |
|---|---|---|---|
| 1 | zona morta linear | 1 (dente de serra) | 3 m reto |
| 2 | **zona morta de giro** ← item nº 1 | 1 (dente de serra) | raio de 1 m |
| 3 | `a_dec` (degrau de giro) | 5 — o nível do meio **×3** | 4 m |
| 4 | curva por velocidade | 9 — cada velocidade **×3** | círculo de 1,2 m de raio |
| 5 | aceleração linear | 3 (**×3**) | 4 m |
| 6 | reta com cutucão, ida × ré | 8 — inclui 1 girada 180° | 4 m nos dois sentidos |

Uns 30 min de robô andando, mais o reposicionamento.

**Por que repetido.** Uma corrida é uma amostra, não uma medida: sem faixa não
dá para dizer se o número serve. Cada grupo de repetições imprime **média,
faixa e dispersão** ao terminar — se a dispersão passar de 15%, ele avisa, e
aí vale repetir ali mesmo, com o robô ligado.

**O ponto 0 é sagrado.** Marcar com fita no chão **e marcar o rumo também**.
Todas as repetições de uma condição saem do mesmo ponto e do mesmo rumo — senão
a dispersão que a gente medir é a sua mão, não a máquina.

**A corrida girada 180° do passo 6 não é repetição, é controle.** Média de três
corridas iguais mata erro aleatório e **não mata erro sistemático**: se o chão
tem caimento, as três desviam igual e a média sai confiante e errada. Girado
180°, o caimento empurra para o mesmo lado do *mundo* e a assimetria do robô
puxa para o mesmo lado do *corpo* — comparando as duas, dá para dizer qual dos
dois você mediu. É a única corrida que separa robô de sala.

**Os ensaios 1 e 2 não repetem em corrida, e é de propósito.** O dente de serra
sobe até ele sair do lugar, desce até ele parar, inverte o sentido e repete 4
vezes — tudo dentro de uma corrida. Saem **4 medidas de saída e 4 de queda**,
nos dois sentidos, sem reposicionar nada. E ele mede duas coisas onde antes
media uma: a **saída** (do repouso, atrito estático — o número do BO-3) e a
**queda** (já andando, sempre menor — é ela que o piso de velocidade precisa).

Parou no meio (bateria, alguém entrou na sala)? Retoma sem perder o que já foi:

```bash
python3 tools/banco/sessao.py --de 4     # do passo 4 em diante
python3 tools/banco/sessao.py --so 2     # só o passo 2
python3 tools/banco/sessao.py --repete 1 # bateria acabando: 1 corrida por
                                         # condição (números sem faixa)
```

## 5. Dois ensaios que pedem atenção humana

**Passo 2 — o robô fica parado com o comando subindo, e volta a ficar.** Não é
travamento e não é para interromper: é a zona morta acontecendo, e é exatamente
o número que viemos buscar. São 4 dentes, então isso acontece 4 vezes,
alternando o sentido do giro. A corrida acaba sozinha quando o 4º fechar — o
teto de tempo (220 s) quase nunca é atingido.

**Passo 6 — filmar a traseira** nas 4 primeiras corridas (`frente-a`, `frente-b`,
`re-a`, `re-b`). O que se procura é a **boba dando meia-volta** quando o robô
anda de ré, e o quanto ele se desvia enquanto ela decide. É o único jeito de
fechar o BO-4: o simulador não tem boba de verdade, então esse vídeo é a medida.
Filmar as duas de frente também — sem elas não há com o que comparar.

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
