# Folha de campo — sessão de bancada no robô

Uma página, para levar ao laboratório. O protocolo e o *porquê* de cada ensaio
estão no `README.md` ao lado; aqui é só a ordem de chegar e rodar.

**Robô LIGADO. Área livre de 5 × 3 m. Alguém de olho, com a mão no disjuntor.**

---

## Para quem for conduzir a sessão (assistente): leia isto primeiro

Este arquivo é executável de cima a baixo. **Não há decisão de projeto a tomar
no laboratório** — tudo já foi decidido em 30-07 e 31-07. Contexto em duas
entradas do `docs/DIARIO.md`: **07-30 (2ª leva)**, a ida ao robô que achou o
giro espelhado, e **07-31**, o protocolo novo.

A ordem é: **bundle → build → base → `--checar` → swap → rebuild → `--checar
--mexer` → sessão**. Cada passo abaixo, em ordem, sem pular.

Cinco coisas que **não** se fazem, cada uma comprada com tempo perdido:

| não faça | por quê |
|---|---|
| medir com o giro espelhado | a sessão inteira sai lixo, e nenhum dos 6 ensaios acusa |
| commitar o swap antes do cutucão validar | decisão de 30-07: só entra no git depois de o robô provar |
| `kill -9` no driver do livox | trava a sessão de dado do Mid-360; derrubar com Ctrl-C e esperar |
| pular o `colcon build` | YAML e xacro são lidos do `install/`; sem build a correção não existe |
| pedir para o dono relatar o console | ele só executa; tudo sai em arquivo (`leituras.txt`, CSV) |

E uma que **se faz sempre**: `--checar` antes de qualquer coisa que ande. Ele
recusa medir sem `/Odometry` e agora também diz **qual calibração a base
carregou** — se a bitola não for 0,270, o número medido não tem unidade.

---

## 1. Levar o código e subir a base

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

Deixe esse terminal vivo. Tudo abaixo roda num **segundo terminal**, com as duas
linhas de `source` repetidas.

**Por que o build, sempre.** A bitola (0,270), o raio (0,080) e os nomes de roda
esquerda/direita moram no `hoverboard_controllers.yaml` e no xacro do
`hoverboard_driver`, e o `tracao.launch.py` lê os dois de `FindPackageShare` —
ou seja, da cópia **instalada**, não do fonte. Trocar o fonte não troca o
`install/`. Sem o build, a correção do giro espelhado **não existe para o robô**,
e todos os limiares saem enviesados sem sinal nenhum de que algo está errado.
É barato e idempotente.

## 2. Conferir antes de medir — e antes de andar

```bash
python3 tools/banco/sessao.py --checar
```

Ele diz `Conferência ok` ou **reprova e para**, e imprime a **calibração viva**:
o que o controlador carregou de `wheel_separation`, `wheel_radius` e nomes de
roda. Isso responde, sem adivinhação, se o build do passo 1 pegou.

Reprovou? O mais provável é o lidar: sem nuvem não há `/Odometry`, e sem
`/Odometry` não há medida nenhuma.

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

## 3. O BLOQUEIO: corrigir o giro espelhado, e provar que corrigiu

O cutucão de 30-07 pegou: comando de **+0,6 rad/s girou −78,5°**. Esquerda e
direita estão trocadas. A reta sai certa (as duas rodas no mesmo sentido) e só o
giro espelha — e **nenhum dos seis ensaios acusaria isso**, porque medem
magnitude. Medir assim é medir errado, e a sessão inteira sairia lixo.

O conserto, aprovado pelo dono e ainda **não aplicado** (a validação exige o robô
andando, com alguém de olho). Aplicar e reconstruir:

```bash
cd ~/Controle_robo_livox
python3 - <<'PY'
import pathlib
p = pathlib.Path('ros2_packages/hoverboard_driver/bringup/config/hoverboard_controllers.yaml')
t = p.read_text()
a = 'left_wheel_names: ["left_wheel_joint"]\n    right_wheel_names: ["right_wheel_joint"]'
b = 'left_wheel_names: ["right_wheel_joint"]\n    right_wheel_names: ["left_wheel_joint"]'
assert a in t, 'ja trocado, ou o arquivo mudou — conferir a mao'
p.write_text(t.replace(a, b)); print('swap aplicado')
PY
colcon build --packages-select hoverboard_driver && source install/setup.bash
```

Reiniciar o `base.launch.py` (Ctrl-C no primeiro terminal e subir de novo), e
então, **com o robô LIVRE e alguém de olho**:

```bash
python3 tools/banco/sessao.py --checar --mexer
```

Ele anda 2 s e gira 2 s, e confere o **sinal** do que aconteceu. Duas coisas
para ler na saída:

- `swap esquerda/direita APLICADO` na seção **calibração viva** — prova que o
  build pegou, antes mesmo de o robô se mexer;
- o cutucão: o giro tem de sair **positivo** (anti-horário).

Saiu positivo? Seguir para o passo 4. Continuou negativo? **Parar e avisar** —
não é o YAML, é fiação, e medir assim é medir errado. Reverter o swap com o
mesmo script trocando `a` e `b`.

**Só commitar o swap depois que o cutucão validar** — é a regra de 30-07.

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
| 1 | **zona morta de GIRO** ← item nº 1 | 1 (dente de serra) | raio de 1 m |
| 2 | zona morta linear | 1 (dente de serra) | 3 m reto |
| 3 | `a_dec` (degrau de giro) | 5 — o nível do meio **×3** | 4 m |
| 4 | curva por velocidade | 9 — cada velocidade **×3** | círculo de 1,2 m de raio |
| 5 | aceleração linear | 3 (**×3**) | 4 m |
| 6 | reta com cutucão, ida × ré | 8 — inclui 1 girada 180° | 4 m nos dois sentidos |

**O giro é o passo 1** porque ele responde a pergunta do pivô **direto**: o
menor `wz` que gira o robô parado *é* o limiar do pivô, em rad/s, sem bitola no
meio. Pelo linear só se chega lá convertendo por `2·zm/L` — confiando de novo
num número medido. Some o risco de sessão cortada (em 30-07 não se mediu nada):
o que fica por último é o que se perde. **Se der para rodar só um ensaio hoje,
é este.**

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
python3 tools/banco/sessao.py --so 1     # só o passo 1 (zona morta de giro)
python3 tools/banco/sessao.py --repete 1 # bateria acabando: 1 corrida por
                                         # condição (números sem faixa)
```

## 5. Dois ensaios que pedem atenção humana

**Passos 1 e 2 — dois números vão sair de cada um, não um.** A leitura separa
a **saída** (do repouso) da **queda** (já andando, sempre menor), e separa
também o **dente #0** dos demais quando ele destoa: só ele parte de repouso
longo, e atrito estático cresce com o tempo parado. Não é ruído — é o caso do
BO-3 aparecendo. Não precisa fazer nada com isso no laboratório além de deixar
rodar; só não se assuste com a faixa larga.

**Passos 1 e 2 — o robô fica parado com o comando subindo, e volta a ficar.**
Não é travamento e não é para interromper: é a zona morta acontecendo, e é
exatamente o número que viemos buscar. São 4 dentes, então isso acontece 4
vezes em cada passo, alternando o sentido. A corrida acaba sozinha quando o 4º
fechar — o teto de tempo (220 s) quase nunca é atingido.

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
