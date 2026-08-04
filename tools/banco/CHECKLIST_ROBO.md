# Folha de campo — sessão de bancada no robô

Uma página, para levar ao laboratório. O protocolo e o *porquê* de cada ensaio
estão no `README.md` ao lado; aqui é só a ordem de chegar e rodar.

**Robô LIGADO. Alguém de olho, com a mão no disjuntor.**

⚠️ **Área livre em TODAS as direções, não só à frente.** Este robô arca com raio
de ~1,2 m indo para a frente: ele varre um disco, não um corredor. Em 04-08 ele
bateu numa cadeira num espaço com "3 m livres à frente". O tamanho do disco você
escolhe no `--espaco` de cada corrida.

---

## Para quem for conduzir a sessão (assistente): leia isto primeiro

> ⚠️ **REVISADO EM 04-08.** A versão anterior deste arquivo mandava **aplicar um
> swap esquerda/direita** que quebraria um robô que está certo, e descrevia um
> passo 6 que não roda nesta máquina. Se você está lendo uma cópia que ainda tem
> a seção "corrigir o giro espelhado", **pare e vá ler a entrada 08-04 do
> `docs/DIARIO.md`**.

Contexto obrigatório em três entradas do `docs/DIARIO.md`: **07-31 (4ª leva)**,
que é a retratação onde os números viraram válidos; **08-04**, esta revisão; e
`docs/dados/2026-08-04-bancada-robo/ambiente.txt`, com os números vivos.

A ordem é: **bundle → build → base → `--checar` → conferir UMA pilha →
corridas**. Sem swap, sem rebuild de correção — não há correção pendente.

Seis coisas que **não** se fazem, cada uma comprada com tempo perdido:

| não faça | por quê |
|---|---|
| 🛑 **aplicar o swap esquerda/direita** | **o giro espelhado NÃO EXISTE** — era `atan2` enrolando em ±180°. O estado atual (sem swap) é o correto. Aplicado em `368ea13`, revertido em `595cf80` |
| dimensionar corrida só pela distância à frente | a trava `--espaco` é **radial** e não vê excursão lateral. Em 04-08 o robô fez círculo e **bateu numa cadeira** num corredor "de 3 m livres" |
| `kill -9` no driver do livox | trava a sessão de dado do Mid-360; derrubar com Ctrl-C e esperar |
| pular o `colcon build` | YAML e xacro são lidos do `install/`; sem build a correção não existe |
| pedir para o dono relatar o console | ele só executa; tudo sai em arquivo (`leituras.txt`, CSV) |

E duas que **se fazem sempre**:

1. **`--checar`** antes de qualquer coisa que ande. Ele recusa medir sem
   `/Odometry` e diz **qual calibração a base carregou** — se a bitola não for
   0,270, o número medido não tem unidade.
2. **Conferir que existe exatamente UMA pilha de localização.** Em 07-31 três
   pilhas órfãs produziram saltos de 1,35 m e custaram horas:
   ```bash
   pgrep -c -f "[f]astlio_mapping"        # tem de ser 1
   pgrep -c -f "[l]ivox_ros_driver2_node" # tem de ser 1
   ```
   O colchete não é frescura: `pgrep -f fastlio` casa com a **própria linha de
   comando do ssh**. E derrubar a base com `pkill -f ros2_control_node` dentro
   de um ssh mata a própria sessão antes de relançar — foi assim que as três
   órfãs nasceram. Matar por PID.

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

## 3. 🛑 NÃO HÁ BLOQUEIO. O giro espelhado não existe.

Esta seção mandava trocar `left`/`right` no `hoverboard_controllers.yaml`.
**Não faça isso.** O que ela chamava de bloqueio era um defeito do instrumento.

O cutucão calcula o giro assim:

```python
giro = math.atan2(math.sin(yaw() - a0), math.cos(yaw() - a0))
```

Isso **enrola em ±180°**. E ele manda `+0,6 rad/s por 2,0 s` — com o patamar da
compensação, muito mais que meia volta. A conta fecha exata com o número que
bloqueou 30-07:

```
giro real +281,5°  −  360°  =  −78,5°     <- o "sentido INVERTIDO"
```

Confirmado em 04-08 com o dono de testemunha: leu **−71,0°** e ele viu **o nariz
ir para a esquerda varrendo bastante** — `289°` enrolados. O robô gira certo.

O swap chegou a ser aplicado (`368ea13`) e foi **revertido** (`595cf80`). O
estado de hoje, **sem swap**, é o correto.

✅ **CONSERTADO em 04-08** (`7a0c364`): o cutucão passou a **acumular** o giro
amostra a amostra em vez de diferenciar as pontas, então ele agora julga o giro
real. Um pivô de mais de meia volta é lido como tal e **aprovado**, e ele avisa
quantas voltas deu. Giro de fato invertido continua sendo pego — travado em
teste, verificado por mutação.

**Então o veredito de giro do `--checar --mexer` voltou a valer.** Se ele acusar
`sentido INVERTIDO` agora, **é para levar a sério** — mas confirme com o olho de
alguém atrás do robô antes de mexer em qualquer coisa. Foi o olho do dono que
desempatou nas duas vezes em que o instrumento mentiu.

## 4. A sessão — o que ainda falta, e o que NÃO roda

**Estado do banco depois de 04-08: 4 passos de 6.**

| passo | o que mede | estado |
|---|---|---|
| 1 | zona morta de GIRO | ✅ 07-31 — 0,095 rad/s (faixa 0,084–0,105) |
| 2 | zona morta linear | ✅ 07-31 — 0,023 m/s, **sem patamar** (é "onde rasteja") |
| 3 | `a_dec` (degrau de giro) | ✅ 08-04, `n=3` — usar a **cauda**, ~1,0 rad/s² |
| 4 | curva por velocidade | ❌ **precisa ser reescrito** |
| 5 | aceleração linear | ❌ **precisa ser reescrito** |
| 6 | reta com cutucão | ✅ 08-04 **em forma corrigida** — ver abaixo |

⚠️ **`sessao.py` NÃO serve como está para conduzir isto.** Três motivos:
não deixa passar `--espaco` por fora (o padrão de 4,0 m é excursão lateral demais
num robô que arca); o passo 6 dele está morto; e os passos 4 e 5 dele medem o
patamar em vez do robô. **Conduza corrida a corrida pelo `ensaio.py`**, como em
04-08:

```bash
python3 tools/banco/ensaio.py --ensaio reta \
  --csv docs/dados/AAAA-MM-DD-bancada-robo/nome.csv \
  --v 0.25 --wz 0 --dur 12 --espaco 1.2 --janela 0.5
```

`--janela 0.5` **sempre no robô**: o padrão de 0,2 s sobre pose a 10 Hz pega
duas amostras e amplifica ruído — foi o que gerou a falsa acusação de "ruído de
0,033 rad/s" contra o LIO em 07-31.

**Por que 4 e 5 não rodam.** Os dois varrem velocidade em 0,2 / 0,4 / 0,6 m/s, e
**as três caem dentro do patamar da compensação**: todo comando entre ~0,008 e
~0,838 m/s vira a mesma coisa na placa. As três corridas dariam o mesmo número,
com cara de varredura. A varredura tem de subir acima de 0,838 m/s, e isso exige
espaço que a bancada não teve.

**Por que o 3 rodou apesar disso:** o `a_dec` é medido **com o comando em ZERO**,
e a compensação só age enquanto há comando. Rodar como **pivô puro** (`--v 0`) —
o `--v 0.3` do protocolo injeta guinada espúria pela curvatura de frente, que
continua depois do corte e vira `a_dec` falso.

**O passo 6 não pode ser rodado como está escrito**, por dois motivos
independentes achados em 04-08:

- **não existe reta de referência.** "O rumo volta ou foge depois da
  perturbação?" não tem sentido num robô que descreve um círculo de 1,22 m de
  raio indo para a frente;
- **o pulso é menor que a latência do atuador.** O cutucão dura 0,5 s e a placa
  leva ~0,5 s para destravar. Medido: wz médio −0,405 antes, −0,358 durante,
  −0,369 depois. Não perturba nada.

O que rodar no lugar dele: **retas puras medindo curvatura**, `--wz 0`, ida e
ré, com o corpo girado 180° entre os sentidos (isso faz o controle de caimento
de piso de graça). Ver `docs/dados/2026-08-04-bancada-robo/ambiente.txt`.

**Por que repetido.** Uma corrida é uma amostra, não uma medida: sem faixa não
dá para dizer se o número serve. Cada grupo de repetições imprime **média,
faixa e dispersão** ao terminar — se a dispersão passar de 15%, ele avisa, e
aí vale repetir ali mesmo, com o robô ligado.

**O ponto 0 é sagrado.** Marcar com fita no chão **e marcar o rumo também**.
Todas as repetições de uma condição saem do mesmo ponto e do mesmo rumo — senão
a dispersão que a gente medir é a sua mão, não a máquina.

**A corrida girada 180° não é repetição, é controle.** Média de três corridas
iguais mata erro aleatório e **não mata erro sistemático**: se o chão tem
caimento, as três desviam igual e a média sai confiante e errada. Girado 180°, o
caimento empurra para o mesmo lado do *mundo* e a assimetria do robô puxa para o
mesmo lado do *corpo* — comparando as duas, dá para dizer qual dos dois você
mediu. É a única corrida que separa robô de sala.

⚠️ **E o rumo inicial é parte da condição, não detalhe.** Em 04-08 o primeiro par
frente × ré foi comparado com **89° de diferença de rumo entre as duas corridas**,
o que deixa caimento de piso entrar na comparação, e a conclusão foi escrita em
cima disso antes de alguém olhar. **Confira o rumo inicial no CSV** (`yaw` da
primeira amostra) antes de comparar duas corridas — se não baterem, não são
comparáveis.

🚨 **DIMENSIONE PELO DISCO VARRIDO, NÃO PELA DISTÂNCIA À FRENTE.** A trava
`--espaco` é **radial** e usa a fonte mais alarmista — que neste robô é a
odometria em `open_loop`, ou seja o **comando ecoado**, que acha que ele andou em
linha reta. Em 04-08 ela disse "2,98 m" enquanto o LIO sabia que o robô estava a
1,79 m da origem fazendo círculo, e **o robô bateu numa cadeira** num espaço com
"3 m livres à frente". Um robô que arca com raio de 1,2 m varre um disco.
Ponha `--espaco` pequeno (1,0 a 1,2): isso limita a distância do ponto 0 **em
todas as direções**, e é a única garantia que existe.

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

**Filmar a traseira nas corridas de ré, e nas de frente para comparar.** O que
se procura é a **boba dando meia-volta**, e o quanto o robô se desvia enquanto
ela decide. É o único jeito de fechar o critério (b) do BO-4: o simulador não
tem boba de verdade, então esse vídeo é a medida. **Em 04-08 foi filmado e não
foi trazido para o repo — é o item mais barato que está faltando.**

## 6. Mandar os dados

Tudo cai em `docs/dados/AAAA-MM-DD-bancada-robo/`.

⚠️ **O NUC não tem autenticação no GitHub.** Os CSVs vêm por `scp` para o dev, e
o commit sai **do dev**:

```bash
scp 'bara@<ip>:~/Controle_robo_livox/docs/dados/AAAA-MM-DD-bancada-robo/*.csv' \
    docs/dados/AAAA-MM-DD-bancada-robo/
git add docs/dados/ && git commit -m "bancada: dados crus da sessão no robô" && git push
```

**Escreva o `ambiente.txt` à mão** se não usar o `sessao.py` — ele é quem
registra piso, bateria, commit e a calibração viva. Sem isso um limiar medido
não volta a ser velocidade de roda; vira número sem unidade. (Em 04-08 piso e
bateria ficaram `NÃO INFORMADO` nas nove corridas.)

**Salve durante a sessão, não no fim.** Bateria e rede caem; em 04-08 os dados
foram commitados em quatro levas, à medida que saíam.

O vídeo vai por fora (é grande demais para o repo).

---

## Se der errado

| sintoma | quase sempre é |
|---|---|
| `/Odometry` não publica | IP do lidar — varredura acima |
| ninguém escuta `cmd_vel` | `base.launch.py` caiu; olhar o primeiro terminal |
| robô não sai do lugar no passo 1 | pode ser o resultado. Repetir com `--rampa-ate 0.6` |
| **gira ao contrário no cutucão** | desde `7a0c364` o cutucão não enrola mais, então **é para levar a sério** — mas confirme com o olho de alguém atrás do robô ANTES de mexer nas rodas |
| ensaio corta cedo | trava de distância (`--espaco`); num robô que arca isso é o normal |
| corrida aborta com "calou por N s" | a fonte morreu. É a trava nova (`--sem-dado`) fazendo o que deve — o CSV parcial vale, a corrida não |
| saltos de ~1,35 m no `/Odometry` | **mais de uma pilha de localização viva**. Contar com `pgrep` |
| ssh cai no meio da corrida | use `-o ServerAliveInterval=15`. A medida não passa pela wifi — o CSV está sendo escrito no NUC |

Nada disso precisa de conserto no laboratório. **Mande os CSVs mesmo assim** —
ensaio que falhou também é dado.
