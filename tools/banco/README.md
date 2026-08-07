# Banco de ensaios — descobrir os limites do robô 2

Protocolo de caracterização. **Antes de escrever qualquer controlador**, medir
o que a máquina consegue fazer. Todos os números que a movimentação precisa
saem daqui, e nenhum deles é herdado ou chutado.

O mesmo banco roda no **robô real** e no **simulador** — mesmos tópicos
(`/hoverboard_base_controller/cmd_vel` e `/Odometry`), mesmo CSV, mesma
leitura. É isso que permite dizer *o quanto* o simulador mente, com medida em
vez de opinião.

## Antes de começar

**Robô LIGADO**, base de pé (`ros2 launch robot_base base.launch.py`), com
`/Odometry` publicando — sem localização não há medida nenhuma, porque toda
velocidade aqui é tirada da pose.

Conferir antes de soltar o primeiro ensaio — e o jeito certo é o condutor, que
faz isso e mais:

```bash
python3 sessao.py --checar            # não anda; recusa medir se faltar algo
python3 sessao.py --checar --mexer    # + cutucão de sanidade (o robô se mexe)
```

Ele mede a taxa dos dois tópicos de odometria, conta os ouvintes do `cmd_vel` e
**pergunta ao controlador que robô ele acha que está dirigindo**:
`wheel_separation`, `wheel_radius` e os nomes de roda esquerda/direita, lidos do
nó vivo e comparados com a trena de 29-07 (0,270 e 0,080).

Isso não é preciosismo. O `tracao.launch.py` lê o YAML e o xacro via
`FindPackageShare` — da cópia em `install/`, não do fonte. Trocar o fonte sem
recompilar deixa os dois divergindo **em silêncio**, e a bitola entra na
conversão comando→roda: 0,32 num robô de 0,270 desloca todo limiar medido em
18,5%. Pior, depois, em casa, esse desvio é **indistinguível de derrapada** —
os dois mexem no mesmo número em sentidos opostos e chegam a se cancelar.

A conferência **não bloqueia** por divergência (ela pode ser deliberada); ela
delata e **grava a calibração no `ambiente.txt`**, ao lado do commit. Commit
descreve o fonte; estas linhas descrevem o que estava dirigindo o robô na hora,
e é o que permite converter um limiar de volta para velocidade de roda.

Os nomes de roda aparecem aí porque é neles que vive a correção do **giro
espelhado** achado em 30-07 (`+0,6 rad/s` girou `−78,5°`) — dá para ver se o
swap está no ar antes de o robô se mexer.

Manualmente, se preferir:

```bash
ros2 topic hz /Odometry                      # tem que estar publicando
ros2 param get /hoverboard_base_controller wheel_separation
```

**Espaço:** uma área livre de pelo menos **5 × 3 m**. O robô anda de verdade.
Todo ensaio tem trava de distância (`--espaco`, 4 m por padrão): estourou, ele
para sozinho. A trava não substitui alguém de olho e com a mão no disjuntor.

**Piso:** o mesmo em todos os ensaios, e anotado no diário. Atrito de piso muda
derrapada e zona morta — medir no carpete e rodar no cimento invalida tudo.

## A ordem, e por que ela é essa

### 1. Zona morta de GIRO — *o robô gira parado com quanto?*

```bash
python3 ensaio.py --ensaio zona_morta_giro --csv zm_giro.csv --dur 220 --rampa-ate 1.5 --dentes 4 --janela 0.5
python3 medir.py zona_morta_giro zm_giro.csv
```

**Dente de serra**, girando no lugar (linear zerada): a rampa sobe até o robô
girar, desce até ele parar, inverte o sentido e repete 4x. Espaço: raio de 1 m
livre em volta; ele não sai do lugar. O `--dur` é teto de tempo, não duração.

> ⛔ **Não use `--fonte roda` neste robô (31-07, 4ª leva).** O controlador está
> com `open_loop: True`, e `/hoverboard_base_controller/odom` devolve o comando
> integrado, não medida. Uma leva inteira de números medidos assim foi
> retratada — ela reproduzia o próprio limiar de detecção. Meça pelo LIO, com
> `--janela 0.5` (a pose vem a 10 Hz; 0,2 s pegam duas amostras e viram ruído
> inventado).

> **Sobre "girar é o pior caso": continua NÃO verificado.** Uma tentativa de
> refutar a frase em 31-07 usou dado inválido e foi retratada junto. A medida
> boa que sobrou (giro ≈ 0,095 rad/s, linear ≈ 0,023 m/s) não compara de forma
> limpa, porque a linear **não tem patamar** — ele rasteja antes de andar, e o
> valor depende de quanto deslocamento se exige (0,029 a 8 cm, 0,040 a 20 cm).
> Fica em aberto. De todo jeito o `CLAUDE.md` proíbe herdar o número do robô 1.

> 🔴 **E o pressuposto do ensaio inteiro caiu:** `cmd_vel` não é velocidade
> neste robô, é acelerador. Sem malha fechada de roda, ele **acelera enquanto o
> comando estiver ligado** — 0,30 rad/s comandados viraram 3,94 rad/s em 1,35 s.
> A rampa do dente de serra assume que um comando sustentado dá velocidade
> sustentada, e isso é falso aqui. O protocolo precisa ser repensado antes de
> valer como caracterização; ver DIARIO 07-31 4ª leva.

Devolve **dois** números, e o projeto precisa dos dois — **saída** (do repouso,
atrito estático, o número do BO-3) e **queda** (já andando, sempre menor, que é
o que o piso de velocidade do seguidor precisa). E sai com **faixa**: um limiar
de atrito é uma distribuição, e cada dente parte com o rotor num ponto
diferente. É por isso que a repetição vive DENTRO da corrida, e que este ensaio
não repete em corrida.

> **O dente #0 é um caso à parte, e a leitura o separa.** Só ele parte de um
> repouso longo; os demais partem da pausa de 1 s entre dentes, e atrito
> estático **cresce com o tempo parado**. São condições físicas diferentes, e
> a média das quatro misturaria as duas bem no número do BO-3 — que é
> exatamente "o robô estava parado e mandaram andar". Quando o #0 destoa mais
> de 30% da mediana dos outros, o `medir.py` mostra os dois separados e diz
> qual usar para cada caso: arrancar do repouso, ou arrancar em manobra
> encadeada.

**Primeiro de todos** — e não por importância, mas porque responde a pergunta do
pivô **diretamente**: o menor `wz` que gira o robô parado *é* o limiar do pivô,
em rad/s, sem conversão. Pelo linear só se chega lá por `2·zm/L`, confiando de
novo na bitola. Some o risco de sessão cortada (em 30-07 não se mediu nada): o
que fica por último é o que se perde.

É o **item nº 1 do projeto**, e a folga é de 4%: com zona morta 0,10 pivotar
exige 0,96 rad/s contra teto de 1,0; com 0,15 exige 1,48 e é impossível. A
resposta vira dentro da faixa, e é por isso que a faixa importa tanto quanto a
média.

No simulador, com zona morta de roda em 0,15 m/s, este é o ensaio em que o robô
fica **22 s parado** com o controlador pedindo 1,0 rad/s. Se acontecer no real,
não é defeito — é a medida.

### 2. Zona morta linear — *o mesmo limiar, pela outra porta*

```bash
python3 ensaio.py --ensaio zona_morta_linear --csv zm_lin.csv --dur 180 --rampa-ate 0.35 --dentes 4
python3 medir.py zona_morta_linear zm_lin.csv
```

O mesmo dente de serra do passo 1, agora andando reto. Espaço: ~3 m (ele vai e
volta, quase não se afasta).

Segundo, mas não menos: andando reto a velocidade da roda **é** a velocidade do
robô, então o que sai daqui é o limiar da roda em m/s **direto**, sem bitola no
meio — e é ele que entra no `v_piso` do seguidor. Vale também como conferência
do passo 1: os dois medem o mesmo atrito por caminhos diferentes e têm de fechar
por `2·zm/L`. Se não fecharem, ou a bitola está errada ou as duas rodas não são
iguais.

Saída e queda, com faixa, como no passo 1. Aqui a **queda** é o número que vai
direto para o `v_piso`: usar a saída no lugar dela deixa o piso alto demais e o
robô mais rápido do que precisa perto do alvo.

Se o robô **não sair do lugar** em dente nenhum, refazer com `--rampa-ate 0.6`.
Isso não é falha do ensaio, é o resultado.

> **A taxa da rampa é parte da medida.** `--rampa-seg` (padrão 20 s de 0 ao
> teto) define quão rápido o comando sobe, e o limiar é lido na primeira
> amostra que passa de `LIMIAR_PARADO` — então rampa mais rápida infla o número
> pelo atraso de detecção. No giro, a taxa de hoje já infla ~0,011 rad/s, e a
> decisão do pivô se joga entre 0,10 e 0,15. Mais dentes custam TEMPO, nunca
> precisão; mexer no `--rampa-seg` é mexer no número.

### 3. Degrau de giro — *quanto ele demora pra PARAR de girar?*

```bash
python3 ensaio.py --ensaio degrau_giro --csv deg_wz03.csv --v 0.3 --wz 0.3 --dur 12
python3 ensaio.py --ensaio degrau_giro --csv deg_wz06.csv --v 0.3 --wz 0.6 --dur 12
python3 ensaio.py --ensaio degrau_giro --csv deg_wz10.csv --v 0.3 --wz 1.0 --dur 12
python3 medir.py degrau_giro deg_wz03.csv     # e assim por diante
```

Anda reto, gira por 2 s, **corta o giro** e mede quantos graus ele ainda roda.
Daí sai o `a_dec` — a desaceleração angular real:

```
a_dec = wz² / (2 · Δrumo depois do corte)
```

**É o número mais importante do projeto.** Ele é a causa medida do S: um
controlador que manda giro até o erro trocar de sinal atravessa o alvo
exatamente por `wz²/(2·a_dec)`. Rodar nos três níveis de `wz` mostra se ele é
constante ou se piora com giro forte.

Espaço: ~4 m, e ele termina apontando para outro lado.

### 4. Curva sustentada — *quanto ele curva a x, 2x, 3x de velocidade?*

```bash
python3 ensaio.py --ensaio curva --csv curva_v02_wz05.csv --v 0.2 --wz 0.5 --dur 12
python3 ensaio.py --ensaio curva --csv curva_v04_wz05.csv --v 0.4 --wz 0.5 --dur 12
python3 ensaio.py --ensaio curva --csv curva_v06_wz05.csv --v 0.6 --wz 0.5 --dur 12
python3 medir.py curva curva_v04_wz05.csv
```

Segura uma curva e compara o **giro pedido com o giro realizado**, mais a
**derrapada** (o quanto a odometria de roda mente em relação à pose do LIO).

É aqui que a geometria do robô 2 aparece: motriz na frente, boba atrás. A
traseira é jogada para fora, e quanto mais rápido, pior. Se o giro realizado
cair conforme a velocidade sobe, está medido — e vira restrição de projeto.

Espaço: o robô descreve um círculo de raio `v/wz` (a 0,4 m/s e 0,5 rad/s são
0,8 m de raio). Deixar folga.

### 5. Aceleração linear — *arranca e freia quanto?*

```bash
python3 ensaio.py --ensaio aceleracao_linear --csv acel.csv --v 0.6 --dur 10
python3 medir.py aceleracao_linear acel.csv
```

Degrau de linear, segura 4 s, corta. Devolve velocidade atingida, aceleração e
desaceleração reais — para conferir se os tetos do
`hoverboard_controllers.yaml` (0,7 m/s e 0,8 m/s²) descrevem esta máquina ou
foram herdados sem medir. Espaço: ~4 m em reta.

### 6. Reta com cutucão — *o rumo volta ou foge? E de ré?*

```bash
python3 ensaio.py --ensaio reta --csv reta_frente.csv --v  0.25 --wz 0.5 --dur 16
python3 ensaio.py --ensaio reta --csv reta_re.csv     --v -0.25 --wz 0.5 --dur 16
python3 medir.py reta reta_frente.csv
python3 medir.py reta reta_re.csv
```

Anda reto, leva um pulso de giro de 0,5 s e **solta**. `--v` aceita negativo, e
é assim que se mede a ré. Espaço: ~4 m em reta, nos dois sentidos.

**É o ensaio que decide se a manobra de ré da decisão 007 é segura.** Andando
para trás a boba deixa de ser arrastada e passa a ser empurrada — vira roda
dianteira, que é a configuração instável do carrinho de supermercado. O que
interessa não é o tamanho do desvio e sim se ele **cresce**: um diferencial não
tem nada que traga o rumo de volta sozinho, então quem denuncia instabilidade é
a velocidade de giro depois de soltar. Assentou e ficou, as motrizes dominam;
sobreviveu ou cresceu, a boba está mandando e a ré precisa de teto menor.

O pulso não é enfeite: sem ele o ensaio não mede nada **no simulador**, porque
lá o robô é perfeitamente simétrico num plano liso e reta pura dá desvio zero
exato nos dois sentidos. No robô real a assimetria perturba sozinha, e vale
rodar também com `--wz 0` para ver o desvio natural.

⚠️ **O simulador não substitui este ensaio.** A boba dele não chega a virar
(BO-4), então lá ré e ida deram idênticas — o que não é evidência de que a ré
seja segura, é evidência de que aquele modelo não tem boba nenhuma.

Vale filmar a traseira: o que se procura é a boba dando meia-volta, e quanto o
robô se desvia enquanto ela decide.

## O que fazer com os números

| ensaio | devolve | vai para |
|---|---|---|
| 1 e 2 | zona morta | `v_piso = zona_morta + wz_max·bitola/2 + margem` |
| 3 | `a_dec` | o parâmetro central da lei de frenagem |
| 4 | giro realizado, derrapada | teto de `wz` por velocidade; calibrar o simulador |
| 5 | aceleração e velocidade reais | tetos do `diff_drive_controller` |
| 6 | rumo assenta ou foge, ida × ré | teto de velocidade da ré (decisão 007); BO-4 |

E, com trena, antes de tudo: **`wheel_separation` e `wheel_radius`**. Os valores
no YAML (0,32 e 0,0825) nunca foram medidos neste robô, e o
`diff_drive_controller` usa os dois para converter comando em rad/s de roda —
errar ali erra todo ensaio acima.

## Rodando no simulador

O mesmo banco, com `--sim` (usa o relógio da simulação):

```bash
ros2 launch robot_base sim.launch.py gui:=false
python3 ensaio.py --ensaio degrau_giro --csv sim_deg.csv --v 0.3 --wz 1.0 --sim
```

**Um simulador por vez.** Dois Gazebos vivos publicam dois `/clock`, o tempo
anda para trás e o CSV sai embaralhado — o banco morre de propósito quando
detecta isso, em vez de gravar dado sujo.

## Percepção — o costmap vê o mundo ou repete o mapa?

Estes dois não medem o robô, medem o que a pilha **sabe**. Nasceram em 07-08
com a decisão 014 (a nuvem entrando nos costmaps) e servem tanto no simulador
quanto no robô parado.

```bash
# o que o costmap marcou que o mapa NÃO conhece — só o sensor explica
python3 tools/banco/percepcao.py --qual local_costmap
python3 tools/banco/percepcao.py --qual global_costmap --csv manchas.csv

# o Nav2 desvia? planeja SEM MOVER o robô (ação compute_path_to_pose)
python3 tools/banco/plano.py --alvo 2.0 7.2 --de 2.0 5.0 --obstaculo 2.0 6.5 0.35
```

**Como se prova percepção, e por que a pista antiga não provava:** mundo e mapa
saem da mesma planta (`tools/mundo/gera_pista.py`), então um robô que desvia
pode estar vendo com o lidar OU repetindo o mapa — indistinguível. Rode com o
mundo que tem o que o mapa não tem:

```bash
python3 tools/mundo/gera_pista.py          # escreve também pista_surpresa.sdf
ros2 launch robot_motion pilha.launch.py sim:=true gui:=false \
     mundo:=$PWD/worlds/pista_surpresa.sdf     # o mapa continua o mesmo
```

⚠️ **`percepcao.py` conta 254, não 253.** No costmap cru do Nav2, 253 é
`INSCRIBED_INFLATED_OBSTACLE` — **inflação**, não obstáculo. Contar 253 faz cada
parede aparecer 0,32 m mais gorda (o `robot_radius`) e parecer marcação
inventada pelo sensor. Custou meia hora em 07-08, com a nuvem crua conferida
ponto a ponto — e ela estava certa.

⚠️ **O robô só marca a FACE que viu.** Uma caixa de 0,25 m² vira 0,075 m² de
costmap. Ao julgar folga de plano, lembre que ele está contornando a lasca
marcada, não o corpo inteiro do obstáculo.

## Pré-voo — o que sobreviveu ao robô?

```bash
python3 tools/banco/checa_pilha.py --csv docs/dados/AAAA-MM-DD-.../preflight.csv
```

30 s, **não move o robô**, e responde item a item se o que foi escrito sem
máquina funciona nela: uma pilha só, nuvem e taxa, as duas TFs (incluindo
`base_link → livox_frame` contra os 0,42 m da trena), fração de nuvem
transformável, `lifecycle` dos quatro servidores, perfil sem mapa (decisão 015),
os dois costmaps marcando e a cadeia de comando inteira. Cada falha traz o
conserto na própria linha.

Duas leituras que ele já embute, porque as duas enganaram alguém neste projeto:

- **conta processo lendo `ps`, nunca `pgrep -c`** — que casa com o próprio shell
  e devolve 2 com zero processos vivos;
- **`/auto_vel` calado com o robô parado é o CERTO**: o `collision_monitor` não
  republica comando nulo (medido 07-08 — 3 s de zeros dão 0 mensagens, 3 s de
  0,05 m/s dão 150). Marca ⚠️, não ❌.

⚠️ Ele mede a TF parada. **Empurre o robô com a mão e rode de novo**: a
translação de `odom → base_link` tem de mudar.
