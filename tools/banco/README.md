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

### 1. Zona morta linear — *o robô sai do lugar com quanto?*

```bash
python3 ensaio.py --ensaio zona_morta_linear --csv zm_lin.csv --dur 180 --rampa-ate 0.35 --dentes 4
python3 medir.py zona_morta_linear zm_lin.csv
```

**Dente de serra**, andando reto: a rampa sobe até o robô sair do lugar, desce
até ele parar, inverte o sentido e repete. Espaço: ~3 m (ele vai e volta, quase
não se afasta). O `--dur` é teto de tempo, não duração — os 4 dentes fecham bem
antes.

Primeiro de todos porque é o número que decide se o robô anda: comando abaixo
dele é silêncio absoluto — nó vivo, tópico publicando, robô imóvel. Já custou
horas de depuração na competição de 2025.

Devolve **dois** números, e o projeto precisa dos dois:

- **saída** — do repouso, atrito estático. É o número do BO-3, e é ele que
  decide se o robô pivota;
- **queda** — já andando, sempre menor, porque manter andando custa menos que
  arrancar. É o que o piso de velocidade do seguidor precisa. Usar a saída no
  lugar dela deixa o piso alto demais perto do alvo.

Sai com **faixa**, não com um número solto: um limiar de atrito é uma
distribuição, e cada dente parte com o rotor numa posição diferente — que é a
fonte real da dispersão. Por isso a repetição vive DENTRO da corrida e este
ensaio não repete em corrida.

Se o robô **não sair do lugar** em dente nenhum, refazer com `--rampa-ate 0.6`.
Isso não é falha do ensaio, é o resultado.

> **A taxa da rampa é parte da medida.** `--rampa-seg` (padrão 20 s de 0 ao
> teto) define quão rápido o comando sobe, e o limiar é lido na primeira
> amostra que passa de `LIMIAR_PARADO` — então rampa mais rápida infla o número
> pelo atraso de detecção. No giro, a taxa de hoje já infla ~0,011 rad/s, e a
> decisão do pivô se joga entre 0,10 e 0,15. Mais dentes custam TEMPO, nunca
> precisão; mexer no `--rampa-seg` é mexer no número.

### 2. Zona morta de giro — *e girando parado?*

```bash
python3 ensaio.py --ensaio zona_morta_giro --csv zm_giro.csv --dur 220 --rampa-ate 1.5 --dentes 4
python3 medir.py zona_morta_giro zm_giro.csv
```

Mesmo dente de serra, girando no lugar (linear zerada). É o **pior caso**: as
duas rodas ficam pequenas ao mesmo tempo, então é aqui que a zona morta morde
com força total. Espaço: raio de 1 m livre em volta.

É o **item nº 1 do projeto**, e a folga é de 4%: com zona morta 0,10 pivotar
exige 0,96 rad/s contra teto de 1,0; com 0,15 exige 1,48 e é impossível. A
resposta vira dentro da faixa, e é por isso que a faixa importa tanto quanto a
média.

No simulador, com zona morta de roda em 0,15 m/s, este é o ensaio em que o robô
fica **22 s parado** com o controlador pedindo 1,0 rad/s. Se acontecer no real,
não é defeito — é a medida.

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
