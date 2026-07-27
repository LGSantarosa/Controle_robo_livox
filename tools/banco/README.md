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

Conferir antes de soltar o primeiro ensaio:

```bash
ros2 topic hz /Odometry                      # tem que estar publicando
ros2 topic echo /Odometry --once             # pose faz sentido?
```

**Espaço:** uma área livre de pelo menos **5 × 3 m**. O robô anda de verdade.
Todo ensaio tem trava de distância (`--espaco`, 4 m por padrão): estourou, ele
para sozinho. A trava não substitui alguém de olho e com a mão no disjuntor.

**Piso:** o mesmo em todos os ensaios, e anotado no diário. Atrito de piso muda
derrapada e zona morta — medir no carpete e rodar no cimento invalida tudo.

## A ordem, e por que ela é essa

### 1. Zona morta linear — *o robô sai do lugar com quanto?*

```bash
python3 ensaio.py --ensaio zona_morta_linear --csv zm_lin.csv --dur 20 --rampa-ate 0.35
python3 medir.py zona_morta_linear zm_lin.csv
```

Rampa lenta de 0 a 0,35 m/s em 20 s, andando reto. Devolve o **menor comando
que produz movimento**. Espaço: ~3 m em linha reta.

Primeiro de todos porque é o número que decide se o robô anda: comando abaixo
dele é silêncio absoluto — nó vivo, tópico publicando, robô imóvel. Já custou
horas de depuração na competição de 2025.

Se o robô **não sair do lugar** em nenhum ponto da rampa, refazer com
`--rampa-ate 0.6`. Isso não é falha do ensaio, é o resultado.

### 2. Zona morta de giro — *e girando parado?*

```bash
python3 ensaio.py --ensaio zona_morta_giro --csv zm_giro.csv --dur 20 --rampa-ate 1.5
python3 medir.py zona_morta_giro zm_giro.csv
```

Mesma rampa, girando no lugar (linear zerada). É o **pior caso**: as duas rodas
ficam pequenas ao mesmo tempo, então é aqui que a zona morta morde com força
total. Espaço: raio de 1 m livre em volta.

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

## O que fazer com os números

| ensaio | devolve | vai para |
|---|---|---|
| 1 e 2 | zona morta | `v_piso = zona_morta + wz_max·bitola/2 + margem` |
| 3 | `a_dec` | o parâmetro central da lei de frenagem |
| 4 | giro realizado, derrapada | teto de `wz` por velocidade; calibrar o simulador |
| 5 | aceleração e velocidade reais | tetos do `diff_drive_controller` |

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
