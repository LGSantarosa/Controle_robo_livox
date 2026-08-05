# Plano de teste no robô real — o que dá para testar, e o que ainda não

> Escrito em **2026-08-05**, depois de uma sessão inteira de simulador.
> Este documento responde três perguntas do dono: **o que testar**, **com que
> mapa**, e **como ele roda**.
>
> Regra da casa que vale aqui: **o dono SÓ RODA.** Nada de relatar console —
> tudo grava CSV que volta por ssh/bundle e eu leio em casa.

---

## 0. A resposta curta

Foi feita muita coisa hoje, mas **ela se divide em duas metades muito
diferentes**, e só uma está pronta para o robô:

| | precisa de mapa? | dá para testar já? |
|---|---|---|
| compensador de rumo (reta e ré) | não | ✅ sim |
| pivô por corte previsto | não | ✅ sim |
| `twist_mux` + teleop (freio de mão) | não | ✅ sim |
| reflexo de colisão (`collision_monitor`) | não | ✅ sim |
| Nav2: planner, seguidor, chegada com ângulo | **sim** | ❌ não |

**Tudo que depende de `map` está bloqueado**, e não por falta de mapa: por
falta de **localização contra o mapa**. Ver a seção 2.

---

## 1. O que testar já — sem mapa nenhum

Estes quatro usam só o `/Odometry` do FAST-LIO e a nuvem do Mid-360. Nenhum
deles sabe o que é `map`.

### ⚠️ Antes de qualquer corrida — a folha de campo continua valendo

`tools/banco/CHECKLIST_ROBO.md`. E os três avisos que já custaram sessões:

1. **Exatamente um** `fastlio_mapping` e **um** `livox_ros_driver2_node`. Em
   31-07, três pilhas órfãs deram saltos de 1,35 m e custaram horas.
2. **`colcon build` antes**, e conferir. `git reset --hard` troca o fonte e
   **não** troca o `install/`; sem o build o robô sobe com a geometria velha.
3. **Anotar piso e bateria.** As nove corridas de 04-08 saíram com
   `NÃO INFORMADO` e por isso não se comparam com as próximas.

### Teste A — o compensador faz o robô andar reto (fatia 4 da decisão 011)

**É o teste mais importante da lista.** Tudo que veio depois se apoia nele, e
no simulador o feedforward é exato *por construção* — no robô a planta do dia
difere e quem paga é o integrador.

**Robô LIGADO. Espaço: 4 m livres à frente, e 2 m para os lados** (se o
compensador falhar, ele arca com raio de 1,2 m).

```bash
# 1) SEM o compensador — a linha de base, para provar que o defeito existe hoje
ros2 launch robot_base base.launch.py
cd tools/banco
python3 ensaio.py --ensaio reta --v 0.25 --wz 0 --dur 12 --espaco 1.2 \
    --janela 0.5 --csv ~/dados/A1-sem-comp-frente-a.csv
# repetir 3x, e 3x com --v -0.25 (ré)

# 2) COM o compensador
ros2 run robot_motion compensador_rumo &
python3 ensaio.py --ensaio reta --v 0.25 --wz 0 --dur 12 --espaco 1.2 \
    --janela 0.5 --topico /compensador_rumo/cmd_vel \
    --csv ~/dados/A2-com-comp-frente-a.csv
# repetir 3x, e 3x de ré
```

**O que eu comparo em casa** (`medir.py --resumo curvatura`):

```
                       o robô fez em 04-08     o simulador com compensador
frente, sem compensar     −0,82 1/m                    —
frente, com compensar         ?                    −0,0025 1/m
ré,     sem compensar     −0,11 1/m                    —
ré,     com compensar         ?                    −0,0003 1/m
```

**Critério (decisão 011):** `|curvatura| < 0,05 1/m` nos dois sentidos.

**O que falsifica:** curvatura acima de 0,05, ou o robô oscilando (o
integrador brigando com a planta). Se acontecer, o suspeito nº 1 é o
`rendimento_giro`, que é do **simulador** e no robô vale 1,0 — o parâmetro
existe, é só passar `-p rendimento_giro:=1.0`.

### Teste B — o pivô fecha (fatia 3, e o número que falta)

**Robô LIGADO. Espaço: 1,5 m de raio livre.**

⚠️ Esta é a varredura que o simulador **não pode** responder: a curva
tempo→ângulo depende da taxa de subida do `wz`, que lá é limite de config
(1,5 exato) e aqui é **inércia** (1,58–2,01, 25% de espalho). Só o robô diz.

```bash
cd tools/banco
for L in 0.3 0.5 0.8 1.2 2.0; do
  python3 ensaio.py --ensaio degrau_giro --v 0 --wz 0.6 --liga $L \
      --dur 10 --espaco 1.0 --csv ~/dados/B-pivo-liga$L-a.csv
done
# repetir a varredura inteira 3x (a, b, c)
```

**O que isso mede**, e o simulador previu:

```
liga [s]   0,3    0,5     0,8     1,2     2,0
giro        0°   3,7°   22,4°   68,9°  215,7°
```

**O número que importa** é a **zona morta de tempo** (abaixo de ~0,4 s ligado
o robô não sai do lugar) e o **pivô mínimo** (~4°). Eles definem a tolerância
do controlador — hoje ela está em 6° por causa do simulador.

### Teste C — o freio de mão funciona

**Robô LIGADO, alguém com a mão no teclado, robô no chão com espaço.**

⚠️ **ANTES, uma vez por máquina** (05-08): o `twist_mux` do apt **não sobe** —
a `diagnostic_updater` do NUC é mais velha que o binário e falta um símbolo. Ele
vem por fonte, e o `git reset --hard` **não** traz o clone (é `.gitignore`):

```bash
./setup_twist_mux.sh      # clona em commit fixado e compila
ros2 pkg prefix twist_mux # tem de apontar para o install/ daqui, não /opt/ros
```

🛑 **NÃO resolva com `sudo apt upgrade ros-jazzy-diagnostic-updater`.** Está
medido: a 4.2.7 **remove** o símbolo que a 4.2.6 exporta, e isso quebraria o
`controller_manager` — a base pode não subir depois. Ver a 10ª leva de 08-05.

✅ **Os itens 2 e 3 abaixo já foram provados sem robô** com
`tools/banco/prova_mux.py` (o humano vence a autonomia; soltar devolve o
comando). No laboratório, o que **precisa** de máquina é o **item 1**.

```bash
ros2 launch robot_motion pilha.launch.py     # sem sim:=true
# noutro terminal ssh:
bin/robot-key
```

**O que se verifica, e é a olho** (este não precisa de CSV):

1. `w` faz andar, `s` faz recuar, `a`/`d` giram. **Solte o teclado: ele para
   sozinho em 0,4 s** (homem-morto).
2. Com a autonomia comandando, apertar `w`/`s` **vence** — o humano manda.
3. Soltar devolve o comando à autonomia.

⚠️ **Se o item 1 falhar, PARE TUDO.** Um teleop que não para sozinho é pior
que teleop nenhum.

### Teste D — o reflexo de colisão para o robô

**Robô LIGADO. Uma caixa de papelão de pelo menos 40 cm de altura**, num
corredor livre de 3 m.

⚠️ **A altura da caixa não é detalhe.** O Mid-360 não vê o chão dentro de
~2 m; a 0,5 m ele só enxerga o que passa de ~21 cm. Caixa baixa **não vai ser
vista**, e isso não é defeito do software.

```bash
ros2 launch robot_motion pilha.launch.py
# empurrar o robô para um corredor com a caixa a ~2 m à frente, e mandar
# ele andar reto pelo teleop OU publicar em /auto_vel_raw
```

**Esperado** (simulador): ele para com o **centro** a ~0,49 m da face da
caixa, ou seja ~0,27 m de folga entre o para-choque e a caixa.

**E o segundo teste, que é o que dá segurança:** com o reflexo segurando,
apertar `s` no teclado — o robô **tem** de recuar. Se ele não recuar, o
humano está preso pelo reflexo, e isso é defeito grave.

---

## 2. O mapa — e por que ele não é o problema principal

### O que existe hoje, e o que não existe

- **`maps/pista_obstaculos.pgm`** é o único mapa do repo, e ele é **sintético**:
  o cabeçalho diz `GERADO por tools/mundo/gera_pista.py — não editar à mão`.
  É a planta da pista do **simulador**. Não corresponde a lugar nenhum do
  mundo real.
- **O mapa do estagiário NÃO está neste repo.** O workspace dele
  (`ESTAGIO-2026/`) está **fora do git**, na máquina — é o que a entrada de
  30-07 do diário descreve. Se houver um mapa lá, ele é do local onde ele
  rodou, e precisaria ser trazido para cá deliberadamente.

### 🔴 Mas o bloqueio de verdade não é o mapa: é a LOCALIZAÇÃO

O `pilha.launch.py` publica hoje uma TF **fixa** `map → odom` (identidade),
com este comentário no código:

> *"A TF que falta: sem localização contra o mapa, `map` e `odom` são o mesmo
> lugar. Provisório."*

No simulador isso funciona porque **mundo e mapa saem da mesma planta** e o
robô nasce no ponto certo. **No robô real não funciona**: o `odom` do
FAST-LIO começa onde o robô ligou, e nada diz onde isso fica no mapa. Sem
essa amarração o costmap global inventa parede onde não tem — e o planner
recusa ou roteia contra obstáculo fantasma.

**Ou seja: ter um mapa não destrava o Nav2. Falta o que registra o robô nele.**

### As três saídas, e a recomendação

| saída | o que dá | o que custa |
|---|---|---|
| **(a) SLAM online** — o mapa nasce enquanto o robô anda, e `map ≡ odom` por construção | destrava o Nav2 **sem** resolver localização | mapa não persiste entre sessões; sem "volte ao ponto X de ontem" |
| **(b) Mapear antes, localizar depois** | mapa reaproveitável, objetivos fixos | precisa de um localizador 3D→mapa que o repo **não tem** |
| **(c) Trazer o que o estagiário usa** | ele já *"anda e faz SLAM"* (diário 30-07) | é pilha de terceiro, e as 3 hipóteses de 30-07 sobre ela nunca foram fechadas |

**Recomendo (a)**, e por uma razão de método: ela é a **menor mudança que
destrava o teste**, e mantém a decisão 003 (localização é LIO, sem AMCL). O
mapa persistente é problema de outro dia — o objetivo agora é ver a pilha
andando num lugar real, não guardar plantas.

⚠️ E há um detalhe técnico que precisa de decisão explícita: o costmap do
Nav2 é **2D** e o Mid-360 entrega **nuvem 3D**. Alguém tem de projetar. Isso
reabre — em escopo restrito — o `/scan` derivado que a decisão 003 descartou,
e vale dizer em voz alta: **derivar 2D para não bater é diferente de derivar
2D para se localizar.** É a mesma pergunta que a decisão 010 deixou aberta.

---

## 3. O que só dá para testar depois disso

Tudo o que hoje está bonito no simulador e **não** tem como ser julgado no
robô ainda:

- **Planner Theta\*** e a troca dele pelo Smac (revisão da 008).
- **Chegada em duas fases** (vai reto, pivota no ponto para o ângulo).
- **Passar no meio da porta** — os números de inflação 0,50 e
  `cost_scaling_factor` 3,0 foram sintonizados contra a pista simulada.
- **As batidas**: de 2509 invasões para zero. Isso vale para *aquela* pista.

**Ressalva honesta:** a sintonia de costmap de hoje foi feita num vão de
0,90 m gerado por script. Num corredor real ela pode ficar conservadora
demais (robô recusando passagens) ou frouxa. **É sintonia, não medida** — e
tem de ser refeita no lugar de verdade.

---

## 4. Como o dono roda, e como o dado volta

1. **Deploy**: `git fetch && git reset --hard origin/main` no NUC. Nunca
   `scp` de arquivo solto (regra do `CLAUDE.md`).
   ⚠️ O NUC **não tem autenticação no GitHub** — ver a entrada de 30-07 do
   diário; o caminho que funcionou foi `git bundle`.
2. **`colcon build`** e conferir que pegou.
3. Rodar os testes na ordem A → B → C → D, **um por vez**, anunciando antes.
4. Os CSV ficam em `~/dados/` no NUC e voltam por bundle/ssh.
5. **`ambiente.txt` junto**, com piso, bateria e commit. Sem isso a sessão
   não se compara com a próxima.

---

## 5. O que falta no repo (próximos passos)

### Do levantamento de segurança (decisão 010)

- ✅ `twist_mux` — feito
- ✅ `collision_monitor` — feito
- ⏳ **detecção por sintoma** (a metade agnóstica do `unstuck_supervisor`),
  com **pivô** como recuperação primária. A premissa "a manobra é sempre ré"
  é do skid-steer do robô 1 e caiu.
- ⏳ **medida de vão livre** contra a nuvem (para quando a ré for necessária)
- ⏳ **`motion_guard`** reescrito pequeno — o do robô 1 é metade filtro
  anti-vidro, defesa contra um defeito do LD06 que este sensor não tem

### Da movimentação

- ⏳ **Fluidez.** Palavra do dono: *"tá parandinho demais"*. Ele pivota com 15°
  de erro. Precisa de **régua** antes de sintonia — sem número, vira gosto.

### Do simulador

- ⏳ **Dispersão (12c)**: ele repete a 0–4%, o robô espalha 21–37%. Um
  simulador determinista faz qualquer controlador parecer mais confiável do
  que vai ser.
- ⏳ **Zona cega**: medir com trena **onde o Mid-360 está montado**. É o item
  mais barato e de maior retorno da lista — a zona cega escala ~8,1× com a
  altura, e ela é o que limita o reflexo de colisão.
- ⏳ **BO-4**: o arco entra no modelo disfarçado de assimetria de roda. O
  encoder simulado mente, e isso quebra se alguém ligar `open_loop: false`.

### Dívidas de instrumento

- ⏳ `rajada_rodas.py` ainda não percebe que o robô sumiu (o `ensaio.py` já
  percebe desde 04-08).
- ⏳ Passos 4 e 5 do banco (curva e aceleração) **precisam ser reescritos**:
  varrem 0,2/0,4/0,6 m/s e as três caem dentro do patamar da compensação.
- ⏳ `sessao.py` não conduz mais uma sessão deste robô (não deixa passar
  `--espaco`, e o passo 6 está morto).
