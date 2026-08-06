# Plano de sintonia do rumo no robô — matar o S

> Escrito em **2026-08-05**, depois de uma sessão inteira de simulador que
> **não conseguiu** reproduzir o S. Este documento responde três perguntas:
> **o que sintonizar**, **com que régua**, e **como o dono roda**.
>
> Regra da casa que vale aqui: **o dono SÓ RODA.** Nada de relatar console —
> tudo grava CSV que volta por ssh e eu leio em casa. Uma corrida por vez,
> anunciando antes e esperando o "pode".

---

## 0. A resposta curta

O compensador de rumo **funciona**: derrubou 95% do arco de frente (raio de
1,10 m para 24 m) e passou o critério na média, nos dois sentidos. O que sobrou
é o que o dono viu a olho:

> *"de frente ele faz um pequeno S para tentar compensar o erro, não é o que eu
> quero, quero ele completamente andando reto"*

**Esse S só pode ser sintonizado no robô.** Não é preguiça de tentar no
simulador — foi tentado, com número, e a seção 1 mostra por quê não dá.

---

## 1. Por que isto NÃO se faz no simulador (leia antes de duvidar)

Em 05-08 o simulador ganhou as duas pontas da placa (atraso de liga e de
desliga) e a taxa do robô (10 Hz). Com isso ele passou a reproduzir o arco
melhor do que nunca — razão frente/ré de **8,74×** contra 7,4× e 9,4× das duas
sessões do robô. Mesmo assim:

```
varredura de ki    1,00 -> 0,50 -> 0,25 -> 0,10 -> 0,00
inversões de rumo     1      1      1      1      1      <- nunca mudou
varredura de kp    1,00 -> 0,40
inversões             1      1                           <- nem isso
                                        (e kp menor PIOROU: amp 8,5° -> 11,0°)
```

E a corrida longa fechou: **27 segundos, 8 metros, 1 inversão só, deriva final
0,0°**. Isso é o transiente da malha pegando o rumo e assentando — não é
ciclo-limite. O robô fez **2 inversões em 4,9 s**, com período de 2,2 s.

**Duas razões estruturais, e as duas são conhecidas:**

1. **No simulador o feedforward cancela o arco por construção.** A placa
   fingida usa `curvatura_frente = −0,817` e o compensador usa
   `curv_frente = −0,817` — o mesmo número. Não sobra erro para o integrador
   trabalhar, então `ki` ali só pode piorar. *(Testado também descasado: planta
   em −0,9116 contra ff de −0,817, que é a situação real do robô. Continuou
   1 inversão.)*
2. **BO-4: a boba do simulador é um patim, não uma boba.** O contato dela não
   participa da dinâmica — multiplicar o atrito por 16 mudou o rumo em 0,4%.
   Uma oscilação de rumo puxada por roda boba arrastada **não pode** aparecer
   nesse modelo.

➡️ Consequência de método: **qualquer valor de ganho decidido no simulador seria
decidido contra um robô que não oscila.** É a forma clássica de o simulador
enganar, e este projeto já pagou por ela três vezes (a bitola em 29-07, a régua
do planner em 29-07, os "20 ms" da bancada de 05-08).

---

## 2. O que se mede, e com que régua

Duas réguas, e elas respondem perguntas diferentes. **Rodar as duas sempre.**

```bash
# quanto ele sai da reta por metro — o critério da decisão 011
python3 tools/banco/medir.py --resumo curvatura ~/dados/*.csv

# ele PENDE ou OSCILA? — a régua nova (05-08)
python3 tools/banco/mede_o_s.py ~/dados/*.csv
```

A segunda existe porque a primeira **não distingue os dois defeitos**: um robô
que serpenteia ±10° e volta sempre dá curvatura média quase nula, igual a um que
pende pouco. Como ler:

```
invs 0  + deriva grande   -> PENDE     (o robô cru, sem compensador)
invs >= 2 + deriva ~0     -> OSCILA    (o S)
invs 1                    -> transiente: corrigiu uma vez e assentou
```

### A linha de base, medida em 05-08 — é contra isto que se compara

```
                          curvatura        amp     invs   período   deriva
sem compensador     −0,9116 (raio 1,10 m)  68,0°     0       —       −68°
com compensador     +0,0417 (raio 24 m)  9,4–18,5°   2    2,2–2,4s   1–4°
                                          critério: |curv| < 0,05
```

⚠️ **3 das 6 corridas compensadas estouraram o critério individualmente**
(frente-b 0,0625; ré-b 0,0525; ré-c 0,0505). Passa na média de três, não em toda
corrida. **O alvo desta sessão é passar em TODA corrida, com `invs` caindo.**

---

## 3. Antes de qualquer corrida

Vale a folha de campo inteira (`tools/banco/CHECKLIST_ROBO.md`), mais os pontos
que esta sessão acrescenta.

### Deploy

```bash
# no PC de dev, com o robô na mesma rede
git bundle create /tmp/repo.bundle --all
scp /tmp/repo.bundle bara@<ip-do-nuc>:/tmp/
# no NUC
cd ~/Controle_robo_livox
git fetch /tmp/repo.bundle main && git reset --hard FETCH_HEAD
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros2_packages --symlink-install \
    --packages-select hoverboard_driver robot_base robot_motion
source install/setup.bash
```

⚠️ **`colcon build` sempre.** `git reset --hard` troca o fonte e **não** troca o
`install/`, e é de lá que o `tracao.launch.py` lê a descrição.

### 🔴 Conferência que esta sessão tornou obrigatória

Em 05-08 o `tracao.launch.py` passou a carregar **outra descrição** do robô
(`robo2.urdf.xacro`, a medida, no lugar do exemplo de demonstração do
`ros2_control`). Os blocos de hardware foram comparados e saem idênticos, mas
**isso nunca rodou no robô**:

```bash
python3 tools/banco/sessao.py --checar
```

**Tem de imprimir `wheel_separation = 0.2700` e `wheel_radius = 0.0800`.** Se
não imprimir, **pare** — a geometria mudou sem querer e nenhum número da sessão
vale. É o primeiro comando da sessão, antes de qualquer coisa que ande.

E o de sempre — exatamente UMA pilha de localização:

```bash
pgrep -a -f "[f]astlio_mapping"        # tem de listar UM processo
pgrep -a -f "[l]ivox_ros_driver2_node" # idem
```

⚠️ **Use `pgrep -a`, que MOSTRA quem casou, e não `pgrep -c`.** Em 05-08 o `-c`
acusou 2 pilhas e era falso: a própria linha de comando do ssh continha a
palavra e o bash casou consigo mesmo. O truque do colchete protege contra o
`pgrep`, não contra o resto do comando.

---

## 4. O protocolo — corrida a corrida

**Robô LIGADO. Espaço: 1,2 m de raio livre EM TODAS AS DIREÇÕES.** Se um ganho
sair ruim o robô volta a arcar, e ele varre um disco, não um corredor.

**Ponto 0 marcado com fita — e o RUMO também.** Todas as corridas de um valor
saem do mesmo ponto e do mesmo rumo; senão a dispersão medida é a mão de quem
posiciona, não a máquina.

### Terminal 1 — a base, fica viva a sessão inteira

```bash
ros2 launch robot_base base.launch.py
```

### Terminal 2 — o compensador, REINICIADO a cada valor

```bash
ros2 run robot_motion compensador_rumo --ros-args -p ki:=0.50
```

🛑 **`ros2 param set` NÃO funciona neste nó.** Ele copia os parâmetros para um
dicionário no construtor e não tem callback — o `param get` responde o valor
novo e o nó segue usando o antigo. Em 05-08 isso invalidou uma varredura
inteira do simulador. **Matar o nó e subir de novo, a cada valor.** E conferir
na primeira linha do log:

```
compensador de rumo (decisão 011): ff -0.817 1/m frente, ...; kp=1.00 ki=0.50
```

### Terminal 3 — as corridas, três por valor

```bash
cd ~/Controle_robo_livox
python3 tools/banco/ensaio.py --ensaio reta --v 0.25 --wz 0 \
    --dur 12 --espaco 1.2 --janela 0.5 \
    --topico /compensador_rumo/cmd_vel \
    --csv docs/dados/AAAA-MM-DD-sintonia-rumo/ki0.50-frente-a.csv
```

Repetir com `-b` e `-c`, reposicionando no ponto 0 entre elas.

⚠️ **`--janela 0.5` sempre no robô.** O padrão de 0,2 s sobre pose a 10 Hz pega
duas amostras e inventa ruído — foi o que gerou a falsa acusação contra o LIO em
31-07.

### A varredura

| ordem | `ki` | por quê |
|---|---|---|
| 1 | **0,50** | a linha de base — repete o estado de 05-08 no piso e bateria de HOJE |
| 2 | **0,25** | metade |
| 3 | **0,10** | um quinto |
| 4 | **0,00** | sem integrador: só feedforward + proporcional |

**12 corridas.** Se a bateria apertar, corte pela ordem inversa (0,25 é o menos
informativo) — mas **nunca corte a de 0,50**: sem ela não há linha de base do
dia, e a comparação com 05-08 fica sem controle de piso e bateria.

### E o `kp`, se sobrar sessão

Só depois que o `ki` fechar, e **só se o `ki` não tiver resolvido**. No
simulador `kp` menor piorou; no robô é outra planta, então vale medir em vez de
supor. Um valor, três corridas: `-p kp:=0.5 -p ki:=<o melhor do dia>`.

---

## 5. Os critérios — o que é sucesso, e o que falsifica

### Sucesso

1. **`|curvatura| < 0,05` em TODA corrida**, não só na média (hoje 3 de 6 falham);
2. **`invs` caindo** de 2 para 0 ou 1;
3. **o seu olho**: *sumiu o S?* — é a mesma testemunha que o encontrou, e ela
   desempatou duas vezes neste projeto quando o instrumento mentiu.

### O que falsifica — e é o resultado mais provável depois de hoje

⚠️ **Se `invs` NÃO mudar em nenhum valor de `ki`, exatamente como no
simulador**, então o S **não é do ganho**. Isso não é fracasso da sessão: é a
resposta, e ela aponta para a **planta** — a boba (BO-4), que é a única peça que
troca de papel entre frente e ré e a única que o simulador não sabe reproduzir.

Nesse caso o caminho deixa de ser sintonia e passa a ser mecânico/estrutural, e
o que a sessão tem de trazer é:

- **o vídeo da traseira** durante uma corrida de frente, com o S acontecendo.
  É o critério (b) do BO-4, aberto desde 28-07, e **já foi filmado em 04-08 e
  não trazido para o repo** — é o item mais barato que falta no projeto;
- **empurrar o robô com a mão, desligado**, e ver se a boba oscila sozinha.

### Uma coisa que NÃO se conclui desta sessão

Se um `ki` sair melhor, ele vale **para o piso e a bateria daquele dia**. Em
05-08 a linha de base repetida no fim da tarde deu −0,8652 contra −0,9116 da
manhã — a planta pode ter enfraquecido ~5% ao longo da sessão, e 5% aqui vale
0,046 1/m, que é **do mesmo tamanho do resíduo inteiro depois de compensar**.
Por isso a corrida de `ki=0,50` no começo não é burocracia: é o controle.

---

## 6. Como o dado volta

Tudo cai em `docs/dados/AAAA-MM-DD-sintonia-rumo/`.

```bash
scp 'bara@<ip>:~/Controle_robo_livox/docs/dados/AAAA-MM-DD-sintonia-rumo/*.csv' \
    docs/dados/AAAA-MM-DD-sintonia-rumo/
git add docs/dados/ && git commit -m "sintonia de rumo: dados crus" && git push
```

**Salve durante a sessão, não no fim** — bateria e rede caem.

### 🔴 `ambiente.txt`, e desta vez é obrigatório

**PISO e BATERIA ficaram `NÃO INFORMADO` em 04-08 e em 05-08.** Nesta sessão
eles deixam de ser burocracia: a bateria é candidata direta a explicar variação
entre valores de `ki`, e sem ela uma varredura de 12 corridas ao longo de uma
tarde **não é comparável consigo mesma**.

```
piso    : <sala, tipo de superfície>
bateria : <tensão no início e no fim, ou pelo menos "cheia"/"caindo">
commit  : <o que o --checar imprimiu>
```

---

## 7. Se sobrar robô na sessão

Nesta ordem, e só se a bateria permitir:

1. **pivô `liga 0,10` e `liga 0,30`, n=3 cada** — a previsão falsificável de
   05-08: se a quantização do laço de 10 Hz estiver certa, `liga 0,10` tem de
   sair **bimodal** (ora ~0°, ora ~16°). Hoje há uma amostra só de cada, e são
   exatamente os dois pontos onde o simulador mais discorda;
2. **teste C, item 1** (homem-morto) — precisa do `./setup_twist_mux.sh` rodado
   uma vez no NUC. Os itens 2 e 3 já estão provados sem robô;
3. **teste D** (reflexo de colisão) — **caixa de 50 cm**, não 40: o Mid-360 foi
   medido a 42 cm e a 0,5 m ele só vê acima de 36 cm.

Detalhes dos três em `docs/PLANO_TESTE_ROBO.md`.
