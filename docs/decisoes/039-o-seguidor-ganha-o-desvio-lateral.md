# 039 — O seguidor ganha o desvio lateral

**Data**: 2026-08-14 (dev + Gazebo, corrida com o dono mandando o goal pelo web)
**Status**: implementada e testada; **medida** só contra o bag da corrida de
hoje. Nasce ligada no simulador (`k_lat: 1.0`) e tem de ir **neutra** (`0.0`)
para a primeira corrida com o robô ligado.
**Toca**: `lei_de_seguimento.py`, `path_follower.py`, `pilha.launch.py`,
`test_lei_de_seguimento.py`
**Vem de**: o dono, olhando o Gazebo: *"ele vai de cara na porta e tem que dar
ré pra atravessar, está praticamente idêntico ao comportamento que vi no real"*

---

## 1. O que a corrida mediu

Corrida gravada em `docs/dados/2026-08-14-porta-gazebo/corrida_a` (bag, 167 s,
22 tópicos). O dono mandou o goal pelo web; ninguém pilotou.

```
                            plano deixa p/ o corpo   robô deixa   o seguidor come
14-08, Gazebo (hoje)              0,175 m              0,065 m       0,110 m
13-08, robô real                  0,118 m              0,036 m       0,082 m
```

No gargalo da porta o vão é de **0,880 m** e o corpo tem **0,455 m** de
largura. O plano cruza a 3,8 cm do centro — bom. O robô cruza a **14,8 cm** do
centro, e o que sobra por lado são **6,5 cm**. Daí para o reflexo disparar
contra parede que está no mapa é um passo, e ele deu: `PolygonStop` em
t=33,25 s, e depois três acionamentos da ré de desencalhe (025).

Desvio de trajeto contra o plano vigente, na aproximação:

```
p50  0,053 m     p90  0,088 m     no instante em que o reflexo cortou  0,094 m
```

⚠️ **Como esse número foi medido, porque a primeira tentativa estava errada.**
Medir o desvio contra o plano do instante dá quase zero por construção: o Nav2
replaneja **a partir de onde o robô está**, então o robô está sempre em cima do
próprio plano. Os números acima são contra o **último plano publicado antes de
o reflexo cortar** — o plano que ele deveria estar seguindo. E a pose foi
levada para o frame do mapa com a correção `map←odom` **interpolada no tempo**;
uma transformada única deixava resíduo de 0,234 m, do tamanho do efeito.

## 2. A causa, e por que ela não é a boba

A lei era só de rumo:

```python
rumo_alvo = rumo_para(x, y, carrot)     # path_follower.py:514
```

Mirar um ponto à frente **corta a curva por dentro** — é o erro permanente
conhecido do pure pursuit, e a entrada da porta vem logo depois de uma curva.
O desvio lateral só voltava ao comando de forma implícita e com atraso.

🔴 **E isto aconteceu no Gazebo**, onde (a) a boba é um patim que não participa
da dinâmica (BO-4) e (b) o arco de frente é cancelado por construção — a
`placa_simulada` usa `curvatura_frente = −0,817` e o compensador usa o mesmo
número. **Logo o defeito da porta não é assinatura de sentido de marcha.**

Isso **encerra a proposta de virar o robô de ré** que estava em cima da mesa
hoje de manhã (o dono: *"eu troco a direção do lidar, vc altera os comandos"*).
A ré é 8,3× mais reta em ARCO — isso continua verdade e continua medido — mas
o arco não é o que derruba na porta. Virar o robô levaria o erro lateral junto,
inteiro. **Abortada pelo dono depois desta leitura**, e o registro fica porque
a alternativa foi descartada com dado, não por gosto.

## 3. A lei que entrou

O termo de Stanley somado ao rumo do carrot:

```
rumo_alvo = rumo_carrot − atan2(k · e_lat, max(|v|, v_ref))
```

`e_lat` é o desvio **com sinal** (+ à esquerda), medido contra o **segmento**
mais próximo e não contra o vértice — ponto a ponto o caminho do planner vem a
~5 cm e a distância ao vértice satura em ~2,5 cm mesmo com o robô bem fora.

**O ganho não veio de varredura, veio da dinâmica que ele impõe.** Com a
correção aplicada,

```
ė = −v·sen(atan(k·e/v)) = −k·e / sqrt(1 + (k·e/v)²)  ≈  −k·e
```

ou seja o erro decai com **constante de tempo 1/k segundos, independente da
velocidade**. É essa a razão de o `v` estar no denominador: perto da porta o
robô anda devagar, e um proporcional puro ficaria fraco justo onde precisa ser
forte. Com `k = 1,0` os 11 cm medidos viram ~1 cm em 3 s — 0,90 m de caminho a
0,30 m/s, que cabe folgado na reta de aproximação.

Dois limites, e os dois têm motivo:

- **`v_ref = 0,20 m/s`** é piso do denominador. Em `v → 0` o termo pediria 90°
  e o robô giraria parado em cima do caminho.
- **teto de 30°**: acima disso deixou de ser correção e virou manobra, e
  manobra tem dono neste robô (o pivô da 036, a ré da 025).

## 4. Alternativas descartadas

- **Trocar o carrot por tangente + cross-track (Stanley puro)**: resolve o
  corte de curva na raiz, mas joga fora o carrot, o `lookahead` derivado do
  raio mínimo e a razão inteira da 008. Mudança grande onde cabia um termo.
- **Encurtar o `lookahead`**: o corte de curva é ~L²/(8R), então encurtar
  ajuda — mas `lookahead_piso` existe contra o ciclo-limite de rumo (o S de
  27-07), e mexer nele troca um defeito por outro que já custou sessão.
- **Subir a inflação para o gradiente centrar mais**: mexe no PLANO, que já
  está bom (3,8 cm do centro). Consertar quem não errou.
- **Virar o robô de ré**: seção 2.

## 5. Verificação

11 testes novos em `test_lei_de_seguimento.py` (**294 verdes** no
`robot_motion`, eram **283** — conferido rodando a suíte com o arquivo antigo
em stash). Somando `robot_base` 75, `robot_planning` 12 e o banco 89, dá
**470** no repo. Os que importam:

- `test_a_lei_NASCE_NEUTRA_ganho_zero_e_o_comportamento_de_hoje` — `k_lat=0`
  devolve o rumo do carrot intacto, para qualquer erro. É o que permite a
  mudança ir ao robô desligada;
- `test_erro_a_esquerda_pede_rumo_a_DIREITA` — sinal trocado aqui AFASTA o robô
  do caminho;
- `test_o_decaimento_NAO_depende_da_velocidade` — a propriedade que escolheu o
  ganho;
- `test_com_o_erro_medido_na_porta_o_ganho_1_resolve_em_um_metro` — a régua do
  conserto, com o número de hoje (0,11 m → < 0,015 m em 3 s).

⚠️ **O que NÃO está verificado**: nenhuma corrida rodou com a lei ligada. O
Gazebo não sobe sem o dono olhando, e a corrida de validação é o próximo passo.
A régua já está no lugar: o `path_follower` agora grava `desvio_lateral` no
CSV, então a próxima corrida mede o conserto sem precisar reabrir o bag contra
o mapa.

## 6. O que vai para o robô

`k_lat:=0.0` na primeira corrida com ele ligado — a lei nasce neutra, como o
freio da 038. Só depois de uma corrida limpa com o dono presente é que o ganho
sobe, e o número dele será remedido lá: o simulador não tem a boba, e o robô
real chega na porta com o arco residual que o compensador não cancelou.

## Referências

- Thrun et al., *Stanley: The robot that won the DARPA Grand Challenge* (2006)
  — a lei `atan2(k·e, v)` e o argumento da constante de tempo.
- Coulter, *Implementation of the Pure Pursuit Path Tracking Algorithm* (1992)
  — o erro permanente em curva, que é o defeito medido aqui.
