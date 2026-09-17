# Estado do Projeto — Controle_robo_livox (PIBIT)

> Documento vivo. Resumo do que está acontecendo, BOs abertos, avanços e o que falta.
> Versionado na `main`. Atualizado em **2026-09-17** (dev — robô 3 ganhou o
> `frente:=-1.0` para trocar frente e ré; não testado no robô).
>
> **Este projeto é um PIBIT** — vai virar artigo. Toda decisão técnica tem um
> registro em `docs/decisoes/`, todo dia de trabalho entra no `docs/DIARIO.md`,
> e escolhas de abordagem são embasadas em literatura (`docs/REFERENCIAS.md`).
> Ritmo deliberadamente devagar: 1 mudança pequena por vez.

---

## 📍 17-09 dev — ROBÔ 2: AMCL SÓ CORRIGE COM AS RODAS ANDANDO

- Contra a pose pulando parado: `tf_odom` com `congela_parado`. Decisão 050.
- 🔴 **NÃO TESTADO NO ROBÔ.** Ensaio: parar 1 min com `localizacao:=amcl` e ver
  se a pose fica quieta; depois andar e ver se não dá degrau.
- 🔴 **REVISADA no mesmo dia, e duas coisas desta seção mudaram:**
  - **`recovery_alpha` voltou** a 0,001/0,1. Zerá-los era mudança independente
    da trava; juntas, um ensaio ruim não diz qual das duas foi. (E nenhum teste
    do repo afirmava esses valores.)
  - **A trava NÃO sobe mais sozinha.** Ela era `true` na `base.launch.py`, e
    assim qualquer `reset --hard origin/main` + `sobe-robo` no NUC a implantava
    sem ninguém pedir. Agora é argumento, **padrão `false`**:
    `ros2 launch robot_base base.launch.py congela_parado:=true`.
  - Corrigido também um defeito que **mentia a pose**: o estado era único para
    as duas rodas, então uma roda muda + a outra publicando zero congelava a TF
    com o robô possivelmente andando. Agora é por roda. Diário de 17-09.

---

## 🤖 15-09 noite — ROBÔ 2 DE PÉ NO NAV2 (lidar em `.158`)

- **O lidar do robô 2 agora está em `192.168.1.158`**; o config dizia `.169`,
  por isso não havia `/Odometry` e o Nav2 abortava. Diário de 15-09.
- NUC: IP `10.127.116.205` (Wi-Fi `Trafico de banana`), cabo do lidar em `enp1s0`.
- 🔴 **O NUC não consegue `git fetch`** (sem chave do GitHub). Deploy de hoje:
  `git push ssh://bara@10.127.116.205/home/bara/Controle_robo_livox main:refs/remotes/origin/main`
  e no NUC `git reset --hard origin/main` + `colcon build`.

---

## 🔁 16-09 dev — A RÉ VIRA A FRENTE (contorno, não conserto)

Decisão do dono: parar de caçar a causa e fazer o robô andar. Como a ré já anda
reto, ela passa a ser a frente.

- `bash bin/sobe-robo3` — **já sobe com a antiga ré como frente**, é o padrão
  agora. Giro **igual** para quem dirige. `frente:=1.0` volta a frente antiga.
- Pega tudo o que dirige o robô 3 hoje (analógico e direcional). O nav2 do robô
  3 ainda não existe; quando for adaptado, o launch novo manda
  `linear_sign: -1.0` no `cmd_vel_to_wheels` e herda o mesmo comportamento.
- O robô 2 fica **fora** disto: lá o Livox define a odometria e os calibres de
  frente e ré são separados.

### 🗺️ E o nav2 do robô 3 ganhou plano: `docs/PLANO_NAV2_ROBO3.md`

Decidido em 16-09: **frente = motoras, traseira = bobas** (eletricamente é o
contrário, e a correção fica no código — é o `linear_sign` de hoje); **o Livox
vem emprestado do robô 2**, no centro do robô, no topo. Atuador se baseia no
robô 1 (MEGA), seguidor e navegação no robô 2 (também diferencial de 2 motoras;
o robô 1 é 4x4 e só faz pivô).

- ⚠️ **Enquanto o robô 3 navega, o robô 2 não navega** — só há um Mid-360.
- 🟢 O chute do URDF (`livox_z_solo` 0,24, centro da caixa) **já casa** com a
  montagem decidida: topo da caixa 0,205 + meio cilindro = 0,2375 m.
- 🔴 O `scan_2d.yaml` **não se herda**: com o sensor a 0,24 m (era 0,42 no robô
  2) o chão entra no campo a 1,95 m em vez de 3,4 m.
🔴 **17-09: o plano virou v2 depois de uma revisão cruzada que derrubou duas
etapas inteiras.** O que a v1 chamava de "etapa 3" (subir a pilha do robô 2
sobre o simulador do robô 3) e de "etapa 5" (localização) **não funcionam como
estavam escritas** — conferido no código, não aceito de boca:

- `pilha.launch.py:556` inclui o simulador do **robô 2**; e o `cmd_vel_to_wheels`
  publica `WheelSpeeds`, que não tem consumidor no Gazebo.
- Nem o controle do robô 3 nem a `localizacao.launch.py` sobem
  `robot_state_publisher` → sem `base_link → livox_frame` o `tf_odom` **recusa**
  publicar `odom → base_link`. Falta um bringup do robô 3.
- O giro de 180° **não** é trocar 3 sinais de x: o trail da boba é fixo em −x.
- A cadeia do robô 2 é toda `TwistStamped` e a do robô 3 toda `Twist` — são dois
  contratos, e dois launches disputam o mesmo mux.

**Nova ordem (v2): etapa 0** ✅ **FEITA em 17-09** — `test_scan_2d.py` estava
vermelho procurando um `robot_radius` que saiu do `nav2.yaml` na decisão 032.
O teste era inválido em dois sentidos (a chave não existe mais, e a
desigualdade era a inversa da certa), foi **removido** com o porquê registrado
no lugar dele, e a suíte do arquivo está **7/7 verde**.

🔴 **E como rodar a suíte, porque isto custa sessão:** `pytest` **da raiz, sem
argumento** (com o ROS carregado) → **845 passed**. Passar `ros2_packages/` como
alvo dá **"no tests collected"** e aborta no primeiro import, com um
`ImportError` de `rclpy.qos` que **não é defeito nenhum** — é `sys.path`. O
mesmo arquivo passa sozinho.

⚠️ Por causa disso, a nota da outra sessão de 17-09 ("a suíte tem as mesmas 2
falhas") está desatualizada: uma era o `test_scan_2d`, consertado agora; a
outra, `test_plano_suavizado`, **não está vermelha** — 10/10 quando invocada
direito.

### ✅ ETAPA 1 FECHADA em 17-09 — geometria medida e placa respondida

- **Placa:** *"mesmo modelo do 1 e do 2, mas não a mesma, cada um tem a sua."*
  → o modelo de atuador **não cai**. ⚠️ **Mas "mesmo modelo" não prova "mesmo
  firmware"**, e a configuração gravada nesta placa é desconhecida: a **etapa 8
  continua sendo medição real do atuador**, não conferência. Os números do
  `MODELO_ROBO2.md` entram como ponto de partida.
- **Quatro valores do URDF mudaram** pela trena: `roda_separacao` 0,3225→**0,320**,
  `roda_largura` 0,058→**0,060**, `boba_raio` 0,025→**0,020**, `altura_solo`
  0,070→**0,065**. Caixa 0,311 × 0,240 × 0,135 confirmada.
- **D3 e D4 encerradas.** A D4 nunca foi caimento: 20,0 − 13,5 = 6,5, e o robô
  está alinhado. O modelo carregava um "suporte de 10 mm" inventado para
  explicar um número errado.
- **Bitola: três caminhos apontam o mesmo lugar** (trena 32,0, Gazebo 32,25, "o
  corpo encosta nas rodas"). ⚠️ **Nominal, não validada** — ela sai da *média*
  de duas leituras "±", onde viés comum **soma** (na largura, que sai da
  *diferença*, cancela). Fecha por **ensaio de pivô** na etapa 8.
- Testes: **104 passed** nos do robô 3; suíte em 847, sem regressão.
- **Ainda aberto:** `roda_raio` (fecha por corrida reta, etapa 8), Livox (etapa
  7) e massas (só depois de Livox e NUC subirem).

### ⏸️ 17-09 — ETAPA 2 ADIADA, a semana é de GAZEBO

**Decisão do dono:** nesta semana **não há ensaio nem implantação em robô
físico**. A etapa 2 fica **adiada, não concluída**.

Estado dos pré-requisitos dela, para quando voltar:

- ✅ placa decidida;
- ✅ protocolo escrito (`docs/PROTOCOLO_ETAPA2_FRENTE_RE.md`);
- 🔴 **parada física independente do Xbox** — hardware, continua faltando;
- 🔴 **o discriminador de canal está QUEBRADO** — achado de 17-09, ver acima.
  Sem ele o ensaio mede curvatura por sentido mas não separa lado de sentido.

**A trilha de Gazebo está aberta** (§8-B do plano). Primeiro passo já feito:
`colcon build --packages-select robot_base` — o `install/` estava **velho** nos
três arquivos do robô 3, e validar sem recompilar seria validar o código de
ontem. Agora está em dia, conferido com `cmp`.

🔴 **O que a trilha NÃO pode declarar validado:** bitola, massas, curvatura e
dinâmica do atuador. O Gazebo prova que a **cadeia de software** funciona; não
prova como o robô se comporta — e usa os números que nós demos, então concordar
consigo mesmo não é evidência.

⚠️ **O smoke test do Gazebo espera você**: a regra da casa é que ele só sobe com
o dono olhando. O build foi seguro porque não simula nada.

O protocolo resolve a medição sem pose — o robô 3 **terá** o Mid-360, ele só
ainda **não está montado** (e a pose dele no URDF é provisória). Até lá: desvio
medido **no chão** contra uma fita, virando curvatura por `κ ≈ 2d/L²`.

🔴 **Mas o discriminador do protocolo está QUEBRADO e não pode ser rodado como
está** (achado de 17-09): eu escrevi que inverter `left_wheel_sign` e
`right_wheel_sign` juntos troca os canais L/R. **Não troca** — aquilo é espelho
(inverte frente e giro), e é o que o `cmd_vel_to_wheels.py` diz em texto escrito
no mesmo dia. E **um parâmetro novo também não resolveria** — não é falta de
código, é aritmética: o `mega_bridge` manda `speed=(L+R)/2` e `steer=(L−R)/2`, e
na reta `L == R`, então trocar L↔R entrega **o frame idêntico**. Swap de
software na reta é no-op.

Trocar canal de verdade é **físico**, nos conectores dos motores — e ⚠️ **isso já
foi feito uma vez, em 14-09**. O RPM por roda daquele dia
(`docs/dados/2026-09-14-robo3-gnd-xbox-reta/`) mostra no ar **−0,1 %** e no chão
**frente +9,8 %, ré −1,8 %**, já com os cabos trocados.

🔴 **Mas essa comparação NÃO é controlada, e eu li errado:** no ar o comando foi
`speed ±250` e no chão `±120` (dpad 0,30 × escala 400). Com zona morta e resposta
não linear, a magnitude sozinha pode explicar o salto. O certo: **compatível com
carga/contato, confundido por comandos diferentes** — separar exige repetir no ar
e no chão com o mesmo setpoint, tensão e configuração.

⚠️ E há um terceiro dado (`docs/dados/2026-09-15-robo3-pivo-parcial/`): no pivô,
nos dois sentidos **a roda que gira para trás é a mais rápida** — NÃO CONCLUSIVO,
mas aponta para sentido de rotação, não para canal.

### 📏 A lista que originou isto: `docs/ETAPA1_MEDIDAS_ROBO3.md` (robô DESLIGADO)

Lista pronta para você passar a trena e anotar — cada medida diz **o que o URDF
carrega hoje** (valor renderizado, não fórmula) e o que ela fecha. Abre com a
única pergunta que não é de trena: **a placa é a mesma ou é outra?** (§5.1 — se
mudar, a etapa 8 recomeça do zero).

🔴 **Achado ao montar a lista:** a tabela-resumo §5.8 da revisão cruzada estava
**velha** — dizia `roda_separacao` **0,425** (decisão da §5.7), mas a **§5.9
revisou para 0,3225** ao fechar a D5 no Gazebo. O **código sempre esteve certo**;
era a tabela que discordava. Grave porque é dela que a etapa 3 copia o contorno
para o `nav2.yaml`: sairia um footprint **9 cm mais largo por lado**, e footprint
inflado não dá erro — só faz o planejador recusar vão por onde o robô passa.
Corrigido, e conferido que ninguém tinha copiado os números velhos para código. **Etapa 1** = fechar a placa e a
geometria por trena. **Etapa 2** = o ensaio frente/ré, agora com protocolo
objetivo (o bag de hoje **não grava pose**, porque o Mid-360 ainda não está
montado no robô 3; até lá não há desvio lateral sem régua). **O Livox só sobe na etapa 7** — até lá o robô 2 continua navegando.
- ⚠️ **Não é o `sinal:=-1.0`** que já existia: aquele é espelho (inverte o giro
  junto, reprovado em 14-09). `frente` é rotação. Decisão 049.
- 🔴 **NÃO TESTADO NO ROBÔ.** Falta subir e rodar LB + direcional cima/baixo.
- ⚠️ Se o puxão só trocar de lado (passar a puxar para a esquerda), o contorno
  não resolve. 🔴 **Mas isso NÃO prova sozinho que a causa é de canal** —
  assimetria de roda, de carga e o transiente das bobas dão o mesmo sinal.
  Quem decide é o ensaio com protocolo do §7 do `PLANO_NAV2_ROBO3.md`, não a
  observação a olho. (Corrigido em 17-09: a redação anterior afirmava a prova.)

O roteiro de pivô limpo abaixo fica **em pausa**, não descartado: a pergunta
segue aberta e vale para o artigo.

---

## 🧩 EM PAUSA (era 16-09) COM O ROBÔ 3 — PIVÔ LIMPO + RETA, NA MESMA BATERIA

**Pergunta (ADIADA pelo contorno acima, não respondida):** por que a frente puxa
para a direita e a ré vai reta (robôs 2 e 3)?

**O que 15-09 já mostrou (parcial, `docs/dados/2026-09-15-robo3-pivo-parcial`):**
no pivô pelo Xbox, **nos dois lados a roda que roda para trás girou mais**
(esq.: L trás 126 × R frente 79 rpm; dir.: R trás 122 × L frente 91). Se
confirmar, **derruba a hipótese elétrica de 15-09** (no pivô as duas rodas estão
no mesmo sentido elétrico e mesmo assim diferem) e aponta um efeito **do sentido
físico de cada roda**, comum às duas. Não conclusivo: pivô esquerdo com
`speed −18` (analógico escapou) e o direito só 1,4 s — o notebook caiu e o USB
da MEGA desconectou (voltou como `/dev/ttyACM1`).

### Passo a passo

**0. Antes de ligar nada** — notebook **fixo no robô**, USB da MEGA firme,
placa **desligada**, robô no chão com ~1 m livre. Os dois PCs na "Trafico de
banana".

**1. Pré-voo (eu, por ssh)** — sincronizo o repo no notebook; confiro porta da
MEGA (`ls /dev/ttyACM*`, serial `55632313039351D05132`) e que não sobrou pilha.
MEGA precisa estar com o `mega_bridge` (está, desde 15-09 17:0x).

**2. Subir a pilha (placa desligada)**
```bash
cd ~/Workspace/Controle_robo_livox
bash bin/sobe-robo3                      # se a MEGA estiver em ACM1: porta:=/dev/ttyACM1
```
Esperar `controle (/joy): 🟢`.

**3. Gravador (eu, por ssh)** — `python3 tools/grava_pivo.py ~/bancada_robo3/pivo_chao_<hora>.csv 420`
(rpm, `/mega/debug`, `cmd_vel`, bateria). Eu aviso a hora de fim.

**4. Ligar a placa.** Mão no botão.

**5. Pivô limpo** — **LB + RB**, analógico **só para o lado, até o fim**:
esquerda **5 s**, solta, direita **5 s**, solta — **3 vezes**. Olhar se o robô
**anda de ré enquanto gira**.

**6. Reta na mesma bateria** — **LB + direcional**: cima **5 s**, solta, baixo
**5 s**, solta — **3 vezes**.

**7. "pronto"** — eu leio. Placa pode ficar ligada; **não derrubo a pilha sem
"pode"**.

### Como eu leio

Só trechos com `/mega/debug` = `speed 0, |steer| 484` (pivô) ou `steer 0` (reta),
regime a partir de 1 s.

| resultado do pivô | leitura | próximo passo |
|---|---|---|
| nos dois lados a roda **para trás** gira mais | efeito do **sentido físico** da roda, comum às duas (bobas, carga, apoio) | teste físico: bobas / peso — definido na hora |
| a mesma roda (L) gira mais nos dois lados | diferença de **canal** (`x`) | trocar os cabos de volta e repetir |
| os dois lados de pivô diferem na média | sentido **elétrico** (`b`) | reabre a hipótese de 15-09 |
| tudo igual | nenhuma das três; a reta volta a ser a pista | — |

**Placas: mesmo modelo, unidades diferentes** nos robôs 2 e 3.

**Firmware da placa:** o dono achou que "quem gravou mexeu", e depois descobriu
que **a mudança foi no ROS do hover, não na placa** — não chega ao robô 3. A
configuração gravada segue **desconhecida**, compatível com o EFeru padrão
(`VLT_MODE`, `STEER 0,5`, `pwmr = -cmdR`) usado na conta de 15-09, não
confirmada.

~~Regravar as placas com o EFeru~~ — **descartado pelo dono em 15-09**.

**Plano B (dono):** se a causa não tiver conserto, inverter frente/ré **só na
reta**, no código, para o robô andar sempre no sentido elétrico da ré (o que vai
reto). Não implementado: primeiro o pivô.

---

## 🎮 14-09 ~21:00 — ROBÔ 3 DIRIGE NO XBOX (decisão 048)

### Como subir o controle manual sozinho (no notebook do robô 3)

```bash
cd ~/Workspace/Controle_robo_livox
bash bin/sobe-robo3          # com a PLACA DESLIGADA
```

1. Xbox ligado e conectado (botão Xbox, luz fixa).
2. Esperar as conferências: `controle (/joy): 🟢`. Placa e bateria ficam 🔴
   enquanto a placa está desligada — é esperado.
3. **Ligar a placa.** Rodas no ar no primeiro teste do dia.
4. Dirigir:
   - **LB segurado** + analógico esquerdo = anda e gira (soltou o LB, para)
   - **RB** = turbo
   - **LB + direcional cima/baixo** = reta pura / ré pura
5. Para derrubar: `bash bin/sobe-robo3 --mata`

Tudo fica gravado em `~/bancada_robo3/controle_<data>/`.

**Se não andar:**

| sintoma | o que fazer |
|---|---|
| `controle (/joy): 🔴` ou "NENHUM Xbox" | ligar/reconectar o Xbox e subir de novo |
| `placa responde: 🔴 /battery/front mudo` | é a MEGA, não a placa: `--mata`, **replugar o USB da MEGA**, subir de novo |
| beep da placa continua e nada anda | religar a placa com a pilha já no ar; conferir o fio do GND |

| o quê | estado |
|---|---|
| Xbox → placa pelo `mega_bridge` | ✅ anda; giro com força e direção boas (dono) |
| `sinal` padrão 1.0 | ✅ frente = a das rodas |
| giro 4,0 / 7,5 | ✅ de partida, rodas no ar |
| direcional reto | 🟡 frente puxa à direita: **canal L +9,8 % de rpm com `speed>0` no chão** (3/3 corridas), ré −1,8 %; no ar igual → só com carga (`reta_chao_211439.csv`) |
| cabos das rodas trocados na placa | ⚠️ frente = lado das bobas; giro continua certo |
| suspensão do notebook | ✅ desligada (derrubava rede, Xbox e o USB da MEGA) |
| pull-up no 19 com GND bom | ❓ não testado |

---

## ✅ 14-09 ~20:25 (BANCADA, PC dev) — ROBÔ 3: ERA O GND

O **fio do GND estava com defeito**; o dono trocou só o fio (mesmos pontos). `teclado_20260914_202729.csv`:

| o quê | estado |
|---|---|
| volta da placa (azul → 19) | ✅ 99,6 quadros válidos/s, 0 ruins |
| bateria das rodas | ✅ 40,8 V pelo retorno |
| comando pelo teclado (`hover_ponte`) | ✅ na hora e constante; `cmd1,cmd2` da placa = mandado |
| giro sozinho | ✅ nenhum em 73 s (só desaceleração ≤ 1,5 s) |
| pull-up no 19 ainda necessário? | ❓ não testado com o GND bom |
| ROS/Xbox pelo `mega_bridge` (decisão 046) | ⬜ próximo |

A tabela abaixo (noite, antes do GND) fica como histórico: tudo nela era o GND.

---

## 🔌 14-09 noite (BANCADA, PC dev) — ROBÔ 3: O AZUL NÃO TRAZ NADA, E A PLACA GIRA SOZINHA

MEGA `55632313039351D05132` no PC dev, com `hover_ponte`. Diário de 14-09
(noite) tem a sequência e os CSVs.

| o quê | estado |
|---|---|
| comando pelo verde, lido como serial | ✅ checksum invertido = roda imóvel |
| teclado (`tools/teclado_placa.py`) | 🟡 anda, com atraso e inconstante |
| **giro sozinho com o PC em zero** | 🔴 3 marcas `m` em 176 s; da placa |
| volta da placa pelo azul (19) | 🔴 só ruído (`0xBF`/`0xFF`) + vazamento do verde; 0 quadros. Em 01-09 chegou |
| pull-up no 19 | ⚠️ muda o beep / arma (3×), mecanismo não medido |
| `hover_probe` + pull-up | ✅ `7baa3fb` no `Controle_robo_web_hover` (`.hex` junto) |
| teclado grava a volta (`placa_*`, `.rx.bin`, tecla `m`) | ✅ `61d8880`, `e3cb011` |

> ⚠️ **As rodas podem girar sozinhas quando a placa arma**, mesmo com zero no
> serial. Rodas no ar e mão no botão antes de qualquer corrida.
>
> Próximo: foto de em qual cabo da placa o fio está hoje × print do vídeo do dia 1.

---

## 🔧 14-09 (BANCADA) — ROBÔ 3: A PLACA ARMA PELO `mega_bridge`, MAS RODA E PARA (decisão 047)

Notebook no `144b739`, MEGA com `mega_bridge` + pull-up no 19 (conferido por
verify do avrdude). Diário de 14-09 (bancada) tem a sequência inteira.

| o quê | estado |
|---|---|
| `twist_mux` (symbol lookup) | ✅ `diagnostic-updater` 4.2.7 via apt |
| `/joy` | ✅ `device_id: 0` (SDL), não o N do `jsN` |
| MEGA → placa: bytes, ritmo, aceitos | ✅ `/mega/debug`: 20/20 cmd/s, 50 Hz, 0 checksum errado |
| arme pelo `mega_bridge` | ✅ com `pinMode(19, INPUT_PULLUP)`; ritual: **ligar a placa segurando comando** (`S` no teclado) |
| roda ~1 s e para com comando contínuo | 🔴 pelos dois firmwares; é da placa |
| retorno da placa (bateria, rpm) | 🔴 nunca chegou |
| ROS no controle com o pull-up | ⬜ não testado |
| `teclado_placa.py --mega` | ✅ mesmo ritual pelo `mega_bridge`, grava `/mega/debug` e erros no CSV |

> ⚠️ Antes de chamar o dono para uma corrida: conferir o firmware da MEGA com
> `avrdude -U flash:v:<hex>:i`. Uma corrida de 14-09 foi perdida com a
> `hover_ponte` gravada sem ninguém saber.

---

## 🎮 14-09 (dev) — ROBÔ 3 NO CONTROLE XBOX, PELA MEGA (decisão 046)

Pronto para testar, provado só sem hardware. Roteiro completo em
`docs/PLANO_CONTROLE_ROBO3.md`. No notebook do robô 3:

```bash
git fetch && git reset --hard origin/main      # (ou o remoto do notebook)
colcon build --packages-select robot_nav
pio run -d firmware/mega_bridge -t upload --upload-port /dev/ttyACM0   # placa DESLIGADA
bash bin/sobe-robo3                             # RODAS SUSPENSAS, placa ligada
bash bin/sobe-robo3 --mata
```

| o quê | estado |
|---|---|
| cadeia joy → mux → cmd_vel_to_wheels → mega_bridge | ✅ provada com MEGA fingida (frente −120, esquerda steer 97, LB solto 0) |
| MEGA com `firmware/mega_bridge` (50 Hz fixos, lê a volta da placa) | ⬜ gravar no lab (substitui a `hover_ponte`) |
| sinal −1 (10-09: speed>0 = ré) | ❓ confirmar suspenso; `sinal:=1.0` se trocado |
| giro 1,5 rad/s acima da zona morta | ❓ se o pivô não sair, subir e anotar |
| bag por sessão em `~/bancada_robo3/controle_<data>/` | ✅ |

Números **de partida**. Movimentação só depois do Livox.

---

## 🔌 10-09 (BANCADA) — ROBÔ 3: A MEGA GIRA AS RODAS NO NOTEBOOK NOVO

Notebook `ubuntu@10.127.116.150` (Latitude 3490, 24.04 + Jazzy) com o repo em
`~/Workspace/Controle_robo_livox`, compilado. Deploy: `git push` para o bare
`~/Workspace/Controle_robo_livox.git` e `fetch/reset` do remoto `notebook` (a
chave de GitHub de lá é de outra conta). O `.bashrc` e a udev são do robô 1 e
**não foram tocados**: carregar à mão `source install/setup.bash`.

| o quê | estado |
|---|---|
| MEGA com `firmware/hover_ponte` (USB↔Serial1, 115200) | ✅ gravada, loopback 256/256 |
| MEGA → placa (18→verde, GND) | ✅ **girou** as rodas |
| arme da placa | ⚠️ comando SEM buraco + religar/girar até o beep mudar; silêncio trava |
| volta da placa (azul → 19) | ❓ nunca lida; PL2303 tem RX com defeito |
| `setup_livox.sh` em máquina limpa | 🔴 falha (compila `robot_base` sem `hoverboard_driver`) |
| driver ROS pela MEGA | ⬜ falta `device:=/dev/ttyACM0` e a geometria do robô 3 |
| teclado de demonstração (`tools/teclado_placa.py`, v1) | 🔴 **não confiável**: às vezes perfeito, às vezes atrasa ou não anda, sem nada mudar |

> 🔴 **Veredito do dono em 10-09: "um lixo" o que estamos testando.** O PC manda
> certo (20 ms cravados em todos os CSVs); a placa responde de forma
> intermitente e **nenhum frame de resposta dela chegou**. Próximo passo
> obrigatório: fazer a volta da placa chegar (bateria, cmd aceito, rpm) antes de
> mexer em qualquer outra coisa. Cinco versões de teclado sem esse instrumento
> foram chute.

Detalhe, as cinco versões e o custo do diagnóstico: `docs/DIARIO.md`, 2026-09-10.

---

## 🔴 20-08 (3ª leva, dev) — A PORTA 2 TEM 70 cm E O ROBÔ 55,5 cm

> **Leia isto antes de qualquer sintonia.** Enquanto esta conta estiver de pé,
> nenhum ajuste de seguidor resolve a porta de forma repetível.

```
vao da porta 2 (medido no mapa, y=19,0)      0,700 m
footprint do nav2.yaml (decisao 032)         0,555 larg x 0,617 compr

largura que o robo ocupa entrando torto por θ:  0,555·cos θ + 0,617·sin θ

   θ =  0°   folga +7,2 cm/lado      θ = 12,4°  +1,3 cm  <- LIMITE
   θ =  5°         +4,7 cm/lado      θ = 13,9°  +0,7 cm
   θ = 10°         +2,3 cm/lado      θ = 23,6°  -2,8 cm  NAO CABE
```

> 🔴 **CORREÇÃO (mesma sessão, 3ª leva)**: a conta acima usou o `footprint` do
> `nav2.yaml` (0,555 × 0,6165), que é o **corpo + ~5 cm de margem por lado**.
> O corpo REAL é **0,455 × 0,433** (decisão 032, medido). Refazendo:
>
> ```
>  θ      corpo real   folga/lado      footprint do costmap   folga/lado
>   0°     0,455 m      +12,2 cm        0,555 m                +7,2 cm
>  15°     0,552 m       +7,4 cm        0,696 m                +0,2 cm
>  20°     0,576 m       +6,2 cm        0,732 m                NÃO CABE
>  43,6°   0,628 m       +3,6 cm  <- projecao MAXIMA do corpo
> ```
>
> ➡️ **O CORPO CABE NO VÃO EM QUALQUER ÂNGULO** (máximo 0,628 m contra 0,700).
> O que não cabe acima de ~15° é o footprint do costmap, e a caixa do reflexo é
> maior ainda. **O dono estava certo: é navegação.** A porta é fisicamente
> passável sempre; quem veta é a camada de segurança.
>
> O que NÃO muda: a descentragem medida. Com `x = 7,20` e o vão indo até 7,29,
> a borda direita do **corpo real** fica em 7,43 — **invade 14 cm**. Centrar
> continua sendo o item nº 1.

**O que o robô fez em 20-08**: travessia 1 com erro de rumo p50 **23,6°** (não
cabia) e travessia 2 com p50 **13,9°** (0,7 cm por lado). Nenhuma passou. É a
explicação do *"passou 1 vez e depois nunca mais"*: com 1 cm de folga por lado,
passar é sorte.

⚠️ **O vão de 0,70 m está nos DOIS mapas** (`andar3todo` e o `alterado`) — não
é parede desenhada na edição. **Falta medir a porta física com trena**: se ela
tiver 0,80–0,90 m, o mapa está engordando a parede e o problema vira
mapeamento, não controle.

### 🟢 E o corredor do andar 3 agora roda no Gazebo

```
ros2 launch robot_motion pilha.launch.py sim:=true gui:=false rviz:=false \
  bag:=false \
  mundo:=$PWD/worlds/andar3todoalterado.sdf \
  mapa:=$PWD/maps/andar3todoalterado/andar3todoalterado.yaml \
  pose_x:=6.14 pose_y:=10.0 pose_yaw:=1.5708
```

Objetivo em `(6,80 · 20,30)`, do outro lado da porta 2. Primeira corrida: subiu
8,7 m do corredor, **não passou a porta**, 7 rés, erro de rumo na aproximação
**p50 14,6°** — contra 13,9° e 23,6° do robô. O defeito aparece fora do robô.

⚠️ **É por isto que o Gazebo "não era eficaz"**: as provas rodavam no
`pista_obstaculos`, de portas **0,90 m**, onde cabe até 32° de erro. Estava
testando um problema três vezes mais folgado. Isso também recontextualiza o
teste que desligou o gargalo em 20-08 — foi reprovado num cenário em que
alinhar não era necessário, e não decide nada sobre a porta de 70 cm.

### 🔎 A cadeia de comando (044) respondeu na estreia

Dos 147 s parados com objetivo vivo nessa corrida:

```
movimentacao_muda[STOP:PolygonStop]   45,6 s
collision                             34,4 s
```

**80 s com o reflexo em STOP dentro da porta.** O handoff registrou o reflexo
como "inocentado" a partir de 4 disparos vistos ao vivo; com registro contínuo
ele é o ator principal da travada.

### As saídas, para decidir juntos

1. **trena na porta real** — decide se é controle ou mapeamento (o mais barato);
2. **alinhar antes de entrar** — o modo `passagem` já existe no código, com
   `passagem_alinha_rumo_deg: 10,0` (o número certo pela conta acima) e hoje
   **desligado** (`passagem_estreita_habilitada: False`);
3. **desviar dessa porta**, se houver outra rota.

⚠️ A gangorra "porta × S" (a mira é um knob só para duas tarefas opostas, e
fica curta **87% do tempo**) continua de pé, mas vem DEPOIS: com 12° de
exigência, sintonia nenhuma entrega repetibilidade.

---

## ⚠️ 20-08 (2ª leva, dev) — CORREÇÃO DO HANDOFF ABAIXO: DUAS AFIRMAÇÕES CAÍRAM

> O handoff da seção seguinte continua valendo quase inteiro, mas **duas
> afirmações dele estão erradas e mandariam a próxima sessão para o lugar
> errado**. Refiz a conta sobre o mesmo arquivo
> (`seguidor_2026-08-20_183507.csv`). Registro completo no `docs/DIARIO.md`
> (2ª leva de 20-08) e na decisão **044**.

**1. ❌ "ele bateu porque chegou a 0,55 m/s, acima do `v_max`" — NÃO HOUVE.**
Em `t=854,4` o robô estava em **RÉ** (`estado=re`, `v_alvo=−0,20`), recuando a
0,36 m/s; os 0,55 vieram de tomar módulo na virada ré→frente. E o `v_alvo` não
pode passar de 0,50 por construção (`velocidade_de_seguimento` devolve um
`min()` com `v_max`). Com `v_alvo` cravado em 0,50, a velocidade real da
corrida inteira dá **p50 0,26 · p90 0,32 m/s** (n=2247): **a máquina nunca
entrega o `v_max`**. A saída proposta no handoff — "não deixar o robô chegar
rápido na porta" — ataca um problema que não existe.

**2. ❌ "a pose saltou a 1,5 m/s" — é o robô sendo CARREGADO.** Dois trechos,
`t=193,7` e `t=859,4`, de 15 s cada, cobrindo 19 m contínuos da porta 2 até a
origem a 1,3 m/s. Trajetória contínua e coerente = passo de gente andando. Salto
de pose é descontínuo; isto é uma viagem. São as duas vezes em que o dono pegou
o robô e o levou de volta.

**✅ O que fica no lugar.** Ele não chega rápido na porta: ele **trava** nela.

```
travessia 1 (t=160-192)  32 s na porta, v real p50 +0,01 m/s com v_alvo 0,50
                         |erro rumo| p50 23,6° · |wz| real p50 2,9 °/s
                         rumo p50 69° (o corredor é 90°)
travessia 2 (t=843-859)  16 s, rumo p50 81°, e 2,3 s PARADO com v_alvo +0,50
                         e rumo travado em 98,4° — empurrando o batente
```

Duas travessias, rumos de chegada bem diferentes, **mesmo ponto de bloqueio**
(y = 19,02 e 18,92). `dist` ao fim do plano **nunca desceu de 4,94 m** — nesta
corrida a porta nunca foi vencida. O dono confirmou por fora: *"depois de tentar
muito passou 1 vez, que foi quando pela primeira vez chegou no objetivo. Depois
nunca mais"* — essa passagem única está em outro CSV, no robô.

**⚠️ Hipótese minha testada e MORTA**: a mira travando em 0,37 m dentro da porta
(gate de espaço `folga_min = 0,60`) NÃO é o diferencial. Variação do `rumo_alvo`
p90 **40°/s no corredor livre** contra **37-40°/s na porta**. Igual.

**🔧 O que mudou de fato (decisão 044)**: a cadeia de comando passou a ser
gravada. `freeze_capture` + `bin/pause_budget.py` já existiam desde o robô 1 e
chegavam **mudos** aqui (assinavam `Twist` numa cadeia `TwistStamped`, com
tópicos do outro robô, sem entrar na `pilha.launch.py`, e com limiares de
skid-steer). Agora sobem na pilha e gravam
`auto_vel_raw → auto_vel → mux → compensador → atuador → odom` mais o
`collision_monitor_state`. **A primeira corrida da próxima sessão no robô
responde, num comando, a pergunta que ficou aberta:**

```
bin/pause_budget.py ~/logs_robo2/freeze_capture.csv

  collision[...]        o reflexo cortou      -> a caixa é o assunto
  movimentacao_muda     ninguém cortou        -> a lei de rumo é o assunto
  vx_zona_morta         comando fraco demais  -> o piso da movimentação é
```

Custa alguns kB/s — contra os 560 MB/min do bag `--all-topics` que travou o LIO.
Vale com o bag DESLIGADO.

---

## 🔴 20-08 (NO ROBÔ, fim da sessão) — HANDOFF: O QUE ESTÁ MEDIDO E O QUE FALTA

> ⚠️ **LEIA A CORREÇÃO ACIMA ANTES**: os itens "0,55 m/s" e "salto de pose de
> 60 cm" desta seção caíram na medida de 20-08 (2ª leva).

> Escrito para quem pegar o projeto a seguir. A sessão terminou com o dono
> insatisfeito e com razão: o robô bateu de frente num batente, e boa parte do
> meu esforço foi na camada errada. O que está abaixo é o que ficou PROVADO
> com número, o que ficou em aberto, e os erros que custaram tempo — para não
> serem repetidos.

### 🔴 PRIMEIRO ITEM DA PRÓXIMA SESSÃO: ELE BATEU, E ISSO É INADMISSÍVEL

Palavras do dono: **"é inadmissível ele bater"**. Não é preferência, é
requisito.

O robô bateu de frente no batente da porta 2. Ele NÃO estava cego — parou na
mesma porta, no mesmo lugar, várias vezes. A causa é margem de parada, e está
medida no CSV `seguidor_2026-08-20_183507.csv`:

    aproximação   velocidade real   margem da caixa
    t=161,1          0,29 m/s          +19,7 cm     parou
    t=169,1          0,25 m/s          +22,1 cm     parou
    t=185,1          0,25 m/s          +21,8 cm     parou
    t=850,4          0,21 m/s          +24,1 cm     parou
    t=854,4          0,55 m/s           +6,2 cm     <-- BATEU

A conta: o `PolygonStop` avança 0,35 m à frente do centro; quando ele corta, a
placa segura a saída por 0,52 s (decisão 020). A 0,55 m/s o robô anda 29 cm
depois do corte e sobram 6 cm. Um quadro de nuvem perdido a 7 Hz custa 7 cm —
**a margem inteira vale menos que um quadro**.

Todas as paradas bem-sucedidas foram a 0,03–0,29 m/s. A única acima de 0,3 foi
a que bateu.

⚠️ E `0,55 m/s` está ACIMA do `v_max` de 0,50. **Por que ele acelerou a 0,55
naquela aproximação, se nas outras chegou a 0,25, é a pergunta que ficou sem
resposta** — e é por onde eu começaria.

⚠️ A nuvem estava a **7–9 Hz**, não aos 10 nominais. Com 10 Hz o quadro custaria
5 cm em vez de 7. Descobrir por que ela está lenta ajuda aqui E no LIO.

O dono NÃO quer aumentar a caixa. As saídas são: não deixar o robô chegar
rápido na porta, ou recuperar a taxa da nuvem.

### O QUE FICOU PROVADO HOJE (com número)

**1. A retenção da placa causava a dança do rumo — CORRIGIDO e funcionou.**
Medido em `seguidor_2026-08-20_172828.csv`, 24 cruzamentos de zero do erro:
o robô varria **29,9°** DEPOIS que o erro já tinha zerado. A conta fecha:
1,0 rad/s × 0,52 s = 29,8°. Cada correção deixava essa sobra, que virava o erro
seguinte, maior — desvio lateral p90 crescia 13 → 30 → 46 cm.

`erro_antecipado()` em `lei_de_rumo.py` desconta `wz_real × retencao` antes da
lei rodar. Ligado no robô por `retencao_giro_s: 0.52` (o `sobe-robo` passa).
Resultado: sobra caiu para **7–16°** e o dono viu o freio de giro funcionando.

⚠️ NÃO é o `FreioDeGiro` (037) — aquele é contra-torque CHEIO e faz o robô
dançar; o dono vetou explicitamente. Aqui o comando DECAI (é a
`wz_de_frenagem`, contínua) e o contra-giro é proporcional ao excesso.

**2. O suavizador nunca rodava — CORRIGIDO, ganho modesto.**
`/plan_smoothed` saía IDÊNTICO ao `/plan`, dígito por dígito. Causa:
`max_smoothing_duration="0.2"` no BT, e o `simples` precisa de mais de 1 s num
caminho de 19 m. O `SmoothPath` vive num `RecoveryNode` com `AlwaysSuccess`,
então a falha passava em SILÊNCIO.

Corrigido para 3,0 s. Ganho medido: quina máxima 24,0° → 19,8°. **É pouco** —
não espere o S sumir.

**3. `cost_check_points` era divisão por zero.**
Era `[-0.185, 1.0, 0.0]`; o formato é `[x, y, peso]` e peso 0 faz o Nav2
dividir por zero (`options.hpp:88-95`), o peso vira NaN e o Ceres falha 504 em
TODO caminho. Corrigido para `[-0.185, 0.0, 1.0]`. O `suave` passou a
convergir — mas foi testado no Gazebo e REPROVOU (bateu a traseira), então
continua fora do BT.

**4. Curvatura do dia: −0,8365** (era −0,9145, de 11-08). Medida com o
compensador fora da cadeia, v=0,30, fonte LIO. n=1 (a decisão 013 pede três).

### O QUE CONTINUA QUEBRADO

**O S do corredor.** O dono localizou e eu NÃO consegui medir:

> "eu CONSIGO ver o plan pelo web, dá pra ver ele seguindo bem o plan até ele
> ter uma quebra, cada quebra o S piora, no ponto que ele não erra é o meio que
> o plan não quebra"

⚠️ A régua que usei (`raio_curva` do CSV) é CEGA para isso: em y=2 ela acusa
raio de 0,38 m e o robô vai bem (3 cm de desvio); em y=14 ela não acusa nada e
o desvio é 65 cm. Ela mede curvatura DENTRO da janela da mira, e uma
descontinuidade do plano passa despercebida.

Existe `/tmp/quebra2.py` no robô, que mede SALTO (descontinuidade) além de
quina, lendo `/plan_smoothed` ao vivo. Nunca chegou a rodar.

⚠️ NÃO é o `inflation_radius` — eu propus baixar de 0,90 para 0,20 (havia
precedente medido, commit `d8216d1`) e o dono recusou com o argumento certo:
inflação simétrica centra, não serpenteia. E o mapa confirma que ele estava
certo por outro lado: o CENTRO do corredor varia **28,6 cm** (de x=6,01 a
x=7,14) no `andar3todoalterado` — mas isso também não explica, porque o dono
apontou que em UM trecho do corredor ele vai reto, e mapa torto seria uniforme.

**As portas.** 2 rés na porta 1, 5 na porta 2. Nas travadas o erro de rumo é
persistente (mediana 50,6° e 33,3°) e o `|wz|` real mediano é **0,00** — a lei
pede giro e o robô não gira. O reflexo está INOCENTE: o monitor ao vivo
registrou só 4 disparos, todos com o seguidor já pedindo `v=0,00`.
**Por que ele não gira com 50° de erro continua sem resposta.**

### ⚠️ ARMADILHAS QUE ME CUSTARAM A SESSÃO — não repita

**1. NUNCA subir o bag `--all-topics` no NUC.** Ele grava 560 MB/min. Eu deixei
DOIS rodando ao mesmo tempo: 97 GB, carga 14,09 em 12 núcleos, o FAST-LIO
travou e a pose saltava **60 cm com o robô parado**. O dono teve que
reposicionar o robô várias vezes por minha causa. Depois de matar: salto de
0,6 cm e carga 2,80.

O CSV do seguidor cobre travessia, S e desvio. Para `/plan` ou
`/collision_monitor_state`, use os scripts ao vivo em `/tmp` do robô
(`quebra2.py`, `reflexo.py`, `cmp.py`, `pose.py`) — eles imprimem e não gravam.

**2. `movimentacao.yaml` é do ROBÔ; `movimentacao_sim.yaml` é do GAZEBO.**
Passei o dia editando o do simulador achando que afetava o robô. O `a_dec: 0.1`
do robô é MEDIDO nele em 12-08 e está certo — não é chute a corrigir.

**3. Matar a pilha exige o padrão completo:** `/opt/ros/jazzy` +
`Controle_robo_livox` + `gz sim` + `robot_state_publisher` + `parameter_bridge`
+ o `ros2 daemon`. Padrões parciais deixaram CINCO pilhas empilhadas no Gazebo
(cinco `path_follower` publicando = robô andando sozinho). Conferir lista vazia.

**4. `ros2 node list` mente** se o daemon travar; conferir por PROCESSO.
E `--no-daemon` NÃO existe em `ros2 topic hz` (o comando falha e a saída vazia
parece "sem dado").

**5. Comparar `map` e `odom` só com o robô PARADO** ou pareado no tempo dentro
do bag. Dois `tf2_echo` sequenciais com o robô a 0,5 m/s dão 1 m de
"divergência" que é só o robô tendo andado — dei dois alarmes falsos assim e
mandei parar uma corrida boa à toa.

**6. Contar EPISÓDIOS, não amostras.** Reportei "57 rés" quando eram 2, e "148"
quando eram 5. O CSV é a 20 Hz.

**7. O robô não alcança o GitHub.** Deploy é
`git push bara@<ip>:/home/bara/Controle_robo_livox main:main` da máquina de
dev (com `receive.denyCurrentBranch updateInstead` já configurado lá).
Usuário `bara`, não `robo`.

**8. Timeout curto nas chamadas.** Nada de 300–500 s: o dono fica sem resposta
e a bateria corre.

### O ERRO DE MÉTODO QUE MAIS CUSTOU

Fiz **três** correções na mira (`tol_estica` 0,03, `rumo_estica` 6°,
`mira_rumo_passo` 0,40) — todas mexendo na régua que LÊ o plano — enquanto o
plano nem estava sendo suavizado e o dono já tinha dito que o defeito era a
quebra do plano. Duas foram reprovadas e revertidas; a terceira (`passo 0,40`)
ficou, mas o mérito dela está em suspenso.

Antes de mexer no seguidor, **verifique se o plano que chega até ele presta**.

---

## 🟢 20-08 (GAZEBO) — ESTADO ATUAL, E ELE VAI PARA O ROBÔ

> **É este o estado que sobe no robô.** Deploy: `git fetch && git reset --hard
> origin/main`. A sessão de 20-08 foi inteira de desfazer uma leva automatizada
> que piorou a porta, e de achar o que dela valia a pena.

### Configuração que está valendo

```
passagem_v_max                    0,5    (teto NEUTRO — 0,25 era a causa da travada)
passagem_estreita_habilitada      False  (o alvo no centro do vão é singularidade)
desencalhe_pivo_habilitado        True   (decisão do dono: mostra onde ele erra)
recuperacao_infinita_com_objetivo True   (idem)
a_dec (sim)                       0,3
mira_tol_estica / encolhe         0,07 / 0,08   (042, medido no robô)
mira_rumo_estica / encolhe        3° / 5°       (baseline)
mira_rumo_passo                   0,40   (a única mudança aprovada no mapa real)
smoother do BT                    simples
cost_check_points                 [-0.185, 0.0, 1.0]  (era divisão por zero)
```

### O que foi medido

Pista, três corridas seguidas sem mexer em nada:

```
corrida_01   111,5 s   0 paradas   1 invasão da caixa   desvlat mediano 6 cm
corrida_02   112,7 s   0 paradas   2 invasões           5 cm
corrida_03   140,3 s   0 paradas   0 invasões           5 cm
```

Mapa real `sala_andar3`, com `mira_rumo_passo: 0,40` — aprovado a olho pelo
dono (*"agora foi bom, gostei dessa"*), **sem repetição**.

### 🔴 Critério de projeto que o dono estabeleceu em 20-08

> *"O collision monitor não foi feito para ajeitar a posição e fazer manobra,
> ele foi feito pra parar impactos inevitáveis, com obstáculo fora do mapa, não
> obstáculo conhecido."*

E, sobre como julgar corrida:

> *"Parar não é um problema, problema é ir errado. Parar por precaução é
> válido."*

Métrica de corrida é **qualidade da aproximação** (yaw na soleira, desvio do
centro do vão, folga do CORPO), não contagem de paradas.

### Fila para a sessão NO ROBÔ

1. **Medir a curvatura do dia** — `curv_frente: -0.817` está com
   `curv_medido_em: HERDADO`, medido no robô em 04-08. O protocolo é três
   corridas retas sem compensador, `medir.py --resumo curvatura`, e subir com o
   valor do dia. Nunca foi feito no simulador; no robô é obrigatório.
2. **Uma passagem de porta**, e ler antes de mexer em qualquer coisa.
3. Se a porta falhar, o primeiro suspeito é a **mira** (`mira_rumo_passo 0,40`
   foi validado só no Gazebo, e o `sala_andar3` não é a porta do prédio).

### ⚠️ Riscos conhecidos

- **`mira_rumo_passo: 0,40` não tem repetição.** Aprovado em n=1.
- **O plano corta quinas**: no mapa real o robô travou com o corpo a 5 cm de
  uma quina, numa área aberta, porque o plano passava rente. Mexe no costmap.
- **CPU satura e estraga a corrida**: com `bag:=true --all-topics` o bag escreve
  560 MB/min e o `/scan` cai para 6,2 Hz. Usar `bag:=false` por padrão.
- **Matar a pilha exige o padrão completo**: `/opt/ros/jazzy` +
  `Controle_robo_livox` + `gz sim`, e conferir lista vazia. Padrões parciais
  deixaram 5 pilhas empilhadas em 20-08.

### Preservado

- Estado do codex: branch `backup/codex-2026-08-20` (não usar como base).
- Dados das 14 corridas dele: `docs/dados/2026-08-20-constancia-gargalos-gazebo/`.

---

## 🟢 19-08 (GAZEBO, última leva) — MELHOR CORRIDA OBSERVADA ATÉ AGORA

> **Baseline atual a preservar.** Veredito do dono: o robô foi e voltou
> perfeitamente, passou a porta, executou o pivô e esta foi a melhor execução
> vista até agora.

- Na ida, o objetivo 1 foi interrompido porque o objetivo 2 foi enviado antes
  da chegada. Isso foi uma preempção, não uma falha: o robô já tinha passado
  pela porta e seguia em condições de concluir.
- Na volta, o objetivo 2 terminou com `Goal succeeded`.
- O pivô foi acionado com erro de `+166°`. Mesmo encerrando o pulso com cerca
  de `20°` restantes, o seguidor retomou o plano e completou a volta sem fazer
  o balão grande que havia aparecido nas tentativas anteriores.
- Evidências exatas: CSV
  `docs/dados/2026-08-19-sem-re-aleatoria/seguidor_2026-08-19_211353.csv` e
  rosbag
  `docs/dados/2026-08-19-sem-re-aleatoria/corrida_2026-08-19_211348/`.

### Configuração congelada como referência

- planejador `nav2_smac_planner::SmacPlanner2D`, com
  `cost_travel_multiplier: 5.0`;
- footprint orientado igual ao `PolygonStop` nos dois costmaps;
- replanejamento a `0.2 Hz` e `timeout_plano: 7.0 s`;
- mira curta restaurada para `0.37 m` — os `0.15 m` foram reprovados;
- mira longa de `1.0 m` somente quando o próximo metro é essencialmente reto;
  havendo curva próxima, a mira fica curta para não antecipá-la.

É o melhor **resultado observado**, ainda com amostra manual `n=1`. A próxima
sessão deve começar repetindo o percurso ida/volta pelo menos três vezes sem
mudar parâmetros. Só depois dessa repetição deve ser aberta uma nova mudança.

## 🟡 19-08 (GAZEBO) — A VOLTA VIROU REFERÊNCIA; A IDA AINDA FREIA DEMAIS

> **Estado atual e prioridade para a próxima sessão.** O robô atravessou a
> porta, cumpriu os dois objetivos e voltou. A volta foi, nas palavras do
> dono, a melhor corrida que ele fez sozinho até hoje. O avanço de hoje deve
> ser preservado; o próximo trabalho é tirar as interrupções da ida sem tirar
> a proteção contra colisão.

### Baseline que está valendo

- O código de movimento/planejamento foi devolvido ao `HEAD` `cd016e8`, que
  voltou a atravessar a porta na ida e na volta. Os experimentos de pivô e de
  corte antecipado do plano que regrediram a porta não fazem parte do baseline.
- O Smac continua ativo. Não voltar ao Theta.
- Foi preservado apenas o ajuste de localização que reduziu a perturbação do
  AMCL durante giros: `alpha1: 0.01` e `alpha4: 0.001`.
- O `PolygonApproach` e o `PolygonStop` continuam ativos. Desligar o stop quase
  levou a contato e foi revertido.
- O seguidor ganhou a saída inspirada no robô 1: se está emperrado, a traseira
  está bloqueada e há frente medida livre, ele avança até 0,20 m, reavaliando
  o vão em todo ciclo. Se a frente não está fisicamente bloqueada, falta de
  progresso sozinha **não autoriza ré**.
- A regra nova da ré está coberta por 11 testes em
  `test/test_re_desligada.py`, todos verdes na última execução.

### Última corrida manual — o que realmente aconteceu

Dados: `docs/dados/2026-08-19-sem-re-aleatoria/`.

```
dois goals                         SUCCEEDED
amostras no estado de desencalhe   49
velocidade negativa nesse estado   0
velocidade positiva nesse estado   49
```

Logo, **não houve ré automática na última corrida**. Houve dois escapes para
frente de aproximadamente 0,20 m quando a traseira estava bloqueada. Na volta,
quando o pivô encerrou com cerca de 22° restantes, o seguidor mediu 2,45 m
livres à frente, recusou corretamente a ré por mero sintoma, voltou a seguir o
plano e terminou. Esse é exatamente o comportamento desejado e deve ficar.

O que pareceu serem três “mini-rés” na ida veio de outro nó: o
`compensador_rumo` recebeu cortes do `PolygonStop` e publicou contra-torque
linear de `-0.50` para cancelar a inércia. O freio está ligado no baseline.
Ele existe porque, sem frenagem, a placa mantém movimento por cerca de 0,52 s
e o robô já percorreu mais 0,10 m após um corte. Portanto, **não desligar o
freio nem enfraquecer o `PolygonStop` às cegas**.

### Próxima melhoria, uma causa por vez

1. Trabalhar somente no freio linear, usando a velocidade longitudinal medida
   como realimentação para encerrar/reduzir o contra-torque antes que ele seja
   percebido como recuo. Não mexer junto em Smac, pivô, AMCL, footprint ou
   lógica de desencalhe.
2. Instrumentar cada atuação com velocidade antes, mínima durante, duração e
   deslocamento longitudinal. Critérios simultâneos: nenhuma velocidade ou
   distância negativa relevante e nenhum avanço residual que comprometa a
   margem do `PolygonStop`.
3. Corrigir o texto de log `fim da ré` quando o sentido for positivo para
   `fim do escape para frente`; hoje o rótulo confunde a leitura, embora o CSV
   mostre corretamente velocidade positiva.
4. Repetir o mesmo percurso `(0,0) -> porta/corredor -> (0,0)` pelo menos três
   vezes. Aceitação: ida sem trancos/mini-rés, volta preservada como está,
   nenhuma colisão, nenhuma ré sem bloqueio frontal e ambos os goals concluídos.

### Estado operacional ao encerrar este registro

Uma simulação permanece aberta para os testes do dono, iniciada com dados em
`docs/dados/2026-08-19-sem-re-aleatoria/`. Antes de uma nova subida, matar e
confirmar a ausência de qualquer `pilha.launch.py`, `gz sim` e `rviz2` antigo.

---

## 🐍 18-08 (NO ROBÔ) — O S TEM UM GATILHO, E ELE ESTÁ NO PLANO, NÃO NO CONTROLE

> 🔴 **É ISTO QUE ESTÁ ABERTO E É A PRIORIDADE.** Achado pelo DONO, a olho nu,
> enquanto eu media atuador. Medido depois. Diário: entrada 18-08.

### A frase que mudou onde procurar

> *"o plan ficava mudando um pouco, não tava reto, aí o robô tentava ir pro
> lado e se perdia e começava o S"* — e, mais preciso: *"ele não fica uma
> linha, ele fica uma linha quebrada em várias partes, aí essas partes que
> tinham diferenças, pequenas, mas isso ferrava o robô"*

### O número, num corredor RETO de 14 m

```
/plan            26 pedacos onde UMA reta bastaria, de 0,32 m cada
                 EMENDA entre pedacos:  p50 11,3°   p90 25,6°   max 49,8°
                 84% das emendas > 5°   ·   60% > 10°

/plan_smoothed   emendas caem para p50 2,6°  (o suavizador GANHA aqui)
                 ...mas sobram 3% acima de 10°, e uma de 45°

ONDULACAO contra a reta   /plan 28,9 cm  ·  /plan_smoothed 29,2 cm   <- IGUAL
```

➡️ **O suavizador conserta as emendas e NÃO conserta o meandro.** Os 29 cm
atravessam ele intactos. Com mira a 1 m, 29 cm valem **16,2° de ordem de rumo**
num corredor onde a ordem certa é zero. O robô tem 0,32 m de raio: o plano
manda ele tecer quase a própria largura numa reta.

### 🔴 E AS CURVAS SÃO FORTES DEMAIS — observação do dono, medida

O raio que a máquina fecha é **0,37 m** (medido 29-07), ou seja, curvatura
máxima de 2,70 1/m. O que os planos pedem:

```
/plan            PICO por plano  p50 2,53   p90 3,10   max 4,24 1/m
                 41% dos planos pedem curva que a maquina NAO fecha
                 raio exigido no pico mediano: 0,395 m

/plan_smoothed   PICO por plano  p50 1,72   p90 2,53   max 3,51 1/m
                 8% dos planos ainda nao cabem
                 raio exigido no pico mediano: 0,580 m
```

**Onde isso nasce**: o planejador é `nav2_theta_star_planner::ThetaStarPlanner`
— grade *any-angle*, **sem nenhuma noção de raio mínimo**. Ele não sabe que o
robô existe. É a mesma raiz da reta quebrada: um planejador sem modelo
cinemático produz caminho que ignora a máquina.

**E o suavizador tem a trava certa, com o peso errado.** No `nav2.yaml`:

```
minimum_turning_radius: 0.37     ✅ o raio da maquina, correto
w_curve:                30.0     <- peso de RESPEITAR o raio
w_smooth:           15000.0      <- peso de ficar LISO
```

➡️ **500 para 1.** Quando "liso" e "cabe no robô" discordam, o liso ganha por
500×. Bate com o medido: as emendas somem (o liso ganha) e 8% dos picos seguem
furando o raio. A trava existe e é atropelada.

### A cadeia do S, com três elos

```
1. GATILHO       plano quebrado / meandro de 29 cm  ->  ~16° de rumo espurio
2. AMPLIFICADOR  a placa so entrega MODULO CHEIO (2,204 rad/s, decisao 023)
3. REALIMENTA    tempo morto 0,94 s — medido nesta sessao: o sentido do giro
                 casa com o do comando em 75% assumindo 0,94 s, contra 47% sem
```

🔴 **É por isso que sintonizar ganho nunca resolveu.** O elo 1 é uma REFERÊNCIA
ERRADA, e nenhum controlador conserta referência errada. Toda a leva 037/038 e
a conversa do `segura_rumo` atacavam os elos 2 e 3.

### O que está medido e o que NÃO está

```
✅ o meandro existe, tem 29 cm, e o suavizador nao o remove
✅ as emendas existem: 11,3° a cada 0,32 m no plano cru
✅ o tempo morto de 0,94 s aparece no proprio dado da sessao
❌ que o meandro CAUSA o S — falta correlacionar no tempo (analise offline,
   NAO precisa de robo)
❌ o teste do PESO na traseira: o `y` da corrida e monotono, entao "onde" e
   "quando" sao a mesma coluna e nao da para separar o peso do trecho.
   O teste que decide: o MESMO trecho duas vezes, com e sem peso
```

### ⚪ De bom: o mapa novo localiza bem

Primeira corrida no `andar3todo`: salto de pose **máximo 4,0 cm** em 499 s, 1
amostra acima de 5 cm em 4946. O AMCL não pula — **o mapa serve**, e isso
derruba a ressalva com que eu o commitei.

---

## 🎮 18-08 (dev, robô desligado) — O ROBÔ GANHA UM CONTROLE XBOX

> Decisão **043**. Pedido do dono: adaptar o robô 2 ao Xbox como foi feito no
> robô 1 (`ecd6e70`). **Nada verificado no robô** — ver o protocolo abaixo.

### O que passou a existir

```
config/teleop_xbox.yaml       mapa de botões + escalas DESTE robô
config/twist_mux.yaml         faixa `joystick`, prioridade 100 (a maior)
launch/joystick.launch.py     joy_node + teleop_twist_joy, auto-detecta o jsN
test/test_joystick_coerente.py  11 casos; 8 mutações conferidas
bin/sobe-robo                 passo 4/5, e o `vivos()` agora mata os dois nós
pair-xbox.sh + scripts/_bluez_fixes.sh + scripts/js_mapping.py   portados
```

Dirigir: **segure o LB** e mexa o analógico esquerdo. **RB** = turbo. Soltou o
LB, ele para — homem-morto, e é o que justifica a prioridade 100.

### 🔴 O QUE NÃO ATRAVESSOU DO ROBÔ 1, e por que copiar teria sido pior

```
publish_stamped_twist   lá false, AQUI TEM DE SER true
                        (a cadeia daqui é TwistStamped; com false o DDS
                         recusa por type hash e o robô ignora o controle
                         SEM UMA LINHA DE ERRO)
scale_angular 6.0       knob anti-skid do 4 rodas; o CLAUDE.md proíbe herdar.
                        Aqui: 0,80 normal / 1,00 turbo (= wz_max da máquina)
entorno do pareamento   lá Pi + `robo` + mDNS; aqui NUC + `bara` + IP que muda
```

### ⚠️ O que o operador vai sentir, e NÃO é o config errado

Girar parado é quantum de ~95° (módulo único de 2,204 rad/s, decisão 023) —
a mesma coisa que o dono viu em 14-08 (*"o pivô dele tá forte demais"*), agora
com o analógico na mão. **Baixar `scale_angular` não resolve** (a placa ignora
o teto) e piora o giro em movimento, que funciona.

### Decisão do dono nesta sessão

O filtro de ir reto **pega o joystick de graça** — o `compensador_rumo` fica
depois do mux. Mas só o feedforward; a malha PI (`segura_rumo`) segue desligada
na pilha, por escolha explícita dele, e mede-se o desvio no campo antes de
mexer. Racional completo na 043 §5.

### ▶️ PROTOCOLO DE CAMPO — `docs/ROTEIRO_XBOX_NO_ROBO.md`

> 📋 **O roteiro da tarde de 18-08 está escrito e é para seguir na ordem.** Ele
> tem uma coisa que o resumo abaixo não tem: a **Fase A inteira roda com a
> bateria das RODAS DESLIGADA** — parear, medir os botões e provar o
> homem-morto não exigem roda nenhuma, e o `joystick.launch.py` sobe sem a
> base. A validação de segurança acontece com o robô incapaz de se mover.

Resumo (o detalhe, os critérios de parada e o que mandar estão no roteiro):

```
1. ./pair-xbox.sh                     no NUC (usuário bara). Sucesso é o
                                      /dev/input/jsN aparecer, não o BlueZ
                                      dizer "Connected"
2. ./scripts/js_mapping.py /dev/input/jsN
   🔴 CONFIRMAR LB=6 e RB=7. Estes números vieram MEDIDOS DO ROBÔ 1, não
      deste controle. Divergiu, o config muda ANTES de qualquer corrida
3. bash bin/sobe-robo                 o passo 4/5 imprime o js e a taxa do /joy
4. homem-morto: com o LB SOLTO, mexer o analógico -> robô tem de ficar PARADO
5. só então dirigir
```

### ⚪ Verificado no dev

```
build                limpo
suíte                340 testes verdes
teste novo           11 casos; as 8 mutações que ele deveria pegar, pegou
auto-detecção        exercida (sem controle, cai em js0 e o joy_node espera)
```

### Achado de lado

`scripts/setup_headless.sh` dava `source` em `scripts/_bluez_fixes.sh`, que
**não existia neste repo** (veio no clone sem o arquivo). Com `set -e`, o setup
headless morria ali. O port do Bluetooth conserta.

---

## 🔴 14-08 (NO ROBÔ) — O SEGUIDOR NÃO DIRIGE NO ROBÔ REAL: `camera_init` É RAIZ SOLTA

> Primeira ida ao robô com o trabalho de 039–042. Deploy feito por `git bundle`
> (o NUC não tem chave do GitHub — igual 30-07). **Nenhuma corrida aconteceu.**

### 🔴 O DEFEITO, e ele é do conserto da 040

Objetivo aceito, plano desenhado, **robô parado**. O log acusa, e é honesto:

```
[path_follower] sem TF camera_init<-map (... not part of the same tree.
                Tf has two or more unconnected trees.); o plano não pode ser
                usado — dirigir com ele seria o defeito de 14-08 de volta
```

`path_follower.py:545`:

```python
destino = self.pose.header.frame_id or 'odom'
```

```
no GAZEBO   /Odometry nasce em `odom`        -> está na árvore, funciona
no ROBÔ     FAST-LIO publica `camera_init`   -> RAIZ SOLTA, o lookup falha
```

O `tf_odom` consome esse mesmo `/Odometry` e publica `odom→base_link` (medido
funcionando: `tf2_echo odom base_link` responde), mas **ninguém liga
`camera_init` à árvore do `map`**.

➡️ **O conserto de frame da 040 foi validado o dia inteiro no simulador, onde o
frame por acaso bate. Ele nunca podia ter funcionado no robô.**

### 🔴 E TEM UM SEGUNDO DEFEITO NO MESMO LUGAR

A pose que o seguidor usa é a do `/Odometry` cru = a pose do **SENSOR**. O
próprio `tf_odom` avisa na subida: *"A pose do LIO é do SENSOR; a composição com
o URDF é o que impede os 42 cm do Mid-360 de virarem erro silencioso de
navegação"*. Mesmo com o frame conectado, ele dirigiria por um ponto 42 cm fora
do centro do corpo.

### ▶️ CONSERTO PROPOSTO, **NÃO IMPLEMENTADO** (o dono quer testar antes)

Ler a pose por **TF** (`map→base_link` ou `odom→base_link`) em vez do
`/Odometry` cru — resolve os dois de uma vez. Ordem combinada: (1) no dev, com
teste que trave o caso `camera_init`, que hoje **nenhum teste cobre**; (2)
protocolo no Gazebo para garantir que o 4/5 não quebrou; (3) só então o robô.

### ⚪ O que ficou VERIFICADO no robô, apesar de tudo

```
deploy por git bundle        c9ff66f -> 9598367, 5 commits
build + 329 testes           verdes NO PRÓPRIO NUC
/plan_smoothed               publicando a 0,95 Hz — o 042 vale lá
caixa do reflexo             quina 0,385 (a protetora) confirmada no nó vivo
odom->base_link              existe e responde
bateria das RODAS            40,36 V   (12-08 rodou bem com 41,13)
/scan                        7,36 Hz   (12-08: 9,998 Hz) — degradação nova, causa desconhecida
```

### ⚠️ Três obstáculos de infra que custaram tempo, de novo

1. **rede**: NUC em `10.244.3.205`, dev em `10.150.x.x` — sub-redes diferentes,
   sem rota. Resolveu quando o DHCP moveu o dev para `10.244.3.5/24`. Usuário do
   NUC é **`bara`**, não `robo`; `robo-desktop.local` não resolve entre redes;
2. **a bateria do NUC morreu no meio do teste** — ela não tem telemetria, e a
   dos 40,36 V é só a das rodas;
3. **o pré-voo não cobria a árvore de TF** — foi por isso que eu disse "pode
   mandar o ponto" para um robô que não podia dirigir.

⚠️ `rosbag2_recorder` assina `/tf` com DURABILITY TRANSIENT_LOCAL e os
publishers são VOLATILE: **os bags gravados no robô saem sem TF**. Não afeta a
navegação, afeta a análise depois.

---

## 🟢 14-08 (4ª leva) — O SEGUIDOR NUNCA VIU O PLANO SUAVIZADO. 4/5 E ZERO RÉ

> Decisão **042**. Dados em `docs/dados/2026-08-14-porta-042/`.
> **Nada foi ao robô.**

### 🔴 O DEFEITO ERA DE FIAÇÃO, NÃO DE SINTONIA

A decisão 026 escreveu uma árvore de comportamento própria só para suavizar o
plano — e o `path_follower` assinava **`/plan`**, o Theta\* CRU. Desde a 026 o
suavizador trabalha e o resultado ia para o lixo, sem uma linha de log. Mesma
família do defeito de frame da 040.

```
curvatura exigida na aproximação da porta, 5 corridas em 5

/plan            raio mínimo  0,215 a 0,275 m    a máquina fecha 0,37 → NÃO CABE
/plan_smoothed                0,402 a 0,477 m    cabe
```

**Conserto**: `topico_plano` default `/plan_smoothed`, com **queda para o cru**
se o suavizado calar mais que `timeout_plano` (o `SmoothPath` recusou 1 vez em
79 numa corrida; sem a queda isso vira robô parado sem culpado no log).

### 🟢 A VERIFICAÇÃO

```
             passou   folga na garganta   corridas com ré
cx041 (antes)  3/5      0,037 a 0,063          3 de 5
cx042 (agora)  4/5      0,109 a 0,158          0 de 5
```

**Zero ré** — que é o alvo que o dono fixou. Em 3 das 5 o reflexo não agiu
nenhuma vez; em 20 corridas de porta isso nunca tinha acontecido.

### 📏 A RÉGUA DO DIA, 20 corridas, separa sem sobreposição

```
passou LIMPO (sem ré)  n=8    folga do corpo na garganta  +0,122 a +0,158
passou com ré          n=5                                +0,047 a +0,105
travou                 n=7                                +0,037 a +0,109
```

⚠️ A que travou agora teve folga **0,109** — a maior de qualquer falha do dia,
encostada no limiar. **Causa não investigada**; não é mais o regime antigo.

### 🔴 TRÊS HIPÓTESES MINHAS MORTAS PELA MEDIDA, no caminho até aqui

| | como morreu |
|---|---|
| a caixa do reflexo (041) | encolhida e rodada: 3/5, igual à base |
| a mira adaptativa esticada | 42% do tempo esticada nos DOIS lados |
| a inflação do costmap | reproduzida offline: de `0,90·3,0` a `1,50·0,7` o caminho não se move |

⚠️ **E duas vezes quase concluí de um artefato**: filtrar por `v_alvo != 0`
mostra as travadas seguindo o plano MELHOR, porque na travada o seguidor
continua pedindo velocidade por ~70 s com o robô parado em cima do plano.
**Filtro de "está andando" olha deslocamento MEDIDO, nunca o pedido.**

### 🔵 A RÉ, medida e deixada em paz por decisão do dono

Das 15 corridas anteriores: a ré disparou em 5 e as 5 passaram; nas 6 que
travaram não disparou nenhuma. O log diz por quê — *"emperrado e sem vão para
recuar, atrás há 0.00 m"*: com o robô a 30° atravessado, o corredor retangular
da ré pega a ombreira. Ele fica preso entre as duas regras de segurança dele.
➡️ **O dono vetou mexer na ré**: o alvo é ele não precisar dela — e agora não
precisa.

---

## 🔎 14-08 (3ª leva) — A CAIXA DO REFLEXO É QUE NÃO CABE NA PORTA

> Decisão **041**. Só análise offline das 10 corridas do A/B da histerese.
> **O Gazebo não subiu, o robô não foi ligado, e o conserto NÃO FOI RODADO.**

### 🔴 A CAUSA DA TRAVADA, MEDIDA EM 4 POSES INDEPENDENTES

Quem veta é o **`PolygonStop`** (caixa estática), 61–64 s por travada. O
`PolygonApproach` aparece 0,0–0,1 s. ⚠️ **A 040 acusou o `PolygonApproach` e
estava errada.**

```
na pose exata da travada (robô a −23° a −27° cruzando o vão de 0,90 m)

  folga do CORPO até a jamba     +0,049 a +0,069 m    cabia, e sobrava
  folga da CAIXA na mesma pose   −0,017 a +0,002 m    vetava
```

🔴 **É aritmética de retângulo**: somar 5 cm por FACE empurra a QUINA em 5·√2 =
**7,1 cm**, e é a quina de TRÁS do lado de dentro da curva que sobra para a
jamba. Num vão de 0,90 m o orçamento por lado é 22,3 cm e o corpo a 25° já
gasta ~17.

### 🟢 O CONSERTO — um parâmetro, margem medida na QUINA

```
traseira  0,2665 -> 0,2375     lateral  0,2775 -> 0,2475
frente    0,35 INALTERADA  (varrida até 0,2165, não move a folga 1 mm)
pior folga nas 4 poses:  −0,017  ->  +0,022 m
```

Invariante nova no `test_configs_coerentes`, verificada por mutação. **324
verdes** em `robot_motion`.

### 🟢 O SEGUIDOR JÁ SEGUE O PLANO — não sobrou o que sintonizar

`|desvio_lateral|` p50 **2,5 cm** (era 11 cm em 13-08); travadas e passagens
**não se separam** em nenhum percentil. E no plano da porta o **plano** cruza a
5–13 cm do centro com −18° a 0°, enquanto o **robô** cruza a 1–7 cm com −20° a
+5°: colar mais no plano (`k_lat`) mandaria o robô para o defeito.

⚠️ **Um artefato meu que quase virou conclusão**: filtrando por `v_alvo != 0`, a
separação saía perfeita e INVERTIDA — porque na travada o seguidor segue
pedindo velocidade por ~70 s com o robô parado em cima do plano. Filtro de
"está andando" tem de olhar deslocamento **medido**, nunca o pedido.

### ▶️ PRÓXIMO PASSO, e precisa do dono na tela

Protocolo de 5 corridas (`tools/banco/protocolo_porta.sh`), máquina limpa.
Linha de base: **3/5**. Previsão escrita antes: as travadas somem; o que
sobrar de falha tem outra causa. ⚠️ O robô **continua entrando torto** — isto
não conserta a entrada, só para de proibir uma passagem que cabe.

---

## 🔧 14-08 (2ª leva) — DOIS DEFEITOS DE VERDADE, E O PIVÔ PASSOU A EXISTIR

> Decisão **040**. Dados em `docs/dados/2026-08-14-porta-gazebo/`, protocolo de
> repetição em `protocolo/`. **Nada foi ao robô.**

### 🟢 O QUE FICOU (defeitos de mecanismo, corrigidos)

**1. O seguidor comparava frames diferentes.** `path_follower` lia pose em
`odom` e plano em `map`, **sem TransformListener nenhum**. A diferença é a
correção do AMCL — salto p90 de 14,3 cm na corrida A, deriva de 76 cm; e 833 cm
na corrida E, quando o AMCL fugiu. Ele dirigia para fechar um desvio que não
existia. Isso explica o S, a entrada torta na porta, e por que **toda melhoria
de responsividade piorava o resultado**.

**2. O pivô era o único caminho da cadeia sem freio.** `heading_controller`
dava `return` dentro do pivô ANTES do bloco do freio de giro (037). Varredura
medida: 150–310° por pulso. Com o freio chegando lá: **91° e 131°** — a ordem
certa para o gatilho de **80°** da decisão 036.

🟢 **É isto que fez o pivô existir**, nas palavras do dono: *"AGORA O PIVO DELE
EXISTE E GIRA NA HORA CERTA PRA PASSAR A PORTA"*. **O 80° era dele e estava
certo o tempo todo.**

**3. Log de toda corrida, por padrão** (pedido do dono). Dois furos: `csv`
nascia vazio **e `grava()` nunca era chamado** — código morto. Nenhuma corrida
deste projeto tinha sido gravada pelo nó. Agora `log_dir` (default
`~/logs_robo2`) liga CSV do seguidor + bag da corrente nos DOIS perfis.

### ⚪ O QUE FOI REPROVADO (quatro tentativas, todas medidas)

| tentativa | mediu | veredito |
|---|---|---|
| `k_lat=1,0` (Stanley sem limite) | amplitude p90 20,0° → **39,8°** | reprovado |
| `k_lat=0,5` + limite de taxa | inversões 26,9 → **30,5**/min | reprovado |
| pivô em 34°, `a_dec` 6,6 | varredura 150–310°/pulso | reprovado |
| idem, já com freio | varredura 91–131° | reprovado |

🔴 **`pivo_a_dec` real medido: 0,79.** O valor original era 0,6; o assistente
inventou 6,6 duas vezes, errando ~8× nas duas. **Derivação não é medida.**

Estado dos knobs: `k_lat: 0`, `limiar_pivo: 80°`, `pivo_a_dec: 0,6`.
No ar e **ainda sem julgamento isolado**: histerese do giro (16°/5°) e mira
adaptativa (0,37 m em curva / 1,00 m em reta).

### 🔴 A HIPÓTESE MAIS IMPORTANTE EM ABERTO

```
período da oscilação de rumo, 7 corridas:  2,0–2,8 s  — CONSTANTE
enquanto mudavam lei, ganho, mira, pivô, frame e histerese
```

Relé com tempo morto `L` oscila em ~4L; `atraso_desliga` = 0,52 s → 2,08 s. A
placa É um relé (decisão 020). ➡️ **Se confirmar, nenhuma reescrita do seguidor
mata o S** — o alvo passa a ser a placa (`deadband_enable`, tempo morto).

### 🔴 O PROTOCOLO DE 5 CORRIDAS: 1 PASSOU, 4 TRAVARAM

Rodado pelo assistente com o dono no almoço (autorização explícita dele, que
inverte a regra de "Gazebo só sobe com o dono olhando"). Pilha NOVA a cada
corrida — `tools/banco/protocolo_porta.sh`.

```
corrida  veredito       parou em       folga   sobra p/ corpo   yaw     reflexo
   1      PASSOU     (+6.15,+3.71)     0.949      +0.722       no alvo  NÃO AGIU
   2      TRAVOU     (+4.64,+1.41)     0.402      +0.175       -10.2°   14x, 51 s
   3      TRAVOU     (+4.65,+1.58)     0.347      +0.120       -16.7°    4x, 69 s
   4      TRAVOU     (+4.72,+1.52)     0.408      +0.180       -33.6°    3x, 70 s
   5      TRAVOU     (+4.92,+1.58)     0.353      +0.126       -33.2°    3x, 69 s
```

🔴 **Em nenhuma falha o corpo invadiu o mapa** — parou com 12 a 18 cm de folga
por lado e ficou congelado ~69 s. Não é colisão: é o reflexo travando com
margem sobrando, porque as quatro **entraram tortas** (−10° a −34°).
⚠️ ~~O `PolygonApproach` é projetado pela velocidade, e a −33° a projeção
alcança a ombreira antes do corpo.~~ **ERRADO, corrigido pela 041**: quem veta
é o `PolygonStop` estático (61–64 s por travada, contra 0,0–0,1 s do
`PolygonApproach`), e o canto que encosta é a **quina de trás**.

➡️ **A dívida nº 1 continua a mesma: ele entra torto na porta.** As duas
correções de mecanismo de hoje eram reais e necessárias, mas atacavam outra
coisa. O próximo alvo é chegar à porta JÁ APONTADO — e o desenho que a medida
pede é o do robô 1 (`door_crossing`: alinhar até `|lat|<8 cm` e `|yaw|<5°`
ANTES de cruzar), mesmo estando desativado lá.

### ⚠️ Higiene da máquina de dev

O grep de limpeza NÃO pega `parameter_bridge` nem `robot_state_publisher` (rodam
de `/opt/ros`). Acumularam a tarde toda e **duas subidas falharam** — o
`planner_server` estoura 65 s de ativação com load 19 em 12 núcleos. Há um
`mysqld` de snap comendo ~1 core, alheio ao projeto.

---

## 🧭 14-08 (dev + Gazebo) — A RÉ MORRE COMO IDEIA, O SEGUIDOR GANHA O DESVIO LATERAL

> Decisão **039**. Bag: `docs/dados/2026-08-14-porta-gazebo/corrida_a`.
> **Nada foi ao robô.** Nenhuma corrida rodou com a lei nova ligada.

### ❌ A VIRADA PARA A RÉ FOI PROPOSTA E ABORTADA — com dado, não por gosto

O dono propôs inverter a frente do robô (girar o lidar + trocar os comandos),
porque **a ré é 8,3× mais reta em arco** (04-08; corroborado 9,4× e 8,74×).
A premissa está certa e continua valendo. **O que a derrubou**: no Gazebo ele
erra a porta do mesmo jeito — e o Gazebo tem boba que é patim (BO-4) e cancela
o arco de frente por construção. Logo **o defeito da porta não é do sentido de
marcha**, e virar o robô levaria o erro lateral junto, inteiro.

⚠️ A carta continua na mesa para o ARCO, e o melhor argumento dela não é o 8×:
é que o `curv_frente` já foi remedido 3 vezes (−0,817 → −0,9116 → −0,9145),
anda ~12% entre sessões, e 12% de 0,82 é ~0,10 1/m sem cancelar — do tamanho do
arco INTEIRO da ré. De frente ele depende de calibração toda sessão; de ré
quase não.

### 🔴 O SEGUIDOR ERA O CULPADO, E AGORA ESTÁ MEDIDO NAS DUAS MÁQUINAS

```
porta de 0,880 m, corpo de 0,455 m

o PLANO cruza a 3,8 cm do centro   -> deixa 0,175 m p/ o corpo   <- o plano está BOM
o ROBÔ  cruza a 14,8 cm do centro  -> deixa 0,065 m
                                      o seguidor come 0,110 m (2/3 da margem)

13-08 no robô real: o seguidor comia 0,082 m.  Mesmo defeito, mesmo tamanho.
```

**Entrou (039)**: o termo de Stanley somado ao rumo do carrot,
`rumo_alvo = rumo_carrot − atan2(k·e_lat, max(|v|, v_ref))`. O ganho veio da
dinâmica (erro decai com constante de tempo 1/k s, independente da velocidade),
não de varredura. 11 testes novos, **283 → 294** no `robot_motion` (470 no
repo). **A lei nasce neutra**: `k_lat:=0.0` reproduz a lei de hoje.

### 🔴 O FREIO LINEAR NÃO FREIA — ELE INVERTE A MARCHA (achado do dono)

*"dá um cutucão e faz o robô recuar uns 10 cm; tá parando e ainda invertendo o
sentido"*. Confirmado no bag, 8 vezes na corrida:

```
engata a +0,319 m/s, manda −0,500 por 0,44 s
SOLTA com o robô ainda a +0,256 m/s   ->   ele termina a −0,224 m/s
```

**A causa é estrutural, não é ganho**: o tempo morto da placa
(`atraso_desliga` 0,52 s) é MAIOR que o evento de frenagem inteiro (0,44 s).
Malha fechada nessas condições não tem como não passar do ponto. Junto vieram
dois defeitos menores: `freio_pico_min` (0,12) é **código morto** (o freio só
engata acima de 0,25, então o portão já nasce satisfeito), e **um limiar só**
faz papel de engate E de solta, o que impede sintonizar os dois.

➡️ **Conserto proposto, NÃO implementado**: contra-torque de duração calculada
no engate (`t_freio = v_ini / a_freio`), aberto, curto — pelo bag, ~0,26 s em
vez de 0,44 s. É redesenho da 038 e **espera o "pode" do dono**.

⚠️ **E isso põe em xeque a verificação da 038** logo abaixo, que registrou
"ré para passar: 6% → 0%". Na corrida de hoje, mesma configuração de costmap,
a ré de desencalhe disparou **3 vezes**. O alvo era outro; a tabela da 038 não
está errada para o alvo dela, mas **não generaliza** — e a porta é o caso que
importa.

### ▶️ O PRÓXIMO PASSO, e ele precisa do dono na tela

1. **Uma corrida no Gazebo com `k_lat=1.0`**, mesmo goal pela porta. A régua já
   está no CSV do `path_follower` (coluna `desvio_lateral` nova) — alvo: sair
   dos 0,110 m comidos para < 0,02 m, sem ré nenhuma;
2. se passar, o "pode" para o redesenho do freio linear;
3. só depois o robô ligado, e lá o `k_lat` vai **0.0** na primeira corrida.

### 🧹 Higiene

Um **web órfão de ontem 16:25** segurava a porta 5000, publicando `/web_vel` na
prioridade 50 desde a sessão passada. ⚠️ **O grep da receita de limpeza não
pega ele** — os args são só `.venv/bin/python app.py`. Terceira vez que órfão
custa tempo (07-31, 13-08, hoje).

---

## 🚀 13-08 (NO ROBÔ) — MAPA PRÓPRIO, AMCL TRAVADO, ELE ATRAVESSOU A PORTA PELO WEB

> Sessão inteira com o robô ligado, no andar 3 do estágio. Decisões **028**
> (mapa próprio) e **029** (inflação). Dados em `~/dados/13-08-*` no NUC;
> entrada 13-08 do diário. Bateria acabou duas vezes no fim.

🟢 **O PASSO 6 FECHOU, E O PASSO 7 ACONTECEU**: o robô saiu da sala **sozinho,
pela porta**, contra mapa próprio, localizado por AMCL, com o destino clicado
numa página web. Palavras do dono: *"FOI LINDO ELE ATRAVESSOU A PORTA E TUDO"*.

## 🚦 ESTADO EM 13-08 (dev + Gazebo) — A BATIDA ACABOU, A RÉ ACABOU, SOBROU O S

> Sessão inteira no simulador com o dono na tela. Decisão **038**. **Nada foi ao
> robô** — a sessão terminou com ele indo para a máquina de verdade.

### 🟢 O que ficou de pé, e é medido

| | antes | depois | evidência |
|---|---|---|---|
| **bater na porta** | folga 0,20 m (encostou) | 0,35 m | freio linear: sobra +0,107 → +0,031 m |
| **ré para passar** | 6% das amostras | **0%** | ganho da cadeia 0,45 |
| **corte do reflexo** | 12% | **0%** | ele nem chega perto |
| **caminho / reta** | 1,40x | **1,21x** | curva suave voltou a existir |
| tempo até a porta | 39,5 s | 30,8 s | mesmo alvo (6,24 · 3,51) |

As duas mudanças fazem coisas diferentes e as duas ficam: o **freio linear** é
rede de segurança (não atuou na corrida boa, e é assim que tem de ser); o
**ganho 0,45** é a causa da ré, corrigida.

### 🔴 O que está aberto, e é o próximo assunto

**O S na reta.** A guinada desperdiçada dobrou (26,9 → 53,5°/m) quando o robô
passou a obedecer ao que se pede. Duas tentativas falharam e estão registradas
na 038 com número: mira do seguidor (não mexeu na reta e amoleceu a curva) e
tolerância de rumo (não chegou ao alvo, reflexo cortou 77%).

⚠️ **A restrição que as duas descobriram**: reta e porta puxam para lados
opostos com um knob GLOBAL. A próxima tentativa (`a_dec` 0,3 → 0,10) não foi
rodada, e o YAML ficou em 0,3 de propósito.

### ⚠️ O QUE MUDA NO ROBÔ REAL, e é para ler antes de ligar ele

```
freio linear     LIGADO por padrão, e NUNCA rodou na máquina de verdade
ganho da cadeia  NEUTRO (1,0) no robô — o 0,45 é default só com sim:=true
a_dec, mira      inalterados; o que tentei e falhou vive só no perfil do SIM
```

A física do freio foi medida NO ROBÔ (retenção de 0,52 s, 04-08), mas os
parâmetros dele saíram do Gazebo. **Primeira coisa lá**: `freio_linear.py` num
trecho livre, comparando a sobra; e `ganho_de_giro.py` para saber se o ganho de
0,45 existe fora do simulador. Se o ganho real for ~1, a ré na porta tem outra
causa no robô e a parte 2 da 038 vale só para o simulador.

## 🧭 ESTADO REAL EM 14-08, FIM DO DIA — leia isto antes de qualquer coisa

> Três levas de trabalho, decisões **030 a 037**. **Nada foi ao robô.** O dia
> terminou com o robô batendo no simulador e o dono encerrando a sessão.

### 🔴 O que está QUEBRADO, e é onde recomeça

**1. O robô bate, e o freio que existe é do eixo errado.**

O freio de GIRO foi medido e implementado (037): corta o giro de 63,8° para
14–28°. Mas a colisão que encerrou o dia foi **LINEAR** — o robô ganha
velocidade e entra na parede — e nessa corrida o freio marcou **zero atuações**.

```
freio de giro      🟢 medido, implementado, ligado
freio LINEAR       🔴 NÃO EXISTE — e é o que estava batendo
```

A física é a mesma nos dois eixos: a placa segura a saída cheia 0,52 s depois do
corte, então zerar o comando não freia nem giro nem linha. O mecanismo do
conserto é idêntico ao do giro — contra-torque, a mesma bancada
(`tools/banco/freio_de_giro.py`), a mesma forma de medir.

⚠️ E isto reenquadra o dia inteiro: quase tudo que mexi no polígono do reflexo
em 14-08 era **compensar com geometria a falta de freio**. Aumentar a caixa
comprava segurança e vetava manobra; diminuir destravava manobra e expunha a
quina. **Geometria não para inércia.**

**2. A premissa não verificada, apontada pelo dono:**

> *"não faz sentido ele piorar no Gazebo, fazer na vida real o que no Gazebo ele
> não faz"*

O robô **atravessou a porta no prédio** em 13-08. Quando a mesma configuração
falhou no simulador, eu tratei como defeito a consertar em vez de estranhar o
simulador. **Tudo que foi sintonizado depois disso está apoiado nessa premissa,
e ela não foi verificada.** Qualquer retomada devia começar por aí.

**3. O salto de pose no mapa**, visto pelo dono e ainda não medido. O requisito,
nas palavras dele: a pose só anda quando as rodas andam e na proporção delas; o
casamento do lidar corrige erro pequeno, não teleporta. Hoje a odometria de roda
**não entra na pose de jeito nenhum**.

### 🟢 O que ficou de pé, e é real

| | evidência |
|---|---|
| **freio de giro** (037) | 63,8° → 14–28° na bancada; perfil bate com o robô real de 06-08 |
| **reflexo em todos os lados** (036) | parou em vez de bater, na tela, depois da 1ª batida |
| **pivô acima de 80°, um pulso** (036) | fechou −149° com 3,4° de resíduo |
| **árvore que não desiste** (035) | estrutura do robô 1; corrida completa **não medida** |
| **planejador com gradiente** (032) | 3 corridas atravessaram, folga mínima 0,35 m |
| **ré só com objetivo vivo** (031) | 4 testes por mutação |
| **fidelidade do simulador no GIRO** | 3,5°/60,3°/1,5 s contra 3°/56–68°/1,9–2,2 s do robô |

**Testes**: 267 em `robot_motion`, 75 `robot_base`, 12 `robot_planning`, 89
`tools`. Ferramentas novas: `tools/banco/freio_de_giro.py` e
`tools/banco/corrida_com_plano.py`.

### O que eu erraria de novo se ninguém anotasse

1. **instrumento errado dá número plausível** — quatro varreduras inválidas no
   freio, duas delas mostradas ao dono como resultado. Raiz das duas primeiras:
   nó de bancada carimbando relógio de PAREDE num mundo em tempo de SIMULAÇÃO.
   Comando descartado por velho, robô parado, **nenhum erro na tela**;
2. **medir o mecanismo antes de trocar parâmetro** — o freio só saiu do lugar
   quando gravei o PERFIL do `wz` (pico 0,65 s depois do corte) em vez de
   continuar varrendo limiar;
3. **dívida anotada não é dívida coberta** — a quina desprotegida estava escrita
   na própria decisão 033 e eu segui assim mesmo. O robô bateu por ela;
4. **limpeza incompleta derruba a sessão seguinte** — três vezes no mesmo dia,
   a última com quatro `robot_state_publisher` órfãos impedindo o Gazebo de
   subir. Sintoma para o dono: "mando o ponto e nada acontece".

## 🚩 COMEÇA AQUI — 14-08 (2ª leva) parou no meio, e a ordem já está escrita

> Sessão de dev + Gazebo, com o dono na tela. Decisões **035** e **036**.
> Nada disso foi ao robô real. O dono ficou sem tokens no meio; esta seção é o
> ponto de retomada.

### 🔴 O QUE ELE QUER FAZER ASSIM QUE VOLTAR, na ordem dele

**1. O SALTO DE POSE NO MAPA — é o item que interrompeu tudo.**

Palavras dele: *"ele tá enlouquecendo no mapa aqui, isso aconteceu com o robô 1
uma vez, resolvemos falando que a posição do robô tem que confiar nas rodas ao
ponto de não poder dar esses saltos absurdos no mapa por conta do lidar querer
casar com outro ponto do mapa... o robô só mexe no mapa quando as rodas mexem,
sempre fazendo sentido com a quantidade mexida também, aí passa pra odometria do
lidar ajustar os pequenos erros só"*.

O requisito, na forma dele:

```
a pose no mapa SÓ anda quando as rodas andam, e na PROPORÇÃO do que elas andaram
o casamento do lidar entra para corrigir erro PEQUENO, não para teleportar
```

⚠️ Ele mesmo desconfia que aqui a causa é outra que a do robô 1 — anotar como
hipótese, não como diagnóstico. **Medir antes**: gravar `/Odometry` (FAST-LIO),
a TF `map→odom` do AMCL e a odometria de roda juntas, e ver QUEM salta. Os
suspeitos são três e o instrumento separa os três:

- o AMCL casando a fatia 2D com o lugar errado do mapa (o mais provável, e é
  onde a fila de desejos já quer 3D — ver a fila abaixo);
- o FAST-LIO perdendo referência;
- a odometria de roda, que hoje **não entra na pose de jeito nenhum**.

**2. Repetir o ensaio de giro na sala, mais controlado** (pedido explícito),
depois de a pose parar de saltar — giro medido contra pose que salta não vale.

**3. O freio de malha fechada** (projetado em `docs/dados/2026-08-14-freio-de-giro/`,
não implementado).

### O que ficou PRONTO nesta leva

| | estado |
|---|---|
| **reflexo protege todos os lados** (036) | 🟢 verificado na tela: parou em vez de bater |
| **pivô acima de 80°, um pulso** (036) | 🟢 fechou −149° com 3,4° de resíduo |
| **árvore que não desiste** (035) | 🟡 sobe e ativa; **corrida completa não medida** |
| **freio de giro** | 🔵 medido e projetado, **não implementado** |

### 🔴 A batida, e ela é minha

Com o dono olhando, o robô **bateu**. Causa: na 033 eu encolhi a caixa do
reflexo e a meia-diagonal do corpo (0,314) ficou fora da lateral protegida
(0,26) — andando reto não aparece, girando a quina sai da caixa. E foi girar
perto de parede que eu destravei na mesma decisão. Consertado na 036, verificado
na tela. **A dívida estava escrita na própria 033 e eu segui assim mesmo** —
dívida anotada não é dívida coberta.

### Regras de trabalho novas (dele, 14-08)

- **o Gazebo só sobe com ele olhando** — nada de rodar corridas em lote sozinho;
- **RViz não; o serviço web** (`controle_web`, `:5000`, `WEB_TELEOP=on
  ROBOT_MODE=nav2`). Nesta máquina de dev o Flask vive num venv com
  `--system-site-packages` (o ROS precisa aparecer lá dentro).

## 🟢 14-08 (DEV + GAZEBO) — ELE ATRAVESSA, TRÊS VEZES, SEM ENCOSTAR EM NADA

> Dia inteiro sem robô (carregando). Decisões **030 a 034**. Dados em
> `docs/dados/2026-08-14-sim-porta/`. Entrada 14-08 do diário — leia, ela é
> feita dos meus erros e eles têm valor de método.

O objetivo que o dono mandou e que travava, **(6,24 · 3,51)**, agora fecha:

```
                            chegou   tempo    cam/reta   reflexo   folga min
como estava de manhã          não      —        1,60x      18%       0,35 m
corridas 1 · 2 · 3            SIM   40,4/40,0/39,7 s ~1,40x  12%     0,35 m
regressão, alvo curto 13-08   SIM     7,0 s     1,00x       0%       0,70 m
```

**Folga mínima 0,35 m** contra 0,2275 de meia-largura do corpo: ele não encostou
em nada nas três — é o critério de 13-08, e ele está cumprido no simulador.

### As três decisões, e elas só funcionam JUNTAS

| | o que era | o que é |
|---|---|---|
| **032** planejador | raio 0,32 · inflação 0,20 | raio **0,26** · inflação **0,90** |
| **033** reflexo | uma caixa reta de 0,57 m | projeção que acompanha o arco + caixa pequena |
| **034** árvore | falha do planner mata a missão | falha de um ciclo é tolerada |

🔴 **O achado que explica o "colar na parede"**: o planejador é o **Theta\***
(o Smac está aposentado desde 05-08) e ele **não tem footprint** — bloqueia
célula com custo > 252. Com `inflation_radius <= raio inscrito` **não existe
faixa graduada**: célula perto é 253, longe é 0, nada no meio. A paisagem de
custo é binária e o caminho raspa a borda do proibido. "Ir pelo meio" nunca foi
uma preferência que existisse.

🔴 **O achado que explica o travar-e-dar-ré**: a caixa estática de 0,57 m é cega
para direção, e como a 023 tirou o pivô **toda curva deste robô é arco**. Das
amostras vetadas pelo reflexo, `|wz|` mediano **0,51 rad/s** contra 0,25 das que
passaram, com folga parecida — ele proibia a própria manobra de contorno. O
seguidor não tinha culpa: erro de trajeto p50 de **8 mm**.

### ⚠️ O que continua ABERTO, em ordem de valor

1. **Nada disso foi ao robô.** É tudo simulador. A porta real de 13-08 continua
   sendo a pergunta, e ela começa por carregar as **duas** baterias e ler tensão;
2. **o `unstuck_supervisor` não entrou** — o pedido do dono (girar para o lado
   livre e devolver ao Nav2). Deixou de ser urgente porque o robô parou de
   encalhar. O desenho está pronto: usar o quantum de ~95° do pivô como manobra
   GROSSA de desencalhe, nunca como controle de rumo;
3. **o pivô segue desligado** (023, `limiar_pivo` 3,20 rad): toda curva é arco,
   e é por isso que o caminho dá 1,40× a reta no trecho do meio;
4. **a projeção do reflexo fica curta abaixo de ~0,47 m/s de comando**, porque a
   placa entrega 0,298 independentemente do pedido. Conserto certo: normalizar o
   comando para o patamar real antes do reflexo, preservando `v/wz`. Dívida
   escrita na 033.

### 🔧 Ferramenta nova

`tools/banco/corrida_com_plano.py` — grava a corrida COM o `/plan` e calcula
erro de trajeto e folga por amostra. Foi ele que matou a hipótese errada (eu
acusava o seguidor) e apontou o reflexo.

## 🗂️ A FILA DE DESEJOS DO DONO — nesta ordem, e a ordem é dele

> Fixada em 14-08. Nada da fila entra antes de o item 1 estar fechado; os dois
> seguintes são melhorias, não consertos, e o dono sabe disso.

```
1. NAVEGAÇÃO BOA        ele atravessa a porta sem bater, repetidamente
                        <- é o único item que bloqueia os outros

2. LOCALIZAÇÃO 3D       casar NUVEM 3D contra MAPA 3D (.pcd) em vez da fatia
                        2D contra grade; a grade 2D FICA, porque o Nav2
                        planeja em 2D e é ela que o dono clica

3. MAPA 3D NA WEB       ver a nuvem 3D no navegador, como no RViz, com o robô
                        se movendo dentro dela
```

**Por que 2 faz sentido** (concordado em 14-08, e o argumento bom não é "o
sensor foi caro"): casamento 2D em corredor reto é **degenerado** — as paredes
laterais fixam a posição transversal e o rumo, e não fixam nada ao longo do
eixo; batente, quina e teto fixam. O próximo passo do roteiro é sair da sala.
Some-se a isso que a fatia usa 0,15–1,00 m de um sensor que varre −7° a +52°, e
que o AMCL casa com `max_beams: 60` de uma nuvem de 20 000 pontos por quadro.

**O 3 é barato porque o mapa é estático**: nuvem sub-amostrada convertida uma
vez (`.pcd` → binário compacto), servida como arquivo, desenhada com Three.js
no navegador; ao vivo trafega só a **pose**, que a página já recebe hoje. Não
precisa de rosbridge nem de streaming de nuvem. O cuidado é o tamanho: o
FAST-LIO grava com `interval: -1` (todos os quadros num arquivo só), então o
`.pcd` cru pode ter centenas de MB e **tem de ser sub-amostrado por voxel antes
de chegar ao navegador**.

⚠️ O `.pcd` para os itens 2 e 3 provavelmente **já existe no NUC**: o
`mid360.yaml` do FAST-LIO está com `pcd_save_en: true`. Toda sessão que
desligou limpo deixou um. Conferir junto com os logs de 13-08.

## 🔴 O CRITÉRIO DE ACEITAÇÃO ENDURECEU (13-08): ELE NÃO PODE BATER

> *"o pior foi ele bater, ele não pode bater de jeito nenhum"* — o dono,
> fechando a sessão, depois da 2ª tentativa de porta.

O critério de 12-08 media **disparo do reflexo** contra obstáculo mapeado. Este
mede **contato**. **Colisão reprova a corrida inteira**, qualquer que seja a
explicação — bateria, mapa ou sintonia. É o critério mais duro do projeto e
está acima dos outros.

### A porta, três vezes, em ordem decrescente de qualidade

```
1ª  atravessou LIMPO, fazendo o que o planejador mandava
2ª  foi DE CARA na porta, emperrou, a ré rodou 8 s sem sair do lugar,
    o planner recusou ("start é obstáculo") e ele BATEU
3ª  de cara na porta de novo, mas curvou forte para a esquerda no fim e passou
```

⚠️ **RESSALVA DO DONO, e ela vale**: as duas últimas foram com **bateria
baixa** (o robô morreu logo depois, e o NUC caiu junto, duas vezes). *"Não dá
para levar essas duas últimas em consideração total; amanhã vou carregar ele
antes."* A tensão **não foi lida** nenhuma vez hoje — a mesma falta de 10-08.

🔴 **ACHADO NOVO, a confirmar com bateria cheia: A RÉ É INEFICAZ.** Ela foi
acionada (o conserto do seguidor funcionou), mediu 1,02 m livres atrás, comandou
recuo e:

```
fim da ré: recuou 0,01 m em 8,0 s      <- bateu o teto e andou 1 cm
```

Duas hipóteses, nenhuma testada: (a) o comando de ré está **abaixo da zona morta
da placa** — a mesma de 020, que escolhe o MÓDULO e ignora pedido pequeno; (b) o
robô estava encravado na quina. Bateria baixa contamina as duas.

➡️ **O mecanismo por trás da batida já tem nome**: ele **entra torto na porta**,
que é a dívida nº 1 (erro de trajeto do seguidor, p50 0,128 m contra 0,118 m de
margem num vão de 0,90 m). A observação do dono é a pista boa: *"na primeira foi
perfeitamente, fez o que o planner mandava, depois começou a errar o caminho do
planner"*.

🔴 **O mapa do estágio não servia, e a prova é exaustiva.** Varri TODAS as poses
possíveis da sala contra `scan_andar3_ajustado`:

```
melhor pose POSSÍVEL       41,4% dos feixes a menos de 0,15 m · mediano 0,300 m
mapa que o robô desenhou   97,7% · depois 99,7% com AMCL travado · mediano 0,000
```

Se a melhor pose possível reprova, o problema não é a pose. O `ajustado` é a
versão **limpa** da planta, e a limpeza que dobrou a folga p10 foi **apagar a
mobília** — a sala real está cheia de coisa. Decisão 028: o robô desenha o
próprio mapa (`maps/sala_andar3/`, versionado).

🔴 **A INFLAÇÃO ERA A CULPADA DO PASSEIO** (decisão 029). Mesmo alvo, mesma
pilha, única mudança:

```
                   chegou    referência    yaw     v não-nula   caminho/reta
inflação 0,50      26,1 s      83,8°      103,2°      41,9%        1,45x
inflação 0,20       8,1 s      23,4°       14,0°      87,1%        1,02x
12-08, SEM mapa     9,5 s      14,7°       10,0°        —          1,09x
```

**1,02× é reta**, e com mapa ele ficou igual à corrida sem mapa — e mais rápida.
A previsão escrita ANTES da corrida (referência caindo para 20–30°) deu 23,4°.
⚠️ Isto **reclassifica a dívida nº 2**: a inflação de 0,50 não proibia planejar
(com o robô parado o plano saía 1,00× reto) — ela **entortava** o plano com o
costmap sendo atualizado durante a marcha.

🟢 **O SERVIÇO WEB ESTÁ NO AR** e o RViz saiu de cena. Nenhum código novo: a
`controle_web/` do robô 1 já publica em `/web_vel`, o canal de prioridade 50 que
o `twist_mux` daqui declara. Só faltava operacional:

```
pip install --user --break-system-packages flask flask-socketio simple-websocket
ROBOT_MODE=nav2   (default 'teleop' — em teleop o clique-para-ir nem existe)
WEB_TELEOP=on     (sem isso o web é só visor)
http://<robô>:5000
```

🔴 **O `/scan` estava morto desde 12-08 às 20:05** — `scan_2d` levou SIGKILL de
um `pkill` de limpeza, e o launch não ressuscita. **As corridas de 12-08 à noite
rodaram sem `/scan`**: a ré da 025 esteve inerte a sessão inteira. Não invalida
"o robô anda reto", mas a rede de segurança não existia.

🔴 **E EU MATEI O SEGUIDOR.** Para atender *"só não manda ele pra trás"* expus
`re_max_seguidas` como argumento e passei `0` — teto zero cai numa guarda que
formata `dist_antes_da_re`, que só existe depois da primeira ré:

```
TypeError: unsupported format string passed to NoneType.__format__
sintoma: goal aceito, plano desenhado, robô parado, ninguém culpado no log
```

O knob certo (`re_habilitada`) **já existia**. Consertado nos dois lados, e a
launch agora expõe o interruptor certo. ➡️ **Knob novo que exercita caminho de
código nunca exercitado é mudança grande disfarçada de parâmetro.**

⏳ **A PRÓXIMA TAREFA, e ela é do dono**: *"o ideal era a ré não sair sozinha e
sim só poder ser ativada quando tiver um destino"*. Hoje o seguidor obedece
**plano**, não objetivo — foi assim que o `tools/banco/plano.py` (documentado
como "planeja sem mover o robô") fez o robô dar uma ré do nada. Amarrar a ré a
objetivo ativo mata a causa.

### 📋 AMANHÃ (14-08), NA ORDEM — decidido com o dono

1. **CARREGAR A BATERIA ANTES**, e ler a tensão no início e no fim. Hoje ela
   acabou duas vezes e derrubou o NUC junto; nenhuma leitura foi feita.
2. **Repetir a porta com o robô cheio.** A 1ª tentativa de hoje diz que ele
   consegue; as duas seguintes podem ter sido tensão. É isto que separa
   "defeito de navegação" de "robô com fome".
3. Se a **ré de 8 s para 1 cm** se repetir com bateria cheia, é a zona morta da
   placa (020) e não geometria — e a ré precisa de comando acima do piso.
4. **Amarrar a ré a objetivo ativo** (pedido do dono, item acima).
5. **O erro de trajeto do seguidor**, que é quem entorta a entrada na porta.

### 🔧 Como subir tudo (o que funcionou hoje, na ordem)

```bash
export ROS_DOMAIN_ID=42        # a pilha de 12-08 estava no domínio 0; combinar
ros2 launch robot_base base.launch.py           # confira /scan ~10 Hz DEPOIS
ros2 launch robot_motion pilha.launch.py sim:=false \
    mapa:=$PWD/maps/sala_andar3/sala_andar3.yaml localizacao:=amcl \
    pose_x:=<x> pose_y:=<y> pose_yaw:=<yaw> \
    curv_frente:=-0.9145 curv_medido_em:=2026-08-11
cd controle_web && WEB_TELEOP=on ROBOT_MODE=nav2 python3 app.py
```

⚠️ **Nunca `pkill -f`**: matou a sessão ssh duas vezes hoje e foi o que matou o
`scan_2d` ontem. Matar por PID e conferir órfãos.
⚠️ **`map_saver_cli` estoura o timeout default de 2 s** neste NUC: passe
`--ros-args -p save_map_timeout:=30.0`. O erro (`Failed to spin map
subscription`) não parece timeout.
⚠️ **`slam_toolbox` com `use_lifecycle_manager: False` NÃO se auto-ativa** no
Jazzy: nasce `unconfigured`, calado, e `/map` nunca aparece. A
`mapeia.launch.py` já sobe o `lifecycle_manager` que resolve.

### 🧭 Achar a pose inicial sem tela

O NUC não tem tela. O que funcionou foi **busca por força bruta**: transformada
de distância do mapa, varredura de (x, y, yaw) sobre as células livres com folga
≥ 0,25 m, nota = fração de feixes a menos de 0,10 m de parede (robusta ao corpo
do dono dentro da sala). Sai pose com ~99% dos feixes dentro de 0,15 m, e vai
para o AMCL por `/initialpose`.
⚠️ **A pose envelhece**: correção calculada de um scan de minutos antes foi
rejeitada pelo filtro (AMCL assentou 0,26 m ao lado). Capturar, buscar e
empurrar **sem intervalo**, com o robô parado.
🔵 Com o mapa desenhado a partir do ponto de partida de sempre, o robô **nasce
na origem do mapa** e esse problema some — foi o que o dono observou no fim.

---

## 🎯 12-08 (NOITE, NO ROBÔ) — O NAV2 DIRIGIU E CHEGOU; O REFLEXO DISPARA CONTRA O PRÓPRIO ROBÔ

> Sessão no robô real com o trabalho das decisões 020–026. Commit `308ad68`,
> bateria 41,13 V, placa a 20,8 °C. Dados em `docs/dados/2026-08-12-robo-nav2/`,
> entrada 12-08 (8ª leva) do diário.

🟢 **A PRIMEIRA NAVEGAÇÃO AUTÔNOMA DESTE ROBÔ NO CHÃO REAL.** Alvo (2,0 · 0,0)
sem mapa: **chegou a 0,054 m em ~8 s**, replanejando 7 vezes. A cadeia inteira
funcionou fora do simulador, e **a previsão da 023 passou**: o `rosout` disse
`pivô por corte FORA DO CAMINHO` e o robô **andou** em vez de girar no lugar.

```
chegada     0,054 m (raio 0,25)      tortuosidade  1,92 (3,77 m para 1,96)
raw_v       77,6% -> ~97% até a chegada          pose  10,0 Hz, pior 0,118 s
```

⚠️ O 77,6% só fica abaixo do critério porque o instrumento não encerra na
chegada: 51 dos 60 s foram robô parado no alvo. **Dívida do `corrida_nav.py`.**

🔴 **O S DE RUMO É REAL, E É DE NARIZ**: yaw +19,5° → −26,0° → +36,6°
(amplitude 63°, envoltória crescendo) com **desvio lateral de só 0,17 m**. O
dono viu a olho. Foi por bambear o nariz e não o corpo que ele chegou.

🔴 **E A CAUSA MAIS PROVÁVEL ESTÁ MEDIDA: o reflexo enxerga o próprio robô.**
Vetou **26 dos 156 comandos (17%)** em 11 rajadas, cada uma zerando o comando
por ~0,1 s dentro de uma malha com 0,94 s de tempo morto. **Com o robô parado
ele continua disparando** — 928 vezes no log:

```
0 a 2 pontos por quadro, sempre no MESMO lugar:
  x ≈ +0,04   y ≈ +0,09   z ≈ 0,42–0,47 m      raio 0,10 m do sensor
  min_points: 2  ->  dispara com exatamente 2
```

10 cm ao lado do Mid-360 e na altura dele: **peça do próprio robô**. Mesma
família da 017 (lá o ponto `(0,0,0)` era o robô, e tratou-se UM ponto só).

🔧 **O DONO ARRANCOU A PEÇA** (havia uma estrutura em volta do Mid-360), e
✅ **a decisão 027 já entrou**: o `nuvem_pontos` descarta pontos dentro de um
**raio do EIXO** do sensor (`raio_cego`, default 0,15 m), não só a origem
exata. Cilindro e não esfera — o corpo fica abaixo do lidar. Descartada a
alternativa de subir `min_points` 2 → 6: esconde o sintoma e cega o reflexo
para obstáculo pequeno de verdade. **Reflexo que dispara sempre é reflexo que
não quer dizer nada.** É reincidência da 017, que tratou UM ponto sem
generalizar. 75 testes no `robot_base` (eram 70), verificados por mutação.

⏳ **A PRÓXIMA SESSÃO COMEÇA POR ESTA PREVISÃO, E A ORDEM É O QUE VALE.** A
peça saiu DEPOIS da medida e ANTES do conserto, o que separa hardware de
software de graça:

```
1º  raio_cego:=0.0 (comportamento velho), robô PARADO
      PolygonStop cala e 0 pontos no polígono -> causa confirmada por intervenção
      continua disparando  -> a peça não era a culpada; suspeito seguinte é a
                              faixa de altura (min_height 0,10) pegando o chão
2º  default 0,15 -> repetir o passo 5: chegada SEM os 17% de vetos
3º  o S encolheu junto?  sim -> o reflexo era o excitador
                         não -> o S é do laço de rumo (família de 06-08)
```

⚠️ **Não misturar os dois numa corrida só** — foi o erro de 11-08 com o
estimador do ff, que deixou a previsão separadora sem teste possível.

🟢 **De graça nesta ida**: `/scan` real a **9,998 Hz** (pior intervalo 0,135 s,
contra o limiar 0,8 s da ré — pergunta 2 respondida, e melhor que o simulador);
vão traseiro **0,858 m** batendo com a trena; a **019 valeu no robô** (o
`tf_odom` subiu sozinho com `livox_frame`); a **018 segue de pé** (`odom` em
z = −0,060); pré-voo **20/20**.

⏳ **O que ficou aberto**: a caixa encostada atrás (o vão tem de dar 0,00,
inclusive na quina) e o homem-morto de verdade — o `[A] = 0,008 s` medido foi o
zero de saída do teleop ao morrer, não a temporização de 0,4 s. E o
`homem_morto.py` mede `[C]` com régua de mentira: lê a odometria de roda, que
está **`open_loop: true`** e vem do comando, não do encoder.

---

## 🤖 SE VOCÊ É O ASSISTENTE E O DONO ESTÁ INDO AO ROBÔ

> **Leia `docs/ROTEIRO_NAV2_NO_ROBO.md` e conduza por ele.** Está em ordem de
> risco crescente, com comando exato, critério de aceitação e o que fazer
> quando falhar. ✅ O passo 0 (puxar os 8 CSV de 11-08 do NUC) **está feito** —
> eles estão em `docs/dados/2026-08-11-estimador/`, commitados.
>
> ⚠️ **Dois consertos que o roteiro precisa**, achados rodando: o passo 3 manda
> rodar o `prova_mux.py` com a pilha de pé, e isso **dirige o robô** (~4,5 s de
> comando, ~3 m no chão); e a confirmação `grep limiar_pivo` procura no lugar
> errado (só o perfil do simulador declara o parâmetro — no robô quem responde
> é o default do nó, e quem prova é o `rosout`).
>
> ⚠️ **Feche o `robot-key` antes de qualquer corrida autônoma**: teleop vivo
> publica zero a 20 Hz em prioridade 90 e trava a autonomia, com cara de "o
> Nav2 não dirige".
>
> Os dois passos que não podem ser pulados: o **freio de mão** (passo 3) e o
> **canal que fura o reflexo** (passo 4). O passo 4 é novo e é o mais
> importante da sessão: hoje entrou um caminho que dirige o robô por fora do
> `collision_monitor`, e a medida que o torna seguro nunca viu o `/scan` real.
>
> Mapa da sessão: `maps/andar3/scan_andar3_ajustado.yaml` (do estágio,
> escolhido por medida — ver `maps/andar3/README.md`).

## 🧭 HANDOFF — LEIA ISTO PRIMEIRO (12-08, fim do dia)

> **Para o assistente que chegar frio.** Estado real, sem enfeite. O trabalho do
> dia está na branch **`slam-meu-mapa`** (não foi para a `main`, e a `main`
> também tem 2 commits não empurrados).

### Onde está cada coisa

```
main            020 (zona morta medida) + corrida_nav/roteiro   [2 commits locais]
slam-meu-mapa   021 (fatia 2D) + 022 (AMCL) + 023 (pivô fora)   [branch de hoje]
```

**405 testes verdes**: `robot_motion` 234 · `robot_base` 70 · `robot_planning` 12
· `tools` 89. Rodar SEMPRE por pacote (`python3 -m pytest ros2_packages/robot_motion`);
por arquivo dá `ModuleNotFoundError`, é dívida antiga.

### 🟢 O que FUNCIONA e está medido

| | evidência |
|---|---|
| **o seguidor dirige a pilha inteira** | **29-07: alvo pela porta de 0,90 m em 12,3 s, 1,04× a reta** |
| **e passa no meio dela** | **05-08: 2509 → 0 invasões, folga 0,449 m de 0,45 teórico** |
| mapa do prédio → mundo do Gazebo | `gz sim` carrega, `walls` vivo |
| fatia 2D da nuvem (`/scan`) | 360 feixes, 0 fora do mapa |
| **scan × mapa** | **100% dentro de 0,15 m, mediano 0,050 m** = 1 célula |
| AMCL contra o mapa | `active`, `map→odom` com correção nula |
| Nav2 planejando | 285 pontos, 15,3 m até um alvo de 13 m |

> ⚠️ **O handoff de 12-08 dizia que o robô "nunca percorreu o plano, nem no
> simulador". Era falso**, e as duas linhas de cima são do próprio diário deste
> repo. A conclusão vinha de uma leitura errada (ver 023): a mediana de erro de
> rumo "zero" era o trecho MORTO do CSV, depois de o seguidor já ter parado —
> 847 zeros para 847 amostras congeladas, batendo na unidade.

### 🔴 O QUE ESTAVA QUEBRADO — e o culpado tem nome (decisão 023)

**O pivô por corte girava o robô no lugar para sempre.** Nas quatro corridas de
12-08: 1321° de giro para 0,20 m de deslocamento, 28 disparos com erro entre 35°
e 81°, e **93–101° de varredura depois do corte** contra 25–32° previstos.

A placa entrega **um único módulo de giro** (2,204 rad/s para qualquer pedido de
0,10 a 1,0) e depois do corte **segura a saída cheia por 0,52 s** — o robô ainda
acelera. A sobra real não depende de quando se corta, então o critério de corte
não é alavanca, e **nenhum `a_dec` salva** (varrido de 0,60 a 0,05).

➡️ **O pivô saiu do caminho do seguidor.** Quem responde por rumo é a lei
contínua da 005, que contra a MESMA placa assenta em 0,5–0,6° de 20° a 180°,
andando 97–99% do tempo. Arco é RAZÃO e a placa preserva razão; pivô é MÓDULO e
é justamente o que ela destrói.

**Quando quebrou**: `9f00854` (a porta funcionando) é **ancestral** de `6aa347a`
(a retenção da placa entrando no simulador, 05-08). A aceitação daquela mesma
leva já dizia *"o pivô NÃO passou"* — e ninguém ligou isso à navegação, porque
entre 05-08 e 12-08 a corrente não voltou a rodar ponta a ponta.

### 🟢 E A CORRIDA ACONTECEU — a 023 passou na máquina

```
alvo (2,0 · 7,2)          12-08 manhã        depois da 023
raw_v não-nulo             1 / 1500         176 / 191  (92,1%)
distância                  2,20 -> 2,34 m   2,20 -> 0,07 m
giro total                 1321°            141°
chegou                     nunca            9,5 s
```

Previsão da 023 (>80% de `raw_v`, distância monótona): **passou nas duas
pontas**, pior recuo 0,038 m, tortuosidade 1,09.

### 🛡️ E O REFLEXO APARECEU PELA PRIMEIRA VEZ

Alvo (6,0 · 1,5), pela porta de 0,90 m: `Robot to stop due to PolygonStop
polygon`. O `ESTADO` listava o `collision_monitor` como *"configurado, NUNCA
visto parando o robô"* — agora está medido, e **ele não bateu**.

### 🟢 E ELE ATRAVESSA A PORTA — decisões 024 e 025

```
alvo (6,0 · 1,5), pela porta de 0,90 m   chegou    dist mín   rés
recuperação inalcançável (pré-024)         não      2,49 m      0
ré alcançável, vetada pelo reflexo         não      2,46 m      0
ré furando o bloqueio, sem teto           65,8 s    0,07 m      9
ré com teto e relógio medido              29,9 s    0,08 m      1
```

Tortuosidade 3,09 → **1,30**, replanejamentos 59 → 28.

Três defeitos em série, cada um escondendo o seguinte:

1. **023** — o pivô por corte girava o robô no lugar para sempre (a placa não
   entrega módulo; ela preserva razão);
2. **024** — a ré existia e era **inalcançável por ordem de linhas**: a guarda
   de plano velho retornava antes da checagem de progresso, e o plano vence
   justamente quando o robô trava;
3. **025** — a ré era **vetada pelo reflexo** (831/831), porque o `PolygonStop`
   é cego para direção. Canal `unstuck_vel` (prioridade 30: acima da autonomia,
   **abaixo do humano**) fura o bloqueio, e só depois de medir o vão traseiro
   em metros num corredor **retangular** da largura do robô.

⚠️ **A primeira 025 fabricou uma fuga** — 9 rés seguidas, 2,50 → 5,10 m de
distância, de costas até acabar o espaço. `re_parado_s` (1,5 s) era menor que a
própria manobra (1,6–2,8 s). Agora 4,0 s, mais `re_max_seguidas=2`, que não
depende de sintonia.

### 🎯 A PRÓXIMA TAREFA — o critério é do dono, e ele é falsificável

> **O reflexo só pode disparar por SURPRESA** — uma pessoa, um obstáculo móvel.
> O que é parado e está no mapa tem de ser desviado ANTES, pela navegação.
> Reflexo disparando contra obstáculo mapeado é falha de navegação, por
> definição.

A ré da 025 é **band-aid** e fica sabendo disso: ela é a rede para a surpresa,
não o conserto de entrar torto. E o culpado já está medido — **é o seguidor,
não o planner**:

```
porta: parede em x=4,0 · vão de y=2,05 a 2,95 · CENTRO 2,50 · corpo 0,63 m

o PLANO do Theta*      cruza em y=2,483   (−0,017 m do centro)   sobra +0,118 m/lado
o CAMINHO REALIZADO    cruza em y=2,401   (−0,099 m do centro)   sobra +0,036 m/lado
```

**O plano está praticamente perfeito** (1,7 cm do centro de um vão de 90 cm). O
seguidor perde **8 cm** e come **2/3 da margem** — sobram 3,6 cm por lado, e é
por isso que o reflexo dispara contra uma parede que está no mapa.

➡️ O alvo da próxima leva é o **erro de trajeto (cross-track) do seguidor**, e
ele é geometria, não sintonia de rumo. Hipóteses na ordem em que eu atacaria:

1. **A lei é só de RUMO.** `rumo_para(x, y, carrot)` mira a cenoura e corrige
   desvio lateral só de forma implícita, com atraso. Pure pursuit tem desvio
   permanente conhecido em curva, e a entrada da porta é logo depois de uma.
2. **Sem pivô (023) ele arca para dentro** em vez de se esquadrejar. Em 05-08
   ele passou *"perfeitamente no meio"* — e tinha o pivô.
3. **`lookahead` 0,37 m** foi escolhido em 05-08 contra um seguidor que
   pivotava. Mira curta com arco puro pode ser outra coisa.

**A régua já existe**: `folga.py` mede contra o MAPA (o sensor tem 2 m de zona
cega e diria "livre" onde o robô raspa), e o `corrida_nav.py` já grava quando o
reflexo agiu. Corrida que dispara o reflexo contra obstáculo mapeado reprova.

### 🔴 O QUE VAI MORDER NO ROBÔ — planejar no `meu_mapa` é MARGINAL

Medido em 12-08 e **quase perdido**: eu derrubei este aviso do handoff ao
reescrevê-lo em 12-08 (4ª leva), e ele voltou porque é o que decide a sessão
no robô.

```
mediana de folga das células livres do meu_mapa   0,35 m
robot_radius configurado                          0,32 m
inflation_radius 0,50  ->  0 de 8 pontos de partida planejam
inflation_radius 0,20  ->  4 de 8
```

A `inflation_radius: 0,50` da produção foi escolhida em 05-08 **para a porta de
0,90 m da pista simulada**, e ela é o que centra o robô no vão. No corredor
real ela **proíbe planejar**. Decisão em aberto, e o roteiro
`docs/ROTEIRO_NAV2_NO_ROBO.md` a trata como passo com número.

### ⚠️ Duas dívidas abertas nesta sessão

- **Ele entra torto na porta**: 0,16 m fora do centro de um vão de 0,90 m com
  corpo de 0,63 m. Custo da 023 (sem pivô ele arca para dentro). Leva própria,
  com o `folga.py` de régua — já decidido pelo dono.
- **`pior intervalo de /Odometry 0,336 s`** com o robô andando. Pode ser carga
  da máquina de dev; não vale conclusão sem repetir.


### O que o dono está otimizando, e o que ele já decidiu

**O critério, nas palavras dele (12-08):** *"o reflexo só deve parar em
momentos que sejam surpresas, uma pessoa, um obstáculo móvel; o que é parado
deve ser desviado previamente pela navegação"*. Reflexo disparando contra
obstáculo que está no MAPA é falha de navegação, por definição. **Este é o
critério de aceitação do projeto agora**, e ele é falsificável: `folga.py` mede
contra o mapa e `corrida_nav.py` registra quando o reflexo agiu.

**A ré da 025 é BAND-AID e ele sabe.** Palavras dele: *"ele dar essa ré em
específico é um band-aid, pois desde o início ele está indo errado até a porta
e precisando dar a ré para não bater; a ré ainda fica viva para esses casos"*.
Não a venda como solução — ela é a rede para a surpresa.

**Decisões já tomadas por ele nesta sessão, não reabra sem motivo:**

- o canal de desencalhe fica em prioridade **30** — acima da autonomia, abaixo
  do humano (teclado 90, web 50);
- o reflexo **da frente** continua intocado; só o recuo fura;
- a recuperação vive **dentro do seguidor**, não em nó novo;
- a entrada torta na porta é **leva separada, com régua** (`folga.py`), e não
  se conserta junto com outra coisa.

### Como esta sessão de fato andou (e o que isso ensina)

**Três observações do dono olhando a tela destravaram a sessão inteira**, e
nenhuma delas estava em CSV nenhum:

```
"está parado agora, tem que ter uma maneira dele sair daí sozinho"  -> 024
"ele ativou de novo a ré mesmo estando reto na porta"               -> o teto da 025
"mandei comandos e ele não fez nada"                                -> era o botão errado do RViz
```

➡️ **Peça o que ele viu.** O CSV diz o que aconteceu; ele diz o que pareceu, e
foi o "pareceu" que apontou o mecanismo nas três vezes.

**E a ideia dele de estratégia estava certa duas vezes**: o furo no bloqueio
(*"o recuo deve ser um furo do bloqueio, ele vê se não tem nada atrás"*) veio
dele, e é o desenho que funcionou. Quando ele descrever um comportamento,
traduza para o que a máquina permite em vez de descartar — a ideia de "diminuir
a velocidade antes da porta" é impossível (a placa escolhe o módulo, decisão
020), mas o miolo dela — chegar já apontado — virou a 026.

### O estado da máquina de dev quando esta sessão fechou

- branch **`slam-meu-mapa`**, empurrada, 21 commits à frente da `main`;
- a `main` tem **2 commits locais não empurrados** (anteriores a hoje);
- **nada foi mesclado na `main`** — a decisão de merge está em aberto;
- havia uma pilha do Gazebo de pé na máquina de dev; se ainda estiver, derrube
  antes de qualquer medida (`ps`, matar por PID, nunca `pkill -f`).

### As dívidas abertas, em ordem de valor medido

1. **O erro de trajeto do seguidor** (p50 0,128 m contra 0,118 m de margem num
   vão de 0,90 m). É o termo dominante. Suspeitas na ordem: a lei é só de rumo
   e não tem termo de desvio lateral; o `lookahead` de 0,37 m foi escolhido
   para um seguidor que pivotava; sem pivô ele arca para dentro do vão.
2. **A inflação no mapa real** — 0,50 é da porta simulada e pode proibir
   planejar. Passo 6 do roteiro.
3. **A velocidade não é comandável** (020): a placa escolhe o módulo, então
   toda a lei de frear na curva é inerte. Reabre o dia que alguém mexer no
   `deadband_enable`.
4. **O objetivo abortado não volta sozinho**: quando o `bt_navigator` desiste,
   ninguém re-manda. Hoje quem manda é o operador.

---

## 🚦 12-08 (2ª leva) — O ROBÔ ESTÁ PRONTO PARA ANDAR SOZINHO (dev)

**344 testes verdes** (eram 337), sete novos, três verificados por mutação.

🟢 **O que já está de pé no robô**: TF fechada sozinha (019), quatro servidores
do Nav2 ativos, percepção marcando, robô andando reto, pré-voo 20/20.

🔴 **O que nunca aconteceu: o robô receber um objetivo e ir.** A corrente
inteira, com o robô andando, nunca rodou — só no Gazebo, que é otimista.

🔴 **As camadas de segurança, no placar honesto:**

```
twist_mux (humano fura a autonomia)   provado sem robô
reflexo (collision_monitor)           configurado, NUNCA visto parando o robô
homem-morto (teclado)                 em 06-08 não publicou NADA em /key_vel
```

O `bin/robot-key` já foi consertado (11-08) e o teleop ganhou três contadores no
`rosout` que separam "não lê a tecla" de "lê e não publica" de "publica e
ninguém escuta". É diagnóstico de cinco minutos de bancada.

🧰 **O instrumento que faltava**: `tools/banco/corrida_nav.py` (ROS) +
`leitura_nav.py` (puro, testado). Manda um objetivo pela **ação** (para poder
cancelar ao sair) e grava a corrente inteira a 20 Hz. ⚠️ Grava as **duas
pontas** do reflexo (`/auto_vel_raw` × `/auto_vel`) porque parar por reflexo e
parar por ter chegado se parecem de fora — e delata corrida em que o humano
meteu a mão, que não prova autonomia.

📋 **`docs/ROTEIRO_NAVEGACAO_NO_ROBO.md`** — a sessão em ordem de risco
crescente: homem-morto → ff do dia → objetivo livre → objetivo com caixa. O robô
só anda sozinho depois do freio de mão provado.

🔵 **A pergunta do CPU do `nuvem_pontos` virou linha medida**, não bloqueio: o
dono decidiu que ele fica vivo na navegação, e o instrumento mede o pior
intervalo de `/Odometry` **durante a corrida** (em 11-08 a conferência foi com o
robô parado). Se engasgar, a saída arquivada é o driver publicar `PointCloud2`
em C++ (`xfer_format = 0`), apagando a ponte Python.

---

## 🎯 12-08 — O PERFIL DO ROBÔ REAL ERA ANTERIOR À BANCADA (dev)

Decisão **020**, segunda leva de navegação. **337 testes verdes** (eram 333),
quatro novos verificados por mutação.

🔴 **`config/movimentacao.yaml` — o perfil que roda em TODA navegação no robô —
abria com "NENHUM DESTES NÚMEROS FOI MEDIDO NESTE ROBÔ AINDA"**, doze dias
depois de 07-31 ter medido. A `bitola` daquela leva foi propagada (por causa do
defeito de 29-07); a zona morta ficou para trás com um chute **8× maior**.

```
                        chute (0,15)      medido 07-31 (0,0178)
wz para pivotar          1,48 rad/s          0,50 rad/s     (teto 1,00)
piso de linear           0,335 m/s           0,203 m/s
raio de chegada mínimo   0,187 m             0,069 m
```

🔴 **Isso não ficava parado no arquivo — ABRIA AS CURVAS.** O driver escala as
duas rodas **juntas** (`k = deadband_speed/mx`), o que preserva a **razão** entre
elas e destrói o módulo: **`cmd_vel` escolhe o RAIO, e a velocidade quem escolhe
é a placa**. A lei da zona morta "escapava" da banda acelerando — e acelerar
muda a razão:

```
pedido (v · wz)   R pedido    com 0,15            com 0,0178
(0,10 · 0,30)      0,333 m    (0,240·0,30) 0,802   (0,108·0,30) 0,361
```

A primeira linha é a corrida de 07-31: o robô entregou **0,333 m** de raio para
esse pedido. A lei mandava **0,802 m** para ele — **2,4× mais aberto**. Em porta
e corredor é a diferença entre passar e raspar.

✅ **O "pivô indisponível" era fantasma**, e tinha escapado do código para a
documentação: estava na lista da próxima ida ao robô como defeito de máquina
(item 4). A máquina gira 147° com `wz=0,30` desde 07-31.

🔧 **O acoplamento que ninguém travava**: `path_follower.py` tinha `v_piso:
0.335` copiado à mão sob o comentário *"TEM QUE BATER com o que a movimentação
calcula"* — e nada conferia. Foi por essa fresta que os dois arquivos andaram
separados. Agora há teste lendo os dois arquivos.

⚠️ **A zona morta de 0,0178 SUPÕE `deadband_enable=true`.** Desligada — como o
banco precisa fazer para caracterizar — a zona morta real (0,25–0,50 m/s) volta
e o piso fica perigosamente baixo. O par está travado em teste: quem desligar a
compensação derruba a suíte.

⚠️ **O que isto NÃO resolve**: a velocidade segue não-comandável. Toda a lei de
velocidade do seguidor (frear na curva, frear perto do objetivo) é **inerte** —
o robô percorre qualquer arco a ~0,30 m/s (medido ontem: 0,305 · 0,292 · 0,298
com `cmd_v=0,25`) e chega no ponto nessa velocidade. Dívida aberta.

⏳ **Não foi ao robô.** Confirmação barata no pré-voo: a subida do
`heading_controller` tem de dizer `pivô DISPONÍVEL acima de 0,13 rad/s`.

🔵 **`a_dec` fica em 0,3 de propósito**, contra os ~3,05 rad/s² medidos — mexer
na frenagem de rumo mexe no S que acabou de assentar em 11-08, e isso merece
leva própria com corrida de controle. Agora está anotado no arquivo como escolha,
não como esquecimento.

---

## 🧭 11-08 (4ª leva) — A NAVEGAÇÃO COMEÇA TIRANDO AS TRAVAS MANUAIS (dev)

Decisão **019**, primeira leva do ritmo acelerado que o dono pediu para a
navegação. **333 testes verdes** (eram 327).

🔴 **A pré-condição de tudo subia quebrada há três sessões.** A
`localizacao.launch.py` não passava `frame_da_pose` ao `tf_odom` → ele caía no
`child_frame_id` do FAST-LIO (`body`, frame da IMU, ausente do URDF) → não
publicava TF → **os quatro servidores do Nav2 não ativam**. Em 08-10 e 11-08
alguém matou o nó e subiu na mão.

🔴 **E `mapa:=nenhum` era obrigatório no robô e opcional na sintaxe** — esquecer
punha o mapa da pista simulada no robô real, misturado com marcação do Livox.

```
localizacao.launch.py   frame_da_pose   '' -> 'livox_frame'
pilha.launch.py         mapa            pista -> 'nenhum' quando sim:=false
pilha.launch.py         rviz            true  -> false    quando sim:=false
```

➡️ **A regra registrada: o default é o caso seguro; o perigoso exige intenção.**
Mesma família do defeito da bitola (29-07) e do `ff HERDADO` (07-08).

🔧 **E o teste desta leva nasceu errado**: a primeira versão **passou com o
condicional invertido na launch**, porque montava a expressão dentro do próprio
teste em vez de ler a do arquivo. A mutação pegou. A versão final lê o
`default_value` por AST e resolve num `LaunchContext`. **Teste que reconstrói o
alvo não testa o alvo** — parente exato do buraco da 017.

⏳ **Não foi ao robô**; confirma-se no pré-voo: a base sozinha deixa a TF
`odom → base_link` de pé, e a pilha sem argumento nenhum sobe sem `map_server`.

---

## 🏁 11-08 (3ª leva) — O ROBÔ ANDA RETO, e o estimador NÃO é o motivo (robô)

13 corridas na cerâmica da sala, bateria 40,17 → 40,65 V. Dados e condições em
`docs/dados/2026-08-11-estimador/ambiente.txt`; entrada 11-08 (3ª leva) do
diário. ⚠️ **Os 8 CSV de 2,5 m ainda estão no NUC** — ele desligou antes do
`scp`. Puxar ANTES do próximo deploy (`git clean -fd` no NUC os apagaria).

🟢 **A PERCEPÇÃO ACORDOU — 017 e 018 funcionam no robô real.** Pré-voo com o
robô parado: **20/20** (o ❌ que apareceu é o falso positivo do `bash -c` do ssh
contado como segunda pilha).

```
                       10-08            11-08
local_costmap        0 células      138 células letais (0,35 m²)
global_costmap       0 células      504 células letais (1,26 m²)
nuvem /livox/pontos     —           10,3 Hz · 10 661 pontos · 100% transformável
odom → base_link     z = −0,477 m   z = −0,060 m
```

⚠️ A previsão que **separava** as duas ficou sem teste: elas entraram juntas no
mesmo deploy. Falsificá-la exigiria reverter a 018 e rodar de novo.

🔴 **O ESTIMADOR DO ff (016) NÃO ESTÁ PROVADO — e a previsão dele PASSOU.**

```
na ordem do tempo →   controle (adapta OFF)    8,9°   21,1°   18,4°
                      adapta ON               10,3°   10,1°    8,2°
                      VOLTA ao controle OFF    8,1°    6,2°
```

A monotonicidade pedida (10,3 > 10,1 > 8,2) apareceu. Quem derrubou foi o
**A-B-A decidido na bancada**: sem estimador, logo depois, o robô deu 8,1° e
6,2° — as melhores do dia. **A melhora era da ordem temporal, não do mecanismo.**

➡️ **Lição de método, e ela vale para o artigo**: previsão falsificável sobre
uma sequência **não** protege contra confundimento de ordem quando a planta
deriva (19% em 5 min, medido em 10-08). Só o retorno à condição de controle
protege. Custou 2 corridas.

✅ **O mecanismo age**: `curv_hat` andou −0,8275 → −0,6935 em três corridas, sem
encostar no grampo. A implementação não está em dúvida; a **utilidade** está.

🔵 **E ele anda para o lado ERRADO** — planta crua do dia −0,9145 (mais), o
estimador foi para −0,69 (menos). Ou o ff efetivo em malha fechada não é a
curvatura crua, ou o sinal da drenagem está invertido. **Resolve-se sem robô**,
semeando a planta de brinquedo com erro dos dois lados. É a próxima tarefa de
dev.

🟢 **O NÚMERO QUE O DONO VIU: 6,2° a 10,3° de excursão em 2,5 m**, deriva final
~1°, contra **35,8°** em 10-08 com o mesmo ff velho. *"No início joga um tico
pra esquerda depois estabiliza lindamente"* — e em 10-08 nenhuma corrida
assentava. **Configuração adotada**: compensador com ff do dia, ganhos
0,25/0,12, `adapta` DESLIGADO.

⚠️ **Ninguém sabe por que ele melhorou ao longo da tarde**, e a planta crua
repetiu 6% de dispersão em três corridas seguidas (o `medir.py` **recusou**
virar ff, corretamente). **Previsão barata que abre a próxima sessão**: três
corridas de 2,5 m com o robô **frio**. ~6° confirma o controlador; 18–21°
significa que a melhora era térmica e o problema não está fechado.

🔧 **Armadilhas que morderam de novo** (detalhe no `ambiente.txt`): a **placa
desligada com a base de pé** (`odom` de roda é `open_loop` e mente — quem prova
é `/hoverboard/connected`); **`pkill -f` matando a própria sessão ssh**, duas
vezes, porque a linha do ssh continha o nome do nó; e o **NUC caindo junto com o
robô** no meio da sessão.

---

## 👁️ 11-08 (2ª leva) — A PERCEPÇÃO SAI DO PAPEL: dois defeitos em série (dev)

Decisões **017** e **018**, as duas do pré-voo de 10-08. **327 testes verdes**
(eram 311).

🔴 **017 — o contrato da nuvem era um NOME, não um tipo.** `/livox/lidar` sai do
driver como `CustomMsg`; costmaps, reflexo e pré-voo assinavam `PointCloud2` e
**nunca receberam nada**. No simulador o `gpu_lidar` publicava PointCloud2 no
mesmo nome — a 014 nasceu inerte e ninguém viu.

```
/livox/lidar    CRU (robô: CustomMsg · simulador: PointCloud2)
/livox/pontos   PointCloud2 SEMPRE — o que a percepção consome
```

No robô converte o nó novo `nuvem_pontos` (sobe na `localizacao.launch.py`); no
simulador a ponte publica direto. ⚠️ O ponto `(0,0,0)` do Mid-360 (raio que não
volta) é **descartado**: no frame do sensor ele é o próprio robô, e marcaria
célula letal em cima dele a cada quadro.

⚠️ **Nenhum dos muitos testes de config pegou isto** — todos conferiam que os
consumidores concordavam ENTRE SI. Faltava perguntar *quem publica, e em que
tipo*. Três testes novos fecham o buraco.

🔴 **018 — o `odom` estava na altura do sensor** (o `z = −0,477` de 10-08). Com
o chão em z ≈ −0,42, a percepção do Nav2 rejeita a nuvem em dois lugares
independentes (faixa de altura e `origin_z` da `VoxelLayer`, no frame global).
Passa a pré-compor com a inversa: **`odom` é a pose do `base_link` na largada**.

⏳ **Previsão que separa as duas**: a 017 sozinha **não** faz os costmaps
marcarem. Se marcarem só com ela, a hipótese da altura está errada e a suspeita
seguinte é a `VoxelLayer`, não a altura.

---

## 🧠 11-08 — O ff DEIXA DE SER UM NÚMERO E VIRA UMA ESTIMATIVA (dev, sem robô)

Decisão **016**, que implementa o caminho 2 da 013 — o que o dono pediu na
bancada de 10-08 (*"o compensador deve conseguir identificar o erro atual"*).

🟢 **O integrador já identificava o erro; o que estava errado era a unidade e a
memória.** Ele é rumo acumulado [rad·s], vive num laço com 0,94 s de tempo morto
e é zerado a cada parada. A mudança é de **escala de tempo**: o que ele segura
em regime é drenado devagar para a `curv_*` [1/m], que sobrevive à parada.

```
transf = integral · dt / adapta_t      Δcurv = −(ki·transf)/v_real
integral −= transf                     (adapta_t = 8 s, valor de partida)
```

⚠️ **Sem solavanco por construção**: o ff cresce exatamente o que o termo
integral encolhe. Degrau de comando num laço com 0,94 s de tempo morto é como se
fabrica a oscilação que o estimador veio matar.

**Opt-in** (`-p adapta:=true`), grampeado a ±0,5 1/m da semente, não estima
abaixo de 0,05 m/s, e o nó publica `curv_hat` no `rosout` a cada 2 s — a
diferença entre "aprendeu" e "encostou no grampo" não aparece no comportamento.

⏳ **NADA DISTO FOI AO ROBÔ.** Previsão falsificável: três corridas de 2,5 m
seguidas, com ff velho de propósito — a corrida 2 tem de ser MENOR que a 1, e a
3 menor que a 2. Se a 2 empatar com a 1, o mecanismo não está agindo.

⚠️ **A planta de brinquedo não arbitra isto**: ela dá 4,1° de pico onde o robô
fez 13–22°, e erra o sobrepasso por 3,7×. Os testes provam o MECANISMO
(converge, é lento, não dá solavanco, grampeia, sobrevive à parada).

**311 testes verdes** (eram 300), cinco novos verificados por mutação.

🔧 **Duas armadilhas de ferramenta, pagas caro nesta sessão** (detalhe no
diário): desfazer mutação com `git checkout --` **apaga trabalho não commitado**
— use cópia de segurança; e mutação do mesmo tamanho em bytes restaurada no
mesmo segundo deixa o `.pyc` velho valendo (Python valida por mtime + tamanho),
fazendo a suíte reprovar um arquivo correto. **Limpar `__pycache__` entre
rodadas de mutação.**

---

## 📏 10-08 — O S ESTAVA LÁ O TEMPO TODO; A RÉGUA É QUE ERA CURTA (robô)

Doze corridas na sala. Dados em `docs/dados/2026-08-10-*`; entrada 08-10 do
diário. **Bateria não foi lida** — pedida duas vezes, a sessão andou sem ela.

🔴 **O ACHADO: a mesma corrida passa ou reprova conforme o tamanho da régua.**

```
comp-longa-a.csv   medida em 1,2 m  ->  −0,0162   PASSA no critério da 011
                   medida em 2,5 m  ->  +0,0882   REPROVA
```

Mesmo robô, mesma corrida, mesmo `medir.py`. O corte de 1,2 m cai no cruzamento
de zero do S: o rumo vai a −13,5°, volta, e de ponta a ponta dá quase reto. Em
2,5 m aparece o ciclo inteiro — **envoltória crescendo 1,65×**, meio-período
~5,3 s.

➡️ **MUDANÇA DE PROTOCOLO: corrida de aceitação tem de durar ao menos um
período (~10 s / 2,5 m).** Isso reclassifica o `+0,0417` que deu "aceito" em
05-08: ele foi medido com a régua curta. A releitura de 06-08 suspeitava; agora
está provado com a mesma corrida medida das duas formas.

🔴 **A PLANTA DERIVA 19% EM 5,4 MINUTOS** (seis retas cruas idênticas, mesmo
ponto e mesmo rumo):

```
+0,0 min 0,8395   +1,5 min 0,9358   +4,8 min 0,9313
+0,9 min 0,8154   +2,5 min 0,9175   +5,4 min 0,9687
                  ajuste +0,022 1/m por minuto (r = +0,80)
```

Velocidade linear igual nas seis; o que muda é o giro. Não é o mundo (mesmo
rumo, mesmo chão). **Os 13,5% entre 04-08 e 05-08 que motivaram a decisão 013
acontecem aqui dentro de uma bancada** — o caminho 3 (medir no começo da sessão)
envelhece dentro da própria sessão. Causa não investigada **por decisão do
dono**, e ela estava certa: térmico ou bateria não muda o que fazer.

🔴 **E FECHAR A MALHA NÃO SALVA UM ff ERRADO** — a conta dizia que caberia
(`ki·int_max` = 0,072 rad/s de autoridade contra 0,038 rad/s necessários), a
máquina disse que não:

```
ff VELHO −0,8275   −13,5° → +22,3°   excursão 35,8°   envoltória cresce
ff HOJE  −0,9383     0,0° → +13,0°   excursão 13,0°   sobrecorrige, não assenta
```

➡️ **O conserto não é medir o ff mais vezes — é o compensador ESTIMAR a
curvatura enquanto anda** (caminho 2 da 013). É o que o dono pediu com todas as
letras: *"o compensador deve conseguir identificar o erro atual para ajeitar"*.
Próxima sessão de dev.

🟢 **O Nav2 SUBIU INTEIRO NO ROBÔ REAL PELA PRIMEIRA VEZ** — os quatro
servidores `active`. O que faltava era a TF `odom → base_link`: o FAST-LIO manda
a pose no frame `body`, que não está no URDF, e o `tf_odom` **se recusava a
publicar, corretamente** (publicar embutiria os 42 cm do Mid-360 sem sintoma).
Conserto: `-p frame_da_pose:=livox_frame`. Pré-voo 10/19 → **16/19**.
⚠️ Dívida: `body` é o frame da IMU, 5 cm do lidar.

🔴 **A DECISÃO 014 ESTÁ INERTE NO ROBÔ**: `/livox/lidar` sai em
**`livox_ros_driver2/msg/CustomMsg`** e costmaps, `collision_monitor` e o
pré-voo assinam `PointCloud2`. O FAST-LIO funciona porque lê CustomMsg — a
localização vai bem e a percepção é zero. No simulador o lidar é `gpu_lidar` e
publica PointCloud2: a 014 foi aceita num ambiente onde o defeito não existe.

⚠️ **Previsão falsificável, não testada**: converter a nuvem **sozinho não faz
os costmaps marcarem**. Com o `tf_odom` compondo a pose do sensor, o `odom` fica
na altura do sensor (`z = −0,477 m` medido), o chão vai para z ≈ −0,42 e a faixa
de altura do costmap (0,10–0,50) rejeita tudo. Dois defeitos em série.

---

## 🎚️ 07-08 (4ª leva) — O ff DO DIA TEM POR ONDE ENTRAR (dev, sem robô)

A decisão **013** foi escolhida de manhã (caminho 3: medir a curvatura crua no
começo da sessão e passar por parâmetro) e, olhando o código à tarde, **não
existia parâmetro para passar**:

- `compensador_rumo.py` declarava `curv_frente` com default **−0,817** — o valor
  de 04-08, o número que a própria decisão diz não valer como verdade;
- `pilha.launch.py` subia os dois compensadores com **só** `use_sim_time` e
  `segura_rumo`; nenhum YAML carrega a curvatura;
- sobrava `ros2 param set`, que o apêndice do roteiro lista como armadilha.

➡️ **O experimento nº 2 da próxima ida produziria um número sem destino.**

🟢 **O protocolo agora fecha ponta a ponta:**

```
três corridas SEM compensador -> medir.py --resumo curvatura
  -> a linha pronta para colar -> pilha.launch.py curv_frente:= curv_medido_em:=
  -> rosout: "ff MEDIDO em <data>"
```

- **`pilha.launch.py`**: `curv_frente`, `curv_re` e `curv_medido_em`, nos DOIS
  compensadores (sim e robô), com os defaults do nó. ⚠️ Numéricos vão como
  `ParameterValue(..., value_type=float)` — argumento de launch chega como texto
  e o parâmetro é double; cru, o compensador **cai na subida** com *parameter
  type mismatch*, e o robô arca 0,82 1/m com a pilha inteira de pé;
- **o nó anuncia a procedência do ff no `rosout`**, em WARN, com default
  `HERDADO`: pilha que sobe sem a medida do dia se denuncia. Ler com
  `ros2 topic echo /rosout --field msg | grep -i "^ff "`;
- **`medir.py --resumo curvatura`** imprime a linha de launch com a média e a
  data de hoje — e **se recusa** com n<3, dispersão acima de 5% (dentro do dia o
  robô repete em 2–3%) ou frente misturada com ré. Linha colável a partir de
  medida ruim é pior que nenhuma: ela seria colada.

⚠️ **O que isto NÃO resolve**: a curvatura segue medida **uma vez por sessão**.
Planta que mude no meio da bancada envelhece o valor dentro da própria sessão —
isso é o caminho (2) da 013 (estimador online), que segue fora.

⏳ **Nada foi ao robô**; o que a próxima sessão confirma é operacional: a linha
sai, sobe a pilha, e o `rosout` diz `ff MEDIDO em <hoje>`. Se disser `HERDADO`,
o número não chegou.

**300 testes verdes** (eram 281), por pacote: `robot_motion` 171, `tools` 70,
`robot_base` 47, `robot_planning` 12. Três novos verificados por mutação.

🔧 **Dívida vista e não paga**: rodar `robot_motion/test` e `tools/` no MESMO
processo pytest derruba `test_o_raio_de_chegada_do_nav2_bate_com_o_do_seguidor`
(o `sys.path.insert` do `test_lei_de_reta.py` quebra o import relativo do
`path_follower`). É anterior a esta leva — confirmado com `git stash` — e não
aparece rodando por pacote.

---

## 👁️ 07-08 — O COSTMAP PASSA A ENXERGAR O LIVOX (dev, sem robô)

Sessão de máquina de dev, robô desligado. Decisão **014**; entrada 07-08 do
diário. **274 testes verdes** (eram 264).

🔴 **O buraco que ninguém tinha na lista: o único consumidor da nuvem era o
reflexo.** O Mid-360 publica em `/livox/lidar` desde 24-07 (robô) e 04-08
(simulador), e **os dois costmaps do Nav2 rodavam só com o mapa estático** — o
comentário no YAML ainda dizia *"o robô simulado ainda não tem lidar"*. O robô
não desviava de obstáculo novo: **parava** na frente dele. A fatia B estava
bloqueada por configuração, não por sensor.

🟢 **Entrou uma `VoxelLayer` com a nuvem NOS DOIS costmaps**, com a mesma faixa
de altura do reflexo (0,10–0,50 m), travada em teste para os dois não
divergirem em silêncio.

⚠️ **`VoxelLayer` e não `ObstacleLayer`, e é o OPOSTO do robô 1** (que trocou
voxel por obstacle no `nav2_params_pi.yaml`). Lá o sensor é planar; aqui ele
olha para cima (−7° a +52°) e enxerga a altura `0,42 − 0,123·d`: uma caixa de
0,30 m **some quando o robô chega a 0,5 m dela**. Com raytrace 2D, o raio que
passa por cima apagaria a marca no exato momento de manobrar.

🔴 **A camada só no LOCAL costmap não resolve — medido.** Mundo
`pista_surpresa.sdf` (obstáculo que o mapa não tem), caminho de (2,0 · 5,0) a
(2,0 · 7,2):

```
                      caminho / reta   folga do centro
só no local             2,20 / 2,20      0,035 m   ATRAVESSA
nos dois                2,74 / 2,20      0,530 m   CONTORNA
```

Quem dirige via a caixa; quem planeja não. Obstáculo só no local **não produz
desvio, produz travamento educado**: o robô vai reto até lá, o reflexo para, e o
replanejamento a 1 Hz devolve o mesmo plano ruim para sempre.

⚠️ **`expected_update_rate` 0,30 → 0,50 s, corrigido por medida.** Com 0,30 o
log enchia de `has not been updated for 0.43 seconds` — e a nuvem estava
perfeita (9,7 Hz, carimbo de 100,0 ms sem cauda em 281 quadros). O atraso é do
consumidor: 20 000 pontos por quadro. **Buffer vencido deixa a camada
não-current, e costmap não-current PARA de atualizar** — errar para baixo aqui
desliga a percepção tendo um aviso amarelo por sintoma.

🧰 **Três instrumentos novos, porque a pista antiga não provava percepção**
(mundo e mapa saíam da mesma planta, então "viu" e "lembrava" eram
indistinguíveis):

- `gera_pista.py` agora escreve **`worlds/pista_surpresa.sdf`** — obstáculos que
  nunca entram no mapa;
- **`tools/banco/percepcao.py`** — células letais do costmap onde o mapa diz
  livre;
- **`tools/banco/plano.py`** — pede caminho por `compute_path_to_pose`, que
  **planeja sem mover o robô** (serve na bancada com a bateria parada).

⚠️ **O robô só conhece a FACE que viu**: 0,075 m² de uma caixa de 0,25 m². O
planejador contorna uma lasca do obstáculo, não o obstáculo.

⏳ **NADA DISTO RODOU NO ROBÔ**, e há uma dívida específica: no global costmap a
marcação é **permanente** (não há janela rolante). Com a TF `map→odom` fixa e
provisória, toda deriva do LIO vira **obstáculo fantasma acumulado**. Seguro no
simulador, dívida no robô — a saída é a mesma que já obriga o mapa a ser
argumento: localização que case `map` com `odom`.

✅ **Decisão 013 RESOLVIDA pelo dono: caminho 3** — medir a curvatura crua no
começo de cada sessão e passar `curv_frente` por parâmetro. Vira passo do
protocolo de bancada; as três corridas sem compensador já são o experimento nº 2
do roteiro.

### 🗺️ E o Nav2 do robô real não tinha mapa — decisão **015**

🔴 **A `pilha.launch.py` subia o mapa da pista SIMULADA no robô real** (uma sala
de 12 × 8 m que não existe), com o `global_costmap` em `StaticLayer` sobre isso.
O cabeçalho da launch já avisava, mas era aviso **sem saída**: não existia o
"sem mapa" para escolher. A decisão 014 piorou — o global passou a misturar
paredes fantasma com marcação real e permanente do Livox.

🟢 **Entra `mapa:=nenhum`** — sem `map_server` (e fora da lista do
`lifecycle_manager`, que aborta o bringup se um servidor não responder), sem
`static_layer`, e o `global_costmap` vira **janela rolante de 20 × 20 m**
alimentada só pelo sensor. É um **overlay** (`config/nav2_sem_mapa.yaml`), não
um segundo `nav2.yaml`: há teste que falha se ele redefinir geometria.

```
                    caminho / reta   folga    manchas no global
com mapa              2,74 / 2,20     0,530 m   2 (as duas surpresas)
sem mapa nenhum       2,78 / 2,25     0,530 m   6 (surpresas + as paredes)
```

Sem mapa, o costmap global é **só o que o sensor viu**, e as seis manchas batem
com a planta (parede oeste 0,19 contra 0,20 real; divisória 3,91 contra 3,90;
parede norte 7,80/7,81 contra 7,80). O robô planeja sem mapa e chega ao mesmo
desvio.

⚠️ **O custo é MEMÓRIA CURTA**: fora dos 20 m ele não sabe de nada e o que nunca
viu conta como livre. Não é regressão — parede fantasma no lugar errado não é
conservadora, é aleatória.

⚠️ **Armadilha nova para o apêndice**: `ros2 launch` **não morre com os nós**.
Matar os filhos por PID e deixar o launch vivo empilhou **3 pilhas simultâneas**
(3 `planner_server`, 5 `tf_map_odom`), e o sintoma foi bringup abortando com
cara de bug no código. Some-se: **`ros2 node list` mostra fantasma** do daemon
mesmo com `ps` provando zero processos (`ros2 daemon stop && start` limpa). Para
saber o que está vivo, `ps`.

### 🛫 O PRÉ-VOO — um comando responde por tudo (`tools/banco/checa_pilha.py`)

30 s, **não move o robô**: uma pilha só, nuvem e taxa, as duas TFs (incluindo
`base_link → livox_frame` contra os 0,42 m da trena), fração de nuvem
transformável, `lifecycle` dos quatro servidores, perfil sem mapa, os dois
costmaps marcando e a cadeia de comando inteira — cada falha com o conserto na
própria linha. **20/20 contra o simulador** no perfil da sessão. É o
experimento 1 do roteiro.

🔵 **HIPÓTESE RETIRADA — o registro de 06-08 estava errado num ponto.** Lá o
`collision_monitor` *"recebe e não publica nem zero"* foi lido como evidência de
que ele não conseguia transformar a nuvem. Medido em 07-08:

```
3 s de comando ZERO  em /auto_vel_raw  ->  /auto_vel recebeu    0
3 s de comando 0,05  em /auto_vel_raw  ->  /auto_vel recebeu  150
```

**Ele não republica comando nulo — é como o Nav2 funciona**, e parado o
`heading_controller` só publica zero. A falta da TF segue provada por outras
duas vias (o `tf2_echo` e o `lifecycle`), mas aquele silêncio não era prova.

**281 testes verdes.**

---

## 🤖 06-08 (tarde) — O ROBÔ CONFIRMOU: o S morre com os ganhos novos

**18 corridas na cerâmica da sala**, bateria 41,16 → 40,92 V. Tudo o que estava
"previsto e nunca visto no robô" foi visto. Dados em `docs/dados/2026-08-06-*`.

🟢 **A previsão principal PASSOU — a oscilação divergente acabou.**

```
                 invs   período medido   envoltória 1ª→2ª
VELHOS  (1,0/0,5)  1,2,2     2 de 3        2,13× 2,53× 1,87×  cresce
NOVOS  (0,25/0,12) 0,1,0     0 de 3        monotônico / 0,58×
```

A evidência limpa é o **período**, e vem do `mede_o_s`: medido em 2 das 3
corridas velhas (2,28 e 2,44 s, dentro dos 2,2–2,4 s de 05-08) e em **nenhuma**
das novas. Sem período não há ciclo. O controle do dia reproduziu 05-08 quase
exato (1,87–2,53× contra 1,86–2,18×) — a comparação é **interna**, não entre
dias. O dono, a olho: *"faz um S mas tá reto, tá bom."*

🔴 **Mas o arco NÃO é do laço, e sobrou inteiro.** `|curvatura|` deu 0,0764
(novos) contra 0,0707 (velhos) — a mesma coisa. Baixar o ganho 4,1× não mexeu
nela, porque ela vem do `ff` (−0,817 1/m, idêntico nas duas). O limiar
`< 0,05` do plano mede erro de **feedforward**, não estabilidade. ➡️ **Acertar
o `curv_frente` é hoje o conserto de maior valor do projeto.**

🟢 **O pivô é bimodal, como a quantização de 10 Hz previa.** `liga 0,10` (1
ciclo) variou **13×** entre corridas idênticas (2,2 · 3,3 · 29,7°); `liga 0,30`
(3 ciclos) ficou em ±8% (59,2 · 67,6 · 69,7°). ⚠️ O modo alto deu ~30°, não os
~16° previstos — o modelo acerta a **estrutura** e erra a **escala** por ~2×.

🟡 **O preditor de Smith EMPATOU** (invs 1,0,1; nenhum período), contrariando a
bancada onde perdia claro. **O desempate não coube na sala**: sustentada ×
assenta só aparece em corrida longa, e a trava de 1,2 m corta em 4,7 s. Fica
desligado — em empate ganha quem não depende de modelo, ainda mais com o pivô
mostrando o modelo errando escala por 2×.

🔵 **Hipótese retirada:** o S era do laço, não da boba (BO-4).

🔴 **OS TESTES C E D FALHARAM, e o D tem causa única: falta o TF `odom →
base_link`.** O FAST-LIO publica `/Odometry` como mensagem e não a
transformada. A árvore TF parte em dois, o `planner_server` não ativa, o
`lifecycle_manager` **aborta o bringup** e leva o `collision_monitor` junto —
ativado na mão, ele recebe e não publica nem zero, porque não consegue
transformar a nuvem. Sensor bom, nuvem a 8 Hz, cadeia de tópicos inteira.
**Falta uma transformada.** No teste C, o teleop não publicou **nada** em
`/key_vel` (44 s de CSV, zero amostras dessa fonte).

---

## 🎚️ 06-08 — O S tem mecanismo, número e conserto projetado

🔴 **A OSCILAÇÃO DO ROBÔ CRESCE — é instabilidade, não transiente.** Olhando a
envoltória das três corridas compensadas de 05-08 (o que eu não tinha feito ao
contar só inversões):

```
corrida    1ª excursão   2ª excursão   cresceu   meio-período
a             −2,97°        +6,46°      2,18×       2,38 s
b             −6,47°       +12,04°      1,86×       2,50 s
c             −5,43°       +11,84°      2,18×       2,50 s
```

✅ **O TEMPO MORTO DO LAÇO É 0,94 s**, e o número fecha por **duas rotas
independentes**: (a) oscilar a 1,277 rad/s com o PI que estava rodando exige
esse atraso; (b) a placa medida tem 0,27 s de liga (01-08) + 0,52 s de desliga
(04-08), mais pose a 10 Hz e a janela de 0,2 s ≈ 0,94 s.

✅ **GANHOS REDUZIDOS 4,1× — `kp` 1,00 → 0,25 e `ki` 0,50 → 0,12.** Crescer
2,07× por meio-período põe o ganho de laço em ~2,07 na travessia de fase, e ele
precisa ficar abaixo de 1. Os dois caem **juntos** (a razão `ki/kp` não muda —
é ganho a menos, não controlador diferente). Margem de ganho ~2×.

🔴 **A PLANTA DE BRINQUEDO DO TESTE TINHA O MESMO PONTO CEGO DO GAZEBO.** O
`_roda_planta` já modelava atraso, mas de **0,26 s** — só a latência de liga.
Com esse valor **ela não oscila com ganho nenhum**, e era por isso que o S não
aparecia em lugar nenhum. Corrigida para 0,94 s, ela reproduz o fenômeno e vira
**o único lugar do projeto onde a estabilidade do rumo se julga sem robô**:

```
                     atraso 0,26 s (antes)      atraso 0,94 s (medido)
ganhos ANTIGOS       4,6 · 0,2 · 0,0            15,2 · 15,3 · 12,2 · 10,0 · 8,2
ganhos NOVOS         8,2 · 0,3 · 0,1            16,5 · 0,4 · 0,2 · 0,1
```

⚠️ **`int_max` FICA EM 0,6, e isso foi decidido com medida.** Baixar `ki` 4,1×
encolhe junto a autoridade do integrador (`ki·int_max`: 0,30 → 0,072 rad/s).
Subi para 2,5 para preservar o produto e **o sino voltou** (9° de segunda
excursão): com tempo morto, esse teto não é só autoridade, é proteção contra
**windup**. O preço de mantê-lo baixo: com o ff 25% errado o rumo assenta ~6,8°
fora da referência. **O conserto disso é acertar o `curv_frente`**, não subir o
integrador.

🔮 **PREDITOR DE SMITH implementado e OPT-IN (`preditor:=true`) — e ele PERDE.**
Em vez de baixar o ganho, desconta o que está a caminho. Prevê **só a correção**,
nunca o feedforward (o efeito futuro do ff é cancelado pelo arco futuro).

```
configuração                   excursões (graus)      assenta   rumo
ANTIGOS 1,0/0,5 sem preditor   15,2 15,3 12,2 10,0     39,9 s   +0,94°
ANTIGOS 1,0/0,5 COM preditor   15,3  2,3  3,7  3,6     nunca    +3,65°
NOVOS 0,25/0,12 sem preditor   16,5  0,4  0,2  0,1      9,6 s   +0,06°
```

Mata a **divergência** (12° viram 3,6°) mas deixa ondulação **sustentada**, onde
o detune assenta abaixo de 0,1°. Fica desligado, com o veredito travado em teste.
⚠️ Ressalva: a planta de brinquedo aplica o arco **imediatamente** enquanto o wz
chega atrasado; no robô os dois chegam juntos. Só a máquina desempata — 3 corridas.

⏳ **NADA DISSO FOI VISTO NO ROBÔ.** A previsão que a próxima sessão testa é uma
só e é falsificável: **a amplitude tem de DECAIR em vez de crescer.** Se
continuar crescendo, o S não é do laço e o caminho passa a ser o **BO-4**.
Plano de campo: `docs/PLANO_SINTONIA_RUMO.md`.

**530 testes verdes.** Ganhos, atraso, comparação com o preditor e o caso do
**modelo errado** travados em teste; quatro verificados por mutação.

---

🟢 **O COMPENSADOR DE RUMO PASSOU NO ROBÔ REAL (05-08)** — 21 corridas, dados e
registro em `docs/dados/2026-08-05-bancada-robo/`, entrada 08-05 (8ª leva) do
diário:

```
                sem compensador          com compensador       redução
FRENTE  n=3    −0,9116 (raio 1,10 m)   +0,0417 (raio 24 m)      95,4%
RÉ      n=3    −0,0968 (raio 10,3 m)   −0,0466 (raio 21 m)      52%
                                    critério (011): |curv| < 0,05
```

As duas médias passam. Ressalvas que o número esconde: 3 das 6 corridas
compensadas estouram o critério **individualmente**; os dois sentidos falham por
motivos **opostos** (de frente **oscila** — o S que o dono viu; de ré fica
**aquém**); e o espalho absoluto **não mudou** (0,035 → 0,044 1/m), porque o
compensador tira viés e não toca variabilidade de planta.

  ⚠️ **RELEITURA DE 06-08, e ela muda o que aquele `+0,0417` significa**: não é
  um viés estável, é a **média de uma oscilação que CRESCE, cortada em 1,2 m**.
  Ou seja, o valor depende de onde a corrida terminou. Consequências:
  · o número **não** diz nada sobre o sinal do erro do feedforward — o que
    obriga a retirar a conclusão de 05-08 de que corrigir o `curv_frente`
    pioraria (ela tinha lido o resíduo como sobrecorreção em regime);
  · os ganhos que produziram essa tabela **não são mais os do repo** (caíram
    4,1× em 06-08), então a próxima sessão tem de rodar os antigos como
    **controle do dia** para a comparação valer.

🔴 **O PIVÔ EM MALHA ABERTA NÃO FUNCIONA NESTE ROBÔ (05-08).** `liga 0,15 s` dá
16,6–33,4°; `liga 0,20 s` dá 30,3–35,8° — **as faixas se sobrepõem**. Duas
corridas com o **mesmo tempo medido** deram 16,6° e 33,4°. A grandeza que o
controlador escolhe **não determina** a que ele quer; pivô tem de ser malha
fechada no yaw. E a "zona morta de tempo abaixo de 0,4 s" que o simulador
previu **não existe**: 0,3 s dão 48°.

  ✅ **A CAUSA FOI ACHADA na 9ª leva do mesmo dia, e é QUANTIZAÇÃO** — não a
  planta. O `controller_manager` roda a **10 Hz**, então o tempo de comando é
  contado em **ciclos de 100 ms**: `liga 0,15 s` são 1,5 ciclos e a fase decide
  se cabem 1 ou 2. Com 16,6° por ciclo, `0,20` = 2 ciclos (33,2° previsto,
  30,3–35,8 medidos) e `0,30` = 3 ciclos (49,8° previsto, 48,0 medidos). E
  **33,4 / 16,6 = 2,012**. Reproduzido dentro do simulador depois de igualar a
  taxa. ⚠️ **Isto corrige o que o `ambiente.txt` da bancada e a 8ª leva do
  diário dizem** (lá a causa foi atribuída a atrito de partida, bateria ou
  comutação do motor).

  ⏳ **Previsão falsificável para a próxima ida**: repetir `liga 0,10` com n=3.
  Se a quantização estiver certa, tem de sair **bimodal** — ora ~0°, ora ~16°.
  O único valor que temos (2,7°) é compatível com ter pego **zero** ciclos.

✅ **O ROBÔ REAL GANHOU CORPO (05-08, 11ª leva).** O `tracao.launch.py` carregava
o `diffbot.urdf.xacro` — o **exemplo de demonstração do `ros2_control`** (caixa
0,10×0,10×0,05, roda 0,015, bitola 0,10, duas bobas, sem Livox). Passou a
carregar o `robo2.urdf.xacro` com `sim:=false`: a arquitetura "uma descrição,
dois hardwares" **já existia** (os dois blocos `ros2_control` chaveados por
`<xacro:arg name="sim">`) e só nunca tinha sido ligada ao robô.

  Os blocos de hardware foram comparados renderizando os dois xacro **antes** da
  troca: plugin, juntas, `device`, `wheel_radius`, `feedback_sign_*` e
  `deadband_*` saem idênticos — **o atuador não muda**. O que muda é a geometria,
  e o **`livox_frame` passa a existir**, que era o bloqueio do teste D.

  ⚠️ **Falta confirmar no robô**: `sessao.py --checar` mostrando
  `wheel_separation = 0.2700` vivo depois da troca. Primeiro passo da próxima ida.

📏 **MID-360 MEDIDO COM TRENA: 42 cm do chão, centrado (05-08)** — não os 27 cm
supostos. A zona cega vai de 2,19 m para **3,40 m**, e a 0,5 m ele só vê acima de
**36 cm** (era 21). O simulador estava 55% otimista nela; corrigido nos dois
lados. **A caixa do teste D tem de ter 50 cm, não 40.** Travado em 4 testes
novos, verificados por mutação.

🔴 **O `collision_monitor.yaml` descreve o robô errado (aberto).** O comentário
que justifica `max_height: 0.50` diz *"o robô tem 0,30 m de alto"* — número do
modelo antigo da decisão 004. A caixa termina a **0,230 m** e o Mid-360 está a
**0,42 m**: é o sensor que define o gabarito. Não mexido de propósito —
`max_height` é parâmetro de segurança e merece decisão própria.

✅ **`twist_mux` RESOLVIDO (05-08, 10ª leva) — `./setup_twist_mux.sh`.** Ele
entra por **fonte em commit fixado** (tag 4.5.0), mesmo padrão do
`setup_livox.sh`; compilado no NUC liga contra os headers que a máquina tem.
Nenhuma lib do sistema é tocada.

  🛑 **E ficou PROVADO que o `apt upgrade` de um pacote só seria perigoso.**
  Lendo os símbolos do `.deb` da 4.2.7 **sem instalar**:

  ```
  diagnostic_updater 4.2.6   exporta  ...NodeTopicsInterfaceEEd     (só)
  diagnostic_updater 4.2.7   exporta  ...NodeTopicsInterfaceEEdh    (só)
  ```

  O símbolo antigo **desaparece** — é substituição, não adição. Subir a lib
  quebraria todo consumidor compilado contra a 4.2.6, inclusive o
  `controller_manager` que a base usa. Aquele caminho só existe como **upgrade
  coerente da pilha inteira**, em sessão própria, com a base conferida depois.

  ⚠️ **Deploy**: o NUC precisa rodar `./setup_twist_mux.sh` uma vez antes do
  teste C. O `git reset --hard` não traz o clone (ele é `.gitignore`, como o
  Livox e o FAST-LIO).

  ✅ **Itens 2 e 3 do teste C já estão provados sem robô**
  (`tools/banco/prova_mux.py`): o humano vence a autonomia, e soltar devolve o
  comando. **Falta o item 1**, que precisa de máquina: soltar o teclado e o robô
  parar sozinho em 0,4 s.

---

## 📋 A PRÓXIMA IDA AO ROBÔ — tudo o que está esperando máquina

> Atualizado depois da sessão de **06-08**, que rodou 18 corridas. Os itens 1, 2
> e 5 da lista antiga **foram fechados**; o 3 e o 4 falharam, e por motivos que
> agora têm causa. O roteiro de operação (`docs/PROXIMA_SESSAO_NO_ROBO.md`)
> segue válido no **método** — o que mudou foi a lista de experimentos.

🔴 **O conserto de maior valor NÃO precisa de robô ligado para ser escrito.**
Dois dos três bloqueios de hoje são de dev:

| # | o que | por quê | onde |
|---|---|---|---|
| 1 | **TF `odom → base_link`** | o FAST-LIO publica `/Odometry` como mensagem e **não** publica a transformada. A árvore TF fica partida, o `planner_server` não ativa, o `lifecycle_manager` **aborta o bringup inteiro** e leva o `collision_monitor` junto. Bloqueia Nav2 **e** o teste D | DIÁRIO 06-08 (3ª leva) |
| 2 | ~~**o `ff` fixo não fecha o arco**~~ ✅ **FECHADO NO DEV (07-08, 4ª leva)** | a planta muda de dia (−0,8031 em 04-08 contra −0,9116 em 05-08; 13,5%, faixas que não se tocam, contra 2–3% *dentro* do dia), então nenhum valor único serve. O dono escolheu o caminho 3, e ele agora existe na máquina: `pilha.launch.py curv_frente:=… curv_medido_em:=…`, a linha sai pronta do `medir.py --resumo curvatura`, e o `rosout` denuncia ff herdado. **Falta só rodar** | decisão **013** |
| 3 | **teleop não publica em `/key_vel`** | 44 s de gravação, **zero** amostras dessa fonte. Não é o `le_tecla()` — é antes disso | `homem_morto.py` |
| 4 | **`bin/robot-key` com `set -u`** | briga com `COLCON_TRACE` e `AMENT_TRACE_SETUP_FILES` dos `setup.bash`. Uma linha | — |

**Só depois disso vale voltar ao robô**, e aí a lista é curta:

```
0. deploy: bundle -> colcon build   (o twist_mux JÁ está compilado no NUC)
1. sessao.py --checar   🔴 wheel_separation TEM de dar 0,2700 (passou em 06-08,
                        reconferir: o NUC reinicia junto com o robô)
```

| # | o que | por quê | onde |
|---|---|---|---|
| 1 | **teste C item 1** (homem-morto) | único teste cuja falha é *pior que não ter a função*. Agora com gravador: `tools/banco/homem_morto.py` mede os três intervalos em CSV | `PLANO_TESTE_ROBO.md` §1 |
| 2 | **teste D** (reflexo) | **bloqueado até o TF existir.** O plano está errado ao listá-lo como executável sem `map` | `PLANO_TESTE_ROBO.md` §1 |
| 3 | **desempate do preditor** | empatou em 06-08; precisa de **corredor longo** — a sala não deu. Sustentada × assenta só aparece em corrida longa | `PLANO_SINTONIA_RUMO.md` §4 |
| 4 | ~~**`heading_controller`: pivô indisponível**~~ ✅ **NÃO ERA DEFEITO — resolvido no dev (12-08)** | a recusa era aritmética de um chute: `zona_morta` estava em 0,15 (nunca medido) contra os **0,0178 medidos em 07-31**. Com o número certo o pivô pede 0,50 rad/s contra teto de 1,00 e existe com folga. A máquina sempre pivotou; o YAML é que proibia. Não consome bancada | decisão **020** |
| 5 | **vídeo da traseira** | segue barato; deixou de ser urgente (o S era do laço) | BO-4 |

⚠️ **O NUC CAI JUNTO COM O ROBÔ.** Parecia ter alimentação separada; não tem.
Em 06-08 um deploy morreu no meio (`No route to host`) e o NUC voltou com
`up 0 min`. Salve em levas para o `origin` — foi o que impediu perda hoje. E
`/tmp/logs` **não sobrevive**: recriar antes de qualquer `nohup`.

🟢 **PISO E BATERIA: resolvido em 06-08.** Cerâmica da sala, 41,16 → 40,92 V,
gravados no `ambiente.txt` de cada pasta de dados. A queda de 0,24 V ao longo de
18 corridas mostra que a suspeita de deriva de planta por bateria **não se
materializou** nesta sessão — as duas condições da sintonia são comparáveis.

---

🟢 **SIMULADOR AGORA BATE COM O ROBÔ EM ARCO E EM PICO DE WZ (04-08):**
- **Arco (1ª leva)**: frente −0,817 1/m (robô −0,838), ré −0,109 (robô −0,113)
- **Pico de wz (3ª leva, planta normal)**: 2,25 rad/s (robô 2,33)
- As duas maiores grandezas de fidelidade estão dentro da dispersão da máquina.
- ~~**Terceira pendência (sobrepasso):** o simulador desacelera 2× mais rápido
  por falta do atraso de desliga da placa (~0,5 s).~~ ✅ **ENTROU em 05-08 (9ª
  leva)**, junto com duas outras correções que o dado obrigou —
  `docs/dados/2026-08-05-aceitacao-atraso-desliga/`:
  1. **atraso de desliga** (0,52 s, decaindo — segurar o valor cheio deixava o
     simulador dando ~37° para 1, 1,5 **e** 2 ciclos de comando);
  2. **a latência de liga deixou de DESCARTAR o comando** — era um `return 0,0`
     que fazia **todo pulso menor que 0,27 s produzir exatamente nada**, e o
     robô gira 32° com pulso de 0,20 s. Virou fila de atraso;
  3. **`update_rate` 50 → 10 Hz, igual ao robô** — remove uma divergência que
     estava documentada como deliberada.

  **Arco (n=3): PASSOU e melhorou** — ré de −0,109 para −0,0938 (robô: −0,0968),
  razão 7,5× → **8,74×** (robô: 7,4× em 04-08, 9,4× em 05-08). O medo do BO-4
  ao baixar a taxa não se concretizou.
  **Pivô (n=3): NÃO passou** — o simulador é chato demais (19° com 1 ciclo, 30°
  com 3; o robô é linear, fator 3,0). Parado de propósito: os dois pontos em
  que ele mais discorda são os dois em que o **robô tem n=1**.

- ✅ **O item 12c (dispersão) veio de graça com a taxa.** O `ESTADO` registrava
  "o simulador é determinista demais". Com 10 Hz ele ficou **bimodal como o
  robô**: tempos múltiplos de 100 ms saem repetíveis, e `liga 0,15 s` (1,5
  ciclos) espalha exatamente entre os valores de 1 e de 2 ciclos.

---

## O robô

- **Tração**: 2 rodas de hoverboard (diferencial) **na frente** + roda boba
  **atrás** (corrigido 07-24: o ESTADO dizia boba na frente). 1 placa
  hoverboard hackeada ligada **DIRETO no PC** por serial — sem Arduino MEGA.
  Protocolo `0xABCD` @115200, realimentação de **18 bytes**.
  ⚠️ Essa geometria (motriz dianteira, boba traseira) é **instável em rumo**:
  a traseira só acompanha por arrasto e amplifica oscilação. É dado de projeto
  do controlador, não detalhe.
- **Sensor**: Livox Mid-360 (LiDAR 3D 360°, IMU embutida, conexão Ethernet).
  Único sensor externo — sem câmera, sem IMU externa, sem optical flow.
  Também sem eletrônica auxiliar: sem relé de luz, LED de marco ou botão.
- **Computador**: Intel NUC (x86), Ubuntu 24.04 + ROS 2 Jazzy. Interface do
  lidar: `enp2s0` (IP `192.168.1.2`).
- 🔋 **ENERGIA: SÃO DUAS BATERIAS INDEPENDENTES** (dono, 14-08 — e não é a
  primeira vez que ele corrige isto):

  ```
  bateria 1   as RODAS (placa hoverboard)
  bateria 2   o NUC e o LIVOX, só eles
  ```

  ⚠️ **Consequência direta para instrumentação**:
  `/hoverboard/battery_voltage` (o único canal de tensão que existe na pilha)
  mede **a bateria 1 apenas**. Ele não diz nada sobre a energia da percepção —
  e é a bateria 2 que governa a hipótese aberta para as corridas ruins de
  13-08 (Livox subalimentado perde qualidade e o robô se perde). Medir "a
  bateria" por esse tópico responde à pergunta errada.

  ⚠️ **Duas fontes do repo estão ERRADAS sobre isto e já me induziram ao erro**:
  o `DIARIO` de 10-08 conclui *"o NUC cai junto com o robô — parecia ter
  alimentação separada, não tem"* (elas descarregam juntas porque são
  carregadas juntas, não porque são a mesma); e o `CONEXOES.txt` da raiz é do
  **robô 1** (Raspberry Pi + LD06 + Arduino MEGA), não descreve esta máquina.
- **Ambiente**: novo (não é o do robô 1). **Não precisa de mapa** — a
  localização é LIO, sem AMCL (decisão 003).
- Estado físico: montado, mas **a elétrica está ruim** (07-27: tentativa de
  medição no laboratório abortada por isso). **Base de software (tração +
  localização) verificada em hardware**; movimentação e navegação ponto a
  ponto escritas e verificadas em simulador, à espera dos números do robô.
  ⚠️ **07-30: o NUC foi reinstalado** — o nosso repo tinha sumido do disco (só
  restava o workspace do estágio) e foi **re-deployado do zero** nesta sessão de
  bancada, via `git bundle` (o NUC não tem auth no GitHub — ver DIARIO 07-30 2ª
  leva). A base foi reconstruída (com um conserto de `launch_ros` no
  `tracao.launch.py`, commitado) e **volta a subir**: `--checar` passou,
  `/Odometry` a 9,9 Hz, placa e lidar de pé (este só depois de um power-cycle).
  📄 **O robô medido está em `docs/MODELO_ROBO2.md`** — atuador, zona morta,
  desvio de rumo, rotação e curva, com o que o modelo NÃO cobre e como conferir
  que o robô está em estado de medir. É o documento a ler antes de tocar no
  simulador ou no controlador.

  ✅ **07-31: bancada no robô real — o modelo está medido.**
  Detalhes completos em `docs/MODELO_ROBO2.md`. Resumo:

  ```
  zona morta linear   0,0178 m/s (frente)   0,0148 m/s (ré)
  zona morta de giro  0,095 rad/s           faixa 0,084 – 0,105
  a_dec angular       ~3,05 rad/s²          faixa 2,08 – 3,67
  curva v=0,10 wz=0,30: obedece a 0,94x, raio 0,333 m
  ```

  🔴 **`cmd_vel` não é obedecido em MAGNITUDE na faixa útil.** O driver escala
  qualquer comando pequeno até a roda maior bater em 100 unidades de firmware,
  com fator `k = 100/mx` — então **todo comando entre ~0,008 e ~0,838 m/s vira a
  mesma coisa na placa**. O teto do robô é 1,0 m/s, ou seja, o patamar cobre a
  operação inteira: `cmd_vel` escolhe **sentido**, não módulo. Latência de
  ~0,35 s para destravar. É o primeiro fato a levar para o Gazebo.
  A compensação **fica ligada**: sem ela o robô não sai do lugar (`v=0,25` andou
  2 mm em 1,5 s).

  🔴 **O robô não anda reto indo para a FRENTE** — roda esquerda 11–12% mais
  rápida e demorando 0,2–0,4 s a mais para parar, dando −9,2°/−6,9° de desvio em
  ~18 cm. De ré as rodas saem simétricas (+0,5°). Confirmado a olho pelo dono.
  **↑ MEDIDO EM PERCURSO LONGO em 08-04, e é maior do que isso** — ver o bloco
  de 08-04 abaixo: de frente ele descreve um **círculo de 1,22 m de raio**; de
  ré desvia 8,3× menos, mas **não** é reto (raio 10,2 m). A causa deixou de ser
  "em aberto": o arco está preso ao corpo (controle de piso feito) e 88% dele é
  o termo que só existe indo para a frente.

  ✅ **O LIO é excelente** — parado, deriva **1,8 mm em 15 s**. Bateu com o olho
  do dono em todas as conferências do dia, inclusive numa de ~270°. As três
  acusações que fiz contra ele durante a sessão eram defeitos meus (janela de
  derivação curta, odometria em `open_loop` como referência, buraco de gravação)
  — ver DIARIO 07-31 4ª leva.

  ⚠️ **`/hoverboard_base_controller/odom` não mede nada**: `open_loop: True`, ele
  integra o comando e devolve. Odometria de roda real só pelos encoders crus
  (`/hoverboard/{left,right}_wheel/velocity`). Resolvida a "anomalia" de
  `-70,1 rad` contra `-0,607 rad`: os motores são espelhados e reportavam sinais
  opostos: corrigido com `feedback_sign_left/right` no `robo2.urdf.xacro`.

  ⚠️ **Antes de medir qualquer coisa**, conferir que há **exatamente um**
  `fastlio_mapping` e **um** `livox_ros_driver2_node` no NUC. Em 07-31 três
  pilhas órfãs publicando em `/Odometry` produziram saltos de ~1,35 m e custaram
  horas de diagnóstico errado.

  ~~❌ **O banco está em 2 passos de 6.**~~ **↑ 08-04: são 4 de 6** — ver abaixo.

  ⚠️ **Passos 4 e 5 precisam ser REESCRITOS antes de rodar.** Os dois varrem
  velocidade (0,2 / 0,4 / 0,6 m/s) e as três caem dentro do patamar da
  compensação — dariam o mesmo resultado. A varredura tem de subir acima de
  0,838 m/s, e isso exige espaço. (O passo 3 **não** tinha esse problema e
  rodou em 08-04: o `a_dec` é medido com o comando em ZERO, e a compensação só
  age enquanto há comando.)

  ~~⚠️ **O modelo só está aferido em rajadas de ~1 s e ~20 cm.**~~ **↑ FECHADO
  para o desvio de rumo em 08-04**: seis corridas de 1,3 a 3,7 m, limpas, com
  uma pilha só de localização. Segue aberto para velocidade sustentada acima do
  patamar.

  ~~🔧 **Dívida de instrumento:** os scripts de rajada não percebem que o robô
  sumiu.~~ **↑ PAGA em 08-04 para o `ensaio.py`** (`7d7fad8`): fonte que cala
  por mais de `--sem-dado` (1,0 s) **aborta** a corrida e sai com código 1, que
  o `sessao.py` já sabia tratar e nunca recebia. 4 testes. **Segue aberta no
  `rajada_rodas.py`.**

## 🎯 2026-08-04 — O bloqueio era do instrumento, e o robô anda em círculo

Nove corridas. **O banco foi de 2 passos de 6 para 4.** Detalhes na entrada
08-04 do `docs/DIARIO.md`; dados crus e leitura em
`docs/dados/2026-08-04-bancada-robo/` (com `ambiente.txt`).

  🔴 **O DESVIO DE RUMO, medido em percurso longo e com controle de piso**
  (`n=2` matched por sentido, `--espaco 1.2`):

  ```
  FRENTE   curvatura −0,817 1/m   raio  1,22 m   faixa −0,73 a −0,90
  RÉ       curvatura −0,098 1/m   raio 10,19 m   faixa −0,08 a −0,12
                                            razão frente/ré = 8,3x
  ```

  **O arco é do ROBÔ, não da sala.** Nas quatro corridas o corpo ficou a ~0°
  (frente) ou ~180° (ré) andando para a mesma faixa de direção do mundo — mesmo
  pedaço de chão, corpo girado. Caimento de piso é força fixa **no mundo** e
  faria a curvatura **no corpo** trocar de sinal. Ela saiu negativa nas quatro.

  Isso **exclui motor/placa fraca de um lado** (daria a mesma curvatura nos dois
  sentidos). Sobra causa dependente do sentido de marcha — a assinatura da roda
  boba, arrastada atrás indo pra frente e dianteira indo de ré. Encaixa com os
  encoders de 31-07 (esquerda 11–12% mais rápida **só** de frente).

  **Lê como duas parcelas somadas**, e a consequência é de projeto:

  ```
  constante nos dois sentidos  ~ −0,10 1/m  (raio 10,2 m) — sobrevive à ré
  só de frente (a boba)        ~ −0,72 1/m  = 88% do arco de frente
  ```

  ➡️ **Consertar a boba NÃO deixa o robô reto.** Sobra raio de ~10 m, que o
  seguidor tem de fechar em malha fechada de rumo.

  🔴 **`a_dec` MEDIDO (passo 3, `n=3`), e ele NÃO é constante:**

  ```
  a_dec EFETIVO  média 3,26   faixa 2,68–4,32   dispersão 50%
  a_dec CAUDA    média 1,03   faixa 0,88–1,12   dispersão 24%
  pico de wz     média 2,33   faixa 2,23–2,51   dispersão 12%
  ```

  **O número que o seguidor deve usar é a CAUDA, ~1,0 rad/s²** — 3× menor que os
  3,05 do `MODELO_ROBO2.md`. O `a_dec` efetivo espalha 50% entre corridas iguais
  e não serve como constante de projeto (é `wz²/(2·Δθ)`, o pico entra ao
  quadrado); os 3,05 caem dentro da faixa dele, então o substituto não errou o
  *valor efetivo* — errou o *uso*. A cauda é onde o robô assenta no rumo, e é o
  lado seguro do erro por 27-07 ("errar para baixo é de graça, para cima traz o
  S de volta").

  ⚠️ **Latência da placa depois do corte: 0,40 / 0,56 / 0,60 s** (média ~0,52 s),
  mais que os ~0,35 s que o registro trazia.

  ✅ **O "giro espelhado" que bloqueou 30-07 NÃO EXISTE** — era o `atan2` do
  cutucão enrolando em ±180°. Com o patamar, `+0,6 rad/s` por 2 s gira bem mais
  que meia volta: `281,5° − 360° = −78,5°`, o número exato do bloqueio. Hoje leu
  −71,0° e o dono viu o nariz ir **para a esquerda varrendo bastante** = 289°
  enrolados. **O swap de rodas NÃO deve ser aplicado. O estado atual é o certo.**

  ✅ **O yaw do LIO NÃO tem sinal invertido** — a retratação da 3ª leva de 31-07
  era ela própria incorreta, tirada do mesmo enrolamento (`−68,2 + 360 =
  291,8°`). Conferido no dado cru daquele dia: comando `+0,30` → yaw desenrolado
  `+147,7°`; comando `−0,30` → `−150,0°`. Os sinais concordam, e esses números já
  estavam na tabela do diário — ninguém cruzou as duas partes do registro.

  ❌ **O passo 6 não pode rodar como está escrito**, por dois motivos
  independentes: não existe reta de referência (o robô faz círculo, então "o rumo
  volta ou foge?" não tem sentido) e o pulso de perturbação dura 0,5 s contra uma
  latência de ~0,5 s. Medido: wz médio −0,405 antes, −0,358 durante, −0,369
  depois. O que rodou hoje foram **retas puras medindo curvatura**, que é a
  pergunta que este robô sabe responder.

  ⚠️ **A trava de `--espaco` é RADIAL e roda por cima da odometria em
  `open_loop`.** Na primeira corrida ela achou que o robô tinha andado 2,98 m em
  linha reta enquanto o LIO sabia que ele estava a 1,79 m da origem, fazendo
  círculo — **cortou pelo motivo errado, e o robô bateu numa cadeira**. Ela não
  protege contra excursão lateral. Num robô que arca, dimensionar por `--espaco`
  pequeno é o jeito de limitar o **disco varrido**.

  ✅ **CONSERTADO no mesmo dia** (`7a0c364`): o `atan2` que enrolava, no cutucão
  do `sessao.py` e no `a_dec` do `medir.py`. Os dois passam a **acumular** o yaw
  amostra a amostra. No `medir.py` veio junto um segundo defeito entrelaçado: a
  janela de frenagem começava no **corte** e não no **pico**, incluindo os 0,4–0,6 s
  em que a placa ainda empurra e o robô ainda ACELERA — consertar só o `atan2`
  deixaria a função devolvendo número errado com cara de consertada. Ele passa a
  imprimir também o `a_dec` da **cauda**. 6 testes, os 4 principais verificados
  por mutação. Suíte: **421 verdes**.

  ⚠️ **Piso e bateria não foram informados** nas nove corridas — o `ambiente.txt`
  registra `NÃO INFORMADO`. Sem eles a sessão não se compara com a próxima.

### 🔗 Como 08-01 e 08-04 se encaixam (escritas independentes, em máquinas diferentes)

  ✅ **CONFIRMAÇÃO CRUZADA, e é forte.** A entrada de 08-01 derivou **do código**
  que a placa entrega um único `wz` de **2,204 rad/s**. Em 08-04 o robô foi
  medido e os picos das três corridas de pivô deram **2,226 / 2,514 / 2,253
  rad/s**. Duas rotas independentes — leitura do driver e LiDAR no robô — no
  mesmo número. O modelo do atuador de 08-01 **está validado em hardware**.

  ⚠️ **As duas "latências" NÃO se contradizem: são coisas diferentes.**
  08-01 mede **0,273 s** para o robô *começar* a andar depois do comando (atraso
  de liga). 08-04 mede **0,40 / 0,56 / 0,60 s** entre o comando *zerar* e o `wz`
  atingir o pico — ou seja, quanto a placa **continua empurrando depois de
  desligada** (atraso de desliga). São dois fenômenos, os dois reais, e o de
  desliga é o maior. Não tratar um como correção do outro.

  🔴 **O `a_dec` de 08-04 fica MAIS importante com a 005 em xeque, não menos.**
  Se `wz` não é modulável — a placa entrega um valor só —, a lei de frenagem
  não roda, e **a única alavanca que sobra é DECIDIR QUANDO CORTAR**. O que
  acontece depois do corte é exatamente o que o `a_dec` descreve. Então ele
  deixa de ser um ganho de controlador e passa a ser **o limite de precisão de
  rumo da máquina**:

  ```
  pico de 2,33 rad/s  ->  47° de giro DEPOIS do corte (faixa 42–53°)
  ```

  ➡️ **Qualquer pivô comandado neste robô custa ~47° de sobrepasso**, mais o que
  ele girou sob comando. Isso é piso, não sintonia: nenhum ganho conserta, e é o
  número que a saída escolhida para o BO da compensação vai ter de derrubar.

  ⚠️ **A "zona morta de giro" de 07-31 (0,095 rad/s) segue valendo como número, e
  08-01 corrige o que ela SIGNIFICA**: é o disparo da compensação (`mx > 1.0` →
  `wz > 0,0621`) mais a latência sobre a rampa, não atrito. Descreve o
  **sistema**, e se move se alguém mexer no `deadband_speed`. **O atrito segue
  não medido** — e, com a compensação ligada, 08-04 não conseguiu medi-lo
  tampouco.

### Medidas ✅ CONFERIDAS COM TRENA (2026-07-29)

```
caixa 433 × 455 × 145 mm, fundo a 85,2 mm do chão
bitola 270 mm   roda Ø160 mm (raio 0,080)   espessura 45 mm
eixo motriz a +81,5 mm do centro (era 0,15 estimado)
```

A bitola era o item nº 1 e **os dois valores herdados estavam errados, em
sentidos opostos**: simulador com 0,20 girava 26% a MENOS que o comandado,
robô real com 0,32 girava 19% a MAIS. Sintonizar rumo na bancada e levar pro
robô erraria duas vezes, em direções contrárias. Raio: 0,0825 → 0,080 (~3% de
odometria, 30 cm a cada 10 m). Detalhes na entrada 07-29 do `docs/DIARIO.md`.

### Medidas que AINDA faltam

1. **Diâmetro da rodinha da boba** — fecha `boba_raio` e `boba_trail` de uma
   vez. Os valores no arquivo hoje são PROVISÓRIOS (0,025 / 0,01), escolhidos
   só para serem possíveis: o anterior (roda de 100 mm) não cabia nos 85,2 mm
   de vão. Governam o comportamento que o projeto inteiro quer reproduzir.
2. **Largura da caixa na altura das rodas** — as rodas ficam 70 mm para dentro
   da parede lateral, então ou há recortes ou a parte de baixo é mais estreita
   que os 455 mm do topo. Não afeta giro nem odometria; afeta o footprint que
   o Nav2 usa pra decidir se passa num vão.
3. **Massas** — os 10 kg (5,8 + 2 + 2 + 0,2) seguem estimados.

## 2026-07-14 — Nascimento do repo: clone do robô 1 + demolição

- Repo criado como **clone com histórico completo** de `Controle_robo_web`
  (`df76a47`) — decisão em `docs/decisoes/000-heranca-do-robo1.md`.
- **`MIGRACAO_LIVOX.md`** (raiz) = plano de demolição/reforma, aprovado pelo dono.
- **Demolição executada** (A `f406bed`, B `fe48a86`, C `2945725`): fora
  teb/costmap_converter, firmwares de diagnóstico, trekking/cone, porta
  (door_crossing e toda a cadeia), mapas/worlds/laudos do robô 1, PS4.
  274 testes verdes após cada fatia.
- Fósseis conscientes (remover na fase ADAPTA): standdown de porta no
  unstuck_supervisor (inerte sem `/door_zone`); `cone_pose_fix.py` (o
  pose_estimator importa `apply_pose_fix`).

## 🧭 2026-07-14 (2ª leva) — Arquitetura-alvo definida + varredura LD06

- **Decisão 001 (`docs/decisoes/001-gui-2d-localizacao-3d.md`)**: humano
  opera em mapa 2D (GUI herdada); o robô se localiza/navega em 3D (LIO no
  Mid-360). Nav2/localização/movimentação repensadas DO ZERO com literatura;
  "fase 1 clone barato" (stack 2D+AMCL como alvo) DESCARTADA — stack herdada
  vira referência/candidata, não caminho assumido.
- **Varredura LD06**: fora test_lidar.sh, lidar.launch.py, retry+watchdog
  serial do launch.sh (agora placeholder explícito do Livox em [3]), passo
  LiDAR do setup_udev.sh (Mid-360 é Ethernet; udev segue só pra MEGA),
  bin/teleop-pernas; README reescrito pro robô 2 (o antigo tinha 1406 linhas
  do robô 1).

## 2026-07-15 — Investigação ADAPTA 1-2: robô 2 NÃO tem MEGA

- Inventário real com o dono: **1 placa hover direto no PC** + Livox + NUC +
  baterias. Nada de MEGA/relé/LED/botão/IMU externa/flow — seção "O robô"
  corrigida acima.
- `cmd_vel_to_wheels.py` já é diferencial puro (knobs anti-skid já não
  existiam) → ADAPTA 2 = só calibração de params com o robô.
- Protocolo da placa (família EFeru/NiklasFauth, `0xABCD` @115200, feedback
  18 B) mapeado de `firmware/mega_bridge/*/hoverboard.{h,cpp}` — base pronta
  pra uma ponte direta em Python, se a bancada confirmar.
- Detalhes + checklist de inspeção: entrada 07-15 do `docs/DIARIO.md`.

## 2026-07-15 (2ª leva) — Meta final + time de 3 + rota 2D→LIO (decisão 002)

- **Meta final explicitada pelo dono**: robô 2 indo de um ponto a outro sem
  bater, igual ao robô 1. É o critério de sucesso.
- **Time vira 3 pessoas**: dono (~6 meses de ROS) + 2 iniciantes totais.
  Divisão em frentes: **A** base/motores · **B** percepção/Livox ·
  **C** infra/GUI. Organização do trabalho a 3 (branches, revisão cruzada,
  DIARIO com autor) a combinar com o time.
- **Decisão 002** (`docs/decisoes/002-rota-2d-primeiro-lio-depois.md`):
  clone 2D primeiro (receita do robô 1: /scan derivado + AMCL + Nav2),
  LIO 3D depois como evolução comparada — o baseline 2D vs LIO vira
  resultado do artigo. Revisa a ordem da 001; o conceito (GUI 2D pro
  humano) permanece.

## 🚀 2026-07-24 — Base de pé: tração + localização (decisão 003)

O robô deixou de ser projeto e virou máquina com base funcionando. **Decisão
003** (`docs/decisoes/003-base-ros2control-e-lio.md`) fecha o BO-1 e revisa a
001 e a 002.

- **Tração**: `ros2_packages/hoverboard_driver/` — interface `ros2_control`
  falando serial direta com a placa + `diff_drive_controller`. Compila limpo
  no Jazzy. **Substitui** `mega_bridge.py` e `cmd_vel_to_wheels.py`.
- **Localização**: Livox Mid-360 + **FAST-LIO** → `/Odometry`. Sem `/scan` 2D,
  sem mapa, sem AMCL — a rota "2D primeiro" da decisão 002 caiu.
- **`ros2_packages/robot_base/`** (novo) amarra as duas:
  `ros2 launch robot_base base.launch.py`.
- **`setup_livox.sh`** (novo) traz os drivers de terceiros em commits fixados,
  compila/instala o SDK nativo da Livox e aplica a config de rede do lidar.
- **Movimentação NÃO foi herdada** — decisão explícita do dono. A camada
  existente andava sempre a fundo (o teto do controlador engolia a
  desaceleração por desalinhamento) com o giro saturado → **anda em S** e não
  fecha curva. Vamos escrever a nossa, em SI real, sem escada de ganhos.

## 🧪 2026-07-24 (2ª leva) — Simulador (decisão 004)

Robô 2 montado no Gazebo Harmonic para ajustar movimentação **sem o robô**.
Premissa da decisão 004: o simulador só serve se **errar como o robô erra**.

- Caixa 0,50 × 0,50 × 0,30, fundo a 0,10 do chão; motrizes de hoverboard na
  frente (separação 0,20); boba no centro da traseira; **10 kg**.
- **Boba com trail de 4 cm e atrito no pivô** — é o que faz a traseira ser
  jogada pra fora no giro. Esfera lisa (o jeito fácil) não reproduz nada.
- **`/Odometry` = pose verdadeira do Gazebo**, mesmo papel do LIO. Odometria de
  roda esconderia a derrapada.
- **Mesmo `diff_drive_controller` e mesmos tetos** do robô real; só a camada de
  hardware muda. Sem isso o ajuste não transfere.
- Arquivos: `ros2_packages/robot_base/description/robo2.urdf.xacro`,
  `worlds/pista_livre.sdf`, `launch/sim.launch.py`,
  `config/hoverboard_controllers_sim.yaml`.
- **Verificado**: física estável no Gazebo (assenta nas 3 rodas em z≈0);
  11 testes travam geometria/massa. Suíte: **285 verdes**.
- ⚠️ Falta `ros-jazzy-gz-ros2-control` (apt, precisa de sudo) para dirigir.

## 🔬 2026-07-27 — O S explicado, e a lei que o elimina

O simulador virou instrumento de medida. `gz-ros2-control` instalado, robô
dirigível, e o defeito **reproduzido e explicado com número**.

- **O S apareceu**: com o controlador velho (linear no teto, giro saturado no
  sinal do erro), o rumo vira **ciclo-limite** — ±18°, período 2,8 s, 21 cm de
  serpenteado a cada 1,9 m percorridos, **sem decair**.
- **A causa é a rampa de desaceleração, não a boba.** Entre "o erro cruzou
  zero" e "o giro parou" existe uma distância de frenagem de rumo de
  `wz²/(2·a_dec)`. Medido em malha aberta: comando cortado a wz=1,0 rad/s e o
  robô **girou mais 25°**. Previsto pela fórmula: 0,276 rad; medido: 0,34 rad.
  A derrapada da boba responde por ~20% do S; a rampa, por ~80%.
- **A lei que resolve** (validada, ainda não implementada):
  `wz = sinal(e)·min(wz_max, √(2·a_dec·|e|))`, linear cedendo com `cos(e)` —
  nunca pede mais giro do que consegue frear no erro que ainda falta.
- **O dono julgou o S do simulador FRACO** perto do robô real (barriga de ~50 cm
  contra os ±10 cm daqui). A barriga escala com `a_dec^-1,43`, o que estima o
  **`a_dec` real em ~0,5 rad/s²** — um terço do que está no YAML. A estimativa
  não vale como medida: entra no banco de ensaios.

### Estresse da lei — 10 corridas na planta degradada (a_dec = 0,3)

Meia-volta de 180°, alvo trocando de sinal, velocidade baixa, malha a 10 Hz
(taxa do robô real) e zona morta injetada: **sobrepasso entre 0,1° e 0,8° em
todos**, contra os ±47° do controlador velho. O S não voltou em nenhuma.

Três achados que não estavam no pedido:

1. **Errar o `a_dec` pra baixo é de graça.** Chutando 3x menos que a planta
   entrega: sobrepasso zero e 4,5 s de assentamento, contra 4,8 s com o valor
   exato. Chutar pra cima é que traz o S de volta (com 5x otimista: oscila,
   mas **decai** — degrada, não quebra).
2. **A zona morta é um precipício, não uma ladeira.** Com zona morta de roda em
   0,10 m/s a meia-volta trava 0,1 s e completa; em **0,15 m/s o robô fica 22 s
   parado**, 100% das amostras, com o controlador pedindo 1,0 rad/s. Ver BO-3.
3. **Defesa dimensionada**: `v_piso = zona_morta + wz_max·bitola/2 + margem`,
   confirmada nos dois valores. O piso não é número solto — depende da zona
   morta MEDIDA.

### Banco de ensaios (`tools/banco/`)

Roda igual no robô e no simulador (mesmos tópicos, mesmo CSV), o que torna os
dois diretamente comparáveis. `README.md` traz o protocolo de caracterização:
zona morta linear e de giro, degrau de giro (`a_dec`), curva sustentada em
várias velocidades e aceleração linear.

## 🧭 2026-07-27 (2ª leva) — Navegação ponto a ponto (decisão 006)

A pilha de movimento própria do robô 2 fechou a fatia A: **ir a um ponto**, sem
obstáculo. Vive em `ros2_packages/robot_motion/` — separada do `robot_nav`, que
ainda guarda os fósseis do robô 1.

- **Rumo alvo = direção até o ponto, recalculada todo ciclo.** Dissolve o erro
  lateral sem controlador extra: se o robô sai da linha, a direção muda e ele
  curva de volta. Resolve os 66 cm de desvio paralelo que o controlador de
  rumo sozinho deixava.
- **Aproximação = a lei de frenagem em distância**: `v = √(2·a_lin·dist)`.
  Mesmo princípio da decisão 005, mesmo tipo de parâmetro físico.
- **Chegada com piso e corte firme.** Não existe "chegar devagarinho": abaixo
  do mínimo viável a placa engole o comando e o robô para longe do ponto
  achando que chegou — o BO-3 disfarçado de sucesso. O nó recusa raio de
  chegada menor que a distância de parada a partir do piso (senão orbita o
  ponto) e avisa no log.
- **Camadas conversam pelo tópico público** (`rumo_alvo`, `velocidade_alvo`):
  a navegação decide *para onde*, a movimentação decide *o que o atuador
  aguenta*.
- **Verificado no simulador**, os dois nós reais empilhados: alvo em (2, 2) a
  2,83 m — chegou e **parou a 7 mm do ponto**, 45 s sem orbitar; alvo em
  (−1, 1), 135° atrás — chegou a **8 mm**.
- **Quatro defeitos achados clicando no RViz** (o dono, em 5 minutos, achou o
  que 10 corridas roteirizadas não acharam): um ponto a 0,65 m era **orbitado
  para sempre**. Causas, todas medidas: (1) a banda morta tem duas saídas e só
  uma estava programada — o pivô era proibido por aritmética; (2) a velocidade
  não tinha teto pela curva; (3) **a linear cedia pelo erro do BICO num robô
  que escorrega** — em órbita o bico ficava a 50° do alvo e o movimento a 87°,
  **37,5° de deriva**, aproximação zero; (4) chegando, ele continuava girando e
  se arrastava para fora (0,06 m viravam 0,27 m).
- **Depois dos quatro**: o ponto de 0,65 m fecha a 0,059 m — ele **para, pivota
  no próprio eixo** e só então arranca. Alvo à frente (2, 2): 9 mm. Alvo atrás
  (−1, 1): 8 mm em 9,8 s, com afastamento máximo de 1,67 m (era 2,15 m num
  laço andando). Parado no ponto: 9 mm de deriva em 30 s.
- **O simulador ganhou uma PLACA FINGIDA** (`robot_base/placa_simulada`), a
  pedido do dono: ela engole comando de roda pequeno demais, como a de verdade.
  Antes eu rodava com zona morta zero — fiel ao Gazebo e infiel ao robô. Agora
  o controle é desenvolvido contra uma zona morta plausível, e quando a bancada
  medir a real troca-se só o número. `sim.launch.py zona_morta:=0.10`.
- **A planta lenta virou perfil versionado** (`planta:=lenta`, padrão), fechando
  uma dívida de reprodutibilidade: ela vivia num arquivo solto e quem subisse o
  simulador pelo caminho oficial pegava a planta ágil e veria um robô melhor do
  que o real.
- **⚠️ Com a zona morta ligada, o pivô some — e a BITOLA é quem decide.**
  O pivô exige `wz_max·bitola/2 ≥ zona_morta + margem`. Com a bitola do modelo
  simulado (0,20) e zona morta 0,10, pivotar exigiria **1,3 rad/s** contra um
  teto de 1,0: impossível, o robô volta a fazer só arcos e **orbita pontos
  próximos** (chegou a 0,168 m de um alvo com raio de chegada de 0,15). Com a
  bitola real presumida (0,32), o mesmo caso dá 0,62 rad/s e o pivô **existe**.
  O nó diz qual dos dois é o caso, em voz alta, na subida.
  **↑ SUPERADO em 2026-07-29** — a bitola foi medida: 0,270. Ela caiu ENTRE os
  dois palpites e **não resolveu a pergunta**. Reescalando: com zona morta 0,10
  o pivô exige 0,96 rad/s (cabe no teto de 1,0, mas com 4% de folga — o que não
  é margem nenhuma); com zona morta 0,15, exige 1,48 rad/s e é impossível, PIOR
  que os 1,25 que se supunha. Quem decide agora é a **zona morta**, ainda não
  medida — ela tomou o lugar da bitola como item nº 1 da bancada.
- **Falta**: desviar de obstáculo (fatia B) — depende do Livox e de percepção
  que o repo ainda não tem.

## 🔙 2026-07-28 — A ré como manobra (decisão 007), e a boba do simulador cai

O ponto perto e de lado deixou de ser inalcançável, e uma premissa da decisão
004 caiu no mesmo dia.

- **A ré entrou** (`docs/decisoes/007-re-como-manobra.md`): quando o alvo exige
  um raio menor do que o robô consegue fazer (`d/(2·sen e) < raio_min_curva`),
  ele **recua reto** até a geometria abrir, e então entra normal. Gatilho
  geométrico — decidido antes de orbitar, não depois.
- **Ré reta por decisão do dono.** Curvar de ré é a manobra sem medida nenhuma:
  andando para trás a boba vira roda dianteira. Reta ainda mantém a zona morta
  simétrica.
- **A lei de rumo NÃO mudou**: `linear_de_avanco` segue com `max(0, cos e)` e o
  `test_nunca_anda_de_re` segue verde. A ré é modo à parte, acionado por
  **velocidade negativa** no tópico que já existia — sem tópico novo.
- **Com orçamento e com voz**: histerese de 1,3× para sair, teto de 1,0 m e 8 s
  para a manobra inteira. Estourou, para e grita com os números.
- **Verificado**: o alvo a 0,65 m de lado, que orbitava a 0,168 m para sempre,
  recua 9 cm em duas mordidas e **chega** (0,150 m, parado 30 s). Alvo (2, 2)
  continua em 8 mm sem acionar ré. **338 testes verdes** (eram 325).
- ⚠️ **Com os parâmetros de hoje o pivô não existe em NENHUM dos dois perfis** —
  no real, pivotar exigiria 1,25 rad/s contra teto de 1,0 (`zona_morta` 0,15,
  `bitola` 0,32). Medida a bitola, `raio_min_curva: 0` desliga a ré sozinho.
  **↑ A expectativa NÃO se cumpriu (07-29).** A bitola medida (0,270) é MENOR
  que os 0,32 supostos, e braço menor exige MAIS wz: o pivô passa a exigir
  1,48 rad/s nesse cenário, contra os 1,25 que se temia. Medir a bitola
  **piorou** o caso em vez de resolvê-lo, e a ré da decisão 007 segue
  necessária. Só a zona morta pode mudar isso agora.
- **A boba do simulador é decorativa** — ver BO-4. Consequência imediata: a ré
  foi validada só no simulador, e a única coisa que preocupa nela (a boba
  virando roda dianteira) é justamente o que aquele modelo não pode mostrar.

## 🔀 2026-07-28 (2ª leva) — Navegação própria APOSENTADA, Nav2 entra

O dono dirigiu o robô no simulador clicando no RViz e reprovou o resultado. O
CSV da sessão está em `docs/dados/2026-07-28-cliques-movimentacao.csv` e o
diagnóstico no diário. O resumo em um número: **0 amostras de giro parado em
2714** — ele nunca virou no próprio eixo; alvo a 0,43 m custou 3,66 m de
caminho e 57 s.

- **Dois defeitos distintos, e só um era da ré.** O ciclo "ré e anda" (12
  entradas, período 2,10 s) é da histerese da decisão 007, que solta a manobra
  assim que o alvo cabe *naquele instante*. O **balão** é da movimentação:
  `raio = v/wz`, com `wz_max` 1,0 (herdado, nunca medido) e o piso de linear
  que a zona morta obriga — girando a 1,0 rad/s ele é obrigado a andar a
  0,23 m/s.
- **Decisão do dono**: aposentar a navegação ponto a ponto (`goal_navigator` +
  a ré da decisão 007 — os dados ficam), trazer o **Nav2**, e por agora testar
  **só o planner**. O seguidor provavelmente será nosso, como no robô 1.
- **O "gira no lugar e anda reto" do robô 1 foi DESCARTADO** pelo dono: aquilo
  era a única saída do chassi de 4 rodas, que não faz arco. Este faz curva boa
  e não deve parar para virar.
- **O que vale trazer do robô 1** (lido a pedido dele): a ré como recuperação
  **por sintoma** com vão traseiro medido em metros (`rear_min_gap`), o
  `twist_mux` com prioridade, e o carrot no plano quando houver seguidor.
- ⚠️ **`wz_max = 1,0` nunca foi medido** e é 4× menor que o do robô 1
  (2,4–4,5 rad/s). É ele que torna o pivô "impossível" por aritmética. Virou
  item de bancada.
  **↑ 07-29: segue sem medida no robô, e piorou no simulador** — em malha aberta
  o comando de 1,0 rad/s entrega 0,79. O teto efetivo é ainda menor que o número
  escrito, e o pivô fica mais longe, não mais perto.

### Bancada do planner (`ros2_packages/robot_planning/`)

Dois cliques no RViz, dois caminhos desenhados, uma tabela de números —
**sem robô, sem simulador, sem sensor**, de propósito: o defeito que trouxe o
Nav2 nasceu na movimentação, e julgar planner junto com quem executa mistura as
culpas. Compara **Theta\*** (o do robô 1) com **Smac Hybrid-A\*** em
Reeds-Shepp, que respeita raio de curva e pode usar ré no próprio plano.

Pista em `tools/mundo/gera_pista.py`, que gera **mapa do Nav2 e mundo do
Gazebo da mesma planta**: porta 0,90 m, bloco solto, aperto 0,80 m, beco sem
saída. Como rodar e como ler: `ros2_packages/robot_planning/README.md`.

**Julgado com dado em 29-07**: `docs/decisoes/008-nav2-planeja-nos-seguimos.md`
foi **ACEITA pelo dono em 29-07**. Decide o **Smac Hybrid-A\*** (único seguível
em toda a faixa de raio plausível — o Theta\* vai a 0/6 no pior caso), a ré
nascendo do planejamento em vez do susto (revisa a 007) e o **seguidor próprio**
(a movimentação da 005 é a única camada com física medida).

## 🎯 2026-07-29 (2ª leva) — A pilha obedece; o giro é que não entrega

`tools/banco/corrida_gazebo.py` (novo) responde o que o `ensaio.py` não
responde: **a pilha montada obedece?** Sobe o Gazebo headless e roda duas fases
na mesma simulação, uma corrida por perfil de zona morta. CSVs em
`docs/dados/2026-07-29-bancada-gazebo-{sim,real}.csv`.

- **10 de 10 alvos alcançados** (5 por perfil), sem órbita e sem travamento.
  Não absolve a navegação aposentada: confirma pela terceira vez que roteiro
  fechado não reproduz o que o dono acha clicando.
- **O giro entrega 79–86% do comandado, e piora subindo** — medido em malha
  ABERTA, com a placa contornada e o controlador de rumo fora do ar. A reta sai
  a 100,4% e esquerda/direita batem em 0,2%, o que exclui bitola e conversão: é
  escorregamento. **`wz_max = 1,0` vale 0,79 rad/s de verdade.**
- **O pivô só existe no perfil otimista**: 181 amostras de giro parado no perfil
  `sim`, **zero** em toda a fase B do perfil `real`. Confirma a aritmética de
  hoje de manhã (0,96 rad/s com zona morta 0,10; 1,48 com 0,15).
- **O piso de linear segura o BO-3**: zero amostras com roda pedida dentro da
  banda morta, nos dois perfis.
- **O ciclo "ré e anda" voltou no perfil pessimista** (5 entradas, período
  2,23 s; 1,85 m de caminho para um alvo a 0,40 m). É o defeito de 28-07, e
  **não será consertado** — vive no `goal_navigator`, já aposentado.
- ⚠️ Tudo isto é Gazebo com a boba do BO-4, que é um patim. O déficit de giro
  pode ser o mesmo contato falso. Não vale como medida do robô.
- **Consequência para a bancada do planner**: `minimum_turning_radius: 0.25`
  está otimista. O raio realizado (p5) foi 0,370 m no perfil sim e **0,463 m no
  real** — ele abre em relação ao pedido justamente por causa do déficit de
  giro. Como o raio mínimo **é** o argumento da comparação Theta\* × Smac,
  julgar com um valor só repetiria a forma de erro da bitola: a bancada
  pareceria boa e o robô pioraria. A bancada passa a rodar uma **faixa** de
  raio.

## 📐 2026-07-29 (3ª leva) — A régua da bancada do planner estava errada

`tools/planner/varredura_raio.py` (novo): 6 casos × 4 raios mínimos × 2
planners, headless, sem cliques. Antes de responder a pergunta, achou um defeito
na régua da bancada.

- **`mede()` lia cúspide de Reeds-Shepp como curva fechadíssima.** No caso real
  `perto_de_lado` com 0,46 m configurado, ela acusava raio 0,125 m, 181° de giro
  e ZERO inversões — as três erradas ao mesmo tempo, e **todas contra quem usa
  ré**. O caminho estava certo (frente · ré por 0,86 m · frente, arcos de
  ~0,41 m); a régua, não. Suavizador e planner foram descartados como causa
  antes do conserto, cada um com seu teste.
- Consertado partindo o caminho nas cúspides. **5 testes novos**, com geometria
  do caminho real. A bancada não tinha teste nenhum — foi assim que sobreviveu.
- **Trecho curto entre cúspides agora é pulado e CONTADO** (coluna `curt`), em
  vez de virar `raio_min = 0,00`. Dentro desse defeito havia um achado de
  verdade: com raio grande, o Smac **treme em cima do alvo** — no `bloco` com
  0,46 m são 4 inversões dentro de uma caixa de 9 cm, depois de 4,86 m limpos.
  Fica para o seguidor.
- ~~**Por consertar**: o `giro` de arco contínuo sai curto.~~ **FEITO na 4ª
  leva** (abaixo), e atrás dele havia mais dois defeitos.

### O veredito da varredura: o ranking não vira, ele se acentua

| raio que a máquina fecha | Theta\*: caminhos seguíveis | Smac: idem |
|---|---|---|
| 0,25 m | 4/6 | 6/6 |
| 0,34 m | 3/6 | 5/6 |
| 0,37 m | 3/6 | 6/6 |
| 0,46 m | **0/6** | 6/6 |

O Theta\* sai **idêntico nos quatro raios** (não conhece raio) — é a testemunha
de que a varredura mexeu só no que devia. Quem se move é a linha que ele precisa
cruzar. Nos dois casos "de lado" ele falha em qualquer raio: desenha reta
lateral, que só serve para robô que pivota. O Smac cobra caminho mais longo, e o
preço sobe com o raio (1,50× → 2,12× no alvo perto e de lado).

**A decisão 008 pode ser assinada sem esperar a zona morta**: a medida que falta
muda o tamanho da vantagem, não quem vence. Falta o julgamento do dono.

## 🧭 2026-07-29 (4ª leva) — O viés do giro, e dois defeitos escondidos atrás dele

O `giro` encolhia arco contínuo (90° lidos como 50°) e **favorecia o Smac na
comparação que ele arbitra** — o canto vivo do Theta\* tem vértice e era contado
inteiro. Consertado somando nos pontos crus, o que exigiu derrubar antes a
premissa da reamostragem: medido, o passo cru é limpo (Theta\* anda 0,05 m com
virada mediana de 0,00°; o Smac 0,086 m com virada máxima de 19,9°, que é o arco
no raio configurado). A justificativa antiga descrevia a cúspide, não ruído.

Somar no cru expôs dois defeitos que a reamostragem escondia:

- **Tocos de ponta**: o planner cola a pose exata de partida/chegada no caminho
  discretizado e sobra um segmento de 7,7 mm em cada ponta, injetando ±125,6°.
  Costurados fora por limite relativo (metade do passo típico). O comprimento
  não muda — o robô percorre o toco, ele só não define rumo.
- **Cúspide rasa**: a dobra geométrica nas inversões do `lado_1m` mede 147° e
  passava por baixo do limiar de 150°. A pose resolve: projetando o passo no
  rumo, o caminho é `+----------------+` — 1 passo à frente, 16 de ré, 1 à
  frente. A detecção passou a usar **pose quando existe, geometria quando não**
  (o Theta\* devolve orientação zerada — faixa de yaw de 0,0° em 144 pontos).

```
lado_1m @0.25    antes: giro 437°  inv 0        depois: giro 144°  inv 2
```

**O veredito não mudou** (a seguibilidade sai do `raio_min`, não tocado). O que
mudou é que a coluna do giro passou a servir: o Smac mexe MAIS o bico em todos
os casos com obstáculo (é o custo de curvar em vez de pivotar), e os dois zeros
do Theta\* nos casos de lado não são virtude — é reta lateral para um robô que
teria de pivotar. A ré do Reeds-Shepp aparece constante nos casos de lado: 2
inversões nos quatro raios. 294 testes verdes.

⚠️ **Próxima correção da régua, não feita aqui**: a reamostragem passa por cima
do canto vivo do Theta\* e devolve 0,37–0,39 m onde a virada é um canto
(curvatura infinita). O `raio_min` faz o Theta\* parecer MAIS seguível do que
é — o veredito de hoje é conservador, e distinguir canto de arco só pode
melhorar o lado do Smac.

## 🔙 2026-07-29 (7ª leva) — A ré volta a ser por gatilho (decisão 009)

O dono derrubou a seção 3 da 008 com razão de campo: ré planejada em robô com
Nav2 faz o robô ficar tentando entrar e seguir os trechos de ré. A bancada não
sabe arbitrar isso (ela desenha, não dirige), então mediu-se o custo de cada
saída antes de decidir.

- **Encarecer a ré não a elimina**: `reverse_penalty` de 2 a 40 não muda as
  inversões — nesses casos ela é geometricamente necessária.
- **Proibir a ré (Dubins) custa**: `bloco` fica sem caminho nos 4 raios, e os
  alvos "de lado" passam de 1,50–2,12 para 3,61–5,66 de desvio (~3 m de caminho
  para um alvo a 0,60 m). `porta`, `aperto` e `beco` não sentem.
- **Recuar NÃO salva o plano Dubins** — hipótese minha, testada e derrubada:
  recuar reto não muda o RUMO, e o alvo segue a 90° do bico.
- **O que sustenta a ré por gatilho é o PIVÔ**, e o robô 1 pivota. O robô 2 não,
  com os parâmetros de hoje.

**Decisão 009 aceita**: planner em `DUBIN`, ré por gatilho no seguidor, disparada
por SINTOMA (não progrediu) e não por geometria. Fica registrado que **a zona
morta reabre esta decisão**: se o pivô não existir, a 008 seção 3 volta com
número. É a terceira razão de peso para o item nº 1 da bancada.

**Fatia B do seguidor feita**: `ProgressoDeAvanco` (mede aproximação, não
velocidade — órbita tem velocidade e não tem progresso), orçamento de ré cego de
0,30 m enquanto o Mid-360 não estiver no modelo, e teto de tempo além do de
metros (se a pose não muda, o orçamento em metros nunca é gasto). 316 testes.

## 🚗 2026-07-29 (9ª leva) — A pilha inteira anda

`ros2 launch robot_motion pilha.launch.py sim:=true` — Gazebo + Nav2 + seguidor
+ movimentação, e o robô vai onde se clicar. Dois alvos verificados: 90° atrás
(15,2 s, 1,67x a reta) e através da porta de 0,90 m (12,3 s, 1,04x).

Quatro defeitos achados e corrigidos, três deles do Nav2 discordando de si mesmo:
a árvore padrão exige `spin`/`backup` (pivô que este robô não faz e a ré que a
009 tirou) → trocada pela `navigate_w_replanning_time.xml`; a segunda árvore
(`navigate_through_poses`) derrubava a subida → `navigators` restrito; e o
`controller_server` **não dirige mas ABORTA** — o detector de colisão dele
derrubou a navegação com o robô já do outro lado da porta.

**Inflação medida como alavanca**: 0,45 trava o robô DENTRO da porta de 0,90 m
(`Start occupied` no replanejamento, porque o vão inteiro fica inflado); 0,30
passa em 1,04x. Produção em 0,30, bancada em 0,45 (é o número dos 48 planos da
008) — divergência deliberada e travada em teste.

⚠️ **Anotado sem conserto**: o alvo a 90° atrás custa 1,67x, e o gatilho de ré
disparou **a 0,43 m do objetivo**. Ré perto da chegada é suspeita e é a primeira
coisa da próxima sessão. E a TF `map→odom` é fixa — vale no simulador (mundo e
mapa saem da mesma planta), **não vale no robô real**.

## 🧰 2026-07-30 — Kit de bancada pronto, e o robô do estágio como testemunha

**`tools/banco/sessao.py`**: o protocolo inteiro do `tools/banco/README.md` num
comando — os 6 passos em ordem, 11 corridas, pausando para reposicionar, tudo
numa pasta só com `ambiente.txt` (piso, bateria, commit) e `leituras.txt`. Chama
o `medir.py` depois de cada corrida, então o número sai ainda com o robô ligado.
Folha de campo: **`tools/banco/CHECKLIST_ROBO.md`**.

Duas defesas que o `ensaio.py` sozinho não tem:

- **conferência que bloqueia** — sem `/Odometry`, sem ouvinte no `cmd_vel`, ele
  recusa medir e diz por quê. CSV gravado com a base incompleta sai limpo e
  errado, e isso só se descobre em casa;
- **cutucão de sanidade** (`--checar --mexer`) — anda 2 s, gira 2 s e confere o
  **sinal**. Roda trocada na fiação dá robô que anda certo e gira ao contrário, e
  nenhum dos 6 ensaios acusa (eles medem magnitude).

Provado ponta a ponta contra o Gazebo headless, e no caso negativo também (pilha
derrubada → recusa). 380 testes verdes.

### O workspace do estágio (`ESTAGIO-2026/`, fora do git) — 3 hipóteses

Mesma máquina, pilha Nav2 de fábrica, **anda e faz SLAM** (o que o nosso ainda
não faz no robô), se perde, e recupera quase só de ré. Lido contra a nossa trena:

1. **Dois raios de roda contraditórios na mesma pilha**: `0.0425` no
   `diff_drive_controller`, `0.0825` no plugin de hardware. A conversão do driver
   (`rad/s ÷ 0,10472`) não usa raio, então quem fixa escala é o controlador —
   com a nossa medida de 0,080, a roda gira **~1,9×** mais que os m/s pedidos.
2. **`wheel_separation: 0.32`** contra os 0,270 medidos → gira ~19% a mais que o
   comandado. É o mesmo desvio que anotamos em 29-07 quando o 0,32 era nosso.
3. **O piso de velocidade do mux deles protege a reta e não o giro**: 0,10 m/s
   no linear, 0,15 rad/s no angular — que com bitola 0,32 são **0,024 m/s de
   roda**, 4× abaixo da faixa de zona morta plausível. O `Spin` do Nav2 decai
   até 0,4 rad/s (0,064 m/s de roda, também abaixo); o `BackUp` é linear puro e
   o piso o levanta sempre. **Hipótese: "só vai de ré" pode não ser a boba — pode
   ser que a ré seja a única recuperação que fisicamente acontece.** É a nossa
   BO-3 vista de fora, num robô que já roda.

Nenhuma é fato nosso. As três se resolvem com os **ensaios 2 e 4** — quarta razão
de peso para o item nº 1 da bancada.

## 📊 2026-07-31 — O protocolo passa a repetir, e a zona morta vira dente de serra

Véspera da ida ao robô. Duas perguntas do dono derrubaram partes do método.

- **Nenhum ensaio repetia.** As 11 corridas eram 11 condições diferentes — os
  "3" dos passos 3, 4 e 6 são três *valores*, não três tentativas. Todo número
  sairia com n=1, sem faixa. Agora são **27 corridas**: reta, curva e aceleração
  ×3 (decisão do dono: repete o que identifica erro), com `--repete 1` para
  encurtar se a bateria cair. Cada grupo imprime **média, faixa e dispersão** ao
  fechar, e delata quando uma corrida morreu e a repetição encolheu.
- **Uma corrida de controle, girada 180°** (passo 6). Não é repetição: média
  mata erro aleatório e **não mata erro sistemático**, então três retas do mesmo
  ponto medem o caimento do piso e a média sai confiante e errada. É a única
  corrida que separa robô de sala. Ponto 0 marcado com fita **e com o rumo**.
- **Zona morta virou DENTE DE SERRA**: sobe até sair do lugar, desce até parar,
  inverte o sentido, 4×. Uma corrida dá **4 saídas e 4 quedas**, nos dois
  sentidos, sem reposicionar — a repetição vive dentro dela. E mede dois números
  onde havia um: **saída** (do repouso, atrito estático — o do BO-3) e **queda**
  (já andando, menor — é ela que o piso de velocidade do seguidor precisa).
- **A primeira versão não cabia na sala**, e quem disse foi o teste: virando por
  tempo, o ensaio linear se afastava **5,25 m** contra trava de 3 m que mata a
  corrida. O dente passou a virar **no evento, não no relógio** — excursão de
  0,059 m e 4 dentes em 18 s (eram 160 s). É o único ensaio em malha fechada.
- **A taxa da rampa virou parâmetro nomeado** (`--rampa-seg`): o limiar é lido na
  primeira amostra que passa de `LIMIAR_PARADO`, então rampa mais rápida infla o
  número. No giro a taxa atual já infla ~0,011 rad/s — 10% do que se distingue.
- **`tools/banco/` ganhou 17 testes** (não tinha nenhum, que é como a régua do
  planner sobreviveu errada em 29-07). O do dente fantasma verificado por
  mutação. Suíte: **397 verdes** (eram 380).
- ⚠️ **O Gazebo provou o mecanismo, não o número**: lá o `ensaio.py` publica
  direto no `cmd_vel` do controlador e passa **por fora da placa fingida**, então
  os 0,023 m/s e 0,036 rad/s medidos são piso de detecção, não zona morta.

### O giro vira o ensaio 1, e o dente #0 é um caso à parte

**Ordem invertida a pedido do dono**: a zona morta de **giro** passa a ser o
passo 1. Ela responde a pergunta do pivô **diretamente** — o menor `wz` que gira
o robô parado *é* o limiar do pivô, sem converter por bitola. Pelo linear só se
chega lá por `2·zm/L`, confiando de novo num número medido. Some o risco de
sessão cortada (em 30-07 não se mediu nada): o que fica por último se perde.
O linear ganhou papel melhor — **conferência do passo 1**, já que os dois medem
o mesmo atrito por caminhos diferentes e têm de fechar por `2·zm/L`.

**O dente #0 destoa, e é física.** Na validação no Gazebo, dispersão de 82% no
passo 1 (#0 em 0,188 rad/s contra 0,035 dos demais). Só o #0 parte de repouso
longo; os outros, da pausa de 1 s. Atrito estático cresce com o tempo parado, e
a média dos quatro misturaria as duas condições **bem no número do BO-3**. O
`medir.py` separa e diz qual usar para cada caso, em vez de corrigir a média —
qual serve depende de quanto tempo o robô fica parado em operação, o que é
decisão de projeto.

### ✅ Fechado no mesmo dia: a conferência lê a calibração VIVA

`sessao.py --checar` pergunta ao `hoverboard_base_controller` **que robô ele
acha que está dirigindo** — `wheel_separation`, `wheel_radius` e os nomes de
roda, do nó vivo — e compara com a trena. **Delata, não bloqueia**, e grava
tudo no `ambiente.txt` com marcador `*** DIVERGE DA TRENA ***`.

Fecha o buraco de que o `ambiente.txt` gravava só o **commit**, que descreve o
fonte: quem dirige o robô é o `install/`. Sem esse registro, um limiar medido
não tem como voltar a ser velocidade de roda — vira número sem unidade.

Os nomes de roda entram junto porque é neles que vive a correção do **giro
espelhado**: a conferência diz `swap APLICADO` / `NÃO aplicado` **antes** de o
robô se mexer, provando que o rebuild pegou sem gastar bateria. Os dois estados
e o caso divergente foram verificados ao vivo contra o Gazebo.

A folha de campo virou **executável de ponta a ponta**: bloco "para quem for
conduzir", ordem fixa, cinco coisas que não se faz, e o swap de rodas como
script copiável (testado e revertido; **não commitado** — só entra no git
depois de o cutucão validar, decisão de 30-07).

### Aberto: `install/` velho envenena todos os limiares

A bitola e o raio da trena moram em arquivos que o `tracao.launch.py` lê de
`FindPackageShare` — da cópia **instalada**. `git reset --hard` troca o fonte e
não troca o `install/`. Se o build faltar, o robô sobe com 0,32/0,0825 e todo
limiar sai 18,5% enviesado, sem sintoma. A folha de campo ganhou o `colcon build
--packages-select hoverboard_driver` e um `grep` de conferência.

**FEITO no mesmo dia** (seção acima). Importava porque a comparação roda × lidar
é limpa na reta (raio de roda puro) mas **não separa bitola errada de
escorregamento** no giro: 0,32 num robô de 0,270 faz girar 18,5% a mais,
derrapar faz girar menos, e `1,185 × 0,82 ≈ 0,97` leria como "quase não
derrapa". Com a calibração gravada ao lado do CSV, os dois voltam a ser
separáveis em casa.

## 🎛️ 2026-08-01 — O atuador medido entra no simulador, e a decisão 005 cai

Sessão sem robô, em cima dos CSV de 31-07.

- **Confirmei os números da bancada refazendo as contas.** A zona morta de giro
  (0,091–0,096 rad/s) é firme: critérios de 2°, 3° e 5° dão 0,091/0,096/0,096,
  com os 4 dentes concordando. O patamar também: comando de 0,10 m/s virou
  **0,26–0,28 m/s de borda** nas quatro rajadas.
- **Fechei o modelo do atuador** com o que faltava: latência **0,273 s** (n=4),
  patamar **0,297 m/s de borda**, aceleração 0,435 m/s², desaceleração
  0,373 m/s², e a escala real do firmware **0,0372 rad/s por unidade** contra os
  0,10472 que o driver assume — **o driver superestima a roda em 2,8×**.
- ⚠️ **A "zona morta de giro" NÃO é atrito, é aritmética do driver.** A
  compensação dispara em `mx > 1.0`, o que dá `wz > 0,0621`; os 0,03 de
  diferença são a latência sobre a rampa. O número descreve o **sistema**, não a
  máquina, e se move se alguém mudar `deadband_speed`. **O atrito segue não
  medido.**
- ⚠️ **A pergunta do pivô foi dissolvida, não respondida.** Com a compensação
  ligada qualquer `wz > 0,062` pivota, na velocidade do patamar. A aritmética da
  folga de 4% ficou sem objeto: o pivô existe, o que não existe é controle da
  velocidade dele. **Mexe na premissa da decisão 009.**

### ⛔ A decisão 005 não sobrevive ao atuador medido

Passando a própria lei da 005 pelo modelo: de **1° a 180°** de erro de rumo, a
lei pede 8 valores distintos de `wz` e a placa entrega **um só** (2,204 rad/s,
2,2× o máximo que a lei pediria). A frenagem de rumo — resultado central da 005,
validado em 10 corridas de simulador — **não existe neste robô** com a
compensação ligada. O mesmo vale para `v = √(2·a_lin·dist)`, e o `v_piso` fica
sem sentido: não há velocidade abaixo do patamar.

**O contrato de que `cmd_vel` está em m/s é falso em toda a faixa que a navegação
usa** — e a movimentação inteira foi escrita sobre esse contrato.

### O simulador agora erra como o robô erra

`placa_simulada.py` reescrito: reproduz a conta do driver linha a linha, depois
aplica escala real, latência e assimetria por sentido. Três modelos —
`medido` (padrão), `cru` (compensação desligada) e `ideal` (fio).
`sim.launch.py placa:=…` e `pilha.launch.py placa:=…`; o `zona_morta:=` saiu.

Provado no Gazebo, mesma ordem nas duas placas:

```
comando      placa ideal      placa medida
0,10 m/s ->  0,200 m em 2 s   0,548 m
0,25 m/s ->  0,500 m          0,541 m
0,50 m/s ->  1,000 m          0,541 m   <- as tres iguais
```

**419 testes verdes** (eram 407), 8 novos travando o modelo — inclusive o do
colapso da lei da 005.

⚠️ **Tropeço meu, igual ao do laboratório:** a primeira prova deu medida ≈ ideal
porque havia **quatro `placa_simulada` órfãs** acumuladas, de lançamentos que
derrubei com `pkill` de padrão largo. Contar processo vivo antes de medir entrou
no procedimento.

## 📋 PARA IR AO ROBÔ: `docs/PLANO_TESTE_ROBO.md`

Escrito em 05-08. Responde o que testar, com que mapa e como rodar. O resumo
que decide a próxima sessão:

- ✅ **Testável JÁ, sem mapa nenhum**: compensador de rumo (reta e ré), pivô,
  `twist_mux` + teleop (freio de mão) e reflexo de colisão. Os quatro usam só
  o `/Odometry` do LIO e a nuvem do Mid-360.
- 🔴 **Bloqueado**: tudo do Nav2 (planner, seguidor, chegada com ângulo).
  E **não é falta de mapa** — é falta de **localização contra o mapa**. A
  `pilha.launch.py` publica uma TF `map→odom` FIXA, que só vale no simulador
  porque mundo e mapa saem da mesma planta.
- ⚠️ **O mapa do repo é sintético** (`GERADO por gera_pista.py`) e não
  corresponde a lugar nenhum. **O mapa do estagiário NÃO está neste repo** —
  o workspace dele está fora do git.
- ➡️ Saída recomendada: **SLAM online** (o mapa nasce enquanto anda, `map ≡
  odom` por construção). É a menor mudança que destrava, e mantém a 003.

## 🎯 O TRABALHO ATUAL: decisão 011 — malha fechada de rumo (o "PID")

Aprovada pelo dono em 04-08 ("focar tudo nesse PID"). Plano em 5 fatias;
registro completo em `docs/decisoes/011-malha-fechada-de-rumo-em-reta.md`.

- ✅ **Fatia 1 — reta e ré no Gazebo: FEITA E ACEITA (04-08, 5ª leva).**
  `compensador_rumo` (nó novo no `robot_motion`, lei pura em
  `lei_de_reta.py`): entra `cmd_vel` desejado, sai `cmd_vel` corrigido,
  yaw do `/Odometry` fechando a malha. ff medido (−0,817/−0,098) + PI
  (kp=1,0, ki=0,5). Bancada n=3 por sentido:

  ```
                   SEM              COM           critério
  frente         -0,82 1/m       -0,0025 1/m       <0,05    passa 20x
  ré             -0,11           -0,0003           <0,05    passa
  ```

  Dados: `docs/dados/2026-08-04-fatia1-compensador/`. O Ki existe para
  zerar o RUMO, não a curvatura (sem ele o robô anda reto, mas ~6° torto)
  — descoberto pelo próprio teste, travado em teste.

- ❌ **Fatia 2 — atraso de desliga na placa fingida: DESCARTADA na medição
  (04-08, 6ª leva).** O gap que a motivava ("sobrepasso 99° no Gazebo contra
  49° no robô") era artefato de comparar sobrepassos **a partir do pico**,
  que repartem diferente nos dois lados. O número que o pivô consome é o
  **giro total entre o comando zerar e o robô parar**, fase de empurrão
  incluída — e nele os dois batem:

  ```
              giro total do corte à parada      tempo até parar
  ROBÔ        102° / 114° / 124°  (~113°)       1,94–2,02 s
  SIMULADOR   117° / 120° / 117°  (~118°)       1,82–1,86 s
  ```

  ⚠️ **Condição de projeto que isso impõe à fatia 3**: o controlador do pivô
  pode consumir o **giro total** e a **detecção de parada**, mas **não a
  forma da frenagem** (pico, wz no meio do caminho) — a forma é onde o
  simulador ainda mente. Se o projeto precisar da forma, a fatia 2 volta.

- ⏳ **Fatia 3 — pivô por corte previsto** (desbloqueada pelo achado acima).
  NÃO é PID: a placa entrega um wz só, então a única alavanca é **decidir
  quando cortar**. Dado de partida: depois do corte o robô ainda varre
  ~113°, o que levanta a pergunta de projeto que a fatia tem de responder
  primeiro — **existe pivô menor que isso?**
- ⏳ **Fatia 4 — robô real.** Mesmo protocolo da bancada de 04-08, o dono
  conduz, `ensaio.py --topico /compensador_rumo/cmd_vel`. É quem julga de
  verdade: no Gazebo o ff é exato por construção; no robô a planta do dia
  difere do ff em até 21% e o integrador é quem paga.
- ⏳ **Fatia 5 — integração permanente** no módulo de movimento (launch,
  prioridade humana no mux, delator do BO-3 nesta camada).

## 🪞 Pendências abertas por 04-08

**Item 12 (calibração do simulador):** o passo 12a (arco) fechou, 12b (aceitação)
está medido e dentro da dispersão do robô. Ficou aberto:

- **12c — Dispersão**: o simulador dá 4% de frente contra 21% do robô, e **0%**
  na ré contra 37%. Determinista demais faz controlador parecer mais repetível
  do que vai ser. É decisão de projeto (injetar ruído, de que tipo), não foi
  tomada. Deixa para quando o controlador já estiver operacional.

- **Atraso de desliga da placa** (novo achado em 04-08, 3ª leva): a placa
  empurra ~0,51 s DEPOIS do comando zerar. O parâmetro `latencia` da
  `placa_simulada` modela o atraso de liga (aquele que trava por 0,27 s no
  arranque), não o de desliga. Com só a latência de liga, o Gazebo desacelera
  2× mais rápido que o robô (sobrepasso 94° contra 49°). Precisaria entrar na
  lógica do `cb` do nó, não é parametrização.

- **Teto de aceleração angular** (corrigido em 04-08, 3ª leva): `angular.z.max_acceleration`
  com valores baixos (0,3 na planta lenta) limitava o pico indiretamente. Com a
  planta normal (1,5) ele sobe de 0,45 para 2,25 rad/s e agora bate. Mas o
  sobrepasso sai 2× maior por falta do atraso de desliga.

- **Velocidade sustentada acima do patamar**: nunca foi medida acima de 0,838 m/s
  em nenhum dos dois lados (é onde o comando volta a ser proporcional).
  Recomputado mas não validado em hardware.

- **Mecanismo (BO-4)**: o arco entra disfarçado de assimetria de roda,
  porque a placa só tem rodas para escrever. Prova de que entra: 24,8% de
  assimetria necessária contra 11–12% medidos pelo encoder. O encoder
  simulado mente e **quebra no dia em que ligarem `open_loop: false`**.
  Reproduz o sintoma, não o mecanismo.

## ⏳ Próximos passos

**Primeiro, com o robô (virou prioridade — a movimentação depende destes
números e hoje eles são chute):**

1. ~~Medir `wheel_separation` e `wheel_radius` com trena.~~ **FEITO 07-29**:
   bitola 0,270 e raio 0,080, os dois já no URDF e nos três YAMLs. Em seu lugar,
   o novo item nº 1 é a **zona morta** — é ela que agora decide se o robô
   consegue pivotar (ver a nota superada acima), e a folga no melhor caso é de
   4%. Medir a rodinha da boba junto, se der.
2. ~~**PRIMEIRO: corrigir o giro espelhado** (bloqueio 07-30).~~
   🛑 **NÃO FAZER. O giro espelhado NÃO EXISTE** (08-04) — era o `atan2` do
   cutucão enrolando em ±180°. O swap foi aplicado (`368ea13`) e **revertido**
   (`595cf80`); **o estado atual, SEM swap, é o correto**. Aplicar aquele script
   quebra um robô que está certo. O `atan2` foi **consertado** em `7a0c364`, então
   o veredito de giro do `--checar --mexer` voltou a valer — se ele acusar agora,
   é para levar a sério, mas confirme com o olho de alguém atrás do robô antes de
   mexer em qualquer coisa.
3. ~~**Rodar a sessão de bancada** (27 corridas).~~ **PARCIALMENTE FEITO**: o
   banco está em **4 de 6** (zona morta de giro e linear em 07-31; `a_dec` e
   desvio de rumo em 08-04). Falta:
   - **passos 4 e 5** (curva e aceleração), que **precisam ser reescritos** — a
     varredura de velocidade cai inteira dentro do patamar da compensação e tem
     de subir acima de 0,838 m/s, o que exige espaço;
   - **filmar a boba** — o vídeo da traseira em corrida de ré é a única medida
     possível dela, e não foi trazido para o repo.
   ⚠️ Rodar pelo `sessao.py` **não serve como está**: ele não deixa passar
   `--espaco` por fora (padrão 4,0 m, que num robô que arca é excursão lateral
   demais) e o passo 6 dele está morto. Conduzir corrida a corrida pelo
   `ensaio.py` com `--espaco` pequeno, como em 08-04.
   O dono só roda; os CSV vêm por bundle/ssh (o NUC não tem autenticação no
   GitHub — ver a dívida de infra na entrada 07-30 do diário).
4. **IP do lidar** — confirmado em `192.168.1.169` na sessão de 07-30, igual ao
   config. Mas o Mid-360 pode ficar **mudo mesmo com o IP certo**: ele tranca a
   sessão de dado se o driver morrer no meio do handshake (não usar `kill -9`;
   SIGINT e esperar). Sintoma: pinga e ACKa, RX de ~6 pacotes/3 s, `/livox/lidar`
   mudo. Cura: power-cycle do lidar. Ver a entrada 07-30 (2ª leva) do diário.

**O simulador está pronto para rodar controlador:**

  Rodá-lo com `planta:=normal` (padrão; a_dec 1,5 rad/s²). A `lenta` (0,3)
  era propositalmente pessimista para validar a lei de frenagem da decisão 005
  — não é a planta do robô real. Com a normal o pico de wz sai 2,25 rad/s
  (robô tem 2,33), que agora bate. O sobrepasso fica 2× maior por falta do
  atraso de desliga, mas isso não vai descer antes de modelar esse atraso.

**Sem o robô, e agora urgentes (01-08):**

5. **Decidir o que fazer com a compensação de zona morta do driver.** É ela que
   cria o patamar e derruba a decisão 005. Três saídas, e nenhuma é de graça:
   (a) desligar e viver com a zona morta física — que só está bracketada entre
   0,25 e 0,5 m/s de borda, e se for isso a velocidade mínima do robô é rápida
   demais para chegar num ponto; (b) mapa estático melhor — **impossível**:
   nenhum preserva razão entre rodas E magnitude abaixo do limiar físico, é o
   que "zona morta" significa; (c) **malha fechada** por roda, que é a saída de
   verdade e o driver já tem o que ela precisa.
6. **`open_loop: false`** — é de graça e teria evitado a sessão perdida de 31-07.
   O driver já exporta posição **e** velocidade por roda (é de lá que saem
   `v_esq`/`v_dir` nos CSV); a odometria de roda hoje é o comando ecoado por
   escolha, não por falta de encoder. Trocar dá a comparação roda × lidar.
7. **Raio de inflação menor que o inscrito** (achado ao rodar a pilha em 01-08):
   `0.300` configurado contra `0.363` de raio inscrito do footprint da trena. O
   Nav2 reclama nos dois costmaps. É defeito, não aviso cosmético.
8. **Comparar a pilha inteira nas duas placas** — não rodou em 01-08 (`base_link`
   ausente na TF na subida). É o que mede o estrago de ponta a ponta.

**Pontas soltas de 08-04 (baratas, e cada uma já custou algo):**

8a. **`rajada_rodas.py` ainda não percebe que o robô sumiu.** O `ensaio.py` foi
   consertado (`7d7fad8`: aborta em `--sem-dado` e sai com código 1), o outro
   não. Foi esse defeito que gravou 7 corridas em branco com cara de sucesso em
   31-07. Mesmo conserto, mesmo teste.

8b. **Trazer o vídeo da traseira** filmado em 08-04. É a única evidência direta
   do garfo da boba e o único caminho para o critério (b) do BO-4. Vai por fora
   do repo (grande demais) — mas o `ambiente.txt` tem de dizer onde ele está.

8c. **Registrar piso e bateria** das nove corridas de 08-04, se ainda der para
   lembrar. Estão como `NÃO INFORMADO`, e sem eles a sessão não se compara com a
   próxima.

8d. **Reescrever os passos 4 e 5 do banco** antes da próxima ida ao robô. Os dois
   varrem 0,2 / 0,4 / 0,6 m/s e as três caem dentro do patamar — medem o driver,
   não o robô. A varredura tem de subir acima de 0,838 m/s, e isso muda o espaço
   que a sessão precisa. **Reescrever em casa é barato; descobrir no laboratório
   custa a sessão.**

8e. **`sessao.py` não conduz mais uma sessão deste robô.** Não deixa passar
   `--espaco` por fora (padrão 4,0 m, que num robô que arca é excursão lateral
   demais), o passo 6 dele está morto e os passos 4 e 5 medem o patamar. Ou
   ganha `--espaco`/`--dur` por fora, ou o protocolo passa a ser conduzido pelo
   `ensaio.py` corrida a corrida — que foi o que funcionou em 08-04.

**Assim que os dados da bancada chegarem:**

9. **Levantamento da camada de segurança do robô 1** (decisão 010) — ler
   `collision_monitor`, `motion_guard` e `unstuck_supervisor` arquivo por
   arquivo e dizer, com número, o que sobrevive ao Mid-360. Expectativa
   preliminar: o `collision_monitor` é config + geometria deste chassi; o
   `motion_guard` **encolhe** (parte dele existe para caçar "fantasma de vidro"
   do LD06); o `unstuck_supervisor` tem a ideia agnóstica e 1433 linhas moldadas
   em varredura planar. Roda DEPOIS dos dados porque os polígonos e limites a
   re-derivar dependem da zona morta e do `a_dec` medidos.

**Sem o robô:**

10. ~~Varrer o raio mínimo e julgar o planner.~~ **FEITO 07-29**: 48 planos, o
   ranking não vira entre 0,25 m e 0,46 m, e a **decisão 008 está escrita**
   (`docs/decisoes/008-nav2-planeja-nos-seguimos.md`) e **ACEITA pelo dono**.
   O Smac Hybrid-A\* é a escolha oficial; o Theta\* sai.
11. **Modelo 3D real do robô** no simulador (o dono vai levantar), com o
   Mid-360 no topo. É ele que troca a fonte de obstáculos do mapa estático
   para o sensor, e corrige footprint e bitola do modelo.
12. ~~**Calibrar o simulador contra o robô**~~ ✅ **FEITO 08-04 (2ª leva)** —
   **12a e 12b fechados; 12c segue aberto.** O simulador arca como o robô arca,
   medido com o mesmo instrumento nos dois lados (n=3 por sentido):

   ```
                   SIMULADOR        ROBO (04-08)
   frente          -0,817 1/m       -0,838 1/m
   re              -0,109           -0,113
   razao               7,5x             7,4x
   ```

   As três caem dentro da dispersão do próprio robô. Dado e leitura em
   `docs/dados/2026-08-04-aceitacao-simulador/`; entrada 08-04 (2ª leva) do
   diário. Antes disso ele dava −0,419 de frente e **zero** de ré.

   🔴 **O ponto de partida escrito abaixo estava errado**: dizia que o
   simulador reproduzia "~1×". Isso era de 28-07 — a reescrita de 01-08 já
   tinha assimetria por sentido. O real era **∞×** (ré perfeitamente reta).

   🔴 **Um defeito meu, que só a corrida no Gazebo pegou:** inverti os
   parâmetros com `curvatura = wz/v` (v com sinal) enquanto a bancada mede
   `Δyaw/caminho` (sem sinal). As duas convenções concordam de frente e **se
   opõem de ré** — o simulador arcava para o lado errado indo de ré, com a
   **suíte verde**, porque o helper do teste usava a mesma convenção do erro.
   Régua e objeto medidos com o mesmo viés, igual à régua do planner em 29-07.
   Conserto de raiz: os parâmetros do nó passaram a ser a **curvatura medida** e
   a assimetria de roda é **derivada** (`a = −2cL/(2s + cL)`), então o sinal da
   ré sai sozinho.

   ⚠️ **`rendimento_giro: 0.80`** (novo): o Gazebo entrega 80% do giro pedido —
   é derrapagem do contato simulado (bate com os 79–86% de 29-07), **não é do
   robô**. Mexer em atrito, massa ou planta do Gazebo obriga a **refazer a
   corrida de aceitação**.

   ⚠️ **`ensaio.py` ganhou `--topico`**, e no simulador ele é obrigatório: sem
   `--topico /cmd_vel_bruto` o ensaio publica direto no controlador e **passa
   por fora da placa fingida** — era a limitação anotada em 31-07 ("o Gazebo
   provou o mecanismo, não o número").

   **Plano original, para registro:**

   **12a. Injetar o arco dependente de SENTIDO** no `placa_simulada.py` (ou logo
   acima dele): `−0,82 1/m` indo para a frente, `−0,10 1/m` de ré. O atuador
   (patamar, latência de liga e de desliga, escala do firmware) **já está** lá
   desde 01-08 e ficou validado em hardware em 08-04 — o que falta é o corpo.

   ⚠️ **Isto entra FENOMENOLÓGICO, e o comentário no código tem de dizer isso.**
   Não dá para derivar o arco da geometria enquanto o BO-4 estiver aberto: a boba
   do simulador é um patim (multiplicar o atrito dela por 16 mudou o rumo em
   0,4%). Consequência que precisa estar escrita ao lado do número: o modelo
   **reproduz o sintoma, não o mecanismo**, então serve para desenvolver
   controlador e **não** serve para responder "e se" — outra carga, outro piso,
   ou depois de consertar a boba.

   **12b. Escrever a corrida de ACEITAÇÃO** contra
   `docs/dados/2026-08-04-bancada-robo/`. Mesma manobra, alvo numérico:

   ```
   frente:  razão caminho/afastamento  2,09x em 3,7 m de percurso
   ré:      1,08x
   razão frente/ré da curvatura:        8,3x
   ```

   Vira teste, não impressão — e o primeiro valor dele é medir **o quanto o
   simulador de hoje já está longe**, antes de mexer em qualquer coisa.

   **12c. Só então decidir se entra DISPERSÃO.** Medida em 08-04: 21% na
   curvatura de frente e 50% no `a_dec` efetivo, entre corridas idênticas. Um
   simulador determinístico devolve sempre o mesmo número e faz qualquer
   controlador parecer mais repetível do que vai ser — que é a forma clássica de
   o simulador enganar. Fica por último porque é decisão de projeto, não de
   ajuste.

   ⏳ **SEGUE ABERTO, e agora com número dos dois lados** (08-04, 2ª leva):

   ```
                    dispersao SIM   dispersao ROBO
   frente                4%              21%
   re                    0%              37%   (0,0002 contra 0,04 absoluto)
   ```

   O simulador é determinista demais. Decidir se entra ruído, e de que tipo,
   é decisão de projeto — não foi tomada.

   **O que o simulador NÃO vai cobrir, e é para estar escrito no `README` dele:**
   acima de **0,838 m/s** nunca foi medido (é onde o comando volta a ser
   proporcional); o **atrito real** segue não medido (os 0,095 rad/s são
   aritmética do driver, não atrito); e as nove corridas de 08-04 saíram com
   **piso e bateria não registrados**, então a rigor não se sabe a que condição
   os números pertencem.
13. **Seguidor próprio** por cima do plano do Nav2 — destravado pela 008, é a
   fatia grande seguinte. Carrot no plano, como no robô 1, por cima da
   movimentação da decisão 005.


## Fósseis conscientes (remover em fatia própria)

`robot_nav/mega_bridge.py` e `robot_nav/cmd_vel_to_wheels.py` deixaram de ser
caminho (decisão 003) mas **não foram apagados**: a GUI e a suíte de testes
ainda os referenciam, e remover tudo junto quebraria as duas. Também seguem
inertes o standdown de porta no `unstuck_supervisor` e o `cone_pose_fix.py`.

## BOs abertos

- ~~**BO-1 — Arquitetura da ponte PC↔placa hover**~~ ✅ **FECHADO 07-24**:
  serial direta, sem microcontrolador. Registro na decisão 003.
- **BO-2 — O que o artigo compara** (aberto 07-24): a decisão 003 tirou do
  caminho o baseline 2D vs LIO que a 002 previa como resultado. Candidatos:
  comparar métodos de LIO entre si, ou comparar estratégias de controle de
  rumo para esta geometria (motriz dianteira + boba traseira) — que é o
  problema real em mãos. **07-27: o segundo candidato ganhou corpo** — o S
  está explicado por `wz²/(2·a_dec)` e há uma lei que o elimina, com 10
  corridas medidas. Falta a comparação valer no robô.

  🟢 **06-08: o segundo candidato virou o mais forte, e por mérito próprio.**
  Há agora DOIS "S" distintos, com mecanismos diferentes, os dois medidos:
  · o de **27-07** (simulador), da rampa de desaceleração — `wz²/(2·a_dec)`;
  · o de **05-08** (robô real), de **tempo morto** — 0,94 s de atraso no laço,
    com a oscilação CRESCENDO 2,07× por meio-período.
  O segundo tem o que um artigo precisa: fenômeno medido no hardware, mecanismo
  confirmado por **duas rotas independentes** (a frequência da oscilação e a
  soma dos atrasos da placa), um conserto **projetado** a partir do número
  (ganhos 4,1× menores), uma alternativa clássica **implementada e comparada**
  (preditor de Smith, que perdeu), e uma **previsão falsificável** esperando o
  robô. E um achado de método que vale por si: o defeito era invisível em TODOS
  os modelos do projeto — Gazebo e teste unitário — porque os dois subestimavam
  o atraso.

- **BO-3 — Zona morta do atuador** (aberto 07-27): comando abaixo da zona morta
  deixa o robô **parado sem erro nenhum** — nó vivo, tópico publicando, log
  limpo, máquina imóvel. Já custou horas de depuração na competição de 2025.
  No simulador é precipício: zona morta de roda em 0,10 m/s passa raspando,
  em 0,15 m/s o robô fica 22 s plantado com o controlador pedindo 1,0 rad/s.
  **O valor real é desconhecido, e a faixa provável de uma placa de hoverboard
  cai bem em cima do precipício.**

  Defesa desenhada, em duas partes — porque prevenção pode falhar (bateria
  fraca, carga, piso diferente) e o custo real do defeito é o tempo de
  diagnóstico:
  1. *prevenir* — `v_piso = zona_morta + wz_max·bitola/2 + margem`;
  2. *delatar* — se há comando de movimento e a pose do LIO não muda por ~0,5 s,
     gritar no log com pedido e efetivo. Nunca parar em silêncio.

  **Fecha quando:** (a) zona morta medida na bancada (`tools/banco`, ensaios 1
  e 2); (b) `a_dec` medido (ensaio 3); (c) `v_piso` calculado pela fórmula com
  esses números; (d) meia-volta no robô real completando sem travar.

  Some junto o caso não resolvido: **girar parado devagar é impossível** —
  abaixo de `2·zona_morta/bitola` as duas rodas ficam na banda proibida. Isso
  não é ajuste de ganho, é limite físico, e cai no colo da navegação.
  **07-28: o alcance disso foi resolvido pela ré** (decisão 007) — o robô
  contorna a falta de pivô recuando. O limite físico continua de pé.

- **BO-4 — A boba do simulador não é uma boba** (aberto 07-28): o garfo do
  pivô **não se alinha com a direção de movimento**. Medido numa curva pra
  frente (v=0,25, wz=0,6, raio 0,42 m): ele deveria assentar a ~157° do corpo
  (`atan(0,18/0,42)` fora do eixo) e ficar lá; em vez disso saiu de 180° e
  girou continuamente até 38°, mantendo o rumo do **mundo**. É um patim, não
  uma boba.

  Multiplicar o atrito da boba por 16 (`mu 0,05 → 0,8`) mudou o rumo da mesma
  curva de 136,161° para 136,675° — **0,4%**. O contato dela não participa da
  dinâmica, e a hipótese do `mu2` baixo como causa foi testada e **descartada**.
  A causa real é desconhecida.

  **Custo:** a decisão 004 apoia-se em "trail de 4 cm + atrito no pivô
  reproduzem a traseira jogada pra fora", e o S de 27-07 foi atribuído ~20% à
  boba. Essa atribuição não se sustenta: a derrapada que o simulador mostra vem
  do `mu` baixo do contato, não da geometria de boba. Some junto a validação da
  ré (decisão 007), que no simulador não testa nada — lá ré e ida deram
  idênticas porque não há boba para virar.

  **Fecha quando:** (a) ~~ensaio 6 do banco rodado no robô real, ida × ré, com a
  boba filmada~~ **PARCIALMENTE FEITO 08-04** — ver abaixo; (b) a causa do garfo
  não alinhar identificada no modelo; (c) simulador reproduzindo o ângulo de
  boba medido no robô, ou a decisão 004 corrigida para dizer o que ele de fato
  reproduz.

  ✅ **08-04: o alvo de comparação existe, e é um número.** Ida × ré rodados no
  robô real com controle de piso (`n=2` cada): de frente ele arca com curvatura
  **−0,817 1/m** (raio 1,22 m), de ré **−0,098 1/m** (raio 10,2 m) — 8,3× de
  diferença, e **88% do arco de frente é o termo que só existe indo para a
  frente**. O arco está preso ao **corpo**, não ao mundo (as quatro corridas
  saíram com curvatura de mesmo sinal, com o corpo girado 180° entre os
  sentidos), o que exclui caimento de piso; e motor/placa fraca de um lado
  também está excluído, porque daria a mesma curvatura nos dois sentidos.

  **Isso vira o critério (c) em teste com número:** o simulador tem de
  reproduzir **8,3× de assimetria entre frente e ré**. ~~Hoje ele reproduz ~1×
  (ré e ida deram idênticas)~~ — essa frase era de 28-07 e já estava vencida.

  ✅ **CRITÉRIO (c) ATENDIDO em 08-04 (2ª leva)**: o simulador entrega **7,5×**
  contra **7,4×** do robô, com as curvaturas dos dois sentidos dentro da
  dispersão da máquina (`docs/dados/2026-08-04-aceitacao-simulador/`).

  ⚠️ **Mas o BO-4 NÃO fecha com isso**, e é importante não confundir: o arco
  entra no simulador **disfarçado de assimetria de roda**, porque a placa só
  tem rodas para escrever. Metade do arco de frente, no robô, **não está nas
  rodas** — a assimetria necessária é 24,8% e o encoder de 31-07 mediu 11–12%.
  O modelo reproduz o **sintoma**; o critério (b) (a causa do garfo não
  alinhar) segue intocado, e o preço é que **o encoder simulado mente** — o que
  quebra no dia em que ligarem `open_loop: false`.

  🔴 **06-08: o BO-4 ganhou um segundo sintoma, e ele é grande.** O S do robô
  não aparece no simulador **em ganho nenhum** — varrer `ki` de 1,0 a 0,0 não
  mudou uma única inversão, e a corrida longa (27 s, 8 m) deu 1 inversão e
  deriva final 0,0°. Duas razões estruturais, e a segunda é este BO: no
  simulador o ff cancela o arco por construção (mesmo −0,817 dos dois lados),
  **e a boba é um patim**. Oscilação de rumo puxada por roda boba arrastada não
  pode aparecer num modelo cujo contato de boba não participa da dinâmica.
  ➡️ Se a sintonia de 06-08 falhar no robô (amplitude continuar crescendo com
  os ganhos novos), o S deixa de ser assunto de laço e passa a ser **este BO**.

  ⚠️ **Falta ainda o vídeo da traseira** — foi filmado em 08-04 mas não trazido
  para o repo. É a única evidência direta do garfo, e sem ela o critério (b)
  não anda. **Virou item da próxima ida** (nº 6 da lista lá em cima): filmar a
  traseira DURANTE o S é o que liga os dois sintomas. E o dono confirmou, empurrando o robô **com a mão** e com ele
  desligado, que o desvio **se repete** — ou seja, o fenômeno não depende de
  acionamento, o que é um dado forte a favor de causa geométrica.
