# 010 — Ficar neste repositório e reaproveitar a camada de segurança do robô 1

**Data:** 2026-07-31
**Estado:** aceita (o *quê*); o *como* depende do levantamento pendente
**Depende de:** [000](000-heranca-do-robo1.md), [003](003-base-ros2control-e-lio.md)

---

## Contexto

Pergunta do dono, com o robô prestes a entregar os primeiros números de bancada:

> vale a pena continuar neste repositório ou pegar o `Controle_robo_web`, que já
> está pronto, e só adaptar a navegação? Aquele já foi todo o sofrimento que
> talvez a gente sofra tudo aqui de novo.

A preocupação é legítima e não é sobre código: é sobre **re-pagar custo de
integração já pago**. O robô 1 rodou em campo, com competição, e cada parâmetro
dele tem um incidente atrás.

## O que a investigação mostrou

**A premissa da pergunta se dissolve: este repositório já É o `Controle_robo_web`.**
Clone com histórico completo (decisão 000), 537 commits. E o que se chamaria de
"reaproveitar o resto" não é hipótese — está no working tree hoje:

```
robot_nav/config/nav2_params_legacy.yaml      368 linhas   (Nav2 do robô 1)
robot_nav/robot_nav/unstuck_supervisor.py    1433 linhas
robot_nav/robot_nav/motion_guard.py           683 linhas
robot_nav/behavior_trees/navigate_w_backup_first_recovery.xml
robot_nav/robot_nav/path_follower.py
```

A demolição de 14-07 removeu só o que era do robô 1 **como missão** (trekking,
cone, travessia de porta, teb, mapas dele, PS4, a cadeia do LD06). Nada da
camada de navegação e segurança foi apagado. Trocar de repositório significaria
refazer a demolição e reimportar o que não serve.

## A parte em que o dono está certo

O desperdício que ele teme **está acontecendo — dentro deste repo, por não
lermos o que já temos.** Evidência medida: em 29-07 brigamos com a inflação do
Nav2, medimos, e chegamos em `inflation_radius: 0.30`. O
`nav2_params_legacy.yaml`, dois diretórios ao lado, diz `0.25` — testado em
campo. Rederivamos por experimento o que estava escrito.

Trocar de repositório não conserta isso: o arquivo estaria igualmente por ler.

## O que não transfere, em qualquer dos caminhos

- **Tração** — o robô 1 tem Arduino MEGA; este fala serial direto (decisão 003).
- **Localização** — o robô 1 tem 3 sensores e 2D/AMCL; este tem só o Mid-360.
- **Os números do chassi** — zona morta, `a_dec`, derrapada, bitola. São desta
  máquina, e pertencem a quem dirige *este* chassi, seja qual for o repositório.
  É por isso que os dados da bancada **não pesam nesta decisão**.

E o custo que não é código: 10 decisões registradas, o diário, a trena, 407
testes. O PIBIT vira artigo em cima desse rastro; recomeçar do repo do robô 1
joga fora a espinha do artigo para reganhar arquivos que já estão aqui.

## Alternativas descartadas

**Migrar para o `Controle_robo_web` e adaptar a navegação lá.** Descartada: é
descrição do que já foi feito. O ganho seria zero e o custo é refazer a
demolição, reimportar MEGA/LD06/knobs de skid-steer e perder o rastro do PIBIT.

**Reescrever a camada de segurança do zero neste repo.** Descartada pelo dono,
com razão: o valor daqueles arquivos não é o código, é o **raciocínio datado**
dentro deles. Exemplo real, do `nav2_params_legacy.yaml`:

> `2026-06-15: lateral ±0.45 -> ±0.32. A ±0.45 pegava as paredes do corredor o
> tempo todo -> robô preso em 30% da velocidade em quase todo lugar (29
> slowdowns no log de campo).`

Isso é experimento de campo pago com tempo. Reescrever do zero é jogá-lo fora.

## Decisão

1. **Ficar neste repositório.**
2. **Reaproveitar a camada de segurança e recuperação do robô 1** —
   `collision_monitor`, `motion_guard`, `unstuck_supervisor` — adaptando-a ao
   Mid-360 em vez de reescrevê-la.
3. **Herdar estrutura e raciocínio; re-derivar os números.** Todo valor daqueles
   arquivos nasceu de um chassi de 4 rodas com atrito de skid-steer. O
   `collision_monitor` do robô 1, por exemplo, traz `angular_limit` justificado
   por uma zona morta de **1,7** — que o `CLAUDE.md` proíbe herdar
   explicitamente. Os polígonos são a geometria dele (±0,25 de meia-largura)
   contra os 0,433 × 0,455 medidos aqui.

## O que ainda NÃO está decidido — o levantamento

"Só adaptar para um sensor melhor" não é uma operação: são três casos
diferentes, e o levantamento arquivo por arquivo é que dá o número. A leitura
preliminar (por `grep` e tamanho, **não** por leitura a fundo) indica:

| componente | acoplamento ao sensor | expectativa |
|---|---|---|
| `collision_monitor` | nó do Nav2; aceita `pointcloud` nativamente | **config**, mais geometria deste chassi. O mais barato, e o que mais ganha com 3D |
| `motion_guard` | assina `scan_safe` **e** `map` — a parte do `OccupancyGrid` existe para caçar "fantasma de vidro" do LD06 | **encolhe**: parte dele o sensor melhor deleta, não adapta |
| `unstuck_supervisor` | vão traseiro por ângulo, corredor retangular, varredura girada por −θ | a **ideia** é agnóstica e já reaparece na decisão 009 (ré por sintoma); as 1433 linhas são moldadas em varredura planar |

### O risco que o levantamento tem de resolver

**Nuvem 3D não é drop-in de varredura 2D, e num aspecto é pior.** O Mid-360 tem
padrão de varredura não repetitivo: num quadro de 100 ms a cobertura é esparsa e
desigual, não um anel uniforme de bins angulares. Código que pergunta *"qual o
alcance mínimo neste setor angular"* recebe resposta instável. Melhor em
informação, mais difícil nesse padrão de acesso.

Saídas possíveis, a decidir com dado: acumular quadros; ou projetar um **anel
sintético** só para a camada de segurança (o que reabre, em escopo restrito, o
`/scan` derivado que a decisão 003 descartou para localização — e é preciso
dizer em voz alta que são coisas diferentes: derivar 2D para *não bater* não é
derivar 2D para *se localizar*).

### Ordem proposta (a confirmar no levantamento)

1. `collision_monitor` — config e geometria; ganho de segurança quase sem código.
2. `unstuck_supervisor` **partido em dois**: detecção por sintoma (pose/odom, sem
   sensor) primeiro e quase inteira; medida de espaço livre depois, contra a
   nuvem, com a escolha do anel sintético feita explicitamente.
3. `motion_guard` por último, e menor do que é hoje.

**Gatilho:** o levantamento roda quando os dados da bancada chegarem — não antes,
porque os polígonos e os limites de velocidade a re-derivar dependem da zona
morta e do `a_dec` medidos.
