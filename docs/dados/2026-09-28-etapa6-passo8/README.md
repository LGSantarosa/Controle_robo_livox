# Etapa 6, passo 8 — gate de não regressão do robô 2 (2026-09-28)

Contrato: `docs/PLANO_ETAPA6_ROBO3.md` §6, itens (a)–(d). PC de dev; robô e
lidar **desligados** em tudo.

## O que está nesta pasta

| arquivo | o que é | quem produziu |
|---|---|---|
| `item_c/` | as quatro listas de nós do item (c), a cópia da ferramenta, `LEIA.txt` e `SHA256SUMS` | cópia de `~/etapa6-gate/c_20260928_130231/` (PC de dev); o `SHA256SUMS` foi gerado junto com a medição |
| `item_d_20260928_130910_robo2-gazebo.SHA256SUMS` | sha256 dos **31** arquivos de `~/validacao_etapa4/20260928_130910_robo2-gazebo/` (PC de dev) | gerado **depois** da corrida, pelo assistente, com `find . -type f \| sort \| xargs sha256sum` na pasta. **Não** é do wrapper: o `bin/valida-etapa4` é anterior ao padrão de manifesto e não assina a pasta. Fica aqui, fora dela, para a pasta não ganhar arquivo |

Conferir o (d) na máquina que tem a pasta:

    cd ~/validacao_etapa4/20260928_130910_robo2-gazebo
    sha256sum -c <repo>/docs/dados/2026-09-28-etapa6-passo8/item_d_20260928_130910_robo2-gazebo.SHA256SUMS

O bag daquela corrida foi para `~/logs_robo2/` (default da pilha do robô 2) e
não está no manifesto nem é versionado.

## Os quatro itens

- **(a) diff contra `1f49981`: passou com exceções aprovadas pelo dono.** Fora
  da lista permitida, só estes oito, cada um nominalmente autorizado:
  1. `README.md` (raiz) — decisão 058 (`6dc13f1`);
  2. `ros2_packages/robot_base/config/MID360_config.template.json` — 058;
  3. `ros2_packages/robot_base/config/README.md` — 058;
  4. `ros2_packages/robot_base/config/livox_host_profiles.json` — 058;
  5. `ros2_packages/robot_base/test/test_config_livox.py` — 058;
  6. `setup_livox.sh` — 058;
  7. `ros2_packages/robot_base/launch/base.launch.py` — **só o docstring**
     (`be8347e`, 2 linhas);
  8. `pytest.ini` — **só a coleta de testes** (`93671b3`, `ea636f7`: +
     `tools/valida_etapa6`, `tools/valida_etapa7` no `testpaths`).

  Qualquer outro caminho fora da lista continua reprovando. ⚠️ Os seis da 058
  mudam a **preparação do Livox real**; o (c) e o (d) cobrem a pilha
  **simulada** do robô 2 e **não** os provam.
- **(b) suíte inteira verde**, piso 1262: **1670/0** no estado final (com
  `718ac0f` e esta documentação).
- **(c) árvore de nós do robô 2 igual byte a byte** entre `1f49981` e
  `f2bb02c`, nas duas combinações: `sim:=true` 23 itens, `sim:=false` 13 itens.
  Método, normalização e as três mutações que o instrumento pega: `item_c/LEIA.txt`.
- **(d) corrida do robô 2 no Gazebo**, `bash bin/valida-etapa4 robo2-gazebo
  6.24 3.51`, domínio 41, código `718ac0f`: RC 0, os seis itens do wrapper
  APROVADO — inclusive "parâmetros: v2 sem permissão, original com passo 2,
  padding". Linhas do `corrida.txt`, copiadas:

      🟢 chegada        0.141 m do alvo (raio 0.25 m) em 120.0 s
         caminho        6.61 m andados para 4.49 m de reta  (tortuosidade 1.47)
         replanejou     6x
      🛡️ reflexo        agiu 2x, 0.0 s no total (1ª vez em t=0.1 s)
         pose           46.9 Hz mediana, pior intervalo 0.042 s

  Referência (etapa 4, `docs/dados/2026-09-22-etapa4-passo7/20260922_125907_robo2-gazebo/corrida.txt`):

      🟢 chegada        0.133 m do alvo (raio 0.25 m) em 120.0 s
         caminho        6.76 m andados para 4.49 m de reta  (tortuosidade 1.51)
         replanejou     7x
      🛡️ reflexo        agiu 2x, 0.2 s no total (1ª vez em t=0.1 s)
         pose           46.4 Hz mediana, pior intervalo 0.051 s

  O "em 120.0 s" é o teto do `corrida_nav.py`, que cancela o objetivo nas duas.
  Primeira entrada no raio de 0,25 m, lida do `corrida.csv` pelo mesmo método
  nas duas: **28,4 s** hoje, **29,55 s** na referência.

  `launch.log`, separado pelo primeiro `signal_handler(SIGINT`: antes dele, 0
  `process has died` e 1 `ERROR` (hoje: o shader do RViz; referência: 3); depois
  dele, os 5 `exit code 1` do teardown (dívida da decisão 057), iguais na
  referência. Os dois controladores ativaram; nenhum timeout de ativação.

🔴 **O que isto NÃO prova:** hardware, desempenho medido ou a zona morta de
nenhum robô; e nada da preparação do Livox real (decisão 058).

## Adendo (2026-09-28, 14h) — a limpeza do item (d) NÃO foi total

O `limpeza.txt` do (d) registra "processos marcados depois: 0" e "domínio 41:
nenhum nó", e isso continua verdade. Mas **sobraram oito segmentos Fast DDS em
`/dev/shm`**, criados às 13:09:19 durante a corrida `20260928_130910` e sem
nenhum processo os segurando. Achados só às 13:46, pela varredura de resíduo
do instrumento da decisão 061 — eu não tinha conferido o `/dev/shm` depois do
(d), e a limpeza do `bin/valida-etapa4` não olha para ele.

Causa, no próprio `limpeza.txt`: "KILL: 2 membro(s) não saíram com INT em 20
s" — o líder da launch e o `ros2-21`, que é o `ros2 bag record` do robô 2 (bag
ligado por padrão na pilha dele). Este Jazzy não responde SIGINT no gravador
(medido na etapa 6), o wrapper congelado só manda INT e depois KILL, e processo
Fast DDS morto por KILL não apaga os próprios segmentos. A referência de 22-09
tem o mesmo KILL no `ros2-21`.

**Não invalida as medições funcionais do (d)** — parâmetros contra a v2,
chegada, subida —, mas corrige a afirmação: a limpeza daquele wrapper deixa
resíduo SHM do gravador, e isso é limitação dele. Registro dos oito, com
horário e `fuser` vazio, e da remoção por `fastdds shm clean` (só ela, que
removeu exatamente os oito): `shm_orfaos_antes_da_limpeza.txt`.
