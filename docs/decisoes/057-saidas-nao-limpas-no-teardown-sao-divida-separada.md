# 057 — Saídas não limpas no teardown são dívida separada, não reprovação da etapa 6

**Data**: 2026-09-24 (PC de dev, robô DESLIGADO)
**Status**: 🟡 **ANOTADA** — a evidência registra o fato; o conserto fica para
depois do passo 8, e o critério de pronto está no §5.
**Toca**: `bin/valida-etapa6`, `tools/valida_etapa6/test_valida_etapa6.py`
(só o registro da evidência).
**Não toca**: `robot_base/`, `robot_nav/`, os nós de `robot_motion/` que saem
com código 1, nem a versão do Nav2 instalada — é exatamente o que esta decisão
recusa a mexer agora.
**Vem de**: a auditoria da quarta corrida da etapa 6 (`5cee221`,
`~/etapa6/20260924_114704`), ampliada pela quinta (`8515a25`,
`~/etapa6/20260924_132510`).

---

## 1. O que apareceu

O `resultado.csv:17` da quarta corrida declarava **0** linhas
`ERROR/FATAL/died`, enquanto o `launch.log:624` **assinado pelo mesmo
`SHA256SUMS`** trazia cinco processos encerrados com código 1. A contagem era
feita **antes** do `limpa`; essas linhas nascem **durante** o teardown. Nada
estava corrompido — mas a pasta afirmava algo falso sobre um arquivo que ela
própria assina, e num PIBIT a pasta é a prova.

A quinta corrida, já com dois exames e três números (`0 / 7 / 7`), mostrou que
o conjunto é maior do que cinco.

## 2. Os seis nós, e o que exatamente se viu

| nó | pacote | saída | constância |
|---|---|---|---|
| `heading_controller` | `robot_motion` | código 1 | 4ª e 5ª corridas |
| `compensador_rumo` | `robot_motion` | código 1 | 4ª e 5ª corridas |
| `path_follower` | `robot_motion` | código 1 | 4ª e 5ª corridas |
| `placa_simulada` | `robot_base` | código 1 | 4ª e 5ª corridas |
| `freeze_capture` | `robot_nav` | código 1 | 4ª e 5ª corridas |
| `collision_monitor` | Nav2 (binário de prateleira) | **código −11 (SIGSEGV)** | **intermitente** — 5ª corrida; na 4ª ele saiu `finished cleanly [pid 230655]` |

Mais **uma linha de erro de TF**, na quinta corrida, `launch.log:463`:

> `[collision_monitor-15] [ERROR] [getTransform]: Failed to get
> "livox_frame"->"base_link" frame transform: Lookup would require
> extrapolation into the future. Requested time 18.501000 but the latest data
> is at time 18.500000`

Um milissegundo. Ela **não existia** na quarta corrida (`grep -c extrapolation`
= 0 lá).

⚠️ **Não se está afirmando que esse erro de TF causou o segfault.** São dois
fatos observados na mesma janela, e é tudo o que se tem. Há discussão upstream
sobre corridas durante o shutdown capazes de derrubar nós do Nav2, mas nada que
prove ser este o defeito, e nenhum backtrace foi colhido (ver §6).

## 3. Por que isso é TEARDOWN, e não erro de execução

A sequência do `launch.log` da quinta corrida (linhas exatas):

| linha | evento |
|---|---|
| 382 | `[INFO] [ros2-20]: process has finished cleanly` — o **gravador** fecha limpo |
| 383–408 | `signal_handler(SIGINT/SIGTERM)` — o sinal geral chega aos nós |
| 459 | `[collision_monitor]: Cleaning up` |
| 463 | o erro de TF acima |
| 631 | `[collision_monitor-15]: process has died ... exit code −11` |

Os dois achados novos caem **depois** de o sinal geral ter sido recebido e
**dentro** do `Cleaning up`. Portanto `0 / 7 / 7` descreve a corrida
corretamente: **execução com a pilha de pé, integração e limpeza sem órfãos
seguem APROVADAS**, e o que está anotado é o encerramento.

A distinção é medida, não argumentada: o validador tira um snapshot do
`launch.log` antes de qualquer sinal e examina o log final depois do `limpa`,
com o trecho posterior recortado pela fronteira de posição (§7).

## 4. O que esta decisão NÃO declara

⚠️ **Não** se está declarando saída não limpa aceitável **para hardware real**.
O que se mediu foi Gazebo headless. **Isto é dívida real, potencialmente
relevante no robô físico** — não é acabamento nem detalhe de log. Um nó que
segfalta ao encerrar pode deixar de rodar o que ia rodar no caminho de saída, e
no robô o caminho de saída é o que zera atuador; a zona morta deste projeto já
ensinou que falha sem sintoma custa hora de competição. E sendo o SIGSEGV
**intermitente**, ele não dá para ser descartado por uma corrida que passou: a
quarta passou. A etapa que subir a pilha no robô ligado avalia isso do zero, e
com o robô fisicamente contido.

## 5. Critério futuro (o que fecha esta dívida)

1. os **seis** nós registram `finished cleanly [pid N]` no `launch.log`;
2. **nenhuma** linha `ERROR/FATAL/died` depois do início do encerramento —
   inclusive a de TF;
3. os três números do resultado em **`0 / 0 / 0`**.

Enquanto isso não acontece, o validador **anota** e não esconde: reprovar aqui
seria reprovar a integração por um defeito que não é dela.

## 6. O Nav2 não se atualiza no meio da etapa

| | versão |
|---|---|
| instalado | `ros-jazzy-nav2-collision-monitor` **1.3.12**-1noble.20260615.160103 |
| candidato | **1.3.13**-1noble.20260903.031125 |

A comparação 1.3.12 → 1.3.13 **não traz correção nominal** para este caso, e
trocar a versão do Nav2 no meio de uma etapa que se fecha por evidência
invalidaria a comparação entre as cinco corridas. Fica para depois do passo 8,
como experimento próprio.

**Não há backtrace nem coredump** deste segfault: `systemd-coredump` não está
instalado nesta máquina (`coredumpctl: command not found`), então o −11 é tudo o
que se sabe da morte. Colher backtrace é parte do conserto futuro, não desta
etapa.

## 7. Por que o conserto fica fora do gate desta etapa

Corrigir os cinco de código 1 alcança `robot_base/`, `robot_nav/` e nós já
existentes de `robot_motion/` — os mesmos pacotes que a decisão
[056](056-a-pilha-escolhe-o-robo-e-a-reescrita-vira-arquivo.md) declara
**intocados**. O sexto alcança versão de pacote do sistema (§6). O gate da etapa
6 é: a pilha sobe o robô 3 no Gazebo sem mexer no que já funciona. Consertar
aqui trocaria a garantia "nada do robô 2 foi tocado" por um conserto no meio da
prova.

**A decisão 056 continua 🟡 PROPOSTA até o passo 8.** Esta decisão não a
promove, não a antecipa e não depende dela.

## 8. Alternativas descartadas

- **Reprovar a corrida (RC 1).** Descartada: a integração em execução está
  provada (grafo, TF, mux, bag legível, limpeza sem órfão), e o encerramento é
  outro assunto. Reprovar aqui ensinaria a ignorar o RC — e é justamente por não
  reprovar que o registro precisa ser explícito.
- **Filtrar as linhas do relatório de erros.** Descartada — é o defeito original
  com outro nome: a pasta voltaria a contradizer o arquivo que assina.
- **Consertar os nós agora.** Descartada pelo §7.
- **Atualizar o Nav2 para 1.3.13.** Descartada pelo §6.
- **Contar só depois do `limpa`, um número só.** Descartada: perderia a
  afirmação que mais importa — "com a pilha de pé, zero erros".
- **Mover a fronteira do snapshot para depois do gravador fechar.** Considerada
  na quinta corrida e descartada: a sequência do §3 mostra que os dois achados
  novos vêm **depois** do sinal geral, então a fronteira atual já os classifica
  certo. Mover não mudaria nenhum dos três números.

## 9. Referências

- `~/etapa6/20260924_114704/` — a quarta corrida, que expôs a contradição.
- `~/etapa6/20260924_132510/` — a quinta, RC 0 e `0 / 7 / 7`, com a sequência
  do §3. Ambas preservadas.
- Decisão [056](056-a-pilha-escolhe-o-robo-e-a-reescrita-vira-arquivo.md) — a
  fronteira de pacotes que esta decisão respeita.
- `tools/valida_etapa6/test_valida_etapa6.py` — as travas de ordem
  (snapshot → `limpa` → exame final → manifesto) e dos três números.
