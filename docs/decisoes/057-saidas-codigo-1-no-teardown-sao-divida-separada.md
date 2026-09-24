# 057 — Saídas com código 1 no teardown são dívida separada, não reprovação da etapa 6

**Data**: 2026-09-24 (PC de dev, robô DESLIGADO)
**Status**: 🟡 **ANOTADA** — a evidência passa a registrar o fato; o conserto
fica para depois do passo 8, e o critério de pronto está no §5.
**Toca**: `bin/valida-etapa6`, `tools/valida_etapa6/test_valida_etapa6.py`
(só o registro da evidência).
**Não toca**: `robot_base/`, `robot_nav/`, nem os nós de `robot_motion/` que
saem com código 1 — é exatamente o que esta decisão recusa a mexer agora.
**Vem de**: a auditoria da quarta corrida da etapa 6 (`5cee221`,
`~/etapa6/20260924_114704`).

---

## 1. O que apareceu

O `resultado.csv:17` daquela pasta declarava **0** linhas `ERROR/FATAL/died`,
enquanto o `launch.log:624` **assinado pelo mesmo `SHA256SUMS`** trazia cinco
processos encerrados com código 1:

| nó | pacote |
|---|---|
| `heading_controller` | `robot_motion` |
| `compensador_rumo` | `robot_motion` |
| `path_follower` | `robot_motion` |
| `placa_simulada` | `robot_base` |
| `freeze_capture` | `robot_nav` |

A contagem era feita **antes** do `limpa`; essas linhas nascem **durante** o
teardown. Nada estava corrompido — mas a pasta afirmava algo falso sobre um
arquivo que ela própria assina, e num PIBIT a pasta é a prova.

## 2. São falhas de TEARDOWN, não erros durante a execução

A distinção não é retórica, e agora ela é medida, não argumentada: o validador
tira um **snapshot do `launch.log` antes de qualquer sinal** e examina o
**`launch.log` final** depois do `limpa`, gravando **três números** no
resultado (execução: 0; final assinado: 5; após o snapshot de pré-limpeza: 5).
As cinco linhas estão todas **depois** da fronteira do snapshot, e antes dela
não há nenhuma. Com a pilha de pé e o bag gravando 89 300 mensagens em 26 s,
não houve erro.

O recorte do trecho posterior sai da **fronteira de posição** (quantidade de
linhas do snapshot), nunca de diferença textual: duas linhas de erro idênticas
se anulariam numa comparação de texto, e sumiria justamente a repetida. E o
rótulo é **"após o snapshot de pré-limpeza"**, não "causado pelo sinal": o que
se mediu foi posição no arquivo. Afirmar causa linha a linha seria afirmar mais
do que a fronteira prova.

## 3. O que esta decisão NÃO declara

⚠️ **Não** se está declarando código 1 no encerramento aceitável **para
hardware real**. O que se mediu foi Gazebo headless, com o `rclpy` saindo depois
de o contexto já ter sido desligado. No robô físico, encerramento que não fecha
limpo pode deixar atuador em estado indefinido — e a zona morta deste projeto já
ensinou que falha sem sintoma custa hora de competição. A etapa que subir a
pilha no robô ligado avalia isso do zero.

## 4. Por que o conserto fica fora do gate desta etapa

Corrigir esses encerramentos alcança `robot_base/`, `robot_nav/` e nós já
existentes de `robot_motion/` — os mesmos pacotes que a decisão
[056](056-a-pilha-escolhe-o-robo-e-a-reescrita-vira-arquivo.md) declara
**intocados**. O gate da etapa 6 é: a pilha sobe o robô 3 no Gazebo sem mexer no
que já funciona. Consertar aqui seria trocar a garantia "nada do robô 2 foi
tocado" por um conserto cosmético de log — mau negócio no meio de uma etapa que
se fecha por evidência.

**A decisão 056 continua 🟡 PROPOSTA até o passo 8.** Esta decisão não a
promove, não a antecipa e não depende dela.

## 5. Critério futuro (o que fecha esta dívida)

Os cinco nós saem com **código 0** ao receber o sinal de encerramento, e o
`launch.log` final registra `finished cleanly [pid N]` para cada um — os três
números do resultado passando a `0 / 0 / 0`. Enquanto isso não acontece, o
validador **anota** e não esconde: reprovar aqui seria reprovar a integração por
um defeito que não é dela.

## 6. Alternativas descartadas

- **Reprovar a corrida (RC 1).** Descartada: a integração em execução está
  provada (grafo, TF, mux, bag legível). Reprovar por ruído de teardown
  ensinaria a ignorar o RC.
- **Filtrar as linhas do `launch_erros`.** Descartada — é o defeito original com
  outro nome: a pasta voltaria a contradizer o arquivo que assina.
- **Consertar os nós agora.** Descartada pelo §4.
- **Contar só depois do `limpa`, um número só.** Descartada: perderia a
  afirmação que mais importa — "com a pilha de pé, zero erros".

## 7. Referências

- `~/etapa6/20260924_114704/` — a pasta que expôs a contradição (preservada).
- Decisão [056](056-a-pilha-escolhe-o-robo-e-a-reescrita-vira-arquivo.md) — a
  fronteira de pacotes que esta decisão respeita.
- `tools/valida_etapa6/test_valida_etapa6.py` — as travas de ordem
  (snapshot → `limpa` → exame final → manifesto) e dos três números.
