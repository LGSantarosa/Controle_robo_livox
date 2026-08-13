# 031 — A ré só existe com objetivo vivo, e ela desencalha PARA IR a um ponto

**Data**: 2026-08-14 (dev, robô carregando)
**Status**: implementada, verificada no simulador de teste (mutação); **falta
ver no Gazebo e no robô**
**Toca**: `path_follower.py`, `pilha.launch.py`, `test_re_desligada.py`
**Vem de**: 13-08 no robô e da corrida de 14-08 no Gazebo — o mesmo defeito nos
dois

## O que o dono viu

> *"a ré deve ser exclusiva de quando um ponto estiver em ação, ontem ela
> ativava do nada sem nada estar acontecendo, e pior, aconteceu no gazebo
> também. A ré é para desencalhar, mas quando ele ENCALHA por conta de um erro,
> é pra desencalhar E IR ATÉ UM PONTO."*

## O mecanismo: o `/plan` fica retido

`self.plano` guarda o último `/plan` recebido e **nunca esvazia**. Terminado o
objetivo — cancelado, abortado, ou o dono simplesmente parando de clicar — o
seguidor continua com um plano na mão. Daí em diante:

```
robô parado                       ->  ProgressoDeAvanco não vê `re_avanco_min`
                                      (0,05 m) em `re_parado_s` (4,0 s)
plano venceu (timeout_plano 2 s)  ->  entra no ramo "plano velho"
res_sem_plano < re_max_sem_plano  ->  entra_na_re()
                                  ->  O ROBÔ RECUA, sem ninguém ter pedido nada
```

Quatro segundos depois de qualquer objetivo morrer, o robô parado dá uma ré.
**Não é sintonia, é alcançabilidade**: o gatilho da decisão 009 é *sintoma*
("não avancei"), e robô parado sem tarefa exibe o sintoma perfeitamente.

Foi o que apareceu na corrida do Gazebo de 14-08, com número:

```
t=3,0   seguidor pede  +0,50   ->  reflexo entrega 0,00
t=5,0   mux entrega    −0,20   <-  a ré, furando o bloqueio (025)
t=5→15  x vai de 0,00 a −0,554 e congela
```

## A decisão

**Guarda nova, a primeira de `entra_na_re`: sem objetivo de navegação vivo, não
se recua.**

```python
if self.par['re_exige_objetivo'] and not self.tem_objetivo():
    ...avisa, progresso.reinicia(), return
```

`tem_objetivo()` lê `navigate_to_pose/_action/status` e
`navigate_through_poses/_action/status` (`GoalStatusArray`) e chama de vivo o
status em `{1, 2, 3}` = ACCEPTED / EXECUTING / CANCELING — a **mesma** tripla
que o `unstuck_supervisor` e o `freeze_capture` já usam.

**Sem timeout, de propósito**: o `GoalStatusArray` é publicado a cada transição
e o último estado vale até a próxima; terminado o objetivo ele vira 4/5/6 e a
função responde `False` sozinha. Ninguém publicou nunca = **não há objetivo**.

**Por que a guarda vem ANTES das outras**: as demais decidem se *esta* ré cabe
(teto de seguidas, vão traseiro, orçamento); esta decide se recuar faz sentido
**algum**. E ela reinicia o contador de progresso, senão o gatilho fica
verdadeiro em todo ciclo e o log vira enxurrada a 20 Hz.

### A outra metade do requisito já estava certa — e agora tem teste

*"desencalhar E ir até um ponto"*: terminada a manobra, `passo_de_re` zera o
canal de desencalhe e devolve `estado = 'seguindo'`, e o ciclo seguinte volta a
perseguir o plano. Isso já funcionava; o que não havia era **teste**, e agora há
(`test_terminada_a_re_o_seguidor_VOLTA_A_SEGUIR`, verificado por mutação).
Com a guarda, essa retomada passa a ser garantida: se a ré só acontece com
objetivo vivo, sempre existe um ponto para onde voltar.

### O knob, e o default

`re_exige_objetivo`, default **`true`**, exposto na `pilha.launch.py`. Default
seguro pela regra da decisão 019: entre "recuar sem ninguém ter pedido" e "não
recuar", o perigoso é o primeiro — e foi ele que apareceu no robô e no Gazebo.

⚠️ **Consequência que é feature**: dirigindo o seguidor por `/plan` cru
(`tools/banco/plano.py`, sem ação do Nav2) a ré fica **inerte**. Foi exatamente
assim que ela fez o robô recuar do nada na bancada. Quem precisar do
comportamento antigo passa `re_exige_objetivo:=false`.

## Alternativas consideradas

**(a) Esvaziar `self.plano` quando o plano vence.** Descartada, e por um motivo
medido: a 4ª leva de 12-08 mostrou que o plano vence JUSTAMENTE enquanto o robô
recua (ninguém replaneja para robô emperrado). Esvaziar mataria a ré legítima —
a que existe para o caso `Start occupied`.

**(b) Exigir plano FRESCO em vez de objetivo vivo.** Mesma armadilha do (a),
pela mesma medida.

**(c) Deixar como está e confiar no teto de rés.** É o que estava no ar: o teto
limita o estrago (2 seguidas), não impede que ele comece. E o dono viu o robô
andando de costas sem tarefa — o teto não responde a "por que ele se mexeu".

## O que ainda NÃO está respondido

Esta decisão conserta **a ré que dispara sem tarefa**. Ela **não** explica por
que o reflexo bloqueou a frente na corrida do Gazebo de 14-08 com 3,59 m
livres — aquilo é o sensor simulado 314 s atrasado em relação ao `/clock`, e
tem investigação própria. Com a guarda, o robô naquela corrida teria ficado
**parado** em vez de andar de costas: melhor, e ainda errado.

## Referências

- decisão 009 — a ré por gatilho, não por plano (de onde vem o sintoma)
- decisão 019 — o default é o caso seguro
- decisão 025 — a ré que fura o bloqueio (o canal que ela usa)
