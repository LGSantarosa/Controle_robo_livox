# 005 — Lei de frenagem de rumo, piso de linear e detector de plantão

**Data:** 2026-07-27
**Estado:** aceita
**Depende de:** [003](003-base-ros2control-e-lio.md), [004](004-simulador-fiel-ao-defeito.md)

---

## Contexto

A decisão 003 descartou a movimentação herdada: ela andava sempre a fundo com o
giro saturado, **andava em S** e não fechava curva. Faltava escrever a nossa.

Antes de escrever, o simulador da decisão 004 foi usado para medir o defeito em
vez de descrevê-lo. Dois números saíram daí:

1. Com o controlador velho, o rumo entra em **ciclo-limite** — ±18°, período
   2,8 s, amplitude constante ao longo de todo o ensaio. Não é sobrepasso que
   decai: é oscilação permanente.
2. Cortando o comando de giro em malha aberta com `wz = 1,0 rad/s`, o robô
   ainda gira **25°**. Existe uma *distância de frenagem de rumo*.

O segundo explica o primeiro. Entre "o erro de rumo cruzou zero" e "o giro
parou de fato", o robô percorre

```
Δrumo = wz² / (2 · a_dec)
```

Com os tetos do `diff_drive_controller` (`a_dec = 1,5 rad/s²`) e giro saturado,
isso dá 0,33 rad — e o ensaio mediu 0,34. **Um controlador que manda giro até o
erro trocar de sinal atravessa o alvo por essa margem em todo ciclo, para
sempre.** O S não é sintoma de ganho mal ajustado; é a consequência aritmética
de ignorar a própria frenagem.

A derrapada da roda boba existe e foi medida (a odometria de roda acusa 4,7° a
mais que a pose verdadeira por curva de 90°), mas responde por ~20% do defeito.
A rampa responde por ~80%.

## Decisão

### 1. A lei de giro

```
wz = sinal(e) · min( wz_max , √(2 · a_dec · |e|) )
```

Em uma frase: **nunca pedir mais giro do que se consegue frear dentro do erro
que ainda falta.** É a mesma conta do sobrepasso, usada como limite em vez de
sofrida como defeito.

Consequências que a fazem preferível a um PID:

- **Um parâmetro, e ele é físico.** `a_dec` é a desaceleração angular que a
  máquina entrega, medível em um ensaio de 5 segundos — não um ganho a ajustar
  por tentativa. Isso atende a regra de "SI real, sem escada de ganhos".
- **O sobrepasso não depende do tamanho do erro.** Medido em erros de 45°, 90°
  e 180°: sempre entre 0,1° e 0,8°. Um PID afinado para 45° não teria essa
  propriedade.
- **Errar o número degrada de forma previsível.** Com `a_dec` 5x otimista, a lei
  oscila mas **decai** (8,2° de erro médio e caindo, contra 24,6° eternos do
  controlador velho); com `a_dec` 3x conservador, sobrepasso **zero** e apenas
  0,3 s a mais de assentamento. É isso que autoriza escrever a movimentação
  antes de o robô ser medido — e que define a regra de ouro: **na dúvida,
  chutar `a_dec` para baixo.**

A linear cede com `cos(e)`: quanto mais torto o rumo, menos avanço.

### 2. O piso de linear

```
v_piso = zona_morta + wz_max · bitola/2 + margem
```

`cos(e)` zera a linear sempre que o erro passa de 90°, e linear zerada põe as
duas rodas simultaneamente perto de zero — dentro da zona morta da placa, que
então engole o giro inteiro e **o robô fica parado encarando o erro**.

Medido: com zona morta de roda em 0,10 m/s a meia-volta trava 0,1 s e completa;
com **0,15 m/s o robô fica 22 s imóvel**, 100% das amostras, com o controlador
pedindo 1,0 rad/s. Não é degradação, é precipício — e o valor real da placa é
desconhecido e cai bem nessa faixa (BO-3).

O piso mantém as duas rodas acima do limiar, tornando qualquer `wz` alcançável.
Confirmado nos dois valores de zona morta (0,10 com piso 0,25; 0,15 com piso
0,30): sobrepasso 0,8°, meia-volta completa.

**Custo aceito:** o robô nunca pivota no lugar — sempre descreve um arco. Numa
meia-volta isso o deixa 66 cm paralelo ao caminho de ida. É custo consciente:
fechar essa distância lateral é trabalho da navegação, que conhece a rota; a
movimentação responde por rumo.

### 3. O detector de plantão

Se há comando de movimento e a pose do LIO não muda por ~0,5 s, o nó **grita no
log** com o valor pedido e o efetivo.

O prejuízo da zona morta não é o robô parar — é ninguém saber por quê. É falha
silenciosa: nó vivo, tópico publicando, log limpo, máquina imóvel. Já custou
horas de depuração numa competição anterior do dono. A prevenção (item 2) pode
falhar quando a bateria cai, a carga muda ou o piso é outro; a delação
transforma uma tarde perdida em uma linha de log.

## Alternativas descartadas

- **PID de rumo.** Precisa de três ganhos sem significado físico, ajustados por
  tentativa contra um simulador que sabidamente mente (decisão 004). E o
  sobrepasso passaria a depender do tamanho do erro, o que a lei de frenagem
  não faz.
- **Herdar a movimentação do robô 1.** Descartado na decisão 003. Os knobs de lá
  (zona-morta 1.7, autoridade de giro 6.0, `slow_wz_cap`, proibição de arco)
  descrevem o atrito de um skid-steer de 4 rodas. Este robô é diferencial e gira
  fácil.
- **Zerar a linear até o rumo fechar** (pivotar e só então andar). Traçaria o
  caminho de volta exato, mas depende justamente da manobra que o atuador não
  entrega: girar parado devagar é impossível abaixo de
  `2·zona_morta/bitola`. A versão de caminho bonito é a que trava o robô.
- **Bater o comando na zona morta por pulsos (dithering).** Resolveria o giro
  parado lento, mas castiga o motor e esconde o limite físico atrás de um
  truque. Se a navegação precisar disso, que seja decisão dela, medida.

## Verificação

Dez corridas no simulador com a planta **degradada de propósito**
(`a_dec = 0,3`, mais pessimista que a estimativa de ~0,5 do robô real).
Sobrepasso entre 0,1° e 0,8° em erro de 45°, 90° e 180°, a 0,2 e 0,7 m/s, em
malha de 50 Hz e de 10 Hz (a taxa do robô real). O S não reapareceu em nenhuma.

Protocolo e leitura em `tools/banco/`.

## O que esta decisão NÃO resolve

- **`a_dec` e zona morta reais são desconhecidos.** Os valores usados aqui são
  do simulador; os do robô saem do banco de ensaios. Até lá os parâmetros do nó
  ficam conservadores de propósito.
- **Girar parado devagar continua impossível.** Limite físico, não ajuste de
  ganho. Cai no colo da navegação.
- **Erro lateral não é tratado.** O controlador trava o rumo e segue paralelo à
  rota. Também é da navegação.
