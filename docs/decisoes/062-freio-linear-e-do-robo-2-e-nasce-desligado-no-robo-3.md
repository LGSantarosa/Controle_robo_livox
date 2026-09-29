# 062 — O freio linear é do robô 2 e nasce desligado no robô 3

**Data**: 2026-09-29 (PC de dev; robô e lidar desligados, Gazebo com RViz)
**Status**: aplicada na branch `etapa6-pilha-robo3`; **experimental**, o A/B que
a valida ainda NÃO foi rodado
**Toca**: `ros2_packages/robot_motion/launch/pilha.launch.py` (argumento novo
`freio_linear`), a mensagem de bringup do `compensador_rumo.py`,
`tools/linha_de_base/argumentos_launch.py` (trava) e
`ros2_packages/robot_motion/test/test_pilha_robo3.py` (cinco testes)
**Não toca**: a lei `lei_de_freio_linear.py` (intacta), o freio de GIRO (037),
o robô 2 em qualquer borda, Nav2, reflexo, mux, perfis

---

## 1. Contexto

O dono dirigiu o robô 3 pelo RViz numa ida e volta na pista
(`~/sessao-robo3/20260929_103954`, domínio 50, Gazebo). O percurso inteiro foi
bom — palavras dele: *"se ele se mover assim na vida real é o melhor q tivemos
até hoje"*. O defeito que ele apontou foi outro: **ao parar, o robô dá um
pulinho de rebote para trás**.

A observação veio com a causa já suspeitada, e correta: *"isso surgiu como
freio do robô 2 e ele pegou, mas esse n precisa de freio"*.

## 2. O que a corrida mediu

Do `freeze_capture.csv` da corrida, tomando como zero o instante em que o
`goal_active` cai:

| t (do fim do goal) | evento |
|---:|---|
| −0,105 s | Nav2 chega: `/auto_vel` e `/compensador_rumo/cmd_vel` vão a **0,000** |
| −0,103 s | `/cmd_vel_bruto` vai a **−0,500 m/s** — o contra-torque da 038 |
| +0,49 s | o freio solta (velocidade abaixo de `solta_em` = 0,25) |
| +0,64 s | o robô **cruza o zero** e passa a andar para trás |
| +0,96 s | pico de ré **−0,245 m/s** |
| +1,49 s | para, **9,6 cm atrás** do ponto onde deveria ter parado |

A entrada do compensador estava em zero durante toda essa janela: o −0,500 não
veio do Nav2 nem do seguidor, foi o próprio `FreioLinear` — é o comportamento
que `compensador_rumo.py` programa, e ele estava ligado
(`freio_linear=True` conferido no nó vivo, por `ros2 param get`).

Isto **não é descoberta nova**: já estava registrado que a retenção de 0,52 s
é maior que o transitório da frenagem e por isso o freio inverte a marcha.
O que a corrida de 29-09 traz é a medida no robô 3 e o número: 9,6 cm.

## 3. Por que o freio existe (e por que fica no robô 2)

A retenção da placa do robô 2 — `atraso_desliga` = **0,52 s** — foi medida **no
robô, em 04-08**. Em **13-08** veio a consequência: na corrida da porta o
reflexo cortou o comando a 0,30 m da parede e o robô andou mais 0,10 m com o
comando em zero, encostando nela. A 038 é a resposta a isso, e a bancada dela
(Gazebo, 13-08) mostrou que sem freio a sobra ficava em +0,107 e +0,127 m,
contra −0,084 a +0,018 m com o freio engatado.

No robô 3 o mesmo contra-torque não para — **inverte**.

⚠️ **Cuidado com a explicação fácil.** É tentador dizer "o robô 3 não tem
aquela inércia para cancelar", mas isso **não foi demonstrado**: a retenção da
planta do robô 3 não foi medida, nem no simulador (que roda com a placa do
robô 2) nem no robô físico. O que está provado é só isto, e é o que sustenta a
decisão: **nesta simulação, o contra-torque produziu inversão da marcha**.

## 4. Decisão

`freio_linear` passa a ser **argumento da pilha**, com default **por robô**:

- `robo:=2` → `true` (o estado de sempre, nas duas bordas: Gazebo e real);
- `robo:=3` → `false`.

A lei não muda, o nó não muda, o robô 2 não muda. Muda quem liga.

## 5. Alternativas descartadas

- **Apagar o `FreioLinear`.** Descartada: ele é a correção de uma batida real
  no robô 2, e o robô 2 continua neste repositório.
- **Escrever `False` direto no `compensador_rumo`.** Descartada: o A/B precisa
  das duas metades na mesma máquina, e default embutido no nó obrigaria a
  recompilar entre elas. Comparação que depende de rebuild não é comparação.
  Está travado em `test_o_freio_do_robo3_e_argumento_e_nao_hardcode`.
- **Baixar `solta_em` em vez de desligar.** Descartada por ora: a bancada de
  13-08 registrou que o portão `pico_min` domina o instante de soltar, então
  ela mede "engatou / não engatou", não "0,15 é melhor que 0,25". Calibrar um
  parâmetro cujo ensaio não separa os valores é ajustar no escuro. Se o A/B
  mostrar que desligar custa distância de parada, este é o próximo caminho.
- **`ros2 param set` em tempo de execução.** Impossível: `self.freio` é
  construído uma vez, na inicialização. Trocar exige reiniciar a pilha.

## 6. O que esta decisão NÃO prova

🔴 **Nada foi rodado depois da mudança.** Ela está escrita, testada em
unidade e com a suíte verde — não há corrida de Gazebo com o freio desligado.

🔴 **A placa do robô 3 não é a do robô 3.** O Gazebo sobe com `placa:=medido`,
que é o modelo de atuador **do robô 2** — está escrito na
`sim_robo3.launch.py`. Os 0,52 s de retenção que ampliam o rebote são
herdados. Sem o freio não haverá ré comandada, mas **ainda pode haver avanço
residual por 0,52 s**, e isso não foi medido.

🔴 **Quatro pontos de apoio não anulam torque retido pela eletrônica.** O
argumento "não precisa de freio porque tem as quatro rodas" não sustenta a
mudança: resistência mecânica é outra coisa. Isso se mede no robô físico.

🔴 **O freio não atuava só na chegada.** Ele atua em **todo** corte de comando,
inclusive no `STOP:PolygonStop` do reflexo — e em 28-09 atuou exatamente
assim, na parada de proteção da segunda porta (vão de 0,80 m).

## 7. Como esta mudança passa (o A/B)

Mesma pilha, mesma máquina, mesmo objetivo, alternando só
`freio_linear:=true|false`. Critérios, todos:

1. zero comando negativo em `/cmd_vel_bruto` depois da chegada;
2. nenhum recuo relevante (o de referência a bater é 9,6 cm);
3. chegada dentro da tolerância viva de 0,25 m;
4. **a passagem da segunda porta repetida**, com nenhum avanço perigoso depois
   de um `STOP:PolygonStop` — parada em área livre não cobre este critério.

## 8. A mensagem de bringup

Com `freio_linear=false` o nó registrava `ERROR` dizendo que o robô "desliza
+0,10 m depois de zerar". Esse número é do **robô 2**. Desligado passou a ser o
default correto de um robô, e `ERROR` em cima de default correto ensina a
ignorar o `rosout` — que é como eu leio a bancada, porque o dono só roda.

Virou `warn`, e não `info`, de propósito: o deslizamento que sobra é real. Sem
freio quem para é só a planta, e o quanto ela escorrega **não foi medido em
nenhum dos dois robôs de verdade**.

## 9. Testes

Em `test_pilha_robo3.py`, cinco, com as duas mutações conferidas:

- `test_o_robo2_continua_com_o_freio_ligado[gazebo|real]` — controle de
  regressão; cai se o default virar `false` para todos;
- `test_o_robo3_nasce_com_o_freio_desligado` — cai se o default virar `true`
  para todos;
- `test_o_freio_do_robo3_e_argumento_e_nao_hardcode`;
- `test_o_freio_chega_como_bool_e_nao_como_texto` — tipo cru derruba o
  compensador na subida com *parameter type mismatch*.

Na trava de argumentos (`argumentos_launch.py`), `freio_linear` entra como
novo permitido com default `'true'`: a extração roda sem `robo:=` e cai no
robô 2, então **aquela linha continua provando que o robô 2 não mudou**.

Suíte no estado final: **1768 passaram, 0 falharam**.

## 10. Referências

- decisão 038 (freio linear e ganho da cadeia de giro), 037 (freio de giro);
- `ros2_packages/robot_motion/robot_motion/lei_de_freio_linear.py` — o ensaio
  de 13-08 e a ressalva de que a varredura não resolve o limiar ótimo;
- dados: `~/sessao-robo3/20260929_103954/` (bag de 9,5 GB fora do Git);
- a sessão de 28-09 em `docs/dados/2026-09-28-robo3-gazebo-manual/`, onde o
  `STOP:PolygonStop` da segunda porta segurou 5,282 s.
