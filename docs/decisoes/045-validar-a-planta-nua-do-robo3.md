# 045 — Validar a planta nua antes de integrar o robô 3

**Data**: 2026-09-09  
**Status**: protocolo e instrumentação prontos; medidas físicas pendentes  
**Toca**: launch mínimo do robô 3, controlador real separado e banco de
caracterização `perfil/mede/sessao_robo3.py`.

## Decisão

O robô 3 será validado primeiro apenas com duas rodas motrizes, duas bobas,
Livox, notebook, alimentação e placa. Nenhum parâmetro de navegação do robô 2
será levado para ele antes de medir a planta.

A compensação de zona morta do driver fica desligada durante a caracterização.
Com ela ligada, qualquer comando baixo é elevado ao patamar de 100 RPM e a
varredura mede o artifício do software, não o limiar nem a modulação da placa.

A ordem é irreversível por segurança: LIO parado → quatro rodas suspensas →
pulsos baixos → retas → pivôs → curvas → inversões. Cada etapa é executada
separadamente, grava comando, encoder, odometria diferencial, LIO, corrente,
tensão e temperatura, e para a sessão na primeira falha.

## Por quê

O robô 3 mudou simultaneamente bitola, comprimento, posição do eixo e número e
lado das bobas. Além disso, a placa ainda não foi identificada e o robô 2 já
mostrou que `cmd_vel` pode não representar velocidade física. Integrar Nav2 ou
sintonizar o Gazebo antes de observar essa cadeia confundiria dinâmica da
planta com erro de navegação.

## Alternativas descartadas

- Montar todos os acessórios e testar navegação completa: aumenta massa,
  variáveis e risco antes de saber se a base vale a pena.
- Copiar a calibração do robô 2: a geometria e o apoio são outros, e a placa
  pode ser outra.
- Confiar apenas na odometria do controlador: o ensaio exige `open_loop=false`
  e compara encoder com LIO, porque ecoar o comando fabricaria a medida.
- Fazer uma corrida longa única: esconderia sentido invertido, patamar,
  retenção e comportamento diferente das bobas na ré.

## Critério para avançar

Só se calibra o Gazebo depois de confirmar as medidas físicas, sentidos e
encoders e obter repetições válidas nos dois sentidos. Só se integra navegação
depois que o modelo reproduzir, dentro da dispersão medida, reta, pivô, arcos e
inversões da planta nua.
