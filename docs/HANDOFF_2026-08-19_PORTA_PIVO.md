# Handoff 2026-08-19 — estado final após a melhor volta

## ATUALIZAÇÃO FINAL — ESTA SEÇÃO SUBSTITUI O ESTADO ANTIGO ABAIXO

O texto histórico mais abaixo registra os experimentos e reversões do dia,
mas não representa sozinho o workspace no fim da sessão. O estado válido é:

- o movimento/planejamento foi restaurado ao `HEAD` `cd016e8`, a versão que
  voltou a passar pela porta na ida e na volta;
- sobre esse baseline ficaram somente o AMCL mais estável (`alpha1=0.01`,
  `alpha4=0.001`) e a recuperação inspirada no robô 1;
- a recuperação pode avançar até 0,20 m quando a traseira está bloqueada e a
  frente está medida livre, sempre rechecando o vão;
- falta de progresso não dispara ré: para recuar, é obrigatório confirmar
  bloqueio físico frontal (`<= 0.20 m`) e espaço traseiro seguro;
- `PolygonApproach` e `PolygonStop` estão ativos e não devem ser removidos;
- os testes específicos de recuperação passaram: 11 verdes.

Última corrida: `docs/dados/2026-08-19-sem-re-aleatoria/`. Os dois goals
terminaram. A volta foi a melhor observada pelo dono: o seguidor viu 2,45 m
livres à frente, não deu ré por mero sintoma, retomou o plano e terminou.
Preservar esse comportamento.

Na corrida inteira, o estado de recuperação teve 49 amostras positivas e
**zero negativas**. Portanto, as “mini-rés” vistas na ida não foram a ré do
`path_follower`; foram contra-torques `-0.50` do freio linear após cortes do
`PolygonStop`. Também ocorreram dois escapes frontais de cerca de 0,20 m. O
log ainda chama o final desses escapes de `fim da ré`; é só um rótulo errado.

### Próximo trabalho

Melhorar apenas o freio linear para eliminar o tranco/recuo percebido na ida,
sem desligá-lo: sem esse freio a placa já foi medida mantendo movimento por
0,52 s e acrescentando 0,10 m depois do corte. Usar a velocidade longitudinal
medida para modular/encerrar o contra-torque antes de cruzar zero e registrar
velocidade mínima, duração e deslocamento de cada frenagem. Não alterar junto
Smac, pivô, AMCL, collision monitor ou recuperação. Depois repetir pelo menos
três idas e voltas; a melhora só vale se a ida ficar lisa, a volta continuar
igual, ambos os goals terminarem e não houver contato nem recuo indevido.

### Processo ainda aberto

No momento deste registro há uma simulação aberta, com saída em
`docs/dados/2026-08-19-sem-re-aleatoria/`. Antes de subir outra, encerrar e
confirmar que não restou nenhum `pilha.launch.py`, `gz sim` ou `rviz2`.

---

## Histórico anterior do mesmo dia

Este é o estado para iniciar uma conversa nova. Não presuma que o último
experimento foi aprovado: ele causou colisão e foi revertido.

## Regra operacional do dono

- Dar pelo menos um `ok` em até 30 segundos; se ficar pensando sem responder,
  o dono interromperá o trabalho.
- Antes de subir qualquer Gazebo novo, matar **todos** os processos anteriores
  de `pilha.launch.py`, `gz sim` e `rviz2`, e confirmar que nenhum restou.
- Nunca deixar duas simulações interferirem uma na outra.
- O protocolo sempre começa no ponto `(0, 0)`.
- Percurso de aceitação: sair do zero, atravessar a porta, entrar cerca de 2 m
  no corredor (alvo usado: aproximadamente `(5.6, 1.2)`), inverter a missão e
  voltar ao `(0, 0)`.
- O objetivo prático é passar pela porta e voltar tranquilamente, sem tocar em
  parede/porta, sem encalhar, sem ré ao parar e sem perder a pose.
- Durante a investigação, não alterar várias causas ao mesmo tempo.

## Preferências e requisitos confirmados

- Manter o **Smac**. O dono gostou do resultado dele e explicitamente não quer
  voltar ao Theta.
- O robô é diferencial e precisa conservar o pivô no próprio eixo.
- Quando o ponto final estiver na reta de chegada, mas pedir orientação oposta,
  o comportamento desejado é: chegar à posição sem fazer balão e, parado no
  ponto, fazer o pivô para ajustar a orientação, como o robô 1.
- O freio linear não pode produzir um "tapinha" para trás quando o robô para.
- Ré automática não pode ser disparada apenas por falta de progresso; exige
  evidência física frontal de bloqueio e espaço seguro atrás.
- O robô real é leve e tem zona morta de giro quase inexistente. Não justificar
  comandos agressivos usando uma zona morta pessimista do simulador.
- Há dados reais de pivô: comando por 0,10 s produziu resultados de cerca de
  2,2°, 3,3° e 29,7° (pico até ~0,89 rad/s); 0,30 s produziu cerca de
  59–70° (pico ~1,44–1,52 rad/s). A resposta é bimodal; não assumir que pulsos
  repetidos de 0,10 s são automaticamente seguros.

## Diagnóstico que levou ao pivô de inversão

Na volta do corredor, o destino estava quase 180° atrás, mas o Smac/DUBIN
entregava ao seguidor uma primeira curva com erro de aproximadamente 30°. O
`heading_controller` nunca via os 180° e, portanto, seu gatilho de pivô de 80°
não disparava. A correção preservada detecta a inversão pelo rumo direto até o
destino, congela o avanço, pede o pivô e só retoma depois de assentamento e plano
novo.

## Último baseline comprovadamente seguro

O teste completo aprovado tecnicamente antes dos experimentos finais está em:

- dados: `docs/dados/2026-08-19-protocolo-completo-v4/`
- log ROS: `~/.ros/log/2026-08-19-15-04-16-793922-luiz-santarosa-750XGK-3321632/launch.log`
- percurso: `(0,0) -> (5.6,1.2) -> (0,0)`
- ida e volta concluídas, com inversão detectada (~169°), pivô, plano novo e
  sem ré;
- pose final medida: odometria aproximadamente `(0.1169, 0.0160)` e AMCL
  aproximadamente `(-0.0932, 0.0399)`.

Esse baseline ainda fazia o balão do Smac perto de orientação final oposta. O
balão continua sendo o próximo problema funcional, mas ele **não deve ser
eliminado cortando o plano cedo dentro da porta**.

## Alterações preservadas no workspace

- `path_follower.py` segue `/plan_smoothed`, não o plano cru.
- Detecção de inversão inicial pelo destino, pivô com avanço congelado,
  assentamento e espera de plano novo.
- Ré condicionada a obstáculo frontal fisicamente confirmado.
- `compensador_rumo.py`: freio linear desligado por padrão, evitando o toque
  para trás.
- `robo2.urdf.xacro`: atrito e damping do caster giratório zerados.
- `localizacao_amcl.yaml`: `alpha1 = 0.01` e `alpha4 = 0.001`, reduzindo o salto
  de pose durante pivô observado no RViz.
- `nav2.yaml`: Smac ativo com o perfil que produziu o baseline acima.

O worktree já continha outras modificações e dados do dono. Não usar
`git reset --hard`, `git checkout --` nem limpar arquivos não rastreados.

## Experimentos inseguros revertidos apó a colisão

Foram removidos e não devem ser reintroduzidos em bloco:

- atalho `sem_balao_dist = 2.0`, que abandonava a cauda segura do Smac cedo
  demais e levou o robô a bater;
- chegada em duas fases ativada nesse experimento (`aponta_no_fim = true`);
- tolerância final grosseira de 1,20 rad;
- pulsos forçados de pivô com 0,10 s;
- aumento de `pivo_max_pulsos` de 1 para 10;
- governador experimental por `pivo_wz_teto_real`;
- parâmetro experimental `giro_puro_proporcional` da placa simulada.

Estado restaurado:

- `pivo_max_pulsos: 1` nos perfis real e simulado;
- `aponta_no_fim: false`;
- `tolerancia_rumo_final: 0.15`;
- sem `sem_balao_*`, `pivo_pulso_s`, `pivo_wz_teto_real` ou
  `giro_puro_proporcional`.

Os arquivos Python modificados compilam com `py_compile`. A coleta da suíte
via Python do shell falhou porque esse interpretador não encontrou `rclpy`; não
foi uma falha de sintaxe. Resolver o ambiente ROS antes de usar esse resultado
como validação completa.

## Estado no encerramento

Todos os `pilha.launch.py`, `gz sim` e `rviz2` foram encerrados e foi confirmado
que nenhum permaneceu vivo. Não foi iniciada outra simulação apó a reversão.

## Como continuar com segurança

1. Ler este documento e inspecionar o diff atual antes de editar.
2. Confirmar novamente que não há Gazebo/launch/RViz antigo.
3. Subir uma única simulação e reproduzir primeiro o baseline v4 sem qualquer
   mudança, sempre saindo de `(0,0)`.
4. Só depois atacar o balão. A solução deve preservar o caminho do Smac pela
   porta e separar orientação final da aproximação apenas quando houver uma
   região comprovadamente livre; não usar distância fixa de 2 m como atalho.
5. Testar uma única mudança por vez, com parada imediata diante de contato ou
   perda de pose.
