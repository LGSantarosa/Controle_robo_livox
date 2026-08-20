# Constância nos dois gargalos — Gazebo — 2026-08-20

Pedido do dono: preservar **todos os dados** e só aceitar resultado repetível,
com o robô fazendo continuamente ida e volta pelos dois vãos.

## Condição congelada antes da primeira corrida

- commit de base: `b6b42a8c4ad3d9d6a03c3fe7376976a3ee07ac3e`
- mundo/mapa: `pista_obstaculos`, spawn `(2.0, 5.0, 0.0)`
- atuador: perfil simulado `medido`, planta `normal`
- ida: `(10.65, 6.60)`; volta: `(2.00, 5.00)`
- mudança sob teste: detecção automática de gargalo no mapa, eixo do plano
  congelado durante a travessia e teto local de `0.25 m/s`
- unitários antes de subir: `360 passed`

`snapshot/` contém o patch completo e cópias dos arquivos de controle e
configuração usados. Cada pasta de rodada recebe o bag de **todos os tópicos**,
CSV do seguidor, CSV por perna, resumo da ação e dumps dos parâmetros vivos.

Nenhuma corrida ruim deve ser apagada ou sobrescrita.
