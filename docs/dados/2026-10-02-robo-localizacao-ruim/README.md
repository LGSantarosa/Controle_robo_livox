# Robô 2 — localização instável no corredor (2026-10-02)

Coleta bruta das duas tentativas feitas no fim da sessão real. O código no NUC
era `04adb94` (decisão 064 implantada). O robô foi desligado depois da coleta.

## Tentativa final — arquivos na raiz

Pilha iniciada por volta de 20:17. Na repetição a partir da origem, o dono
observou oscilação e pulos do robô no mapa; na volta ele perdeu a localização
no corredor e entrou contra a parede. Este conjunto ainda não foi analisado.

- `pilha.log`, `base.log`, `web.log`, `joystick.log`;
- `seguidor_2026-10-02_201720.csv`;
- `freeze_capture.csv` e `freeze_diag.csv`;
- `nav_metrics_20261002.csv`.

## Primeira tentativa — `primeira_tentativa-heartbeat/`

Pilha iniciada por volta de 20:02. O `collision_monitor` perdeu quatro
heartbeats; o `lifecycle_manager` reiniciou toda a pilha e cancelou goals. A
decisão 064 não disparou. O plano aceito depois dos reinícios estava muito
serrilhado. Esta tentativa fica separada para não atribuir seus sintomas ao
episódio de localização da tentativa final.

## Pergunta da próxima análise

Separar a contribuição de `/Odometry` (FAST-LIO) e de `map → odom` (AMCL) nos
pulos observados. O alvo reafirmado pelo dono é a arquitetura das decisões 001
e 003: localização global usando a geometria 3D do Mid-360, com projeção 2D
somente para Nav2 e para a interface web. O AMCL funciona bem na maior parte
das rotas e não será apagado: o estado atual fica preservado na branch
`baseline-amcl-2d-2026-10-02` como baseline. O localizador 3D o substituirá no
caminho ativo porque os poucos saltos e perdas restantes são inaceitáveis para
o sensor disponível.
