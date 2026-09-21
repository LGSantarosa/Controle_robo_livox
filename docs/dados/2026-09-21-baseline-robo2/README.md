# Linha de base do robô 2 no Gazebo — etapa 4, passo 0

Capturada em 2026-09-21 com o robô físico desligado e o comando antigo, sem
argumentos adicionais:

```bash
ros2 launch robot_motion pilha.launch.py sim:=true
```

O wrapper recompilou `robot_base`, `robot_motion` e `robot_nav`, conferiu todos
os `data_files` instalados e só então iniciou a pilha. A captura aprovada está
em `02-baseline-aprovada/` e identifica o commit `feb064a`.

## Resultado que fica como régua

- veredito: **APROVADO**;
- grafo: 30 nós visíveis, exatamente os 30 esperados;
- parâmetros: dumps bruto e normalizado dos 25 nós que oferecem os serviços;
- lifecycle: todos os servidores gerenciados em `active`;
- nós comuns: todos respondendo aos serviços de parâmetros;
- auxiliares internos sem esses serviços: um nó fixo do `BtActionServer` e
  quatro `TransformListener`, presentes na cardinalidade esperada;
- `footprint_padding` vivo nos dois costmaps:
  **`0.009999999776482582`**.

O valor longo é o valor efetivamente devolvido pelo Nav2, não um número
arredondado para documentação. O passo 1 deve usá-lo como referência para que
a comparação semântica não esconda alteração.

## Primeira captura reprovada — preservada como achado

`01-descoberta-grafo/` é a primeira captura, no commit `a6bd4cb`. Os 25 nós
previstos estavam presentes, mas o grafo mostrou cinco auxiliares internos:

- `/bt_navigator_navigate_to_pose_rclcpp_node`, criado pelo `BtActionServer`;
- quatro `/transform_listener_impl_<hex>`, vindos do BT Navigator, Collision
  Monitor e dos dois `Costmap2DROS`.

Eles não oferecem `list_parameters/get_parameters`. A captura reprovou em vez
de aceitá-los silenciosamente. O commit `feb064a` passou a tratá-los como nós
somente de grafo: nome fixo para o primeiro e regex ancorada, cardinalidade
exata 4 e alias estável para os listeners. Nomes parecidos ou quantidade
diferente continuam reprovando.

## Logs e limites

`logs/build.txt` e `logs/launch.txt` contêm o build e o launch da captura
aprovada. A única linha marcada como erro é do shader GLSL do RViz; ela não
derrubou o RViz nem qualquer nó e não interferiu na captura de parâmetros. Os
bags, por serem grandes e não fazerem parte desta prova, não foram copiados
para o repositório.

Esta linha de base prova apenas o conjunto de nós e parâmetros efetivos do
robô 2 antes dos perfis. Não prova desempenho, geometria nem hardware.
