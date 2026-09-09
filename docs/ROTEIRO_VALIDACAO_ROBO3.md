# Roteiro de laboratório — planta nua do robô 3

Objetivo de hoje: voltar com as medidas finais e com dados suficientes para
fazer o Gazebo reproduzir **reta, ré, pivô, curva curta, curva longa e inversão
de marcha**. A montagem é deliberadamente mínima: duas motrizes, duas bobas,
Livox, notebook, alimentação e placa. Navegação e acessórios ficam fora.

> Não execute tudo de uma vez. Cada passo é um comando separado e só começa
> depois de olhar o anterior. O software publica zero em falha e no final, mas
> isso não substitui uma pessoa com a mão no corte físico da tração.

## 1. Levar e preparar

- trena/paquímetro, nível do celular e balança;
- dois apoios firmes para suspender **as quatro rodas** pelo chassi;
- fita para marcar posição e rumo no chão;
- câmera/celular apontado para as duas bobas;
- área plana de 5 × 5 m, 4 m de corredor e 2 m livres para trás;
- corte físico da energia das rodas alcançável por outra pessoa.

No notebook:

```bash
cd ~/Workspace/Controle_robo_livox
git switch robo3-bancada-validacao
colcon build --base-paths ros2_packages/hoverboard_driver \
  ros2_packages/robot_base --packages-select hoverboard_driver robot_base
source install/setup.bash
```

## 2. Medir com a tração desligada

Preencha [FICHA_MEDIDAS_ROBO3.md](FICHA_MEDIDAS_ROBO3.md). Tire uma foto de
cada trena ainda posicionada; isso elimina ambiguidade entre face interna,
face externa e centro. As medidas críticas são:

1. pneus: externo–externo e interno–interno;
2. eixo motor até frente e até traseira da caixa;
3. caixa: comprimento, largura, altura e fundo–chão na frente e atrás;
4. bobas: diâmetro, largura, *trail*, posição dos pivôs e tamanho da chapa;
5. Livox: centro óptico em `x/y/z` relativo ao meio do eixo motor e sua
   orientação;
6. massa da configuração exata ensaiada e posição das baterias/notebook;
7. tudo que ultrapassa a caixa, placa usada, tensão e identificação USB.

Envie a ficha e as fotos **antes dos ensaios no chão**. Os valores atuais do
modelo (bitola `0,3225 m`, raio `0,0825 m`) são conferências, não licença para
ignorar uma divergência.

## 3. Passo 0 — só Livox, nenhuma roda energizada

Terminal 1:

```bash
source install/setup.bash
ros2 launch robot_base base_robo3.launch.py
```

Terminal 2:

```bash
source install/setup.bash
python3 tools/banco/sessao_robo3.py --so 0
```

O robô fica parado 30 s. Só prossiga se houver `/Odometry` estável, sem saltos
ou deriva incompatível com o chão. Encerre o launch antes de mudar a montagem.

## 4. Passo 1 — rodas suspensas

Suspenda pelo chassi. Pneus e bobas não podem encostar em nada. Energize a
tração e mantenha o corte físico armado.

Terminal 1:

```bash
source install/setup.bash
ros2 launch robot_base base_robo3.launch.py tracao:=true
```

Se a placa não estiver em `/dev/ttyUSB0`, use o caminho visto em
`/dev/serial/by-id`, por exemplo `device:=/dev/serial/by-id/...` no mesmo
comando. Não escolha a porta por tentativa com as rodas no chão.

Terminal 2:

```bash
source install/setup.bash
python3 tools/banco/sessao_robo3.py --so 1
```

O programa aplica quatro pulsos de 0,35 s: frente, ré, giro esquerdo e giro
direito. Confirme visualmente:

- frente: a superfície de contato dos dois pneus caminharia para trás no chão;
- ré: o inverso;
- giro esquerdo (`wz > 0`): roda direita para frente e esquerda para trás;
- os dois encoders mudam, trocam de sinal na ré e não disparam sozinhos.

Qualquer troca de lado/sinal, ruído contínuo ou roda que não pare reprova o
passo. Corte a energia e envie `leituras.txt`, CSV e JSON antes de alterar fio
ou parâmetro.

## 5. Passo 2 — chão, pulsos de risco baixo

Depois de aprovar as medidas e o passo suspenso:

```bash
python3 tools/banco/sessao_robo3.py --so 2
```

São pulsos curtos de reta/ré em `0,10`, `0,25`, `0,40` e de giro nos dois
sentidos. A compensação de zona morta fica desligada nesta bancada: comando
baixo que não mover é um dado válido. Os pulsos dizem se `cmd_vel` regula
velocidade ou apenas escolhe um patamar, e medem latência e retenção. Se o robô
acelerar sem estabilizar, não seguir para os passos longos.

## 6. Ensaios de comportamento

Execute um passo, confira o resumo e reposicione. O programa pergunta antes de
cada corrida, para automaticamente ao perder LIO/encoder e impõe limites de
trajeto, velocidade e rotação.

```bash
python3 tools/banco/sessao_robo3.py --so 3   # reta curta: frente e ré, 3×
python3 tools/banco/sessao_robo3.py --so 4   # reta longa + robô girado 180°
python3 tools/banco/sessao_robo3.py --so 5   # pivôs 150/300/600 ms, 2 lados
python3 tools/banco/sessao_robo3.py --so 6   # curvas curtas, 4 combinações
python3 tools/banco/sessao_robo3.py --so 7   # curvas longas, inclusive ré
python3 tools/banco/sessao_robo3.py --so 8   # frente↔ré; filme as duas bobas
```

Os passos 4 e 7 precisam do espaço maior. No passo 4, preserve a linha marcada
quando o roteiro pedir o controle com o robô girado 180°: isso separa tendência
da máquina de caimento do piso. No passo 8, o vídeo deve começar um segundo
antes da inversão e mostrar as duas bobas completando o giro.

Para uma primeira leitura rápida, force uma repetição sem mudar os perfis:

```bash
python3 tools/banco/sessao_robo3.py --so 6 --repete 1
```

Não use `--sem-perguntas` no robô no chão.

## 7. Quando parar

Pare a etapa e corte fisicamente a tração se ocorrer qualquer um:

- sentido físico diferente do texto acima;
- uma roda continua após zero ou encoder fica sem dado;
- salto/perda do LIO, roda boba travada, cambaleio diagonal;
- aceleração contínua, trajetória inesperada ou aproximação do limite da área;
- cabo, notebook, bateria ou Livox se deslocando.

Não “compense no controle” durante a coleta. Filme/anote o evento e preserve a
corrida reprovada: ela vale mais para o modelo do que uma corrida retocada.

## 8. O que mandar ao voltar

Todos os dados ficam em:

```text
docs/dados/AAAA-MM-DD-bancada-robo3/
```

Envie a pasta inteira: `ambiente.txt`, `observacoes.txt`, `leituras.txt`, todos
os CSV/JSON, `resumo.csv`, `resumo.json`, ficha de medidas e vídeos. Desses
arquivos saem raio efetivo, bitola efetiva, mapa comando→velocidade, latência,
frenagem, atrito/derrapagem, assimetria, dinâmica das bobas e dispersão — os
parâmetros que faltam para casar Gazebo e planta real.
