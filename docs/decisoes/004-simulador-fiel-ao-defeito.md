# 004 — Simulador que reproduz o defeito, não o robô ideal

**Data:** 2026-07-24
**Estado:** aceita
**Depende de:** [003](003-base-ros2control-e-lio.md)

---

## Contexto

A meta imediata é fazer o robô andar direito: hoje ele **anda em S** e não fecha
curva. Ajustar isso no robô real é caro — depende de bateria, de espaço, de o
robô estar disponível — e é lento, porque cada tentativa custa uma montagem.

Um simulador resolveria isso, mas só se ele **errar do mesmo jeito que o robô**.
Simulador que anda perfeito não serve para ajustar controlador: a gente ajustaria
contra um robô que não existe, e o ajuste não sobreviveria ao contato com o real.

## Decisão

Construir um modelo em Gazebo Harmonic (`ros2_packages/robot_base/description/`)
com três compromissos inegociáveis, cada um endereçando uma forma específica de
o simulador mentir.

### 1. A roda boba é uma boba de verdade

Modelada como **duas juntas** — pivô vertical + roda — com a roda deslocada
`trail = 4 cm` **atrás** do eixo do pivô, e com **atrito e amortecimento no
pivô**.

O caminho fácil seria uma esfera lisa sob a traseira. Isso não reproduz nada: a
esfera não tem orientação, então nunca precisa "dar meia volta" para acompanhar,
e nunca empurra a traseira para o lado. O deslocamento entre o eixo do pivô e o
ponto de contato é **a causa física** de a traseira ser jogada para fora no
giro; sem ele o defeito desaparece do simulador e continua no robô.

O atrito do pivô é o segundo detalhe: pivô ideal se alinha instantaneamente.
Boba real emperra, e é durante esse atraso que a traseira sai.

### 2. A pose vem do chão, não das rodas

`/Odometry` publica a **pose verdadeira do Gazebo**, não odometria de roda.

Isso espelha o robô real, onde quem localiza é o LIO — que enxerga o movimento
que de fato aconteceu, incluindo a derrapada. Odometria de roda reportaria o
movimento *comandado*: ela integraria as voltas das rodas e concluiria que o
robô foi para onde mandaram. Ou seja, **esconderia exatamente o erro que
viemos estudar**.

A odometria de roda do `diff_drive_controller` continua publicada, mas para
comparação: a diferença entre as duas *é* a derrapada, e é uma medida útil.

### 3. A cadeia de controle é a mesma do robô

O simulador roda o **mesmo `controller_manager` e o mesmo
`diff_drive_controller`** do robô real, trocando apenas a interface de hardware
(`gz_ros2_control/GazeboSimSystem` no lugar da placa serial). A URDF é única,
com um argumento `sim` que decide qual camada de hardware entra.

Os **tetos de velocidade e aceleração são mantidos idênticos** aos do robô. Isso
é deliberado: foi um teto engolindo a modulação de velocidade que produziu o
defeito original (a regra "desacelere quando desalinhado" gerava um mínimo já
acima do teto, então o robô andava sempre a fundo). Um simulador sem esses
limites não teria como reproduzir a falha.

## Alternativas descartadas

| Alternativa | Por que não |
|---|---|
| **Esfera sem atrito no lugar da boba** | O jeito padrão de modelar roda boba em tutorial. Não tem orientação nem trail, então não reproduz o defeito — que é o único motivo de existir este simulador. |
| **Plugin `DiffDrive` embutido do Gazebo** | Dispensaria instalar o `gz_ros2_control`, mas pularia o `diff_drive_controller` — justamente onde moram os limites que causaram o problema. O ajuste feito contra ele não transferiria para o robô. |
| **Odometria de roda como fonte de pose** | Mais simples e não exige plugin de pose verdadeira, mas mascara a derrapada. Seria um simulador que sempre concorda consigo mesmo. |
| **Mundo com paredes/obstáculos** | O que estamos caçando é comportamento do próprio robô em trajetória livre. Parede encurta a trajetória antes de o erro se acumular e vira ruído. Obstáculo entra depois, quando o assunto for desvio. |

## Números estimados (e por que estão todos num lugar só)

O robô real **ainda não foi medido com trena**. Tudo abaixo é estimativa
informada pelo dono e vai ser calibrado comparando o comportamento do simulador
com o do robô. Por isso todos moram no bloco de propriedades no topo da URDF:

| Parâmetro | Valor | Origem |
|---|---|---|
| caixa | 0,50 × 0,50 × 0,30 m | informado |
| altura do fundo ao solo | 0,10 m | informado |
| separação das motrizes | 0,20 m | informado |
| raio da roda motriz | 0,0825 m | roda de hoverboard |
| massa total | 10 kg | informado ("ele é leve") |
| trail da boba | 0,04 m | **estimado** |
| raio da boba | 0,05 m | **estimado** (tem que alcançar o chão a partir de 0,10) |
| atrito/amortecimento do pivô | 0,05 / 0,02 | **estimado** — o parâmetro mais importante para casar simulador e robô |

## Consequências

- **Divergência aberta e conhecida**: separação das rodas é `0,20` no simulador
  e `0,32` no robô (valor herdado, nunca medido). Enquanto não forem
  reconciliadas, um controlador ajustado no simulador comanda giro diferente no
  robô. Há teste garantindo que URDF e YAML do simulador não divirjam **entre
  si**; a reconciliação com o real depende da trena.
- 11 testes travam a geometria (`test/test_urdf_robo2.py`), incluindo um que
  falha se o trail da boba for zerado — porque essa mudança transformaria o
  simulador em robô ideal silenciosamente.
- Passa a haver dependência de `ros-jazzy-gz-ros2-control`.
- A física foi verificada: o robô assenta sobre as três rodas em z≈0, estável,
  sem pular nem afundar.

## Em aberto

- **Calibrar o simulador contra o robô.** O critério não é "parece certo": é o
  simulador fazer um **S parecido** com o do robô real na mesma manobra. Até lá
  ele serve para desenvolver e descartar ideias ruins rápido, não para fechar
  números finais.
- Sem Livox no modelo por enquanto — o controle de movimentação não precisa
  dele, já que a pose vem pronta. Entra se formos simular percepção.
