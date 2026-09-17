# Robô 2: atualizar o NUC com a trava do AMCL (decisão 050)

**O que muda:** a pose do robô no mapa para de pular enquanto ele está parado.
O AMCL só corrige a pose quando as rodas andam.

**Antes de começar**
- O PC de dev e o NUC precisam estar na rede **"Trafico de banana"**. O NUC fica
  em `10.127.116.205`, usuário `bara`.
- 🔴 **A PLACA FICA DESLIGADA** durante os passos 1 a 3 (deploy, build e
  primeira subida da pilha) — ou o robô com as **rodas suspensas**, se for mais
  prático. Só energize no passo 4.
  **Por quê:** já houve registro da placa **girar as rodas sozinha** com a MEGA
  mandando zero, e a decisão 020 mediu **0,52 s de retenção** depois do comando
  zerar. Derrubar a pilha (`--mata`), compilar e subir tudo de novo com a placa
  energizada é exatamente o cenário em que isso morde — e desta vez com um nó
  novo, nunca rodado no robô, assumindo o `odom → base_link`.
- 🔴 **A trava não sobe mais sozinha** (mudança de 17-09): o
  `base.launch.py` a mantém desligada por padrão, de propósito, para que nenhum
  deploy normal a ative sem querer. Ligar é explícito, no passo 3.

## 1. Mandar o código para o NUC (rodar no PC de dev)

O NUC não consegue baixar do GitHub, então o código vai do PC de dev direto
para ele:

```bash
cd ~/Workspace/Controle_robo_livox
git pull
git push ssh://bara@10.127.116.205/home/bara/Controle_robo_livox main:refs/remotes/origin/main
```

## 2. Atualizar e compilar (rodar no NUC)

```bash
ssh bara@10.127.116.205
cd ~/Controle_robo_livox
bash bin/sobe-robo --mata            # derruba a pilha, se estiver de pé
git reset --hard origin/main
git log --oneline -1                 # tem de mostrar 1c3feb2 ou mais novo
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_base robot_motion
```

⚠️ Não pule o `colcon build`: sem ele o robô continua rodando a versão antiga,
sem dar erro nenhum.

## 3. Subir a pilha COM a trava ligada

🔴 **`bash bin/sobe-robo` sozinho NÃO liga a trava** — desde 17-09 ela é
opt-in, de propósito, para nenhum deploy normal a ativar sem querer. Para o
ensaio ela tem de ser pedida explicitamente.

**Só agora a placa pode ser energizada** (passos 1 e 2 são com ela desligada), e
com a mão no botão.

```bash
bash bin/sobe-robo congela=on
```

Confira se a trava está ligada:

```bash
grep congela_parado ~/logs_sessao/base.log
```

Tem de aparecer `congela_parado: rodas paradas = odom->base_link parado.`.
Se **não** aparecer, são duas causas possíveis, nesta ordem:
1. faltou o `congela=on` (o padrão é desligado — não é defeito);
2. o build não pegou: volte ao passo 2.

## 4. Teste rápido

1. Deixe o robô **parado por 1 minuto** e olhe a pose no mapa (web ou RViz).
   Ela **não pode se mexer**.
2. Ande uns 2 m com o controle e pare. A pose tem de acompanhar o robô **sem
   saltar** na saída.
3. Pivô no lugar: a pose tem de girar junto.

**Se der errado:** anote o que viu (parado pulando, salto ao sair ou pose que
não acompanha) e **não mexa no código**. Para voltar à versão anterior:

```bash
git reset --hard c8c7cf6 && colcon build --packages-select robot_base robot_motion
```

**Cuidado:** empurrado com a mão, o robô não mexe no mapa. A pose só acompanha
quando as rodas giram, e isso é esperado.
