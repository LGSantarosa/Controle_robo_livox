# Robô 2: atualizar o NUC com a trava do AMCL (decisão 050)

**O que muda:** a pose do robô no mapa para de pular enquanto ele está parado.
O AMCL só corrige a pose quando as rodas andam.

**Antes de começar**
- O PC de dev e o NUC precisam estar na rede **"Trafico de banana"**. O NUC fica
  em `10.127.116.205`, usuário `bara`.
- O robô pode estar ligado, mas **parado e sem ninguém dirigindo**.

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

## 3. Subir a pilha

```bash
bash bin/sobe-robo
```

Confira se a trava está ligada:

```bash
grep congela_parado ~/logs_sessao/base.log
```

Tem de aparecer `congela_parado: rodas paradas = odom->base_link parado.`. Se
não aparecer, o build não pegou: volte ao passo 2.

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
