# Roteiro — pré-condição da etapa 6: auditar o NUC

> **Só a pré-condição.** O roteiro de bancada (rede, lidar, nuvem,
> `/Odometry`) **não existe ainda, de propósito**: ele nasce depois do
> relatório real do NUC, com fatos da máquina, não com suposição feita do PC
> de dev.

## 0. Antes de começar

- 🔌 **O robô pode estar DESLIGADO.** Nada aqui fala com hardware: o
  `bin/audita-livox` só lê arquivos e pergunta ao `ros2 pkg prefix`. Não sobe
  pilha, não pede sudo, não escreve nada fora do arquivo de saída.
- Quem roda é o dono; quem lê o relatório sou eu. Não precisa relatar console.

**Por que auditar antes de qualquer coisa:** em 23-09 o PC de dev tinha três
defeitos que nenhum teste pegava e que não davam sintoma até a hora errada
(decisão **055**): o SDK compilado desde julho e **nunca instalado**; um
`install/livox_ros_driver2` que era **casca vazia**; e o `MID360_config.json`
de runtime com o IP **`.169`**, que a varredura de 15-09 achou **mudo**, em
vez do `.158`. Esse último é o pior: IP errado = `bind failed` = FAST-LIO sem
nuvem = `/Odometry` nunca publicado — e quem investiga vai olhar o FAST-LIO.

O NUC pode estar em qualquer um dos três. Do PC de dev **não dá para saber**:
`third_party/` e `install/` não vêm pelo git.

## 1. Atualizar a branch, sem destruir evidência

⚠️ **Não use `git reset --hard` aqui.** A regra do `CLAUDE.md` (`git fetch &&
git reset --hard`) é de **deploy**; isto é **auditoria**. Sujeira local no NUC
pode ser justamente o que se quer medir — alguém pode ter editado o
`MID360_config.json` ou o setup lá. Apagar antes de olhar destrói a evidência.

No NUC, na raiz do repo:

```bash
git status --short --branch
git fetch origin
git switch etapa6-pilha-robo3 || git switch --track origin/etapa6-pilha-robo3
git merge --ff-only origin/etapa6-pilha-robo3
```

**Regra de parada:** se o `status` vier **sujo**, ou se o `--ff-only`
**falhar**, **pare aqui**. Não force, não resete, não faça `stash`. Preserve o
estado como está e me mande a saída desses quatro comandos — a divergência é
informação sobre o robô, não obstáculo.

## 2. Rodar o auditor

```bash
bin/audita-livox --saida ~/audita-nuc.txt
echo "rc=$?"
```

O `rc=` importa tanto quanto o relatório.

## 3. Trazer o arquivo

Do PC de dev (ou por pendrive, se a rede do laboratório não ajudar):

```bash
scp NUC:~/audita-nuc.txt .
```

> Isto é **puxar log**, não deploy — a proibição de `scp` do `CLAUDE.md` é
> sobre **mandar código solto para o robô**, que continua valendo.

## 4. O que cada código de saída significa

| `rc` | veredito | o que fazer |
|---|---|---|
| **0** | **APROVADO** — tudo que dava para provar, provou | seguir para o roteiro de bancada, que eu escrevo com o relatório na mão |
| **1** | **REPROVADO** — alguma coisa está errada | **parar**. O relatório diz qual linha; a correção costuma ser `sudo apt install ros-jazzy-pcl-ros` ou rodar o `setup_livox.sh` no NUC |
| **2** | **INCONCLUSIVO** — nada errado, mas algo não deu para provar | **não é reprovação.** Ex.: sem clone do SDK, a lib instalada pode estar perfeita — só não dá para provar a procedência dali. Eu leio e digo se dá para seguir ou se vale investigar |

Três vereditos porque **"não consegui provar" não é "está errado"**. Tratar
inconclusivo como reprovação faz perder tarde de laboratório atrás de defeito
que não existe.

## 5. Regra de parada, explícita

🛑 **Nenhuma ação de hardware antes de eu ler o relatório.** Não ligar o
lidar, não subir a pilha de localização, não mexer na rede do Mid-360, não
rodar `base.launch.py`. O auditor responde "a máquina está montada para
tentar" — **não** "a localização funciona". Essas duas coisas só se separam
medindo, e medir é a etapa seguinte.

## 6. O que vem depois

Com o `audita-nuc.txt` na mão, eu escrevo o roteiro de bancada: ordem dos
testes, o que precisa estar ligado em cada passo, o que gravar em CSV e qual
é o critério de parada de cada um. Com fatos do NUC.

---

**Referências:** decisão `055`, `bin/audita-livox` (e seus 19 testes em
`tools/audita_livox/`), `ros2_packages/robot_base/config/README.md` (a
varredura que achou o `.158` e descartou o `.169`).
