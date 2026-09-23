# Roteiro — pré-condição da etapa 6: auditar o NOTEBOOK de bordo

> **Só a pré-condição.** O roteiro de bancada (rede, lidar, nuvem,
> `/Odometry`) **não existe ainda, de propósito**: ele nasce depois do
> relatório real da máquina, com fatos dela, não com suposição feita do PC de
> dev.
>
> ⚠️ **O computador de bordo da etapa 6 é o NOTEBOOK, não o NUC.** O NUC fica
> **adiado** até o chassi e o Nav2 serem aprovados. Nada aqui conclui coisa
> alguma sobre o NUC: ele não é auditado nesta etapa, e um relatório do
> notebook não fala por ele.

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

O notebook pode estar em qualquer um dos três — e provavelmente está em mais
de um, porque nunca foi preparado para esta camada. Do PC de dev **não dá
para saber**: `third_party/` e `install/` não vêm pelo git.

### O defeito mais provável no notebook, e por que o `cmp` sozinho não pega

O `host_net_info.cmd_data_ip` do `MID360_config.json` é **`192.168.1.2`**, e
está documentado como o IP **do NUC**. O notebook tem outro endereço. Os dois
`cmp` (fonte contra runtime) continuariam **verdes** — eles provam que os dois
arquivos são iguais, não que o endereço existe na máquina. Resultado: `bind
failed`, FAST-LIO sem nuvem, `/Odometry` nunca publicado, com tudo verde.

Por isso o auditor ganhou (23-09) a checagem **IP do host**, que compara o
JSON contra o `ip -brief addr` real e separa três casos:

| na máquina | veredito |
|---|---|
| o `192.168.1.2` está numa interface | **APROVADO** (diz qual, e se está UP) |
| há interface na `192.168.1.0/24` com **outro** endereço | **REPROVADO** — é a rede certa, com host errado: o caso do notebook |
| não há nada nessa sub-rede | **INCONCLUSIVO** — pode ser só cabo fora hoje |

## 1. Atualizar a branch, sem destruir evidência

⚠️ **Não use `git reset --hard` aqui.** A regra do `CLAUDE.md` (`git fetch &&
git reset --hard`) é de **deploy**; isto é **auditoria**. Sujeira local no notebook
pode ser justamente o que se quer medir — alguém pode ter editado o
`MID360_config.json` ou o setup lá. Apagar antes de olhar destrói a evidência.

No notebook, na raiz do repo:

```bash
git status --short --branch
git fetch origin
git switch etapa6-pilha-robo3 || git switch --track origin/etapa6-pilha-robo3
git merge --ff-only origin/etapa6-pilha-robo3
```

**Regra de parada:** se o `status` vier **sujo**, ou se o `--ff-only`
**falhar**, **pare aqui**. Não force, não resete, não faça `stash`. Preserve o
estado como está e me mande a saída desses quatro comandos — a divergência é
informação sobre a máquina, não obstáculo.

## 2. Rodar o auditor

```bash
bin/audita-livox --saida ~/audita-notebook.txt
echo "rc=$?"
```

O `rc=` importa tanto quanto o relatório.

## 3. Trazer o arquivo

Do PC de dev (ou por pendrive, se a rede do laboratório não ajudar):

```bash
scp NOTEBOOK:~/audita-notebook.txt .
```

E, no notebook, junto:

```bash
ip -brief addr
```

O auditor já lê isso, mas a saída crua ajuda a decidir **qual** endereço o
notebook deve assumir — e se é ele que muda, ou se é o JSON.

> Isto é **puxar log**, não deploy — a proibição de `scp` do `CLAUDE.md` é
> sobre **mandar código solto para o robô**, que continua valendo.

## 4. O que cada código de saída significa

| `rc` | veredito | o que fazer |
|---|---|---|
| **0** | **APROVADO** — tudo que dava para provar, provou | seguir para o roteiro de bancada, que eu escrevo com o relatório na mão |
| **1** | **REPROVADO** — alguma coisa está errada | **parar**. O relatório diz qual linha. **Num notebook virgem o esperado é reprovar em quase tudo** — é isso que diz o que preparar no passo 2 |
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

A sequência da etapa 6, decidida em 23-09:

1. **auditar o notebook** (este roteiro);
2. **preparar nele** SDK, drivers e overlay — `setup_livox.sh`, que já recusa
   revisão errada e dependência faltando antes de tocar em `/usr/local`;
3. **verificar a rede notebook↔Mid-360** — inclusive decidir o endereço do
   host, que hoje é o do NUC;
4. **só então** escrever e executar o roteiro de bancada com Nav2;
5. se o chassi superar o robô 2, **migrar e auditar o NUC** — aí sim.

Com o `audita-notebook.txt` na mão, eu escrevo o passo 2 com fatos da
máquina.

---

**Referências:** decisão `055`, `bin/audita-livox` (e seus 23 testes em
`tools/audita_livox/`), `ros2_packages/robot_base/config/README.md` (a
varredura que achou o `.158` e descartou o `.169`).
