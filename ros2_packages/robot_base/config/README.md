# Rede do Livox Mid-360

A configuração do driver tem **dois eixos independentes**:

1. a máquina que recebe os pacotes (`host_net_info.*_ip`);
2. a unidade física do sensor (`lidar_configs[].ip`).

Um sensor pode migrar entre computadores e um computador pode receber outra
unidade. Por isso não existe mais um `MID360_config.json` universal nem perfis
`robo2`/`robo3` que congelem os dois lados juntos.

## O que fica versionado

| arquivo | papel |
|---|---|
| `livox_host_profiles.json` | perfis **só das máquinas**: `nuc` = `192.168.1.2/24`; `notebook` = `192.168.1.5/24` |
| `MID360_config.template.json` | portas e estrutura do JSON, com tokens para host e sensor |

O arquivo ativo continua sendo
`ros2_packages/livox_ros_driver2/config/MID360_config.json`, dentro do clone
ignorado. O `setup_livox.sh` sempre o gera a partir dos dois eixos; editar essa
cópia à mão continua sendo provisório e se perde no próximo setup/reclone.

## Uso seguro

Com o Mid-360 ligado, deixe o setup descobrir a unidade pelo OUI da Livox:

```bash
./setup_livox.sh --perfil notebook
# ou
./setup_livox.sh --perfil nuc
```

Antes de instalar dependência, clonar, tocar `/usr/local` ou compilar, o script:

1. exige que o IP **e a máscara** do perfil existam em exatamente uma interface
   local;
2. varre a sub-rede e cruza quem respondeu com a tabela de vizinhos;
3. aceita automaticamente somente **um** MAC com OUI `e4:7a:2c`;
4. reprova se nenhum ou mais de um Livox responder.

Para preparar a máquina com o sensor desligado, ou para desambiguar duas
unidades presentes, o IP precisa aparecer explicitamente no comando:

```bash
./setup_livox.sh --perfil notebook --lidar-ip 192.168.1.169
```

Se o endereço responder, o MAC precisa ser Livox. Se não responder, o setup
prossegue porque a escolha foi deliberada, mas imprime **`NÃO confirmado`** em
destaque. Esse modo só prepara o arquivo; não é evidência de que o sensor está
presente nem de que publica nuvem.

O perfil nunca é inferido por hostname, nome de interface ou número do robô.
Interfaces já mudaram de `enp2s0` para `enp1s0`; o contrato é o IP local vivo:

```bash
ip -brief -4 addr show
```

## Conferência independente do IP do sensor

A varredura continua sendo a fonte da verdade:

```bash
for i in $(seq 1 254); do ping -c1 -W1 192.168.1.$i >/dev/null 2>&1 & done; wait
ip neigh | grep -i 'e4:7a:2c'
```

O último octeto do Mid-360 acompanha a identidade da unidade. Nunca transfira
um IP de sensor para o perfil de uma máquina.

## Inventário ainda aberto

Há duas observações incompatíveis com a afirmação antiga de que o projeto tem
uma única unidade:

| data/contexto | IP | MAC observado |
|---|---|---|
| 2026-09-15, NUC / bancada do robô 2 | `192.168.1.158` | `e4:7a:2c:95:df:da` |
| 2026-09-24, notebook / robô 3 | `192.168.1.169` | `e4:7a:2c:90:1d:f1` |

MACs distintos sugerem duas unidades, mas isso ainda precisa ser reconciliado
fisicamente com etiqueta/número de série e nova varredura. Até lá:

- não chamar `.158` de “lidar do robô 2” nem `.169` de “lidar do robô 3”;
- tratar as linhas acima como observações datadas, não inventário confirmado;
- não usar a frase “só há um Mid-360” como fato atual.

## Sintoma de combinação errada

Se `host_net_info` não for o IP desta máquina, ou o IP da unidade não for o que
está na rede, o driver pode subir sem entregar nuvem. O FAST-LIO fica sem dados
e `/Odometry` nunca aparece. Foi exatamente o caso de 2026-09-24: host `.2` no
JSON, interface `.5` no notebook; casar o host fez o fluxo chegar.
