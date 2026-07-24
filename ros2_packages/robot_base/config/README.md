# Configs da base do robô

## `MID360_config.json` — rede do Livox Mid-360

**Por que este arquivo mora aqui e não dentro do driver:** o `livox_ros_driver2`
é clonado do upstream pelo `setup_livox.sh` e fica fora do git (ver `.gitignore`).
Qualquer edição feita dentro dele se perde no próximo reclone. Então a config
vive aqui, versionada, e o script a copia para dentro do driver no setup.

**Por que ela é crítica:** se `host_net_info` ou `lidar_configs[].ip` não casarem
com a rede real, o driver loga `bind failed` / `Init lds lidar fail`, nenhuma
nuvem chega ao FAST-LIO e **`/Odometry` nunca é publicado** — o robô fica sem
localização e nada acima disso funciona. É uma falha silenciosa: os nós sobem
normalmente, só não sai dado.

### Os dois valores que precisam estar certos

| Campo | O que é | Como conferir |
|---|---|---|
| `host_net_info.*_ip` | IP da **NUC** na interface ethernet ligada ao lidar | `ip -brief addr` — na NUC a interface é `enp2s0`. Valor atual: `192.168.1.2` |
| `lidar_configs[].ip` | IP do **lidar** | varredura da sub-rede (abaixo). Valor atual: `192.168.1.169` |

Descobrir o IP do lidar:

```bash
for i in $(seq 1 254); do ping -c1 -W1 192.168.1.$i >/dev/null 2>&1 & done; wait
ip neigh | grep -i 'e4:7a:2c'   # e4:7a:2c = OUI da Livox
```

> ⚠️ O último octeto do Mid-360 acompanha o número de série da unidade — trocar
> de lidar muda esse valor. Já foram observados **dois** valores diferentes neste
> robô (`.169` e `.158`), então **a varredura é a fonte da verdade**; o número
> commitado aqui é só o último conhecido. Se mudar, editar este arquivo (não o
> do driver) e rodar `setup_livox.sh` de novo.
