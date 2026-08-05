#!/usr/bin/env bash
# Traz o `twist_mux` como FONTE dentro de ros2_packages/, em commit fixado.
#
# POR QUE NÃO O PACOTE DO APT — e este é o ponto todo deste script.
#
# O `ros-jazzy-twist-mux` do apt INSTALA e NÃO SOBE no NUC:
#
#     undefined symbol: diagnostic_updater::Updater::Updater(
#         NodeBase, NodeClock, NodeLogging, NodeParameters,
#         NodeTimers, NodeTopics, double, unsigned char)
#     [ros2run]: Process exited with failure 127
#
# Não é versão errada de `twist_mux` — é a `diagnostic_updater` da máquina
# estar atrás. Medido nos dois lados (05-08), lendo os símbolos das libs:
#
#     diagnostic_updater 4.2.6   exporta  ...NodeTopicsInterfaceEEd     (só)
#     diagnostic_updater 4.2.7   exporta  ...NodeTopicsInterfaceEEdh    (só)
#
# O construtor ganhou um parâmetro, o nome mangled mudou, e **o símbolo antigo
# DESAPARECE na 4.2.7**. Isso é substituição, não adição.
#
# 🛑 CONSEQUÊNCIA, e é por isso que NÃO se resolve com `apt upgrade` de um
# pacote só: subir a `diagnostic_updater` no NUC quebraria **todo consumidor
# compilado contra a 4.2.6** — inclusive o `controller_manager`, que a base
# usa e que estava com a lib mapeada no processo vivo. O robô poderia não
# subir na sessão seguinte. Só um upgrade COERENTE da pilha inteira (com o
# `controller_manager` reconstruído junto, e a base conferida depois) resolve
# por aquele caminho, e isso é sessão própria.
#
# Compilando aqui, o `twist_mux` liga contra os headers que a máquina TEM:
# ele passa a pedir `...d`, que existe. Nenhuma lib do sistema é tocada.
# Verificado por inspeção do fonte: `twist_mux_diagnostics.cpp` constrói o
# Updater com `make_shared<Updater>(mux)` — um nó só, os demais parâmetros vêm
# por default do header, então ele compila contra as duas versões.
#
# Mesmo padrão e mesma razão do `setup_livox.sh`: upstream de terceiro entra
# por clone em commit fixado, não vendorizado no repo. Subir versão é mudança
# deliberada, não efeito colateral de rodar o setup.
#
# Rodar de dentro da raiz do repo. Idempotente.
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKGS="$WS/ros2_packages"

TWISTMUX_URL="https://github.com/ros-teleop/twist_mux.git"
TWISTMUX_TAG="4.5.0"       # mesma versão do apt; o problema nunca foi ela
TWISTMUX_COMMIT="136394c"  # o que a tag 4.5.0 aponta, fixado explicitamente

echo "==> 1/3  twist_mux (fonte, @ $TWISTMUX_TAG / $TWISTMUX_COMMIT)"
if [ -d "$PKGS/twist_mux/.git" ]; then
    echo "    já presente — mantendo (apague o diretório para refazer)."
else
    git clone --quiet "$TWISTMUX_URL" "$PKGS/twist_mux"
    git -C "$PKGS/twist_mux" checkout --quiet "$TWISTMUX_COMMIT"
    echo "    clonado."
fi

echo "==> 2/3  Conferindo que o pacote do apt NÃO vai vencer o nosso"
# O `install/setup.bash` do workspace entra na frente de `/opt/ros` no
# AMENT_PREFIX_PATH, então o nosso vence. Mas se alguém tiver o do apt
# instalado, vale saber — dois twist_mux no caminho é confusão garantida no
# dia em que um deles quebrar.
if dpkg -l ros-jazzy-twist-mux 2>/dev/null | grep -q '^ii'; then
    echo "    ⚠️  ros-jazzy-twist-mux (apt) ESTÁ instalado."
    echo "        O do workspace vence pelo AMENT_PREFIX_PATH, mas o do apt é"
    echo "        justamente o que não sobe. Se algo estranho acontecer,"
    echo "        confira com:  ros2 pkg prefix twist_mux"
    echo "        (tem de apontar para install/, não para /opt/ros)"
else
    echo "    não instalado pelo apt — caminho limpo."
fi

echo "==> 3/3  Build"
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
SHELL=/bin/bash colcon build --base-paths ros2_packages --symlink-install \
    --packages-select twist_mux_msgs twist_mux 2>/dev/null \
    || SHELL=/bin/bash colcon build --base-paths ros2_packages --symlink-install \
       --packages-select twist_mux

echo
echo "==> Pronto. Confira que o nó SOBE (é o teste que importa):"
echo "    source install/setup.bash"
echo "    ros2 pkg prefix twist_mux      # tem de ser o install/ daqui"
echo "    ros2 run twist_mux twist_mux --ros-args \\"
echo "        --params-file install/robot_motion/share/robot_motion/config/twist_mux.yaml"
