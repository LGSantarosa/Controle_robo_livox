#!/usr/bin/env bash
# Prepara a camada de localização: clona os drivers de terceiros (Livox Mid-360
# e FAST-LIO) dentro de ros2_packages/ e instala a config de rede do lidar.
#
# POR QUE UM SCRIPT, e não os drivers commitados no repo:
#   - são upstreams grandes e de terceiros; vendorá-los inchava o repo e
#     escondia qual commit exato estamos usando;
#   - o livox_ros_driver2 precisa de um preparo pró-ROS2 que o upstream faz num
#     build.sh próprio (renomear package_ROS2.xml e launch_ROS2/). Sem esse
#     passo o colcon quebra com "package.xml does not exist" / member_of_group.
#   - a config de rede do Mid-360 mora DENTRO do clone. Como o clone é
#     descartável, a config versionada fica em robot_base/config/ e é copiada
#     para dentro aqui — senão se perde a cada reclone.
#
# Commits fixados de propósito: são os que foram verificados neste robô.
# Subir versão é uma mudança deliberada, não um efeito colateral de rodar o setup.
#
# Rodar de dentro da raiz do repo. Idempotente.
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKGS="$WS/ros2_packages"

LIVOX_URL="https://github.com/Livox-SDK/livox_ros_driver2.git"
LIVOX_COMMIT="6b9356c"
FASTLIO_URL="https://github.com/Ericsii/FAST_LIO.git"
FASTLIO_COMMIT="18418bc"
SDK_URL="https://github.com/Livox-SDK/Livox-SDK2.git"
SDK_LIB="/usr/local/lib/liblivox_lidar_sdk_shared.so"

clonar() {
    local dir="$1" url="$2" commit="$3" extra="${4:-}"
    if [ -d "$PKGS/$dir/.git" ]; then
        echo "    $dir já presente — mantendo (apague o diretório para refazer)."
        return
    fi
    echo "    clonando $dir @ $commit"
    # shellcheck disable=SC2086
    git clone $extra "$url" "$PKGS/$dir"
    git -C "$PKGS/$dir" checkout --quiet "$commit"
}

echo "==> 1/5  SDK nativo da Livox (Livox-SDK2)"
# O livox_ros_driver2 NÃO embute o SDK: o CMakeLists dele faz
#   find_library(LIVOX_LIDAR_SDK_LIBRARY liblivox_lidar_sdk_shared.so /usr/local/lib REQUIRED)
# ou seja, exige a lib C++ já instalada em /usr/local. Sem ela o colcon falha com
# "Could not find LIVOX_LIDAR_SDK_LIBRARY" — que não diz o que fazer. Daí este passo.
# É o único ponto do setup que precisa de sudo (instala em /usr/local).
if [ -f "$SDK_LIB" ]; then
    echo "    já instalado ($SDK_LIB)."
else
    # Fonte em local fixo (fora do git, ver .gitignore) e não em /tmp: o passo de
    # instalação pede sudo, e se a senha falhar dá pra repetir o script sem
    # recompilar o SDK inteiro de novo.
    SDK_SRC="$WS/third_party/Livox-SDK2"
    echo "    fonte em $SDK_SRC"
    [ -d "$SDK_SRC/.git" ] || git clone --depth 1 "$SDK_URL" "$SDK_SRC"
    # '-include cstdint' contorna um bug do upstream: vários headers do SDK usam
    # std::uint8_t / uint64_t sem incluir <cstdint>, e o GCC 13+ (Ubuntu 24.04)
    # deixou de puxar esse header transitivamente. O erro que aparece sem isso é
    # "'uint8_t' in namespace 'std' does not name a type", em arquivos variados.
    # Force-include resolve todos de uma vez, sem editar fonte de terceiro.
    cmake -S "$SDK_SRC" -B "$SDK_SRC/build" -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_CXX_FLAGS="-include cstdint"
    cmake --build "$SDK_SRC/build" -j"$(nproc)"
    echo "    instalando em /usr/local (pede sudo)"
    sudo cmake --install "$SDK_SRC/build"
    sudo ldconfig
fi

echo "==> 2/5  Drivers de terceiros em ros2_packages/"
clonar livox_ros_driver2 "$LIVOX_URL" "$LIVOX_COMMIT"
clonar FAST_LIO "$FASTLIO_URL" "$FASTLIO_COMMIT" --recursive

echo "==> 3/5  Preparo pró-ROS2 do livox_ros_driver2"
# Equivale ao que o build.sh do upstream faz para a edição ROS2.
( cd "$PKGS/livox_ros_driver2"
  cp -f package_ROS2.xml package.xml
  [ -d launch_ROS2 ] && cp -rf launch_ROS2/. launch/ 2>/dev/null || true
)

echo "==> 4/5  Config de rede do Mid-360"
# A fonte da verdade é robot_base/config/ (versionada). Ver o README de lá:
# IP errado = 'bind failed' = FAST-LIO sem nuvem = /Odometry nunca publicado.
CFG="$PKGS/robot_base/config/MID360_config.json"
cp -f "$CFG" "$PKGS/livox_ros_driver2/config/MID360_config.json"
python3 - "$CFG" <<'PY'
import json, sys
c = json.load(open(sys.argv[1]))
print(f"    host (NUC): {c['MID360']['host_net_info']['cmd_data_ip']}")
print(f"    lidar:      {c['lidar_configs'][0]['ip']}")
PY
echo "    (conferir contra a rede real — ver ros2_packages/robot_base/config/README.md)"

echo "==> 5/5  Build"
# O setup.bash do ROS lê variáveis não definidas; 'set -u' aborta nele.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
# Os cmake-args são exigidos pelo livox_ros_driver2 (edição ROS2); sem eles ele
# compila como se fosse ROS1 e não gera as mensagens.
SHELL=/bin/bash colcon build --base-paths ros2_packages --symlink-install \
    --packages-select livox_ros_driver2 fast_lio robot_base \
    --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble

echo
echo "==> Pronto. Para subir a base (robô LIGADO):"
echo "    source install/setup.bash"
echo "    ros2 launch robot_base base.launch.py"
