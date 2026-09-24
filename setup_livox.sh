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
#     descartável, o modelo e os perfis de MÁQUINA ficam versionados em
#     robot_base/config/. O IP do SENSOR vem de varredura ou argumento
#     explícito: unidade física e computador não são o mesmo eixo.
#
# Commits fixados de propósito: são os que foram verificados neste robô.
# Subir versão é uma mudança deliberada, não um efeito colateral de rodar o setup.
#
# Rodar de dentro da raiz do repo. Idempotente.
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKGS="$WS/ros2_packages"

uso() {
    cat <<'EOF'
Uso:
  ./setup_livox.sh --perfil <nuc|notebook> [--lidar-ip A.B.C.D]

O perfil escolhe somente o IP desta máquina. Sem --lidar-ip, o script varre a
sub-rede e exige exatamente um MAC Livox respondendo. Com --lidar-ip, aceita a
escolha explícita mesmo se o sensor estiver desligado, mas avisa que não houve
confirmação.
EOF
}

PERFIL=""
LIDAR_IP=""
while [ "$#" -gt 0 ]; do
    case "$1" in
        --perfil)
            [ "$#" -ge 2 ] || { echo "ERRO: --perfil exige um valor" >&2; uso >&2; exit 2; }
            PERFIL="$2"
            shift 2
            ;;
        --lidar-ip)
            [ "$#" -ge 2 ] || { echo "ERRO: --lidar-ip exige um valor" >&2; uso >&2; exit 2; }
            LIDAR_IP="$2"
            shift 2
            ;;
        -h|--help)
            uso
            exit 0
            ;;
        *)
            echo "ERRO: argumento desconhecido: $1" >&2
            uso >&2
            exit 2
            ;;
    esac
done
if [ -z "$PERFIL" ]; then
    echo "ERRO: --perfil é obrigatório; não há máquina padrão segura" >&2
    uso >&2
    exit 2
fi

# Esta pré-condição vem ANTES de dependência, clone, /usr/local e build. O
# defeito de 24-09 foi exatamente o driver subir com host .2 numa máquina .5:
# sem validar o IP local, o setup parecia bom e a nuvem nunca voltava.
CFG_MODELO="$PKGS/robot_base/config/MID360_config.template.json"
CFG_PERFIS="$PKGS/robot_base/config/livox_host_profiles.json"
CFG_GERADA="$(mktemp "${TMPDIR:-/tmp}/MID360_config.XXXXXX")"
limpa_config_temporaria() {
    rm -f -- "$CFG_GERADA"
}
trap limpa_config_temporaria EXIT

echo "==> Pré-condição: máquina e unidade do Mid-360"
PREPARA_CONFIG=(
    python3 "$WS/tools/prepara_config_livox.py"
    --perfis "$CFG_PERFIS"
    --modelo "$CFG_MODELO"
    --perfil "$PERFIL"
    --saida "$CFG_GERADA"
)
if [ -n "$LIDAR_IP" ]; then
    PREPARA_CONFIG+=(--lidar-ip "$LIDAR_IP")
fi
"${PREPARA_CONFIG[@]}"

LIVOX_URL="https://github.com/Livox-SDK/livox_ros_driver2.git"
LIVOX_COMMIT="6b9356c"
FASTLIO_URL="https://github.com/Ericsii/FAST_LIO.git"
FASTLIO_COMMIT="18418bc"
SDK_URL="https://github.com/Livox-SDK/Livox-SDK2.git"
# O SDK também é fixado, como os dois drivers. Até 23-09 ele era clonado com
# '--depth 1' sem revisão: quem rodasse o script hoje pegaria o `main` do dia,
# e a lib que vai para /usr/local — código nativo, fora do git, que nenhum
# teste nosso cobre — seria outra a cada máquina. A tag é a que está compilada
# neste PC desde 24-07; o SHA existe porque tag pode ser movida no upstream.
SDK_TAG="v1.3.1"
SDK_COMMIT="f5d9375f84efe2b15bc0a052d3e18482ed13adf4"
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

echo "==> 0/5  Pré-condição: dependências ROS dos drivers de terceiros"
# ANTES de qualquer coisa em /usr/local, e antes de clonar/compilar: se faltar
# dependência ROS, o script ia até o passo 5/5 e só lá quebrava — com erro de
# CMake ("By not providing Findpcl_ros.cmake...") que não diz o que instalar,
# e já tendo mexido no sistema. Achado em 23-09: o fast_lio reprovou assim,
# com o SDK já instalado.
#
# A lista é fixa, e não lida dos package.xml, porque esta conferência roda
# ANTES do clone — em máquina nova os package.xml ainda nem existem. São as
# dependências dos drivers de terceiros que a instalação padrão do ROS não
# traz; subir a lista é mudança deliberada, como os commits fixados.
ROS_SHARE="/opt/ros/jazzy/share"
DEPS_ROS=(pcl_ros pcl_conversions)
faltando=()
for dep in "${DEPS_ROS[@]}"; do
    [ -d "$ROS_SHARE/$dep" ] || faltando+=("$dep")
done
if [ ${#faltando[@]} -gt 0 ]; then
    apt_nomes=()
    for dep in "${faltando[@]}"; do
        apt_nomes+=("ros-jazzy-${dep//_/-}")
    done
    echo "ERRO: faltam dependências ROS dos drivers de terceiros:" >&2
    for dep in "${faltando[@]}"; do
        echo "      - $dep (nada em $ROS_SHARE/$dep)" >&2
    done
    echo >&2
    echo "      Instale e rode o script de novo:" >&2
    echo "        sudo apt install ${apt_nomes[*]}" >&2
    echo >&2
    echo "      (nada foi alterado no sistema nem clonado)" >&2
    exit 1
fi
echo "    ok: ${DEPS_ROS[*]}"

echo "==> 1/5  SDK nativo da Livox (Livox-SDK2)"
# O livox_ros_driver2 NÃO embute o SDK: o CMakeLists dele faz
#   find_library(LIVOX_LIDAR_SDK_LIBRARY liblivox_lidar_sdk_shared.so /usr/local/lib REQUIRED)
# ou seja, exige a lib C++ já instalada em /usr/local. Sem ela o colcon falha com
# "Could not find LIVOX_LIDAR_SDK_LIBRARY" — que não diz o que fazer. Daí este passo.
# É o único ponto do setup que precisa de sudo (instala em /usr/local).
# Fonte em local fixo (fora do git, ver .gitignore) e não em /tmp: o passo de
# instalação pede sudo, e se a senha falhar dá pra repetir o script sem
# recompilar o SDK inteiro de novo.
SDK_SRC="$WS/third_party/Livox-SDK2"

# A conferência da revisão vem ANTES da guarda do $SDK_LIB, de propósito: se o
# clone existir com outra revisão, a gente quer saber disso mesmo que a lib já
# esteja instalada — é justamente o caso em que o instalado e o fonte divergem
# em silêncio. E aqui o script RECUSA em vez de atualizar ou resetar sozinho:
# mexer em fonte de terceiro é mudança deliberada, com registro, não efeito
# colateral de rodar o setup.
if [ -d "$SDK_SRC/.git" ]; then
    SDK_HEAD="$(git -C "$SDK_SRC" rev-parse HEAD)"
    if [ "$SDK_HEAD" != "$SDK_COMMIT" ]; then
        echo "ERRO: $SDK_SRC está em outra revisão do Livox-SDK2." >&2
        echo "      esperado: $SDK_COMMIT ($SDK_TAG)" >&2
        echo "      presente: $SDK_HEAD" >&2
        echo "      Decida e registre: ou apague o diretório para reclonar a" >&2
        echo "      tag fixada, ou mude SDK_TAG/SDK_COMMIT neste script." >&2
        exit 1
    fi
    echo "    fonte em $SDK_SRC @ $SDK_TAG ($SDK_COMMIT)"
fi

if [ -f "$SDK_LIB" ]; then
    echo "    já instalado ($SDK_LIB)."
else
    if [ ! -d "$SDK_SRC/.git" ]; then
        echo "    clonando Livox-SDK2 @ $SDK_TAG em $SDK_SRC"
        git clone --depth 1 --branch "$SDK_TAG" "$SDK_URL" "$SDK_SRC"
        SDK_HEAD="$(git -C "$SDK_SRC" rev-parse HEAD)"
        # A tag do upstream pode ser movida; o SHA é quem fecha.
        if [ "$SDK_HEAD" != "$SDK_COMMIT" ]; then
            echo "ERRO: a tag $SDK_TAG do upstream não aponta mais para o SHA fixado." >&2
            echo "      esperado: $SDK_COMMIT" >&2
            echo "      veio:     $SDK_HEAD" >&2
            exit 1
        fi
    fi
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
# A fonte da verdade é a combinação validada no início: perfil da máquina +
# sensor descoberto/explicitado. O arquivo ativo continua dentro do clone e é
# descartável; por isso ele é sempre regenerado pelo setup.
CFG_ATIVA="$PKGS/livox_ros_driver2/config/MID360_config.json"
cp -f "$CFG_GERADA" "$CFG_ATIVA"
if ! cmp -s "$CFG_GERADA" "$CFG_ATIVA"; then
    echo "ERRO: a config gerada não foi copiada integralmente para o driver" >&2
    exit 1
fi
python3 - "$CFG_ATIVA" <<'PY'
import json, sys
c = json.load(open(sys.argv[1]))
print(f"    host:       {c['MID360']['host_net_info']['cmd_data_ip']}")
print(f"    lidar:      {c['lidar_configs'][0]['ip']}")
PY
echo "    config ativa: $CFG_ATIVA"

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
