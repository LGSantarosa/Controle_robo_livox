"""Os testes importam o FONTE dos pacotes, não a cópia do install/.

Achado de 21-09 (etapa 4, passo 2): com o overlay carregado, `import
robot_motion` resolvia para `install/robot_motion/lib/.../site-packages` (e o
`robot_planning` para `build/`). O teste do `vao_frente()` seguiu VERMELHO com
o código já certo até um `colcon build` — e o inverso é pior: um módulo novo
que ainda não existe no install/ faria o vermelho depender de disciplina
manual. O `conftest.py` da raiz põe os quatro diretórios-fonte na frente do
`sys.path`.

O que NÃO muda, e também é travado aqui: launches e arquivos do `share/`
continuam vindo do install/ (`get_package_share_directory`, AMENT_PREFIX_PATH
intactos). Gazebo, console scripts e testes de instalação usam o install/,
recompilado pelos wrappers.
"""
import importlib
import os

import pytest

RAIZ = os.path.dirname(os.path.abspath(__file__))
PACOTES = ['robot_base', 'robot_motion', 'robot_nav', 'robot_planning']


@pytest.mark.parametrize('pacote', PACOTES)
def test_o_pacote_importado_e_o_fonte(pacote):
    mod = importlib.import_module(pacote)
    fonte = os.path.join(RAIZ, 'ros2_packages', pacote) + os.sep
    # O caminho LITERAL, sem realpath: o robot_planning deste PC vem de um
    # build/ com --symlink-install (28-07) que aponta para o fonte, e o
    # realpath o aprovaria por estado de build da máquina, não por garantia.
    assert os.path.abspath(mod.__file__).startswith(fonte), \
        f'{pacote} veio de {mod.__file__}, não de {fonte}'


def test_o_share_continua_vindo_do_install():
    ament = pytest.importorskip('ament_index_python.packages')
    share = ament.get_package_share_directory('robot_motion')
    assert os.path.join(RAIZ, 'install') + os.sep in share + os.sep, share
