"""Os testes importam o FONTE dos pacotes ROS, não a cópia do install/.

Com o overlay carregado, `import robot_motion` resolvia para o install/ (ou o
build/), e o vermelho/verde dependia de um `colcon build` feito antes — achado
de 21-09, etapa 4, passo 2. Aqui os quatro diretórios-fonte entram no INÍCIO do
`sys.path`, resolvidos a partir deste arquivo. Travado por
`test_import_do_fonte.py`.

Só o import de Python muda. `AMENT_PREFIX_PATH` e `get_package_share_directory`
ficam intactos: launches, configs e tudo do `share/` continuam vindo do
install/. Gazebo, console scripts e testes de instalação usam o install/,
recompilado pelos wrappers.
"""
import os
import sys

# E o pytest NÃO desce nos produtos do colcon. O padrão dele pula `build/` mas
# não `install/`: lá ele achava os `__init__.py` dos pacotes instalados, o
# plugin `launch_testing` os importava, e o `import_path` do pytest punha o
# site-packages do install/ NA FRENTE do sys.path — por cima do que vem abaixo.
# Nenhum desses diretórios tem teste. `*/` pega também o lixo de colcon que
# existe dentro de pacotes (robot_motion/install, robot_base/description/...).
collect_ignore_glob = ['install', 'log', 'build', '*/install', '*/log', '*/build']

_RAIZ = os.path.dirname(os.path.abspath(__file__))

for _pacote in reversed(('robot_base', 'robot_motion', 'robot_nav', 'robot_planning')):
    _fonte = os.path.join(_RAIZ, 'ros2_packages', _pacote)
    if _fonte in sys.path:
        sys.path.remove(_fonte)
    sys.path.insert(0, _fonte)
