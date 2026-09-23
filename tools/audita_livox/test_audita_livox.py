"""`bin/audita-livox` — o contrato dos TRÊS vereditos, contra máquina fingida.

O auditor responde sobre uma máquina que não é esta (o NUC). Não dá para
testá-lo rodando na máquina certa: por definição, a máquina errada é que
interessa. Então aqui se monta uma máquina inteira de mentira — `/opt/ros`,
workspace, `/usr/local`, clone do SDK — e se confere o veredito de cada
defeito, um por um.

**Por que três vereditos, e por que isso é o coração do teste:** "não consegui
provar" NÃO é "está errado". Clone do SDK ausente com a lib instalada é
INCONCLUSIVO, não reprovação — a lib pode estar perfeita, só não dá para
provar a procedência dali. Chamar isso de reprovação seria mentir sobre o
robô, e é o tipo de mentira que faz perder tarde de laboratório atrás de
defeito que não existe.

Códigos de saída: 0 aprovado, 1 reprovado, 2 inconclusivo sem reprovação. E
REPROVADO ganha de INCONCLUSIVO: o pior veredito manda.

Calços: `ros2`, `dpkg-query` e `ldconfig` entram por um PATH próprio (como no
`sobe_robo3`). O `git` é o de verdade, sobre repositórios de verdade criados
no tmp — fingir `git rev-parse` esconderia justamente o que se quer provar.
"""
import json
import os
import shutil
import subprocess

import pytest

RAIZ = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
AUDITOR = os.path.join(RAIZ, 'bin', 'audita-livox')
SHA_FIXADO = 'f5d9375f84efe2b15bc0a052d3e18482ed13adf4'

CONFIG = {'MID360': {'host_net_info': {'cmd_data_ip': '192.168.1.2'}},
          'lidar_configs': [{'ip': '192.168.1.158'}]}
CONFIG_VELHA = {'MID360': {'host_net_info': {'cmd_data_ip': '192.168.1.2'}},
                'lidar_configs': [{'ip': '192.168.1.169'}]}


def _escreve(caminho, texto, executavel=False):
    os.makedirs(os.path.dirname(caminho), exist_ok=True)
    with open(caminho, 'w') as f:
        f.write(texto)
    if executavel:
        os.chmod(caminho, 0o755)


class Maquina:
    """Uma máquina fingida, montada certa por padrão; cada teste estraga uma coisa."""

    def __init__(self, base):
        self.base = base
        self.ros_opt = os.path.join(base, 'opt', 'ros')
        self.ws = os.path.join(base, 'ws')
        self.calcos = os.path.join(base, 'calcos')
        self.sdk_lib = os.path.join(base, 'usr', 'local', 'lib',
                                    'liblivox_lidar_sdk_shared.so')
        self.prefixos = {}
        self.apt = {'pcl_ros': '2.6.5-1noble'}
        self.ldconfig = 'liblivox_lidar_sdk_shared.so (libc6,x86-64) => /usr/local/lib/x.so'
        # Por padrão a máquina TEM o IP do host do JSON; cada teste muda.
        self.ip_addr = ('lo               UNKNOWN        127.0.0.1/8\n'
                        'eth0             UP             192.168.1.2/24')

        _escreve(os.path.join(self.ros_opt, 'jazzy', 'setup.bash'), '')
        _escreve(os.path.join(self.ws, 'install', 'setup.bash'), '')
        _escreve(os.path.join(self.ws, 'ros2_packages', 'robot_base', 'config',
                              'MID360_config.json'), json.dumps(CONFIG))

        for pacote in ('pcl_ros', 'livox_ros_driver2', 'fast_lio'):
            prefixo = os.path.join(self.ws, 'install', pacote)
            os.makedirs(prefixo, exist_ok=True)
            self.prefixos[pacote] = prefixo
        self.config_runtime = os.path.join(
            self.prefixos['livox_ros_driver2'], 'share', 'livox_ros_driver2',
            'config', 'MID360_config.json')
        _escreve(self.config_runtime, json.dumps(CONFIG))

        self.clona_sdk(SHA_FIXADO)
        self.instala_sdk(conteudo=b'LIB-FIXADA')

    # --- montagem -----------------------------------------------------------
    def clona_sdk(self, _=None):
        """Repositório de VERDADE — fingir o `git rev-parse` esconderia o que se prova.

        O SHA de um repo criado agora não tem como ser o fixado, então é o
        ESPERADO que se injeta (`AUDITA_SDK_COMMIT`), apontando para este
        HEAD. O default do script, que é o que vale no robô, fica travado
        contra o `setup_livox.sh` em `test_sha_esperado_bate_com_o_setup`.
        """
        clone = os.path.join(self.ws, 'third_party', 'Livox-SDK2')
        os.makedirs(clone, exist_ok=True)
        git = ['git', '-C', clone, '-c', 'user.email=t@t', '-c', 'user.name=t']
        subprocess.run(['git', 'init', '-q', clone], check=True)
        subprocess.run(git + ['commit', '-q', '--allow-empty', '-m', 'x'], check=True)
        self.clone = clone
        self.sha_esperado = subprocess.run(
            ['git', '-C', clone, 'rev-parse', 'HEAD'],
            capture_output=True, text=True, check=True).stdout.strip()

    def build_sdk(self, conteudo=b'LIB-FIXADA'):
        alvo = os.path.join(self.ws, 'third_party', 'Livox-SDK2', 'build',
                            'sdk_core', 'liblivox_lidar_sdk_shared.so')
        os.makedirs(os.path.dirname(alvo), exist_ok=True)
        with open(alvo, 'wb') as f:
            f.write(conteudo)

    def instala_sdk(self, conteudo=b'LIB-FIXADA'):
        os.makedirs(os.path.dirname(self.sdk_lib), exist_ok=True)
        with open(self.sdk_lib, 'wb') as f:
            f.write(conteudo)

    # --- execução -----------------------------------------------------------
    def _monta_calcos(self):
        listagem = '\n'.join(f'{k}:{v}' for k, v in self.prefixos.items())
        _escreve(os.path.join(self.calcos, 'ros2'), f'''#!/usr/bin/env bash
[ "$1" = pkg ] && [ "$2" = prefix ] || exit 1
while IFS=: read -r nome prefixo; do
    [ "$nome" = "$3" ] && {{ echo "$prefixo"; exit 0; }}
done <<'LISTA'
{listagem}
LISTA
exit 1
''', executavel=True)
        apt = '\n'.join(f'{k}:{v}' for k, v in self.apt.items())
        _escreve(os.path.join(self.calcos, 'dpkg-query'), f'''#!/usr/bin/env bash
alvo="${{@: -1}}"
while IFS=: read -r nome versao; do
    [ "ros-jazzy-${{nome//_/-}}" = "$alvo" ] && {{ printf '%s' "$versao"; exit 0; }}
done <<'LISTA'
{apt}
LISTA
exit 1
''', executavel=True)
        # '%b' e não '%s': o printf só expande \n dentro do FORMATO, não do
        # argumento — com %s o `ip` fingido devolvia tudo numa linha só, e o
        # auditor lia a interface errada. Defeito do calço, achado em 23-09.
        _escreve(os.path.join(self.calcos, 'ip'),
                 f'#!/usr/bin/env bash\nprintf "%b\\n" {self.ip_addr!r}\n',
                 executavel=True)
        _escreve(os.path.join(self.calcos, 'ldconfig'),
                 f'#!/usr/bin/env bash\nprintf "%s\\n" {self.ldconfig!r}\n',
                 executavel=True)

    def audita(self, *args):
        self._monta_calcos()
        ambiente = dict(os.environ)
        ambiente.update({
            'AUDITA_ROS_OPT': self.ros_opt,
            'AUDITA_WS': self.ws,
            'AUDITA_SDK_LIB': self.sdk_lib,
            'AUDITA_SDK_COMMIT': self.sha_esperado,
            'PATH': self.calcos + os.pathsep + os.environ['PATH'],
        })
        return subprocess.run([AUDITOR, *args], env=ambiente,
                              capture_output=True, text=True, timeout=120)


@pytest.fixture()
def maquina(tmp_path):
    return Maquina(str(tmp_path))


# --- o caminho feliz --------------------------------------------------------

def test_maquina_certa_aprova(maquina):
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 0, r.stdout
    assert 'VEREDITO: APROVADO' in r.stdout
    assert 'REPROVADO' not in r.stdout


def test_relatorio_sempre_diz_o_que_nao_testa(maquina):
    """O auditor não pode ser lido como 'a localização funciona'."""
    maquina.build_sdk()
    r = maquina.audita()
    for palavra in ('rede', 'lidar', 'nuvem', '/Odometry'):
        assert palavra in r.stdout


# --- distro -----------------------------------------------------------------

def test_sem_distro_reprova(maquina):
    shutil.rmtree(os.path.join(maquina.ros_opt, 'jazzy'))
    r = maquina.audita()
    assert r.returncode == 1
    assert 'nenhuma' in r.stdout


def test_outra_distro_reprova_nomeando_a_suportada(maquina):
    shutil.rmtree(os.path.join(maquina.ros_opt, 'jazzy'))
    _escreve(os.path.join(maquina.ros_opt, 'humble', 'setup.bash'), '')
    r = maquina.audita()
    assert r.returncode == 1
    assert 'humble' in r.stdout and 'jazzy' in r.stdout


def test_duas_distros_e_inconclusivo_nao_reprovacao(maquina):
    _escreve(os.path.join(maquina.ros_opt, 'humble', 'setup.bash'), '')
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 2, r.stdout
    assert 'sem escolha inequívoca' in r.stdout


# --- as três camadas do ROS -------------------------------------------------

def test_apt_sem_pcl_ros_reprova_com_o_comando_exato(maquina):
    maquina.apt = {}
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1
    assert 'sudo apt install ros-jazzy-pcl-ros' in r.stdout


def test_overlay_sem_fast_lio_reprova(maquina):
    """O defeito de 23-09: apt e ROS base certos, workspace sem o pacote."""
    del maquina.prefixos['fast_lio']
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1
    assert 'fast_lio' in r.stdout


def test_sem_install_setup_reprova_dizendo_que_falta_construir(maquina):
    os.remove(os.path.join(maquina.ws, 'install', 'setup.bash'))
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1
    assert 'não construído' in r.stdout


# --- o SDK: presença e procedência são perguntas diferentes -----------------

def test_lib_ausente_reprova(maquina):
    os.remove(maquina.sdk_lib)
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1
    assert 'ausente' in r.stdout


def test_clone_ausente_com_lib_presente_e_inconclusivo(maquina):
    """A lib pode estar perfeita; só não dá para provar a procedência dali."""
    shutil.rmtree(os.path.join(maquina.ws, 'third_party'))
    r = maquina.audita()
    assert r.returncode == 2, r.stdout
    assert 'REPROVADO' not in r.stdout


def test_sha_diferente_reprova(maquina):
    maquina.sha_esperado = SHA_FIXADO      # o clone está em outro commit
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1
    assert SHA_FIXADO in r.stdout


def test_sha_esperado_bate_com_o_setup(maquina):
    """A trava: o default do auditor é o mesmo commit que o setup instala.

    Sem isto, o override do teste deixaria o valor real livre para derivar, e
    o auditor aprovaria no robô uma revisão que o setup nunca instalaria.
    """
    def constante(caminho, nome):
        with open(os.path.join(RAIZ, caminho)) as f:
            for linha in f:
                if nome in linha and '=' in linha:
                    return linha.split('"')[-2] if '"' in linha else None
        return None

    with open(AUDITOR) as f:
        fonte = f.read()
    assert SHA_FIXADO in fonte, 'o default do auditor mudou'
    assert constante('setup_livox.sh', 'SDK_COMMIT=') == SHA_FIXADO


def test_lib_instalada_diferente_do_build_reprova(maquina):
    """O caso que o SHA do clone sozinho NÃO pega."""
    maquina.build_sdk(conteudo=b'OUTRA-LIB')
    r = maquina.audita()
    assert r.returncode == 1
    assert 'DIFERE do build fixado' in r.stdout


def test_sem_build_local_a_procedencia_e_inconclusiva(maquina):
    r = maquina.audita()
    assert 'não verificável' in r.stdout


# --- a config que o launch de fato lê ---------------------------------------

def test_config_de_runtime_com_ip_velho_reprova_e_mostra_o_ip(maquina):
    """O .169 mudo: bind failed, FAST-LIO sem nuvem, /Odometry nunca publicado."""
    _escreve(maquina.config_runtime, json.dumps(CONFIG_VELHA))
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1
    assert '192.168.1.169' in r.stdout


def test_config_de_runtime_ausente_reprova(maquina):
    os.remove(maquina.config_runtime)
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1


def test_audita_o_runtime_e_nao_o_clone(maquina):
    """Sem --symlink-install o install/ tem cópia real: o clone pode mentir."""
    clone_cfg = os.path.join(maquina.ws, 'ros2_packages', 'livox_ros_driver2',
                             'config', 'MID360_config.json')
    _escreve(clone_cfg, json.dumps(CONFIG))          # clone certo
    _escreve(maquina.config_runtime, json.dumps(CONFIG_VELHA))   # runtime errado
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1, 'auditou o clone em vez do runtime'


# --- prioridade dos vereditos e saída em arquivo -----------------------------

def test_reprovado_ganha_de_inconclusivo(maquina):
    shutil.rmtree(os.path.join(maquina.ws, 'third_party'))   # inconclusivo
    maquina.apt = {}                                          # reprovação
    r = maquina.audita()
    assert r.returncode == 1
    assert 'INCONCLUSIVO' in r.stdout and 'VEREDITO: REPROVADO' in r.stdout


def test_saida_grava_arquivo_com_o_relatorio(maquina, tmp_path):
    maquina.build_sdk()
    destino = str(tmp_path / 'relatorio.txt')
    r = maquina.audita('--saida', destino)
    assert r.returncode == 0
    with open(destino) as f:
        conteudo = f.read()
    assert 'VEREDITO: APROVADO' in conteudo


# --- o IP do host contra a interface real -----------------------------------
# O cmp fonte/runtime prova que os dois arquivos são iguais; NÃO prova que o
# endereço deles existe nesta máquina. Com o notebook no lugar do NUC (23-09),
# esse é o defeito mais provável: config idêntica dos dois lados e host que
# nunca sobe. bind failed com tudo verde.

def test_ip_do_host_presente_aprova(maquina):
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 0, r.stdout
    assert '192.168.1.2 em eth0 (UP)' in r.stdout


def test_maquina_na_rede_do_lidar_com_outro_ip_reprova(maquina):
    """O caso notebook: está na rede certa, com o endereço errado."""
    maquina.ip_addr = 'eth0             UP             192.168.1.77/24'
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1
    assert '192.168.1.77' in r.stdout and '192.168.1.2' in r.stdout


def test_sem_nada_na_sub_rede_e_inconclusivo_nao_reprovacao(maquina):
    """Cabo fora hoje não é máquina errada."""
    maquina.ip_addr = 'wlan0            UP             10.0.0.5/24'
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 2, r.stdout
    assert 'cabo fora' in r.stdout


def test_sub_rede_parecida_nao_conta_como_a_do_lidar(maquina):
    """Ponto é curinga em regex: 192.168.1. casava com 192.168.18.9.

    O auditor REPROVAVA uma máquina por estar numa sub-rede que só PARECE a do
    lidar. Achado em 23-09 rodando o próprio script no PC de dev.
    """
    maquina.ip_addr = 'wlan0            UP             192.168.18.9/24'
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 2, r.stdout
    assert 'REPROVADO' not in r.stdout
