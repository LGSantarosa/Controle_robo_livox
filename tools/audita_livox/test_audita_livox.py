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
import ipaddress
import json
import os
import shutil
import subprocess
import sys

import pytest

RAIZ = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
AUDITOR = os.path.join(RAIZ, 'bin', 'audita-livox')
SHA_FIXADO = 'f5d9375f84efe2b15bc0a052d3e18482ed13adf4'

# A árvore da decisão 058: só template e perfis são versionados. O JSON ativo
# é o template materializado pelo MESMO gerador que o setup usa.
CONFIG_VERSIONADA = os.path.join(RAIZ, 'ros2_packages', 'robot_base', 'config')
TEMPLATE = os.path.join(CONFIG_VERSIONADA, 'MID360_config.template.json')
PERFIS = os.path.join(CONFIG_VERSIONADA, 'livox_host_profiles.json')
sys.path.insert(0, os.path.join(RAIZ, 'tools'))
from prepara_config_livox import renderiza_config  # noqa: E402


def config_ativa(host='192.168.1.2', lidar='192.168.1.158'):
    from pathlib import Path
    return renderiza_config(Path(TEMPLATE), ipaddress.ip_address(host),
                            ipaddress.ip_address(lidar))


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
        self.config_ws = os.path.join(self.ws, 'ros2_packages', 'robot_base', 'config')
        os.makedirs(self.config_ws, exist_ok=True)
        shutil.copy(TEMPLATE, self.config_ws)
        shutil.copy(PERFIS, self.config_ws)

        for pacote in ('pcl_ros', 'livox_ros_driver2', 'fast_lio'):
            prefixo = os.path.join(self.ws, 'install', pacote)
            os.makedirs(prefixo, exist_ok=True)
            self.prefixos[pacote] = prefixo
        self.config_runtime = os.path.join(
            self.prefixos['livox_ros_driver2'], 'share', 'livox_ros_driver2',
            'config', 'MID360_config.json')
        self.grava_runtime(config_ativa())

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

    def grava_runtime(self, config):
        _escreve(self.config_runtime, json.dumps(config, indent=2))

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


# --- a config que o launch de fato lê (contrato da decisão 059) ------------

def linha(r, nome):
    """A linha do relatório de uma checagem, para não casar texto de outra."""
    for texto in r.stdout.splitlines():
        if f'] {nome}' in texto:
            return texto
    raise AssertionError(f'sem linha "{nome}" em:\n{r.stdout}')


def test_arvore_058_sem_json_universal_aprova(maquina):
    """O defeito da composição etapa 6 + 058: a máquina certa reprovava.

    Template e perfis versionados, NENHUM robot_base/config/MID360_config.json,
    e um JSON ativo válido no runtime. Antes da 059 o auditor dizia
    "sem o versionado" aqui, em qualquer máquina.
    """
    assert not os.path.exists(os.path.join(maquina.config_ws, 'MID360_config.json'))
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 0, r.stdout
    assert 'sem o versionado' not in r.stdout
    assert '[APROVADO' in linha(r, 'config de rede')


def test_aprovado_nomeia_perfil_host_sensor_e_nao_alega_o_sensor(maquina):
    maquina.build_sdk()
    r = maquina.audita()
    detalhe = linha(r, 'config de rede')
    for trecho in ('nuc', '192.168.1.2/24', '192.168.1.158', 'não consultado'):
        assert trecho in detalhe, detalhe


def test_perfil_notebook_da_bancada_de_24_09_aprova(maquina):
    """O par que entregou nuvem em 24-09: notebook .5 e sensor .169."""
    maquina.grava_runtime(config_ativa('192.168.1.5', '192.168.1.169'))
    maquina.ip_addr = 'enp1s0           UP             192.168.1.5/24'
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 0, r.stdout
    assert 'notebook' in linha(r, 'config de rede')
    assert '192.168.1.5/24 em enp1s0 (UP)' in linha(r, 'IP do host')


def test_template_versionado_ausente_reprova(maquina):
    os.remove(os.path.join(maquina.config_ws, 'MID360_config.template.json'))
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1
    assert 'MID360_config.template.json' in linha(r, 'config de rede')


def test_perfis_versionados_ausentes_reprova(maquina):
    os.remove(os.path.join(maquina.config_ws, 'livox_host_profiles.json'))
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1
    assert 'livox_host_profiles.json' in linha(r, 'config de rede')


def test_placeholder_no_runtime_reprova(maquina):
    config = config_ativa()
    config['MID360']['host_net_info']['imu_data_ip'] = '__HOST_IP__'
    maquina.grava_runtime(config)
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1
    assert '__HOST_IP__' in linha(r, 'config de rede')


def test_campos_de_host_divergentes_reprova(maquina):
    config = config_ativa()
    config['MID360']['host_net_info']['push_msg_ip'] = '192.168.1.5'
    maquina.grava_runtime(config)
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1
    assert '[REPROVADO' in linha(r, 'config de rede')


def test_host_net_info_que_nao_e_objeto_reprova(maquina):
    """JSON estruturalmente inválido é defeito da máquina, não do conferidor."""
    config = config_ativa()
    config['MID360']['host_net_info'] = [config['MID360']['host_net_info']]
    maquina.grava_runtime(config)
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1, r.stdout
    detalhe = linha(r, 'config de rede')
    assert '[REPROVADO' in detalhe and 'host_net_info' in detalhe
    assert 'conferidor falhou' not in r.stdout


def test_entrada_de_lidar_que_nao_e_objeto_reprova(maquina):
    config = config_ativa()
    config['lidar_configs'] = ['192.168.1.158']
    maquina.grava_runtime(config)
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1, r.stdout
    detalhe = linha(r, 'config de rede')
    assert '[REPROVADO' in detalhe and 'lidar_configs[0]' in detalhe
    assert 'conferidor falhou' not in r.stdout


def test_host_sem_perfil_reprova(maquina):
    maquina.grava_runtime(config_ativa('192.168.1.9', '192.168.1.158'))
    maquina.ip_addr = 'eth0             UP             192.168.1.9/24'
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1
    assert 'perfil' in linha(r, 'config de rede')
    # Sem perfil não há máscara esperada: a 8 não inventa uma.
    assert '[INCONCLUSIVO' in linha(r, 'IP do host')


@pytest.mark.parametrize('sensor', ['192.168.2.169', '192.168.1.2',
                                    '192.168.1.0', '192.168.1.255'])
def test_sensor_invalido_para_o_perfil_reprova_e_mostra_o_ip(maquina, sensor):
    """Fora da rede, igual ao host, endereço de rede ou broadcast."""
    config = config_ativa()
    config['lidar_configs'][0]['ip'] = sensor
    maquina.grava_runtime(config)
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1
    assert sensor in linha(r, 'config de rede')


def test_campo_editado_a_mao_reprova(maquina):
    """IPs válidos, mas o resto não é o template: materializar pega a porta."""
    config = config_ativa()
    config['MID360']['host_net_info']['point_data_port'] = 56399
    maquina.grava_runtime(config)
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1
    assert 'template' in linha(r, 'config de rede')


def test_config_de_runtime_ausente_reprova(maquina):
    os.remove(maquina.config_runtime)
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1


def test_audita_o_runtime_e_nao_o_clone(maquina):
    """Sem --symlink-install o install/ tem cópia real: o clone pode mentir."""
    clone_cfg = os.path.join(maquina.ws, 'ros2_packages', 'livox_ros_driver2',
                             'config', 'MID360_config.json')
    _escreve(clone_cfg, json.dumps(config_ativa()))          # clone certo
    errada = config_ativa()
    errada['lidar_configs'][0]['ip'] = '__LIDAR_IP__'        # runtime errado
    maquina.grava_runtime(errada)
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
# A checagem 7 prova que o JSON ativo é uma materialização válida; NÃO prova
# que o host dele existe nesta máquina. Host lido do RUNTIME (059), máscara do
# perfil que casou.

def test_ip_do_host_presente_aprova(maquina):
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 0, r.stdout
    assert '192.168.1.2/24 em eth0 (UP)' in linha(r, 'IP do host')


def test_maquina_na_rede_do_lidar_com_outro_ip_reprova(maquina):
    """O caso notebook: está na rede certa, com o endereço errado."""
    maquina.ip_addr = 'eth0             UP             192.168.1.77/24'
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1
    detalhe = linha(r, 'IP do host')
    assert '192.168.1.77' in detalhe and '192.168.1.2' in detalhe


def test_host_com_mascara_errada_reprova(maquina):
    maquina.ip_addr = 'eth0             UP             192.168.1.2/16'
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1
    assert '/16' in linha(r, 'IP do host')


def test_host_em_duas_interfaces_reprova(maquina):
    maquina.ip_addr = ('eth0             UP             192.168.1.2/24\n'
                       'eth1             UP             192.168.1.2/24')
    maquina.build_sdk()
    r = maquina.audita()
    assert r.returncode == 1
    detalhe = linha(r, 'IP do host')
    assert 'eth0' in detalhe and 'eth1' in detalhe


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
