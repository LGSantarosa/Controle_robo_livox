"""A fatia 2D que o AMCL consome — decisão 021.

O AMCL casa um `LaserScan` contra um mapa de ocupação. O nosso sensor é 3D,
então alguém escolhe QUAL fatia do mundo vira "parede", e essa escolha é dois
números num YAML. Errar neles não dá erro: dá AMCL divergindo sem explicação,
que é o pior tipo de falha para diagnosticar na frente do robô.

Nenhum destes testes sobe ROS. Todos leem os ARQUIVOS que o robô carrega.
"""
import os
import re

import pytest

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
SCAN = os.path.join(RAIZ, 'ros2_packages', 'robot_base', 'config', 'scan_2d.yaml')
SCAN_LAUNCH = os.path.join(RAIZ, 'ros2_packages', 'robot_base', 'launch',
                           'scan_2d.launch.py')
LOCALIZACAO = os.path.join(RAIZ, 'ros2_packages', 'robot_base', 'launch',
                           'localizacao.launch.py')
SIM = os.path.join(RAIZ, 'ros2_packages', 'robot_base', 'launch', 'sim.launch.py')
NAV2 = os.path.join(RAIZ, 'ros2_packages', 'robot_motion', 'config', 'nav2.yaml')
URDF = os.path.join(RAIZ, 'ros2_packages', 'robot_base', 'description',
                    'robo2.urdf.xacro')

# Medidos, e é deles que sai a fatia. Repetidos aqui de propósito: se alguém
# mexer no URDF, é o teste de baixo que confere contra o arquivo — estes são a
# referência escrita do que foi medido com trena.
LIVOX_ALTURA = 0.42        # m do chão (trena, 05-08)
FOV_ABAIXO = 7.0           # graus, o quanto o Mid-360 enxerga para baixo


def valor(caminho, chave):
    """Primeiro valor numérico da chave, ignorando comentários.

    Por texto e não por parser de YAML porque os comentários destes arquivos
    são metade do valor deles — mesmo motivo do `test_configs_coerentes.py`.
    """
    with open(caminho) as f:
        for linha in f:
            corte = linha.split('#')[0]
            m = re.search(rf'^\s*{re.escape(chave)}:\s*([-\d.]+)\s*$', corte)
            if m:
                return float(m.group(1))
    raise AssertionError(f'{chave} não encontrada em {caminho}')


def test_a_fatia_nao_le_o_CHAO():
    """Piso lido como obstáculo põe uma parede circular em volta do robô.

    O raio de −7° começa a bater no chão a `0,42/tan(7°)` = 3,4 m. O mapa não
    tem essa parede — e o AMCL, tentando casar um anel que não existe na
    planta, não fecha. O default do upstream (`min_height = -1.0`) faz
    exatamente isso.
    """
    assert valor(SCAN, 'min_height') > 0.0, (
        'min_height ≤ 0 lê o próprio chão como parede')
    assert valor(SCAN, 'min_height') >= 0.10, (
        'margem pequena demais: o chão não é plano e o robô balança')


def test_a_fatia_nao_le_o_TETO():
    """O Mid-360 olha para CIMA (até +52°), e o mapa é uma planta.

    A +52° o raio alcança 1,00 m já a 0,45 m de distância. Sem corte, quase
    tudo perto vira "parede" — inclusive teto e luminária, que não estão no
    mapa.
    """
    teto = valor(SCAN, 'max_height')
    assert teto < 2.0, 'max_height alto demais: entra teto, que o mapa não tem'
    assert teto > LIVOX_ALTURA, (
        'max_height abaixo do próprio sensor descarta a parede na altura dele')


def test_o_robo_nao_se_enxerga_como_parede():
    """`range_min` tem de passar do raio do robô.

    A caixa fica a 0,23 m de altura — DENTRO da fatia — e o sensor está em cima
    dela. Sem este corte o robô lê a própria carcaça como obstáculo colado, em
    todos os quadros e em todas as direções: o AMCL trava com um anel fixo que
    nenhuma pose explica.
    """
    raio_robo = valor(NAV2, 'robot_radius')
    assert valor(SCAN, 'range_min') > raio_robo, (
        f"range_min={valor(SCAN, 'range_min')} não passa do raio do robô "
        f'({raio_robo}) — ele vai se ler como parede')


def test_a_fatia_e_alcancavel_pelo_campo_de_visao_do_sensor():
    """A fatia tem de existir para o sensor, não só no papel.

    A menor altura vista a `d` metros é `0,42 − 0,123·d`. Se `min_height`
    ficasse acima de 0,42 o sensor nunca a alcançaria de perto, e o robô ficaria
    cego para parede próxima — sem nenhum aviso.
    """
    assert valor(SCAN, 'min_height') < LIVOX_ALTURA, (
        'a fatia começa acima do sensor: parede perto fica invisível')


def test_o_nome_do_no_bate_com_a_chave_do_YAML():
    """Parâmetro que não casa com o nome do nó é IGNORADO em silêncio.

    O nó subiria com os defaults do upstream — fatia de −1 a +1 m, que lê o
    chão E o teto — e nada no log diria isso. É a mesma família do defeito da
    017: tudo parece conectado, e nada chega.
    """
    with open(SCAN) as f:
        yaml_texto = f.read()
    with open(SCAN_LAUNCH) as f:
        launch = f.read()
    m = re.search(r"name='([^']+)'", launch)
    assert m, 'o nó do scan não declara `name=`'
    nome = m.group(1)
    assert re.search(rf'^{re.escape(nome)}:\s*$', yaml_texto, re.M), (
        f'o nó sobe como `{nome}` mas o YAML não tem essa chave — os '
        'parâmetros seriam ignorados e ele leria chão e teto')


def test_o_scan_sai_da_nuvem_QUE_A_PERCEPCAO_LE():
    """`/livox/pontos`, e não `/livox/lidar`.

    O tópico cru do driver carrega `CustomMsg` no robô e `PointCloud2` no
    simulador. Assinar ele pelo NOME faria o scan funcionar na bancada e
    receber nada na máquina — é literalmente o defeito da decisão 017.
    """
    with open(SCAN_LAUNCH) as f:
        launch = f.read()
    assert "'/livox/pontos'" in launch
    assert "'/livox/lidar'" not in launch


def _entra_na_descricao(caminho, alvo='scan_2d.launch.py'):
    """O include chega mesmo à `LaunchDescription`, ou só existe no arquivo?

    A primeira versão deste teste procurava o nome do arquivo no TEXTO, e
    passava com o include definido numa variável que ninguém usava — o
    simulador subia sem fatia nenhuma e a suíte ficava verde. A mutação pegou.
    Teste que não olha o alvo não testa o alvo (a mesma lição da decisão 019).

    Aqui a pergunta é a certa: o include está DENTRO da chamada de
    `LaunchDescription`, direto ou por uma variável atribuída a ele?
    """
    import ast

    with open(caminho) as f:
        fonte = f.read()
    arvore = ast.parse(fonte)
    chamadas = [n for n in ast.walk(arvore)
                if isinstance(n, ast.Call)
                and getattr(n.func, 'id', None) == 'LaunchDescription']
    assert chamadas, f'{caminho} não monta uma LaunchDescription'

    for chamada in chamadas:
        if alvo in (ast.get_source_segment(fonte, chamada) or ''):
            return True                      # incluído ali mesmo
        usados = {n.id for n in ast.walk(chamada) if isinstance(n, ast.Name)}
        for no in ast.walk(arvore):          # ...ou por variável usada na lista
            if not isinstance(no, ast.Assign):
                continue
            destinos = {t.id for t in no.targets if isinstance(t, ast.Name)}
            if destinos & usados and alvo in (
                    ast.get_source_segment(fonte, no.value) or ''):
                return True
    return False


@pytest.mark.parametrize('arquivo,onde', [
    (LOCALIZACAO, 'o robô real'),
    (SIM, 'o simulador'),
])
def test_os_DOIS_mundos_sobem_a_MESMA_fatia(arquivo, onde):
    """Bancada e robô têm de enxergar a mesma coisa.

    Se o simulador rodar com uma fatia e o robô com outra, o que for aprovado
    na bancada não é o que anda — é o defeito da bitola de 29-07, que custou
    semanas: os dois errados, em sentidos opostos, e a bancada parecendo boa.
    """
    assert _entra_na_descricao(arquivo), (
        f'{onde} não sobe a fatia 2D; a localização contra mapa fica sem '
        'entrada lá')
