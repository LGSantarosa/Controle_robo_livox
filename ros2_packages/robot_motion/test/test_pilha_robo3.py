"""A `pilha` sobe o robô 3 no Gazebo — etapa 6 (`docs/PLANO_ETAPA6_ROBO3.md`).

🔴 ESTE ARQUIVO NASCE VERMELHO, de propósito (passo 2 do §7 do plano). Ele
descreve o contrato da etapa 6 contra a pilha de hoje, que ainda recusa
`robo:=3`. Cada teste tem de falhar pelo motivo PREVISTO — não por import, não
por fixture, não por contexto mal montado. Os controles positivos abaixo
(`robo:=2`) existem justamente para separar as duas coisas: se eles falharem,
o problema é o arquivo de teste, não a pilha.

O que está descrito aqui, na ordem do plano:

  §2  a matriz: (2,sim) e (2,real) intactos; (3,sim) sobe; (3,real) RECUSADO;
  §3  D1 — o robô vem do `argv`, e a trava de coerência tem três bocas;
      D2 — os dois YAMLs do robô 3 na pasta da corrida, que é a raiz de
           evidências, com o bag num subdiretório ainda inexistente;
      D3 — mux próprio, `twist_mux_pilha_robo3.yaml`, com QUATRO faixas;
  §4.7 a escrita só acontece DEPOIS da validação — recusa não deixa rastro;
  §0-C o include do simulador só recebe argumento que o alvo declara.

⚠️ Sobre o `sys.argv`: a seleção do perfil é uma PONTE deliberada (D1), e por
isso todo teste que quer o robô 3 tem de dizer isso nas DUAS bordas — no
`argv` e no contexto. Montar só uma é justamente o que a trava mata, e há
teste para cada uma dessas montagens erradas.
"""
import importlib
import os

import pytest
import yaml

pytest.importorskip('launch_ros')

from ament_index_python.packages import get_package_share_directory  # noqa: E402
from launch import LaunchContext  # noqa: E402
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,  # noqa: E402
                            IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.launch_description_sources import (  # noqa: E402
    get_launch_description_from_python_launch_file)
from launch.utilities import perform_substitutions  # noqa: E402
from launch_ros.actions import Node  # noqa: E402
from launch_ros.utilities import evaluate_parameters  # noqa: E402

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PILHA = os.path.join(PKG, 'launch', 'pilha.launch.py')
# ⚠️ Os `share/` do install/, e não os diretórios-fonte: é deles que a launch
# monta os caminhos, e este arquivo compara CAMINHO INTEIRO com o que o nó
# recebe. Comparar contra o fonte daria diferença onde não há.
SHARE_MOTION = get_package_share_directory('robot_motion')
SHARE_BASE = get_package_share_directory('robot_base')
OPERACIONAIS = (Node, IncludeLaunchDescription, ExecuteProcess)
NOS_DO_NAV2 = ('map_server', 'planner_server', 'smoother_server',
               'controller_server', 'bt_navigator', 'amcl')
# As quatro faixas do mux do robô 3 (D3). O `unstuck_vel` está aqui porque o
# perfil do robô 3 CONFIGURA o desencalhe (`perfil.py:108`) e quem publica
# nele é o `path_follower` — sem a faixa, o desencalhe fala para o vazio.
FAIXAS_ROBO3 = {'dpad_vel': 110, 'joy_vel': 100, 'unstuck_vel': 30,
                'auto_vel': 10}
# O que a pilha tem de passar ao `sim_robo3` (achado C do §0): os seis que ele
# declara, e nada além — `planta`, que a pilha passa hoje ao sim do robô 2,
# mataria o include na subida.
ARGS_DO_SIM_ROBO3 = {'mundo', 'x', 'y', 'yaw', 'gui', 'placa'}


def _perfil():
    return importlib.import_module('robot_motion.perfil')


def _txt(x):
    """O texto de uma substituição (ou lista delas) sem precisar de contexto.

    `ExecuteProcess` e `IncludeLaunchDescription` guardam até as strings
    literais como `TextSubstitution`; `str()` neles devolve o `repr` do objeto,
    e comparar isso com um nome de arquivo dá verde/vermelho por acidente.
    """
    if isinstance(x, str):
        return x
    if isinstance(x, (list, tuple)):
        return ''.join(_txt(i) for i in x)
    return getattr(x, 'text', str(x))


def _argv(monkeypatch, *args):
    """A linha de comando como o `ros2 launch` a entrega (D1: a ponte)."""
    monkeypatch.setattr(
        'sys.argv',
        ['ros2', 'launch', 'robot_motion', 'pilha.launch.py', *args])


def _descricao():
    return get_launch_description_from_python_launch_file(PILHA)


def _percorre(ld, argumentos, devolvidas=None):
    """Visita argumentos e validações num contexto real; não sobe nada.

    `devolvidas`, se vier, recolhe as ações que as `OpaqueFunction` devolvem —
    é por lá que a materialização anuncia os caminhos que escreveu.
    """
    ctx = LaunchContext()
    ctx.launch_configurations.update(argumentos)
    alcancadas = []
    for e in ld.entities:
        if isinstance(e, OPERACIONAIS):
            alcancadas.append(e)
        elif isinstance(e, (DeclareLaunchArgument, OpaqueFunction)):
            try:
                saida = e.visit(ctx)
            except Exception as erro:  # noqa: BLE001 - o teste examina qual
                return alcancadas, erro
            if devolvidas is not None and saida:
                devolvidas.extend(saida)
    return alcancadas, None


def _nos(ld, nome):
    return [e for e in ld.entities
            if isinstance(e, Node) and e._Node__node_name == nome]


def _contexto(ld, **argumentos):
    ctx = LaunchContext()
    ctx.launch_configurations.update(argumentos)
    for e in ld.entities:
        if isinstance(e, DeclareLaunchArgument):
            e.visit(ctx)
    return ctx


def _params(ctx, no):
    return [str(p) if isinstance(p, os.PathLike) else p
            for p in evaluate_parameters(ctx, no._Node__parameters)]


def _fonte(include, ctx=None):
    """O caminho do arquivo que um `IncludeLaunchDescription` inclui.

    ⚠️ NÃO se lê pelo `.location` da source: ele é uma **string** que contém o
    `repr` da substituição (`'<launch.substitutions…TextSubstitution object at
    0x…>'`), e comparar isso com um nome de arquivo dá vermelho eterno — o
    teste continuaria falhando depois da implementação, que é a pior espécie
    de teste. O caminho de verdade está na lista de substituições guardada em
    `__location`, e só existe depois de resolvida num contexto.
    """
    loc = vars(include._IncludeLaunchDescription__launch_description_source
               )['_LaunchDescriptionSource__location']
    return perform_substitutions(ctx or LaunchContext(),
                                 loc if isinstance(loc, list) else [loc])


def _pasta_da_corrida(tmp_path):
    """A raiz de evidências (D2): `<log_dir>/corrida_<carimbo>/`.

    O carimbo é da subida, então o teste não o adivinha: ele olha o que
    apareceu embaixo do `log_dir` que ele mesmo deu.
    """
    filhos = sorted(p for p in tmp_path.iterdir() if p.is_dir())
    assert len(filhos) == 1, f'esperava UMA pasta de corrida, achei {filhos}'
    return filhos[0]


# ─── §2: a matriz das quatro combinações ─────────────────────────────────────

@pytest.mark.parametrize('sim', ['true', 'false'], ids=['gazebo', 'real'])
def test_robo2_segue_intacto_nas_duas_bordas(monkeypatch, sim):
    """CONTROLE POSITIVO — tem de estar VERDE hoje e depois da etapa 6.

    Se este cair, o vermelho dos outros não significa nada: quer dizer que o
    arquivo de teste está errado, não a pilha.
    """
    _argv(monkeypatch, f'sim:={sim}')
    alcancadas, erro = _percorre(_descricao(), {'sim': sim})
    assert erro is None, erro
    assert alcancadas


def test_robo3_no_gazebo_sobe(monkeypatch):
    """A linha que a etapa 6 existe para virar verde."""
    _argv(monkeypatch, 'robo:=3', 'sim:=true')
    alcancadas, erro = _percorre(_descricao(), {'robo': '3', 'sim': 'true'})
    assert erro is None, erro
    assert alcancadas


def test_robo3_no_robo_real_continua_recusado(monkeypatch):
    """E a recusa é TRAVA, não pendência: a mensagem nomeia o que falta.

    Não basta morrer — morrer dizendo "etapa 6" (o texto de hoje) convida a
    tentar de novo depois do merge. O texto tem de dizer as duas coisas que
    impedem: a fronteira do atuador real, que não está no grafo, e a
    localização, que é a etapa 7.
    """
    _argv(monkeypatch, 'robo:=3', 'sim:=false')
    alcancadas, erro = _percorre(_descricao(), {'robo': '3', 'sim': 'false'})
    assert isinstance(erro, RuntimeError), erro
    texto = str(erro).lower()
    assert 'atuador' in texto, str(erro)
    assert 'localiza' in texto, str(erro)
    assert alcancadas == [], 'recusa depois de ação operacional não é recusa'


# ─── §3 D1: a ponte do `argv`, e as três bocas da trava ──────────────────────

def test_o_perfil_do_robo3_e_montado_pelo_perfil(monkeypatch):
    """Trocado o perfil, troca o que o nó recebe — e o robô pedido é o 3."""
    chamadas = []
    real = _perfil().parametros

    def espia(robo, share, **kw):
        chamadas.append(robo)
        return real(robo, share, **kw)

    monkeypatch.setattr(_perfil(), 'parametros', espia)
    _argv(monkeypatch, 'robo:=3', 'sim:=true')
    _descricao()
    assert chamadas == [3], f'a pilha pediu o perfil {chamadas}, não o do robô 3'


def test_robo_repetido_no_argv_morre(monkeypatch):
    """Boca 1: `robo:=2 robo:=3` não é ambiguidade para resolver, é erro.

    Qualquer desempate (o primeiro, o último) escolheria robô por acidente —
    exatamente o que o `_recusa_robo` recusa fazer com " 2" e "2.0".
    """
    _argv(monkeypatch, 'robo:=2', 'robo:=3', 'sim:=true')
    try:
        _, erro = _percorre(_descricao(), {'robo': '3', 'sim': 'true'})
    except Exception as cedo:  # noqa: BLE001 - morrer na descrição também vale
        erro = cedo
    assert erro is not None, 'nada morreu com `robo:=` repetido'
    # E morreu POR ISSO: hoje a pilha morre por outro motivo (a recusa do
    # robô 3), o que faria este teste passar sem a trava existir.
    assert 'repetid' in str(erro).lower() or 'duas vezes' in str(erro).lower(), \
        str(erro)


def test_argv_e_contexto_divergindo_morre(monkeypatch):
    """Boca 2: o `argv` diz 2 e o contexto diz 3.

    É o modo de falha caro da D1, e o único que justifica a ponte existir com
    trava: o perfil viria do 2 e a pilha subiria como 3 — footprint do robô
    errado, sem uma linha de aviso.
    """
    _argv(monkeypatch, 'sim:=true')
    alcancadas, erro = _percorre(_descricao(), {'robo': '3', 'sim': 'true'})
    assert isinstance(erro, RuntimeError), erro
    assert 'argv' in str(erro).lower() or 'linha de comando' in str(erro).lower()
    assert alcancadas == []


def test_include_programatico_do_robo3_morre(monkeypatch):
    """Boca 3: outra launch incluindo esta com `robo:=3`.

    Não passa pelo `argv`, então cai na boca 2 — e o teste existe para fixar
    que o caso é ESTE, e não "o include funciona por acaso".
    """
    _argv(monkeypatch)  # nenhum argumento: é o default `2` que o argv entrega
    _, erro = _percorre(_descricao(), {'robo': '3', 'sim': 'true'})
    assert isinstance(erro, RuntimeError), erro
    # Pela boca 2, e não pela recusa genérica de hoje — senão este teste
    # ficaria verde sem a trava existir.
    assert 'argv' in str(erro).lower() or 'linha de comando' in str(erro).lower(), \
        str(erro)


# ─── §3 D2 + §4.7: os dois arquivos, e quando eles nascem ────────────────────

def _corrida_do_robo3(monkeypatch, tmp_path):
    """Sobe a descrição do robô 3 até o fim das validações e devolve o que saiu.

    Devolve (ld, pasta da corrida, ações devolvidas pelas OpaqueFunction).
    """
    _argv(monkeypatch, 'robo:=3', 'sim:=true', f'log_dir:={tmp_path}')
    ld = _descricao()
    devolvidas = []
    _, erro = _percorre(ld, {'robo': '3', 'sim': 'true',
                             'log_dir': str(tmp_path)}, devolvidas)
    assert erro is None, erro
    return ld, _pasta_da_corrida(tmp_path), devolvidas


def _materializados(pasta):
    """Os dois YAMLs escritos, por papel — e são exatamente dois."""
    escritos = sorted(p for p in pasta.glob('*.yaml'))
    assert len(escritos) == 2, f'esperava os DOIS YAMLs, achei {escritos}'
    nav2 = [p for p in escritos if 'nav2' in p.name]
    cm = [p for p in escritos if 'collision_monitor' in p.name]
    assert len(nav2) == 1 and len(cm) == 1, escritos
    return nav2[0], cm[0]


def test_os_dois_yamls_do_robo3_sao_materializados(monkeypatch, tmp_path):
    """Nav2 e collision_monitor, escritos, e IGUAIS a `aplica_reescritas`.

    Reescrita pedida e não aplicada é costmap lendo o arquivo sem ela, em
    silêncio — a guarda que a pilha tem hoje existe por isso. A etapa 6 troca
    a guarda pela aplicação, e isto aqui é o que prova que trocou.
    """
    _, pasta, _ = _corrida_do_robo3(monkeypatch, tmp_path)
    nav2, cm = _materializados(pasta)

    perfil = _perfil()
    esperado = perfil.parametros(3, SHARE_MOTION, share_base=SHARE_BASE)
    for arquivo, chave, reescritas in (
            (nav2, 'nav2', esperado['nav2_rewrites']),
            (cm, 'collision_monitor', esperado['collision_monitor_rewrites'])):
        with open(esperado[chave]) as f:
            base = yaml.safe_load(f)
        with open(arquivo) as f:
            assert yaml.safe_load(f) == perfil.aplica_reescritas(base, reescritas), \
                arquivo.name


def test_os_nos_do_robo3_recebem_os_yamls_materializados(monkeypatch, tmp_path):
    """Escrever o arquivo certo e entregar o arquivo velho ao nó é o mesmo
    defeito que a guarda de hoje impede: o costmap lendo o YAML sem a
    reescrita. Prova de existência não substitui prova de entrega."""
    ld, pasta, _ = _corrida_do_robo3(monkeypatch, tmp_path)
    nav2, cm = _materializados(pasta)
    ctx = _contexto(ld, robo='3', sim='true', log_dir=str(tmp_path))

    for nome in NOS_DO_NAV2:
        for no in _nos(ld, nome):
            recebido = _params(ctx, no)[0]
            assert recebido == str(nav2), f'{nome} recebeu {recebido}'
            assert os.path.isabs(recebido), recebido
    recebido = _params(ctx, _nos(ld, 'collision_monitor')[0])[0]
    assert recebido == str(cm), recebido
    assert os.path.isabs(recebido), recebido
    # E os dois na MESMA pasta de corrida — evidência espalhada não é evidência.
    assert os.path.dirname(str(nav2)) == os.path.dirname(str(cm)) == str(pasta)


def test_o_robo2_nao_materializa_nada(monkeypatch, tmp_path):
    """CONTROLE POSITIVO do §6: o caminho É o valor do parâmetro.

    Materializar para o robô 2, mesmo com bytes idênticos, mudaria o caminho
    que o nó recebe e tornaria impossível a comparação byte a byte com
    `1f49981` — a trava que protege o robô que funciona. Por isso a comparação
    é do caminho INTEIRO: outro `nav2.yaml`, em outra pasta, passaria por
    basename e seria exatamente o defeito.
    """
    _argv(monkeypatch, 'sim:=true', f'log_dir:={tmp_path}')
    ld = _descricao()
    _, erro = _percorre(ld, {'sim': 'true', 'log_dir': str(tmp_path)})
    assert erro is None, erro
    assert not list(tmp_path.rglob('*.yaml')), 'o robô 2 escreveu YAML'

    ctx = _contexto(ld, sim='true', log_dir=str(tmp_path))
    perfil2 = _perfil().parametros(2, SHARE_MOTION)
    for nome in NOS_DO_NAV2:
        for no in _nos(ld, nome):
            assert _params(ctx, no)[0] == perfil2['nav2'], nome
    assert _params(ctx, _nos(ld, 'collision_monitor')[0])[0] == \
        perfil2['collision_monitor']


def test_combinacao_recusada_nao_deixa_falsa_evidencia(monkeypatch, tmp_path):
    """§4.7: pasta de corrida com YAML dentro seria prova de corrida que não
    houve. Num projeto em que a pasta É a prova, isso é pior que não ter."""
    _argv(monkeypatch, 'robo:=3', 'sim:=false', f'log_dir:={tmp_path}')
    _, erro = _percorre(_descricao(), {'robo': '3', 'sim': 'false',
                                       'log_dir': str(tmp_path)})
    assert erro is not None
    assert not list(tmp_path.rglob('*')), f'sobrou {list(tmp_path.rglob("*"))}'


def test_a_escrita_fica_entre_a_recusa_e_a_primeira_acao_operacional():
    """A ordem é recusa < materialização < primeira ação operacional.

    O lado de baixo protege contra falsa evidência; o de CIMA protege contra
    nó tentando abrir um arquivo que ainda não foi escrito — e esse defeito
    apareceria como "arquivo não existe" num servidor do Nav2, longe da causa.
    """
    ents = _descricao().entities

    def _indice(nome):
        return [i for i, e in enumerate(ents) if isinstance(e, OpaqueFunction)
                and e._OpaqueFunction__function.__name__ == nome]

    recusa = _indice('_recusa_robo')
    escrita = _indice('_materializa_perfil')
    operacionais = [i for i, e in enumerate(ents) if isinstance(e, OPERACIONAIS)]
    assert len(recusa) == 1 and len(escrita) == 1, (recusa, escrita)
    assert recusa[0] < escrita[0] < min(operacionais), \
        (recusa[0], escrita[0], min(operacionais))


def test_a_escrita_e_atomica(monkeypatch, tmp_path):
    """D2: falha no meio da serialização não pode deixar destino parcial.

    Um YAML truncado é o pior dos mundos: o costmap ABRE o arquivo e lê uma
    configuração pela metade. O contrato é escrever ao lado e trocar por
    `os.replace`, então uma falha na serialização não deixa nenhum destino.

    ⚠️ A falha entra por `yaml.safe_dump`, e **só** por ele: o `yaml.dump` é
    usado pelo próprio `launch_ros` para normalizar parâmetros
    (`normalize_parameters.py:152`), e derrubá-lo mataria a descrição inteira
    antes de chegar à materialização. Fica, então, como contrato: quem escreve
    serializa com `safe_dump` — o par do `safe_load` que o `perfil` já usa.

    ⚠️ E a falha acontece **depois de já ter escrito um pedaço**, porque é
    isso que separa atômico de não atômico. Uma implementação

        texto = yaml.safe_dump(dados); open(destino, 'w').write(texto)

    também sobreviveria a uma exceção lançada ANTES de abrir o destino — ela
    passaria num teste ingênuo sem ser atômica. Aqui o `safe_dump` recebe o
    arquivo aberto, escreve um fragmento nele e só então explode.
    """
    escritos = []

    def explode(dados, fluxo=None, *_a, **_kw):
        if fluxo is not None:            # escrita em arquivo: suja antes de cair
            fluxo.write('# fragmento\n')
            fluxo.flush()
            escritos.append(getattr(fluxo, 'name', '?'))
        raise RuntimeError('falha proposital no meio da serialização')

    monkeypatch.setattr(yaml, 'safe_dump', explode)
    _argv(monkeypatch, 'robo:=3', 'sim:=true', f'log_dir:={tmp_path}')
    _, erro = _percorre(_descricao(), {'robo': '3', 'sim': 'true',
                                       'log_dir': str(tmp_path)})
    assert erro is not None, 'a falha da serialização foi engolida'
    # E é a falha INJETADA que tem de chegar aqui. Sem esta linha o teste
    # ficaria verde hoje pela recusa do robô 3 — sem nunca ter escrito nada,
    # que é exatamente o que ele deveria estar medindo.
    assert 'falha proposital' in str(erro), str(erro)
    # Nem destino, nem temporário: o fragmento tem de ter ido embora junto.
    sobrou = [p for p in tmp_path.rglob('*') if p.is_file()]
    assert not sobrou, f'sobrou depois da falha: {sobrou}'


def test_a_escrita_troca_por_os_replace(monkeypatch, tmp_path):
    """Atomicidade pelo lado de cima: na corrida que DÁ CERTO, cada um dos dois
    arquivos nasce ao lado e entra no lugar por uma troca.

    Sem este teste, o de cima aceitaria `open(destino, 'w')` direto — bastaria
    a exceção cair antes do `open`. O que prova atomicidade é a TROCA, e que o
    temporário esteja no MESMO diretório do destino (`os.replace` entre
    sistemas de arquivos diferentes não é atômico, e nem funciona)."""
    trocas = []
    real = os.replace

    def espia(origem, destino, *a, **kw):
        trocas.append((str(origem), str(destino)))
        return real(origem, destino, *a, **kw)

    monkeypatch.setattr(os, 'replace', espia)
    _, pasta, _ = _corrida_do_robo3(monkeypatch, tmp_path)
    nav2, cm = _materializados(pasta)

    destinos = {d for _, d in trocas}
    assert destinos == {str(nav2), str(cm)}, trocas
    for origem, destino in trocas:
        assert os.path.dirname(origem) == os.path.dirname(destino), \
            f'temporário fora do diretório do destino: {origem} → {destino}'


def test_os_dois_caminhos_absolutos_aparecem_no_log(monkeypatch, tmp_path):
    """D2: o caminho impresso na subida é o que liga a corrida à evidência.

    Sem ele, quem lê o console depois não sabe QUAL arquivo os costmaps
    leram — e a pasta de evidência vira um monte de YAML anônimo.
    """
    ld, pasta, devolvidas = _corrida_do_robo3(monkeypatch, tmp_path)
    nav2, cm = _materializados(pasta)
    ctx = _contexto(ld, robo='3', sim='true', log_dir=str(tmp_path))
    dito = ' '.join(perform_substitutions(ctx, e.msg)
                    if isinstance(e.msg, list) else str(e.msg)
                    for e in devolvidas if isinstance(e, LogInfo))
    for caminho in (str(nav2), str(cm)):
        assert caminho in dito, f'{caminho} não foi anunciado; log: {dito!r}'


def test_o_bag_vai_para_subdiretorio_da_pasta_da_corrida(monkeypatch, tmp_path):
    """D2: a pasta da corrida é a RAIZ de evidências, e `ros2 bag record -o`
    exige diretório inexistente — então o bag desce um nível, e desce DENTRO
    da mesma pasta dos YAMLs, não numa pasta irmã parecida."""
    ld, pasta, _ = _corrida_do_robo3(monkeypatch, tmp_path)
    ctx = _contexto(ld, robo='3', sim='true', log_dir=str(tmp_path))
    bags = [e for e in ld.entities if isinstance(e, ExecuteProcess)
            and 'bag' in [_txt(p) for p in e.cmd]]
    assert len(bags) == 1, f'esperava UM gravador, achei {len(bags)}'
    partes = [perform_substitutions(ctx, p) if isinstance(p, list) else _txt(p)
              for p in bags[0].cmd]
    destino = partes[partes.index('-o') + 1]
    assert destino == os.path.join(str(pasta), 'bag'), destino


# ─── §3 D3: o mux, e as quatro faixas ────────────────────────────────────────

def test_o_mux_do_robo3_tem_as_quatro_faixas(monkeypatch):
    """Inclusive o `unstuck_vel`: o perfil do robô 3 configura o desencalhe, e
    faixa que não existe não dá erro — dá silêncio."""
    _argv(monkeypatch, 'robo:=3', 'sim:=true')
    ld = _descricao()
    ctx = _contexto(ld, robo='3', sim='true')
    mux = _nos(ld, 'twist_mux')
    assert len(mux) == 1, 'um mux só'
    arquivo = _params(ctx, mux[0])[0]
    assert os.path.basename(arquivo) == 'twist_mux_pilha_robo3.yaml', arquivo
    with open(arquivo) as f:
        topicos = yaml.safe_load(f)['twist_mux']['ros__parameters']['topics']
    assert {v['topic']: v['priority'] for v in topicos.values()} == FAIXAS_ROBO3


def test_o_mux_do_robo2_nao_muda(monkeypatch):
    """CONTROLE POSITIVO: o arquivo do robô 2 é outro, e continua o mesmo."""
    _argv(monkeypatch, 'sim:=true')
    ld = _descricao()
    ctx = _contexto(ld, sim='true')
    arquivo = _params(ctx, _nos(ld, 'twist_mux')[0])[0]
    # Caminho INTEIRO: outro `twist_mux.yaml`, em outro pacote, passaria por
    # basename — e existem quatro arquivos com esse nome curto no repo.
    assert arquivo == os.path.join(SHARE_MOTION, 'config', 'twist_mux.yaml'), \
        arquivo


# ─── §0-C: o include do simulador ────────────────────────────────────────────

def test_o_include_do_sim_aponta_para_o_sim_robo3(monkeypatch):
    _argv(monkeypatch, 'robo:=3', 'sim:=true')
    fontes = [_fonte(e) for e in _descricao().entities
              if isinstance(e, IncludeLaunchDescription)]
    assert any(f.endswith('sim_robo3.launch.py') for f in fontes), fontes


def test_o_include_do_sim_so_passa_argumento_declarado_pelo_alvo(monkeypatch):
    """Achado C: a pilha passa `planta`, que o `sim_robo3` não declara — e
    include com argumento não declarado morre na subida. A lista de
    argumentos é LIDA do alvo, nunca escrita à mão aqui."""
    _argv(monkeypatch, 'robo:=3', 'sim:=true')
    alvos = [e for e in _descricao().entities
             if isinstance(e, IncludeLaunchDescription)
             and _fonte(e).endswith('sim_robo3.launch.py')]
    # Sem esta linha o teste ficaria VACUOSO: hoje não há include do
    # `sim_robo3`, e um `for` sobre lista vazia passa dizendo nada.
    assert len(alvos) == 1, 'não há include do sim_robo3 para conferir'
    declarados = {a.name for a in get_launch_description_from_python_launch_file(
        _fonte(alvos[0])).get_launch_arguments()}
    passados = {_txt(k) for k, _ in alvos[0].launch_arguments}
    # 🔴 CONJUNTO EXATO, não `passados <= declarados`: aquilo aceitaria lista
    # VAZIA, e um include que não passa pose nem mundo nasce o robô no lugar
    # errado sem reclamar de nada.
    assert passados == ARGS_DO_SIM_ROBO3, sorted(passados ^ ARGS_DO_SIM_ROBO3)
    # E o conjunto esperado tem de ser mesmo o que o alvo declara — senão o
    # teste estaria conferindo a lista contra ela mesma.
    assert ARGS_DO_SIM_ROBO3 <= declarados, sorted(ARGS_DO_SIM_ROBO3 - declarados)
