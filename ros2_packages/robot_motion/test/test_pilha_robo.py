"""A pilha escolhe o robô por `robo:=` e monta o robô 2 pelo perfil — passo 3.

Dois contratos, testados com a launch de verdade num `LaunchContext` real:

1. **Recusa antes de tudo.** `robo:=2` (o texto exato, e o default) segue;
   `robo:=3` morre dizendo que o perfil entra no passo 4 mas a pilha do robô 3
   só na etapa 6 (D1); qualquer outro texto morre dizendo que o robô não
   existe. Sem aparar espaço nem converter número: `" 2"`, `"02"` e `"2.0"` não
   são o robô 2. E a recusa vem antes da validação que já existia e antes de
   QUALQUER ação que sobe processo — uma pilha meio de pé com o robô errado é
   pior do que nenhuma.
2. **O perfil é consumido de verdade.** Os caminhos que os nós recebem saem de
   `perfil.parametros(2, pkg)` — trocado o perfil, troca o que o nó recebe. E
   com a sobreposição do `path_follower` vazia, a lista de parâmetros dele fica
   literalmente a de antes.

Percorrer a descrição não resolve condição nem sobe nada (§7.2 do plano): é o
aviso cedo; a prova é o grafo vivo no Gazebo.
"""
import importlib
import os

import pytest
import yaml

pytest.importorskip('launch_ros')

from launch import LaunchContext  # noqa: E402
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,  # noqa: E402
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import (  # noqa: E402
    get_launch_description_from_python_launch_file)
from launch_ros.actions import Node  # noqa: E402
from launch_ros.utilities import evaluate_parameters  # noqa: E402

PILHA = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'launch',
                                     'pilha.launch.py'))


def _perfil():
    return importlib.import_module('robot_motion.perfil')


OPERACIONAIS = (Node, IncludeLaunchDescription, ExecuteProcess)


def _descricao():
    return get_launch_description_from_python_launch_file(PILHA)


def _percorre(ld, argumentos):
    """Visita, em ordem e num contexto real, argumentos e validações.

    Ação operacional não é executada — só anotada como ALCANÇADA. Devolve
    (alcançadas, exceção ou None).
    """
    ctx = LaunchContext()
    ctx.launch_configurations.update(argumentos)
    alcancadas = []
    for e in ld.entities:
        if isinstance(e, OPERACIONAIS):
            alcancadas.append(e)
        elif isinstance(e, (DeclareLaunchArgument, OpaqueFunction)):
            try:
                e.visit(ctx)
            except Exception as erro:  # noqa: BLE001 - o teste examina qual
                return alcancadas, erro
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


# ─── o argumento e a recusa ──────────────────────────────────────────────────

def test_a_pilha_declara_robo_com_default_2():
    args = {a.name: a for a in _descricao().get_launch_arguments()}
    assert 'robo' in args
    assert [s.text for s in args['robo'].default_value] == ['2']


@pytest.mark.parametrize('argumentos', [{}, {'robo': '2'}],
                         ids=['default', 'robo:=2'])
def test_robo2_passa_e_a_pilha_sobe(argumentos):
    alcancadas, erro = _percorre(_descricao(), argumentos)
    assert erro is None, erro
    assert alcancadas


def test_robo3_recusa_ate_a_etapa6():
    alcancadas, erro = _percorre(_descricao(), {'robo': '3'})
    assert isinstance(erro, RuntimeError), erro
    assert 'passo 4' in str(erro) and 'etapa 6' in str(erro), str(erro)
    assert alcancadas == []


@pytest.mark.parametrize('robo', ['', ' 2', '2 ', '02', '2.0', '1', '4', 'robo2'])
def test_robo_que_nao_existe_recusa(robo):
    alcancadas, erro = _percorre(_descricao(), {'robo': robo})
    assert isinstance(erro, RuntimeError), erro
    assert 'não existe' in str(erro), str(erro)
    assert repr(robo) in str(erro), 'a mensagem tem de mostrar o texto recebido'
    assert alcancadas == []


def test_a_recusa_do_robo_vem_antes_da_validacao_que_ja_existia():
    """Com o robô E a localização inválidos, quem fala é o robô."""
    _, erro = _percorre(_descricao(), {'robo': '3', 'localizacao': 'xyz'})
    assert 'etapa 6' in str(erro), str(erro)


def test_a_recusa_do_robo_vem_antes_de_qualquer_acao_operacional():
    ents = _descricao().entities
    recusas = [i for i, e in enumerate(ents) if isinstance(e, OpaqueFunction)
               and e._OpaqueFunction__function.__name__ == '_recusa_robo']
    operacionais = [i for i, e in enumerate(ents) if isinstance(e, OPERACIONAIS)]
    assert len(recusas) == 1
    assert recusas[0] < min(operacionais)


# ─── o perfil consumido ──────────────────────────────────────────────────────

NOS_DO_NAV2 = ('map_server', 'planner_server', 'smoother_server',
               'controller_server', 'bt_navigator', 'amcl')


def test_a_pilha_monta_pelo_perfil(monkeypatch, tmp_path):
    """Troca o perfil e confere que o que os nós recebem trocou junto."""
    nav2 = tmp_path / 'nav2_do_perfil.yaml'
    cm = tmp_path / 'cm_do_perfil.yaml'
    for f in (nav2, cm):
        f.write_text('{}\n')
    chamadas = []

    def falso(robo, share):
        chamadas.append((robo, share))
        return {'nav2': str(nav2), 'nav2_rewrites': {},
                'collision_monitor': str(cm), 'collision_monitor_rewrites': {},
                'path_follower': {'passagem_margem': 0.99}}

    monkeypatch.setattr(_perfil(), 'parametros', falso)
    ld = _descricao()
    assert chamadas and all(r == 2 for r, _ in chamadas)
    ctx = _contexto(ld, sim='true')
    for nome in NOS_DO_NAV2:
        for no in _nos(ld, nome):
            assert _params(ctx, no)[0] == str(nav2), nome
    assert _params(ctx, _nos(ld, 'collision_monitor')[0])[0] == str(cm)
    assert _params(ctx, _nos(ld, 'path_follower')[0])[-1] == {'passagem_margem': 0.99}


FOLHAS = {
    'nav2_rewrites': (('bt_navigator', 'ros__parameters',
                       'bt_loop_duration'), 999),
    'collision_monitor_rewrites': (('collision_monitor', 'ros__parameters',
                                    'stop_pub_timeout'), 9.99),
}


@pytest.mark.parametrize('chave', list(FOLHAS))
def test_reescrita_conhecida_passa_a_ser_aplicada(monkeypatch, tmp_path, chave):
    """🔄 ETAPA 6, PASSO 3: aqui a pilha ABORTAVA, porque não sabia reescrever.

    Agora sabe (`perfil.materializa`), e o teste vai até o fim: sobe a
    descrição, RODA o `_materializa_perfil` e abre o arquivo que nasceu. Só
    `_descricao()` não provaria nada — a reescrita nem chega a ser avaliada
    ali, e uma reescrita inválida passaria batida.
    """
    caminho, valor = FOLHAS[chave]
    alvo = chave[:-len('_rewrites')]
    real = _perfil().parametros

    def com_reescrita(robo, share):
        return {**real(robo, share), chave: {caminho: valor}}

    monkeypatch.setattr(_perfil(), 'parametros', com_reescrita)
    ld = _descricao()

    ctx = LaunchContext()
    ctx.launch_configurations.update({'log_dir': str(tmp_path)})
    escreveu = [e for e in ld.entities if isinstance(e, OpaqueFunction)
                and e._OpaqueFunction__function.__name__ == '_materializa_perfil']
    assert len(escreveu) == 1
    escreveu[0].visit(ctx)

    nascidos = [p for p in tmp_path.rglob('*.yaml')]
    assert len(nascidos) == 1, nascidos
    with open(nascidos[0]) as f:
        dados = yaml.safe_load(f)
    no = dados
    for passo in caminho:
        no = no[passo]
    assert no == valor, f'{alvo}: a reescrita não chegou ao arquivo'
    # E o nó recebe ESSE arquivo, não o do `share/`.
    ctx_nos = _contexto(ld, sim='true', log_dir=str(tmp_path))
    consumidor = 'collision_monitor' if alvo == 'collision_monitor' else 'bt_navigator'
    assert _params(ctx_nos, _nos(ld, consumidor)[0])[0] == str(nascidos[0])


def test_reescrita_que_a_materializacao_nao_conhece_mata_a_subida(monkeypatch):
    """O que sobrou da guarda, e é o que ela de fato protegia: chave de
    reescrita que ninguém aplica sumiria em silêncio, e o nó leria o arquivo
    sem ela — costmap com o footprint do robô errado, sem uma linha de aviso."""
    real = _perfil().parametros

    def com_chave_estranha(robo, share):
        return {**real(robo, share), 'inventado_rewrites': {('a',): 1}}

    monkeypatch.setattr(_perfil(), 'parametros', com_chave_estranha)
    with pytest.raises(RuntimeError, match='inventado_rewrites'):
        _descricao()


def test_path_follower_sem_sobreposicao_fica_como_era():
    ld = _descricao()
    ctx = _contexto(ld, sim='true')
    params = _params(ctx, _nos(ld, 'path_follower')[0])
    assert [list(p) for p in params] == [
        ['lookahead_piso'], ['k_lat'], ['log_dir'], ['desvio_taxa_deg_s'],
        ['re_habilitada'], ['re_max_seguidas'], ['re_exige_objetivo'],
        ['passagem_estreita_habilitada'], ['use_sim_time']]
