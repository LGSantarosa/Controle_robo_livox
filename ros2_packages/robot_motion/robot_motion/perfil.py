"""O que cada nó da pilha recebe, por robô — etapa 4 (PLANO_ETAPA4_ROBO3.md §2).

Função pura, sem ROS: recebe o número do robô e o `share/` do `robot_motion` e
devolve caminhos e dicionários. A `pilha.launch.py` monta o robô 2 por aqui, e
é isso que faz desta função o consumidor real do perfil, e não código paralelo.

    nav2               o YAML dos servidores do Nav2 e dos costmaps
    nav2_rewrites      chaves a reescrever nesse YAML na subida ({} = nenhuma)
    collision_monitor  o YAML do reflexo
    path_follower      sobreposição dos defaults do nó ({} = nenhuma)

Reescrita é por CAMINHO COMPLETO (tupla de chaves), aplicada por
`aplica_reescritas` — os costmaps são nós dentro dos servidores do Nav2, e
dicionário passado ao `Node` do servidor não chega neles (plano §3).

O robô 2 NÃO tem arquivo de sobreposição: o perfil dele é a base de hoje, sem
camada nova para quebrar. O do robô 3 entra no passo 4.

⚠️ Perfil montável não é robô liberado: quem decide se a pilha SOBE para um
robô é a própria pilha (`_recusa_robo`), não esta função.
"""
import copy
import os


class RoboSemPerfil(ValueError):
    """Pediram o perfil de um robô que não tem (ou ainda não tem) perfil."""


def parametros(robo: int, share: str) -> dict:
    # `type is int` e não `== 2`: `2.0` e `True` comparam igual a inteiro, e
    # perfil de robô não se escolhe por conversão.
    if type(robo) is int and robo == 2:
        return {
            'nav2': os.path.join(share, 'config', 'nav2.yaml'),
            'nav2_rewrites': {},
            'collision_monitor': os.path.join(share, 'config',
                                              'collision_monitor.yaml'),
            'path_follower': {},
        }
    if type(robo) is int and robo == 3:
        raise RoboSemPerfil('o perfil do robô 3 entra no passo 4 da etapa 4')
    raise RoboSemPerfil(f'robô {robo!r} não tem perfil (só o robô 2, por enquanto)')


def aplica_reescritas(dados: dict, reescritas: dict) -> dict:
    """Cópia de `dados` com cada FOLHA existente em `reescritas` trocada.

    Caminho = tupla não vazia de textos, do topo do YAML até a folha. Reprova
    (ValueError, sem efeito parcial — o original nunca muda) caminho que não
    existe, que para num ramo (trocaria um dicionário inteiro) ou que atravessa
    uma folha. Criar chave em silêncio seria o nó lendo parâmetro que ninguém
    declarou, e o valor pedido sumindo sem aviso.
    """
    novo = copy.deepcopy(dados)
    for caminho, valor in reescritas.items():
        if (not isinstance(caminho, tuple) or not caminho
                or not all(isinstance(k, str) for k in caminho)):
            raise ValueError(f'caminho de reescrita tem de ser tupla não vazia '
                             f'de textos: {caminho!r}')
        no = novo
        for i, chave in enumerate(caminho[:-1]):
            if not isinstance(no, dict) or chave not in no:
                raise ValueError(f'reescrita {caminho!r}: {chave!r} não existe '
                                 f'em {caminho[:i]!r}')
            no = no[chave]
        folha = caminho[-1]
        if not isinstance(no, dict):
            raise ValueError(f'reescrita {caminho!r}: {caminho[-2]!r} é folha, '
                             'não dá para descer nela')
        if folha not in no:
            raise ValueError(f'reescrita {caminho!r}: {folha!r} não existe')
        if isinstance(no[folha], dict):
            raise ValueError(f'reescrita {caminho!r}: {folha!r} é ramo, não folha')
        no[folha] = valor
    return novo
