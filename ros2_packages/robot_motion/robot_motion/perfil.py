"""O que cada nó da pilha recebe, por robô — etapa 4 (PLANO_ETAPA4_ROBO3.md §2).

Função pura, sem ROS: recebe o número do robô e o `share/` do `robot_motion` e
devolve caminhos e dicionários. A `pilha.launch.py` monta o robô 2 por aqui, e
é isso que faz desta função o consumidor real do perfil, e não código paralelo.

    nav2               o YAML dos servidores do Nav2 e dos costmaps
    nav2_rewrites      chaves a reescrever nesse YAML na subida ({} = nenhuma)
    collision_monitor  o YAML do reflexo
    path_follower      sobreposição dos defaults do nó ({} = nenhuma)

O robô 2 NÃO tem arquivo de sobreposição: o perfil dele é a base de hoje, sem
camada nova para quebrar. O do robô 3 entra no passo 4.

⚠️ Perfil montável não é robô liberado: quem decide se a pilha SOBE para um
robô é a própria pilha (`_recusa_robo`), não esta função.
"""
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
