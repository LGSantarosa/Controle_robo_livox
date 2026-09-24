#!/usr/bin/env python3
"""Julga a corrida da etapa 6 a partir da captura — sem ROS, offline.

    confere.py <captura> <pasta_da_corrida> <esperados.yaml> <saida.yaml>

A captura (`tools/linha_de_base/captura.py`) já espera o grafo ficar PRONTO e
ESTÁVEL e grava os parâmetros VIVOS de cada nó. Este programa não consulta nada:
ele lê aquele dump e decide. É de propósito — assim o veredito é reproduzível a
partir da pasta de evidências, meses depois, sem levantar Gazebo nenhum.

Quatro itens do §4 do plano, e cada um tem uma armadilha própria:

  1. §4.1 o mux vivo: as QUATRO faixas e `use_stamped`. Lido do parâmetro do nó,
     não do arquivo — arquivo certo com nó lendo outro é o defeito que se quer
     pegar.
  2. §4.3 `use_sim_time` verdadeiro em TODOS os nós da lista versionada, e nó no
     dump fora da lista REPROVA. É esta segunda metade que impede a prova de
     envelhecer em silêncio a cada nó novo.
  3. §4.4 `footprint` e `footprint_padding` vivos nos dois costmaps, comparados
     com o que `perfil.parametros(3, …)` devolve — 🔴 NUNCA redigitados aqui. Um
     número redigitado prova que dois lugares concordam com o teste, não que o
     costmap recebeu o que o perfil mandou.
  4. §4.7 + D2 os dois YAMLs materializados na pasta da corrida, e cada um igual
     a `aplica_reescritas(base, reescritas)` — relido do disco.

⚠️ `use_sim_time` vem do `parametros_brutos.yaml`, e não do normalizado: o
normalizador EXCLUI os automáticos (`normaliza.py:EXCLUIDOS_EXATOS`), e
`use_sim_time` é um deles. Ler o normalizado daria "ausente" em todo nó e a
prova passaria por vacuidade.
"""
import argparse
import importlib.util
import os
import sys

import yaml

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
COSTMAPS = ('/global_costmap/global_costmap', '/local_costmap/local_costmap')
# As quatro faixas do robô 3 (D3). Aqui é legítimo estar escrito: é o CONTRATO
# da decisão, não um valor derivado de outro lugar do código.
FAIXAS = {'dpad_vel': 110, 'joy_vel': 100, 'unstuck_vel': 30, 'auto_vel': 10}


def _modulo(nome, caminho):
    spec = importlib.util.spec_from_file_location(nome, caminho)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def perfil_do_robo3(share_motion, share_base):
    """O perfil, montado pela MESMA função que a pilha usa.

    Importado por caminho para este programa não depender de o `robot_motion`
    estar no `PYTHONPATH` — ele roda depois da corrida, quando o ambiente já
    pode ter sido desmontado.
    """
    p = _modulo('perfil_etapa6', os.path.join(
        RAIZ, 'ros2_packages/robot_motion/robot_motion/perfil.py'))
    return p, p.parametros(3, share_motion, share_base=share_base)


def plano(d, prefixo=''):
    """Achata `{'a': {'b': 1}}` em `{'a.b': 1}`.

    O dump bruto pode vir aninhado (como o `ros2 param dump`) ou já com pontos,
    dependendo do nó. Achatar os dois jeitos evita um "ausente" que é só forma.
    """
    saida = {}
    for chave, valor in (d or {}).items():
        nome = f'{prefixo}{chave}'
        if isinstance(valor, dict):
            saida.update(plano(valor, f'{nome}.'))
        else:
            saida[nome] = valor
    return saida


def params_por_no(brutos):
    return {no: plano((v or {}).get('ros__parameters'))
            for no, v in (brutos or {}).items()}


# ─── os quatro itens ─────────────────────────────────────────────────────────

def item_mux(params):
    """§4.1 — as quatro faixas e o `use_stamped`, VIVOS no nó."""
    p = params.get('/twist_mux')
    if not p:
        return False, {'erro': '/twist_mux sem parâmetros no dump'}
    faixas, priori = {}, {}
    for chave, valor in p.items():
        partes = chave.split('.')
        if len(partes) == 3 and partes[0] == 'topics' and partes[2] == 'topic':
            faixas[partes[1]] = valor
        if len(partes) == 3 and partes[0] == 'topics' and partes[2] == 'priority':
            priori[partes[1]] = valor
    vivas = {faixas[k]: priori.get(k) for k in faixas}
    # `is not True` e não `!= True`: em Python `1 == True`, e um `use_stamped`
    # que chegou como inteiro 1 não é o booleano que o mux declara.
    stamped = p.get('use_stamped')
    ok = vivas == FAIXAS and stamped is True
    return ok, {'faixas_vivas': vivas, 'faixas_esperadas': FAIXAS,
                'use_stamped': stamped,
                'sobrando': sorted(set(vivas) - set(FAIXAS)),
                'faltando': sorted(set(FAIXAS) - set(vivas))}


def item_use_sim_time(params, esperados):
    """§4.3 — todos true, e a lista versionada tem de cobrir o grafo.

    Duas metades, e a segunda é a que importa a longo prazo: nó no dump que não
    está na lista REPROVA. Sem ela, um nó novo entraria sem ninguém conferir se
    ele também está no tempo do simulador.
    """
    esperados = set(esperados)
    fora = sorted(set(params) - esperados)
    ausente, falso = [], {}
    for no in sorted(set(params) & esperados):
        if 'use_sim_time' not in params[no]:
            ausente.append(no)
        elif params[no]['use_sim_time'] is not True:
            falso[no] = params[no]['use_sim_time']
    return (not fora and not ausente and not falso), {
        'nos_conferidos': len(set(params) & esperados),
        'fora_da_lista_versionada': fora,
        'sem_use_sim_time': ausente,
        'use_sim_time_nao_true': falso}


def item_footprint(params, perfil_robo3):
    """§4.4 — o que o costmap tem VIVO contra o que o perfil manda.

    🔴 Os valores esperados saem de `perfil.parametros(3, …)`, nunca redigitados.
    O contraste é o controle positivo: o `nav2.yaml` do robô 2 traz
    `[[0.35, 0.2775], …]`, e ler ISSO com `robo:=3` significa que a reescrita
    não pegou.
    """
    esperado = {}
    for caminho, valor in perfil_robo3['nav2_rewrites'].items():
        # caminho = (costmap, costmap, 'ros__parameters', folha)
        esperado[(f'/{caminho[0]}/{caminho[1]}', caminho[-1])] = valor
    diferencas, conferidos = {}, 0
    for (no, folha), valor in sorted(esperado.items()):
        vivo = params.get(no, {}).get(folha, 'AUSENTE')
        conferidos += 1
        if not _igual(vivo, valor):
            diferencas[f'{no}:{folha}'] = {'vivo': vivo, 'perfil': valor}
    faltando = [c for c in COSTMAPS if c not in params]
    return (not diferencas and not faltando and conferidos == 4), {
        'conferidos': conferidos, 'costmaps_sem_dump': faltando,
        'diferencas': diferencas}


def _igual(vivo, esperado):
    """Número compara como número; texto como texto — e bool nunca como número.

    O `footprint` viaja como TEXTO (o Nav2 exige), e o `footprint_padding` como
    double: o mesmo item tem os dois tipos, e comparar tudo como texto deixaria
    `0.01` passar por `'0.01'`.
    """
    if isinstance(vivo, bool) or isinstance(esperado, bool):
        return vivo is esperado
    if isinstance(vivo, (int, float)) and isinstance(esperado, (int, float)):
        return abs(float(vivo) - float(esperado)) <= 1e-9
    return vivo == esperado


def item_materializados(perfil_mod, perfil_robo3, pasta_corrida):
    """D2 + §4.7 — os dois YAMLs na pasta da corrida, e o conteúdo certo."""
    detalhe = {'pasta': pasta_corrida, 'arquivos': {}}
    ok = True
    destinos = perfil_mod.destinos(perfil_robo3, pasta_corrida)
    for chave, destino in sorted(destinos.items()):
        registro = {'caminho': destino}
        if destino == perfil_robo3[chave]:
            registro['erro'] = 'o perfil do robô 3 deveria materializar este'
            ok = False
        elif not os.path.isfile(destino):
            registro['erro'] = 'não existe'
            ok = False
        else:
            with open(perfil_robo3[chave]) as f:
                base = yaml.safe_load(f)
            with open(destino) as f:
                lido = yaml.safe_load(f)
            esperado = perfil_mod.aplica_reescritas(
                base, perfil_robo3[f'{chave}_rewrites'])
            registro['igual_a_aplica_reescritas'] = lido == esperado
            if lido != esperado:
                ok = False
        detalhe['arquivos'][chave] = registro
    # O bag desce para `<pasta>/bag`, porque `ros2 bag record -o` exige
    # diretório inexistente e abortaria contra a raiz de evidências.
    bag = os.path.join(pasta_corrida, 'bag')
    detalhe['bag'] = {'caminho': bag, 'existe': os.path.isdir(bag)}
    if not os.path.isdir(bag):
        ok = False
    return ok, detalhe


def item_fronteira_de_hardware(nos):
    """§4.2, pela negativa: a fronteira do atuador real NÃO está no grafo.

    No Gazebo a cadeia termina no `hoverboard_base_controller`, via
    `placa_simulada`. `cmd_vel_to_wheels` ou `mega_bridge` aqui significaria
    comando saindo por um caminho que ninguém está medindo — e foi isso que
    derrubou a "etapa 3" da v1.
    """
    proibidos = ('cmd_vel_to_wheels', 'mega_bridge')
    achados = sorted(n for n in nos
                     if n.rsplit('/', 1)[-1] in proibidos)
    exigidos = ['/placa_simulada', '/hoverboard_base_controller']
    faltando = [n for n in exigidos if n not in nos]
    return (not achados and not faltando), {
        'proibidos_no_grafo': achados, 'exigidos_faltando': faltando}


# ─── juntando ────────────────────────────────────────────────────────────────

def julga(captura, pasta_corrida, esperados_yaml, share_motion, share_base):
    with open(os.path.join(captura, 'parametros_brutos.yaml')) as f:
        brutos = yaml.safe_load(f) or {}
    with open(esperados_yaml) as f:
        esperados = yaml.safe_load(f) or {}
    with open(os.path.join(captura, 'resumo.yaml')) as f:
        resumo = yaml.safe_load(f) or {}
    params = params_por_no(brutos)
    perfil_mod, p3 = perfil_do_robo3(share_motion, share_base)
    nos = set(params) | set(resumo.get('grafo', {}).get('visiveis') or [])

    itens = {}
    itens['4.1 mux vivo: as quatro faixas e use_stamped'] = item_mux(params)
    itens['4.2 fronteira de hardware fora do grafo'] = \
        item_fronteira_de_hardware(nos)
    itens['4.3 use_sim_time true na lista versionada'] = \
        item_use_sim_time(params, esperados.get('nos') or [])
    itens['4.4 footprint/padding vivos = perfil.parametros(3)'] = \
        item_footprint(params, p3)
    itens['D2 os dois YAMLs materializados + bag em subpasta'] = \
        item_materializados(perfil_mod, p3, pasta_corrida)
    return itens


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('captura')
    ap.add_argument('pasta_corrida')
    ap.add_argument('esperados')
    ap.add_argument('saida')
    ap.add_argument('--share-motion', required=True)
    ap.add_argument('--share-base', required=True)
    a = ap.parse_args(argv)

    itens = julga(a.captura, a.pasta_corrida, a.esperados,
                  a.share_motion, a.share_base)
    veredito = 'APROVADO' if all(ok for ok, _ in itens.values()) else 'REPROVADO'
    saida = {'veredito': veredito,
             'itens': {nome: {'veredito': 'APROVADO' if ok else 'REPROVADO',
                              **detalhe}
                       for nome, (ok, detalhe) in itens.items()}}
    with open(a.saida, 'w') as f:
        yaml.safe_dump(saida, f, sort_keys=True, allow_unicode=True, width=1000)
    for nome, (ok, _) in sorted(itens.items()):
        print(f'{"APROVADO" if ok else "REPROVADO"}  {nome}')
    print(f'veredito: {veredito}  ({a.saida})')
    return 0 if veredito == 'APROVADO' else 1


if __name__ == '__main__':
    sys.exit(main())
