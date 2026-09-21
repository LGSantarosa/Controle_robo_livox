#!/usr/bin/env python3
"""Captura a linha de base de parâmetros com o grafo PRONTO (etapa 4, §10.2).

    python3 tools/linha_de_base/captura.py <esperados.yaml> <pasta_saida>
            [--prazo 180] [--intervalo 2] [--estavel 3]

`esperados.yaml` separa os nós com parâmetros dos auxiliares que existem apenas
no grafo. Estes últimos podem ser nomes exatos ou famílias de nome variável,
sempre com expressão ancorada, cardinalidade e alias estável.

Ordem, e por quê:

  1. Espera o grafo ficar PRONTO: todos os esperados presentes, nenhum a mais,
     **nenhum nome duplicado**, lifecycle em `active` e nós comuns respondendo
     ao `list_parameters`. E ESTÁVEL: o mesmo instantâneo (nós + estados) em
     `--estavel` consultas seguidas — um spawner transitório, que aparece e
     some, não entra na captura.
  2. Duplicado reprova ANTES de qualquer consulta de parâmetro: com dois nós
     do mesmo nome, o serviço responde por um deles sem dizer qual.
  3. Lê os parâmetros pelos mesmos serviços que o `ros2 param dump` usa
     (`list_parameters` recursivo + `get_parameters`), direto no rclpy: os
     tipos chegam intactos e não dependemos do CLI.

Se o prazo vence sem ficar pronto, AINDA ASSIM grava os parâmetros de quem está
lá (sem duplicado): uma sessão de Gazebo custa caro, o dado fica — com o
veredito REPROVADO e o motivo.

O nó desta ferramenta é oculto (`_linha_de_base`), e nós ocultos (nome que
começa com `_`) ficam fora da comparação do grafo, como no `ros2 node list`.

A leitura NÃO confia no lote do `get_parameters` (tudo-ou-nada no rclcpp): se
a contagem difere da pedida, lê nome a nome. Parâmetro listado e ilegível
(sem valor, ou PARAMETER_NOT_SET) e dump vazio de nó da lista REPROVAM.

Saída na pasta: `resumo.yaml` (veredito, motivos, footprint_padding dos dois
costmaps, ilegíveis e dumps vazios), `consultas.csv` (uma linha por consulta ao
grafo), `estado_nos.csv`, `grafo.txt`, `parametros_brutos.yaml`,
`parametros_normalizados.yaml` e `ilegiveis.yaml` (sempre gravado; `{}` é a
afirmação de que não houve nenhum). Sai com código 1 se reprovar.
"""
import argparse
import csv
import importlib.util
import os
import re
import sys
import time

import yaml

AQUI = os.path.dirname(os.path.abspath(__file__))
NOME_PROPRIO = '_linha_de_base'
COSTMAPS = ('/global_costmap/global_costmap', '/local_costmap/local_costmap')
PRONTOS = ('active', 'responde', 'presente_sem_parametros')
TIMEOUT_SERVICO = 3.0


def _carrega_normaliza():
    spec = importlib.util.spec_from_file_location(
        'linha_de_base_normaliza', os.path.join(AQUI, 'normaliza.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


nz = _carrega_normaliza()


# ─── lógica pura (testada sem ROS) ───────────────────────────────────────────

def nome_completo(ns, nome):
    return f'/{nome}' if ns in ('', '/') else f'{ns.rstrip("/")}/{nome}'


def oculto(nome):
    return nome.rsplit('/', 1)[-1].startswith('_')


def _valida_nomes(nomes, rotulo):
    ruins = [n for n in nomes if not isinstance(n, str) or not n.startswith('/')]
    if ruins:
        raise ValueError(f'{rotulo}: nome não completo (tem de começar com /): {ruins}')
    repetidos = sorted({n for n in nomes if nomes.count(n) > 1})
    if repetidos:
        raise ValueError(f'{rotulo}: nome repetido: {repetidos}')


def le_configuracao(caminho):
    with open(caminho) as f:
        bruto = yaml.safe_load(f) or {}
    nos = bruto.get('nos') or []
    gs = bruto.get('grafo_somente') or {}
    exatos = gs.get('exatos') or []
    volateis = gs.get('volateis') or []
    _valida_nomes(nos, 'nos')
    _valida_nomes(exatos, 'grafo_somente.exatos')
    conflito = sorted(set(nos) & set(exatos))
    if conflito:
        raise ValueError(f'nó está em nos e grafo_somente.exatos: {conflito}')
    aliases = []
    for i, regra in enumerate(volateis):
        if not isinstance(regra, dict):
            raise ValueError(f'grafo_somente.volateis[{i}] não é dicionário')
        padrao = regra.get('padrao')
        alias = regra.get('alias')
        quantidade = regra.get('quantidade')
        if not isinstance(padrao, str) or not padrao.startswith('^') or not padrao.endswith('$'):
            raise ValueError(f'padrão volátil tem de ser expressão ancorada: {padrao!r}')
        try:
            rx = re.compile(padrao)
        except re.error as e:
            raise ValueError(f'padrão volátil inválido {padrao!r}: {e}') from e
        conflito = sorted(n for n in nos + exatos if rx.fullmatch(n))
        if conflito:
            raise ValueError(f'padrão volátil também casa com nó fixo: {conflito}')
        if not isinstance(quantidade, int) or isinstance(quantidade, bool) or quantidade < 1:
            raise ValueError(f'quantidade volátil inválida: {quantidade!r}')
        _valida_nomes([alias], f'grafo_somente.volateis[{i}].alias')
        aliases.append(alias)
    if len(set(aliases)) != len(aliases):
        raise ValueError('alias volátil repetido')
    return {'nos': sorted(nos),
            'grafo_somente': {'exatos': sorted(exatos), 'volateis': volateis},
            'ilegiveis_permitidos': _le_ilegiveis_permitidos(
                bruto.get('ilegiveis_permitidos') or [], nos)}


CHAVES_ILEGIVEL = ('no', 'parametro', 'origem')


def _le_ilegiveis_permitidos(entradas, nos):
    """Exceções EXATAS nó + parâmetro, cada uma com a evidência que a justifica.

    Nada de padrão ou prefixo. O nó tem de estar em `nos` (nó só de grafo não
    tem dump a ler) e cada par aparece uma vez.
    """
    vistos = set()
    for i, e in enumerate(entradas):
        if isinstance(e, dict) and any(not isinstance(k, str) for k in e):
            # YAML 1.1: `no:` sem aspas vira o booleano False.
            raise ValueError(f"ilegiveis_permitidos[{i}]: chave não-texto {e!r} — "
                             "escreva 'no': com aspas")
        if not isinstance(e, dict) or set(e) != set(CHAVES_ILEGIVEL):
            raise ValueError(f'ilegiveis_permitidos[{i}] tem de ter exatamente '
                             f'{CHAVES_ILEGIVEL}: {e!r}')
        for chave in CHAVES_ILEGIVEL:
            if not isinstance(e[chave], str) or not e[chave].strip():
                raise ValueError(f'ilegiveis_permitidos[{i}].{chave} vazio ou não é texto')
        if e['no'] not in nos:
            raise ValueError(f"ilegiveis_permitidos[{i}]: {e['no']} não está em `nos`")
        par = (e['no'], e['parametro'])
        if par in vistos:
            raise ValueError(f'ilegiveis_permitidos repetido: {par}')
        vistos.add(par)
    return list(entradas)


def le_esperados(caminho):
    """Compatibilidade: devolve somente os nós cujo dump é obrigatório."""
    return le_configuracao(caminho)['nos']


def _configuracao(esperados):
    if isinstance(esperados, dict):
        return {'ilegiveis_permitidos': [], **esperados}
    return {'nos': sorted(esperados),
            'grafo_somente': {'exatos': [], 'volateis': []},
            'ilegiveis_permitidos': []}


def avalia_grafo(vistos, esperados, grafo_somente=None):
    """`vistos` pode ter repetição (é o que o grafo devolve). Ocultos saem."""
    gs = grafo_somente or {'exatos': [], 'volateis': []}
    visiveis = [v for v in vistos if not oculto(v)]
    unicos = set(visiveis)
    exatos = set(gs.get('exatos') or [])
    casados = set()
    aliases = {}
    volateis_invalidos = []
    for regra in gs.get('volateis') or []:
        rx = re.compile(regra['padrao'])
        achados = sorted(n for n in unicos if rx.fullmatch(n))
        if len(achados) != regra['quantidade']:
            volateis_invalidos.append({
                'alias': regra['alias'], 'esperados': regra['quantidade'],
                'encontrados': len(achados), 'nomes': achados})
        for nome in achados:
            if nome in casados:
                volateis_invalidos.append({
                    'alias': regra['alias'], 'erro': f'{nome} casa com mais de uma regra'})
            casados.add(nome)
            aliases[nome] = regra['alias']
    fixos = set(esperados) | exatos
    somente_grafo = exatos | casados
    return {
        'visiveis': sorted(unicos),
        'duplicados': sorted({v for v in visiveis if visiveis.count(v) > 1}),
        'faltando': sorted(fixos - unicos),
        'sobrando': sorted(unicos - fixos - casados),
        'somente_grafo': sorted(somente_grafo),
        'aliases': aliases,
        'volateis_invalidos': volateis_invalidos,
    }


def pronto(grafo, estados):
    return (not grafo['duplicados'] and not grafo['faltando']
            and not grafo['sobrando']
            and not grafo.get('volateis_invalidos')
            and all(estados.get(n) in PRONTOS for n in grafo['visiveis']))


def estavel(historico, n):
    return len(historico) >= n and all(h == historico[-1] for h in historico[-n:])


PARAMETER_NOT_SET = 0   # rcl_interfaces/ParameterType


def _legivel(v):
    return v is not None and getattr(v, 'type', PARAMETER_NOT_SET) != PARAMETER_NOT_SET


def resolve_lote(nomes, lote, le_um):
    """Casa nomes com valores SEM confiar que o lote veio inteiro.

    O `get_parameters` do rclcpp é tudo-ou-nada: um nome que falha zera o lote
    (o `collision_monitor` com `Polygon*.max_points`, 21-09). Qualquer contagem
    diferente da pedida — não só lote vazio — leva à leitura nome a nome, e
    cada leitura tem de devolver EXATAMENTE um valor. PARAMETER_NOT_SET é
    ilegível. Devolve ({nome: valor}, [ilegíveis]).
    """
    if lote is not None and len(lote) == len(nomes):
        pares = list(zip(nomes, lote))
    else:
        pares = []
        for n in nomes:
            um = le_um(n)
            pares.append((n, um[0] if um is not None and len(um) == 1 else None))
    ok = {n: v for n, v in pares if _legivel(v)}
    return ok, [n for n, v in pares if not _legivel(v)]


def avalia_ilegiveis(observados, permitidos):
    """Confronta os ilegíveis observados com a lista versionada, por par exato.

    Devolve os inesperados (reprovam), os permitidos que apareceram (só
    registro) e as permissões sem uso — exceção cadastrada que ficou legível ou
    sumiu, e que também reprova, para obrigar a revisar a exceção.
    """
    perm = {(p['no'], p['parametro']) for p in permitidos}
    obs = {(no, n) for no, nomes in observados.items() for n in nomes}

    def por_no(pares):
        d = {}
        for no, n in sorted(pares):
            d.setdefault(no, []).append(n)
        return d
    return {'inesperados': por_no(obs - perm),
            'permitidos_observados': por_no(obs & perm),
            'permissoes_sem_uso': [f'{no}:{n}' for no, n in sorted(perm - obs)]}


def motivos(grafo, estados, estabilizou, erros, padding, ilegiveis=None, vazios=None):
    m = []
    if grafo['duplicados']:
        m.append(f"nome duplicado: {grafo['duplicados']}")
    if grafo['faltando']:
        m.append(f"faltando: {grafo['faltando']}")
    if grafo['sobrando']:
        m.append(f"sobrando: {grafo['sobrando']}")
    if grafo.get('volateis_invalidos'):
        m.append(f"cardinalidade/nome volátil: {grafo['volateis_invalidos']}")
    nao = {n: e for n, e in estados.items() if e not in PRONTOS}
    if nao:
        m.append(f'não ativos/sem resposta: {nao}')
    if not estabilizou:
        m.append('grafo não ficou estável dentro do prazo')
    if erros:
        m.append(f'erro lendo parâmetros: {erros}')
    if ilegiveis and ilegiveis['inesperados']:
        m.append(f"parâmetros listados e ilegíveis, sem exceção: {ilegiveis['inesperados']}")
    if ilegiveis and ilegiveis['permissoes_sem_uso']:
        m.append('exceção de ilegível sem uso (ficou legível ou sumiu — revisar): '
                 f"{ilegiveis['permissoes_sem_uso']}")
    if vazios:
        m.append(f'dump vazio (todo nó expõe ao menos use_sim_time): {vazios}')
    sem = [c for c, v in padding.items() if v == 'AUSENTE']
    if sem:
        m.append(f'footprint_padding ausente em: {sem}')
    return m


def resumo_padding(normalizado):
    return {c: normalizado[c].get('footprint_padding', 'AUSENTE')
            for c in COSTMAPS if c in normalizado}


# ─── ROS ─────────────────────────────────────────────────────────────────────

class Captura:
    def __init__(self, node):
        import rclpy
        from lifecycle_msgs.srv import GetState
        from rclpy.parameter import parameter_value_to_python
        from rclpy.parameter_client import AsyncParameterClient
        self._rclpy = rclpy
        self._GetState = GetState
        self._valor = parameter_value_to_python
        self._Cliente = AsyncParameterClient
        self.node = node
        self._clientes = {}

    def _espera(self, fut, timeout=TIMEOUT_SERVICO):
        self._rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        return fut.result() if fut.done() else None

    def vistos(self):
        return [nome_completo(ns, n)
                for n, ns in self.node.get_node_names_and_namespaces()]

    def _cliente(self, no):
        if no not in self._clientes:
            self._clientes[no] = self._Cliente(self.node, no)
        return self._clientes[no]

    def _e_lifecycle(self, no):
        ns, nome = no.rsplit('/', 1)
        try:
            servs = self.node.get_service_names_and_types_by_node(nome, ns or '/')
        except Exception:
            return False
        return any(s == f'{no}/get_state' for s, _ in servs)

    def estado(self, no):
        if self._e_lifecycle(no):
            cli = self.node.create_client(self._GetState, f'{no}/get_state')
            try:
                if not cli.wait_for_service(timeout_sec=TIMEOUT_SERVICO):
                    return 'sem_resposta'
                r = self._espera(cli.call_async(self._GetState.Request()))
                return r.current_state.label if r else 'sem_resposta'
            finally:
                self.node.destroy_client(cli)
        cli = self._cliente(no)
        if not cli.wait_for_services(timeout_sec=TIMEOUT_SERVICO):
            return 'sem_resposta'
        return 'responde' if self._espera(cli.list_parameters()) else 'sem_resposta'

    def parametros(self, no):
        cli = self._cliente(no)
        if not cli.wait_for_services(timeout_sec=TIMEOUT_SERVICO):
            raise RuntimeError('serviços de parâmetro ausentes')
        lista = self._espera(cli.list_parameters(depth=None))  # recursivo, como o dump
        if lista is None:
            raise RuntimeError('list_parameters sem resposta')
        nomes = sorted(lista.result.names)
        if not nomes:
            return {}, []
        r = self._espera(cli.get_parameters(nomes))

        def le_um(n):
            um = self._espera(cli.get_parameters([n]))
            return list(um.values) if um is not None else None

        ok, ilegiveis = resolve_lote(nomes, list(r.values) if r else None, le_um)
        return {n: self._valor(v) for n, v in ok.items()}, ilegiveis


def captura(esperados, pasta, prazo, intervalo, n_estavel):
    import rclpy
    config = _configuracao(esperados)
    esperados = config['nos']
    grafo_somente = config['grafo_somente']
    permitidos = config['ilegiveis_permitidos']
    os.makedirs(pasta, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node(NOME_PROPRIO)
    cap = Captura(node)
    historico, grafo, estados = [], None, {}
    inicio = time.monotonic()
    ok = False
    with open(os.path.join(pasta, 'consultas.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'n_visiveis', 'duplicados', 'faltando', 'sobrando',
                    'nao_prontos', 'pronto'])
        while True:
            vistos = cap.vistos()
            grafo = avalia_grafo(vistos, esperados, grafo_somente)
            # Duplicado: nem pergunta estado — a resposta seria de um qualquer.
            estados = ({} if grafo['duplicados'] else {
                n: ('presente_sem_parametros' if n in grafo['somente_grafo']
                    else cap.estado(n))
                for n in grafo['visiveis']})
            p = pronto(grafo, estados)
            historico.append((tuple(grafo['visiveis']), tuple(sorted(estados.items())), p))
            nao = sorted(n for n, e in estados.items() if e not in PRONTOS)
            w.writerow([f'{time.monotonic() - inicio:.1f}', len(grafo['visiveis']),
                        ' '.join(grafo['duplicados']), ' '.join(grafo['faltando']),
                        ' '.join(grafo['sobrando']), ' '.join(nao), p])
            f.flush()
            if p and estavel(historico, n_estavel):
                ok = True
                break
            if time.monotonic() - inicio > prazo:
                break
            time.sleep(intervalo)

    with open(os.path.join(pasta, 'grafo.txt'), 'w') as f:
        for v in sorted(vistos):
            f.write(f"{v}{'   (oculto, fora da comparação)' if oculto(v) else ''}\n")
    with open(os.path.join(pasta, 'grafo_normalizado.txt'), 'w') as f:
        for v in sorted(set(grafo['visiveis']) - set(grafo['aliases'])):
            f.write(f'{v}\n')
        for regra in grafo_somente.get('volateis') or []:
            f.write(f"{regra['alias']} x{regra['quantidade']}\n")
    with open(os.path.join(pasta, 'estado_nos.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['no', 'estado'])
        for n in sorted(estados):
            w.writerow([n, estados[n]])

    brutos, erros, ilegiveis = {}, {}, {}
    if not grafo['duplicados']:
        for n in sorted(set(grafo['visiveis']) - set(grafo['somente_grafo'])):
            try:
                valores, ruins = cap.parametros(n)
                brutos[n] = {'ros__parameters': valores}
                if ruins:
                    ilegiveis[n] = sorted(ruins)
            except Exception as e:  # o dado dos outros nós fica
                erros[n] = str(e)
        with open(os.path.join(pasta, 'parametros_brutos.yaml'), 'w') as f:
            yaml.safe_dump(brutos, f, sort_keys=True, allow_unicode=True, width=1000)
    # Sempre gravado, mesmo vazio: "nenhum ilegível" é afirmação, não silêncio.
    with open(os.path.join(pasta, 'ilegiveis.yaml'), 'w') as f:
        yaml.safe_dump(ilegiveis, f, sort_keys=True, allow_unicode=True, width=1000)
    vazios = sorted(n for n in esperados
                    if n in brutos and not brutos[n]['ros__parameters'])
    normalizado = nz.normaliza(brutos)
    if brutos:
        nz.grava(normalizado, os.path.join(pasta, 'parametros_normalizados.yaml'))

    padding = resumo_padding(normalizado)
    avaliacao = avalia_ilegiveis(ilegiveis, permitidos)
    m = motivos(grafo, estados, ok, erros, padding, avaliacao, vazios)
    resumo = {
        'veredito': 'REPROVADO' if m else 'APROVADO',
        'motivos': m,
        'footprint_padding': padding,
        'ilegiveis': ilegiveis,
        'ilegiveis_permitidos_observados': avaliacao['permitidos_observados'],
        'ilegiveis_inesperados': avaliacao['inesperados'],
        'permissoes_ilegivel_sem_uso': avaliacao['permissoes_sem_uso'],
        'dumps_vazios': vazios,
        'nos_esperados': (len(esperados) + len(grafo_somente.get('exatos') or [])
                          + sum(r['quantidade']
                                for r in grafo_somente.get('volateis') or [])),
        'nos_com_parametros': len(esperados),
        'nos_visiveis': len(grafo['visiveis']),
        'consultas': len(historico),
        'segundos': round(time.monotonic() - inicio, 1),
    }
    with open(os.path.join(pasta, 'resumo.yaml'), 'w') as f:
        yaml.safe_dump(resumo, f, sort_keys=False, allow_unicode=True, width=1000)
    node.destroy_node()
    rclpy.shutdown()
    return resumo


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('esperados')
    ap.add_argument('pasta')
    ap.add_argument('--prazo', type=float, default=180.0)
    ap.add_argument('--intervalo', type=float, default=2.0)
    ap.add_argument('--estavel', type=int, default=3)
    a = ap.parse_args(argv)
    r = captura(le_configuracao(a.esperados), a.pasta, a.prazo, a.intervalo, a.estavel)
    print(yaml.safe_dump(r, sort_keys=False, allow_unicode=True, width=1000))
    return 0 if r['veredito'] == 'APROVADO' else 1


if __name__ == '__main__':
    sys.exit(main())
