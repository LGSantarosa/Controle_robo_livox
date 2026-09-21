#!/usr/bin/env python3
"""Captura a linha de base de parâmetros com o grafo PRONTO (etapa 4, §10.2).

    python3 tools/linha_de_base/captura.py <esperados.yaml> <pasta_saida>
            [--prazo 180] [--intervalo 2] [--estavel 3]

`esperados.yaml` é `{nos: [/nome/completo, ...]}` — a lista explícita do passo 0.

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

Saída na pasta: `resumo.yaml` (veredito, motivos, footprint_padding dos dois
costmaps), `consultas.csv` (uma linha por consulta ao grafo),
`estado_nos.csv`, `grafo.txt`, `parametros_brutos.yaml` e
`parametros_normalizados.yaml`. Sai com código 1 se reprovar.
"""
import argparse
import csv
import importlib.util
import os
import sys
import time

import yaml

AQUI = os.path.dirname(os.path.abspath(__file__))
NOME_PROPRIO = '_linha_de_base'
COSTMAPS = ('/global_costmap/global_costmap', '/local_costmap/local_costmap')
PRONTOS = ('active', 'responde')
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


def le_esperados(caminho):
    with open(caminho) as f:
        nos = (yaml.safe_load(f) or {}).get('nos') or []
    ruins = [n for n in nos if not isinstance(n, str) or not n.startswith('/')]
    if ruins:
        raise ValueError(f'nome não completo (tem de começar com /): {ruins}')
    repetidos = sorted({n for n in nos if nos.count(n) > 1})
    if repetidos:
        raise ValueError(f'nome repetido na lista esperada: {repetidos}')
    return sorted(nos)


def avalia_grafo(vistos, esperados):
    """`vistos` pode ter repetição (é o que o grafo devolve). Ocultos saem."""
    visiveis = [v for v in vistos if not oculto(v)]
    unicos = set(visiveis)
    return {
        'visiveis': sorted(unicos),
        'duplicados': sorted({v for v in visiveis if visiveis.count(v) > 1}),
        'faltando': sorted(set(esperados) - unicos),
        'sobrando': sorted(unicos - set(esperados)),
    }


def pronto(grafo, estados):
    return (not grafo['duplicados'] and not grafo['faltando']
            and not grafo['sobrando']
            and all(estados.get(n) in PRONTOS for n in grafo['visiveis']))


def estavel(historico, n):
    return len(historico) >= n and all(h == historico[-1] for h in historico[-n:])


def motivos(grafo, estados, estabilizou, erros, padding):
    m = []
    if grafo['duplicados']:
        m.append(f"nome duplicado: {grafo['duplicados']}")
    if grafo['faltando']:
        m.append(f"faltando: {grafo['faltando']}")
    if grafo['sobrando']:
        m.append(f"sobrando: {grafo['sobrando']}")
    nao = {n: e for n, e in estados.items() if e not in PRONTOS}
    if nao:
        m.append(f'não ativos/sem resposta: {nao}')
    if not estabilizou:
        m.append('grafo não ficou estável dentro do prazo')
    if erros:
        m.append(f'erro lendo parâmetros: {erros}')
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
        r = self._espera(cli.get_parameters(nomes)) if nomes else None
        if nomes and r is None:
            raise RuntimeError('get_parameters sem resposta')
        valores = r.values if r else []
        return {n: self._valor(v) for n, v in zip(nomes, valores)}


def captura(esperados, pasta, prazo, intervalo, n_estavel):
    import rclpy
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
            grafo = avalia_grafo(vistos, esperados)
            # Duplicado: nem pergunta estado — a resposta seria de um qualquer.
            estados = ({} if grafo['duplicados']
                       else {n: cap.estado(n) for n in grafo['visiveis']})
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
    with open(os.path.join(pasta, 'estado_nos.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['no', 'estado'])
        for n in sorted(estados):
            w.writerow([n, estados[n]])

    brutos, erros = {}, {}
    if not grafo['duplicados']:
        for n in grafo['visiveis']:
            try:
                brutos[n] = {'ros__parameters': cap.parametros(n)}
            except Exception as e:  # o dado dos outros nós fica
                erros[n] = str(e)
        with open(os.path.join(pasta, 'parametros_brutos.yaml'), 'w') as f:
            yaml.safe_dump(brutos, f, sort_keys=True, allow_unicode=True, width=1000)
    normalizado = nz.normaliza(brutos)
    if brutos:
        nz.grava(normalizado, os.path.join(pasta, 'parametros_normalizados.yaml'))

    padding = resumo_padding(normalizado)
    m = motivos(grafo, estados, ok, erros, padding)
    resumo = {
        'veredito': 'REPROVADO' if m else 'APROVADO',
        'motivos': m,
        'footprint_padding': padding,
        'nos_esperados': len(esperados),
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
    r = captura(le_esperados(a.esperados), a.pasta, a.prazo, a.intervalo, a.estavel)
    print(yaml.safe_dump(r, sort_keys=False, allow_unicode=True, width=1000))
    return 0 if r['veredito'] == 'APROVADO' else 1


if __name__ == '__main__':
    sys.exit(main())
