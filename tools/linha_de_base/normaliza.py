#!/usr/bin/env python3
"""Normaliza e compara dumps de parâmetros — a régua da etapa 4 (§1, §10.2).

A etapa 4 prova que o robô 2 NÃO muda comparando os parâmetros vivos no Gazebo
com a linha de base do passo 0. Texto bruto de `ros2 param dump` não serve para
isso: ordem de chave, formato de float (`1` vs `1.0`) e parâmetros automáticos
variam sem significado. Aqui cada dump vira um dicionário plano
`{nó: {parâmetro.com.pontos: valor}}` com:

  · os automáticos excluídos pela lista escrita abaixo (chave COMPLETA);
  · número como número (1 == 1.0), mas **bool nunca igual a número** — em
    Python `True == 1`, e na comparação isso não pode valer;
  · chaves ordenadas no arquivo, para o diff de texto também ser legível.

A comparação lista o que entrou, saiu ou mudou, por caminho exato `nó:param`
(ou só `nó`, se o nó inteiro sumiu/apareceu). Diferença só passa se houver uma
permissão com o MESMO caminho, o mesmo tipo e — quando dado — o mesmo valor.
Permissão que não autorizou nada é avisada: não reprova, mas não some calada.

    python3 tools/linha_de_base/normaliza.py normaliza <bruto.yaml> <saida.yaml>
    python3 tools/linha_de_base/normaliza.py compara <base.yaml> <novo.yaml> [--permitidas p.yaml]

`compara` sai com código 1 se reprovar.
"""
import argparse
import math
import sys
from dataclasses import dataclass, field

import yaml

# Parâmetros que o rclcpp/rclpy declara sozinho em todo nó. Mudar esta lista é
# mudar a régua: o teste `test_lista_de_exclusao_e_explicita` trava os dois.
EXCLUIDOS_EXATOS = ('use_sim_time', 'start_type_description_service')
EXCLUIDOS_PREFIXO = ('qos_overrides.',)

TIPOS = ('entrou', 'saiu', 'mudou')


# ─── normalização ────────────────────────────────────────────────────────────

def _excluido(nome):
    return nome in EXCLUIDOS_EXATOS or nome.startswith(EXCLUIDOS_PREFIXO)


def _valor(v):
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return float(v)
    if isinstance(v, (list, tuple)):
        return [_valor(x) for x in v]
    return v


def _achata(d, prefixo, saida):
    for k, v in d.items():
        nome = f'{prefixo}{k}'
        if isinstance(v, dict):
            _achata(v, f'{nome}.', saida)
        elif not _excluido(nome):
            saida[nome] = _valor(v)


def normaliza(dump):
    """`{nó: {'ros__parameters': {...aninhado...}}}` → `{nó: {a.b.c: valor}}`.

    Aceita também o próprio formato normalizado (nó → dicionário plano), para
    que normalizar duas vezes dê o mesmo resultado.
    """
    saida = {}
    for no, corpo in dump.items():
        params = corpo.get('ros__parameters', corpo) if isinstance(corpo, dict) else {}
        plano = {}
        _achata(params or {}, '', plano)
        saida[no] = dict(sorted(plano.items()))
    return dict(sorted(saida.items()))


def como_dump(normalizado):
    """Volta ao formato de dump (plano dentro de `ros__parameters`)."""
    return {no: {'ros__parameters': dict(p)} for no, p in normalizado.items()}


def grava(normalizado, caminho):
    with open(caminho, 'w') as f:
        yaml.safe_dump(normalizado, f, sort_keys=True, default_flow_style=None,
                       allow_unicode=True, width=1000)


def le(caminho):
    with open(caminho) as f:
        return normaliza(yaml.safe_load(f) or {})


# ─── comparação ──────────────────────────────────────────────────────────────

def _chave(v):
    """Forma comparável que separa bool de número (True != 1.0 aqui)."""
    if isinstance(v, bool):
        return ('bool', v)
    if isinstance(v, float):
        return ('num', 'nan' if math.isnan(v) else v)
    if isinstance(v, list):
        return ('lista', tuple(_chave(x) for x in v))
    return (type(v).__name__, v)


@dataclass(frozen=True)
class Diferenca:
    caminho: str
    tipo: str
    antes: object = None
    depois: object = None


@dataclass(frozen=True)
class Permitida:
    """Diferença autorizada. `antes`/`depois` None = valor não conferido."""
    caminho: str
    tipo: str
    antes: object = None
    depois: object = None

    def __post_init__(self):
        if self.tipo not in TIPOS:
            raise ValueError(f'tipo {self.tipo!r} inválido; use um de {TIPOS}')

    def autoriza(self, d):
        if (d.caminho, d.tipo) != (self.caminho, self.tipo):
            return False
        for esperado, visto in ((self.antes, d.antes), (self.depois, d.depois)):
            if esperado is not None and _chave(_valor(esperado)) != _chave(visto):
                return False
        return True


@dataclass
class Resultado:
    diferencas: list = field(default_factory=list)
    nao_autorizadas: list = field(default_factory=list)
    permissoes_sem_uso: list = field(default_factory=list)

    @property
    def aprovado(self):
        return not self.nao_autorizadas


def compara(base, novo, permitidas=()):
    difs = []
    for no in sorted(set(base) | set(novo)):
        if no not in novo:
            difs.append(Diferenca(no, 'saiu'))
            continue
        if no not in base:
            difs.append(Diferenca(no, 'entrou'))
            continue
        a, b = base[no], novo[no]
        for p in sorted(set(a) | set(b)):
            caminho = f'{no}:{p}'
            if p not in b:
                difs.append(Diferenca(caminho, 'saiu', antes=a[p]))
            elif p not in a:
                difs.append(Diferenca(caminho, 'entrou', depois=b[p]))
            elif _chave(a[p]) != _chave(b[p]):
                difs.append(Diferenca(caminho, 'mudou', antes=a[p], depois=b[p]))

    usadas = set()
    nao_aut = []
    for d in difs:
        quem = [i for i, p in enumerate(permitidas) if p.autoriza(d)]
        if quem:
            usadas.update(quem)
        else:
            nao_aut.append(d)
    sem_uso = [p for i, p in enumerate(permitidas) if i not in usadas]
    return Resultado(difs, nao_aut, sem_uso)


def le_permitidas(caminho):
    with open(caminho) as f:
        itens = yaml.safe_load(f) or []
    return [Permitida(i['caminho'], i['tipo'], i.get('antes'), i.get('depois'))
            for i in itens]


# ─── linha de comando ────────────────────────────────────────────────────────

def relatorio(r):
    linhas = []
    for d in r.diferencas:
        marca = '❌' if d in r.nao_autorizadas else '✅ permitida'
        linhas.append(f'{marca}  {d.tipo:6}  {d.caminho}  {d.antes!r} → {d.depois!r}')
    for p in r.permissoes_sem_uso:
        linhas.append(f'⚠️  permissão sem uso: {p.tipo} {p.caminho}')
    linhas.append('APROVADO' if r.aprovado else
                  f'REPROVADO: {len(r.nao_autorizadas)} diferença(s) não autorizada(s)')
    return '\n'.join(linhas)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    n = sub.add_parser('normaliza')
    n.add_argument('bruto')
    n.add_argument('saida')
    c = sub.add_parser('compara')
    c.add_argument('base')
    c.add_argument('novo')
    c.add_argument('--permitidas')
    a = ap.parse_args(argv)

    if a.cmd == 'normaliza':
        grava(le(a.bruto), a.saida)
        return 0
    perm = le_permitidas(a.permitidas) if a.permitidas else []
    r = compara(le(a.base), le(a.novo), perm)
    print(relatorio(r))
    return 0 if r.aprovado else 1


if __name__ == '__main__':
    sys.exit(main())
