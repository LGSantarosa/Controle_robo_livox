"""O `testpaths` do `pytest.ini` alcança TODO teste versionado — e nada morto.

A coleta da raiz passou a ser por seleção positiva (23-09): `pytest` sem
argumento coleta só os caminhos do `pytest.ini`. Isso resolve o teste de
terceiro que entrava sozinho (`ros2_packages/twist_mux`, `ESTAGIO-2026/`), mas
cria o risco simétrico: diretório de teste NOVO que ninguém lembra de listar
some da suíte sem uma linha de aviso — verde por não ter rodado.

Esta trava fecha os dois lados:
- todo arquivo de teste versionado no git está sob algum `testpaths`;
- todo `testpaths` existe no disco (entrada morta é lista podre).

A fonte da verdade é o `git ls-files`, não o disco: o que conta é o que está
versionado, e é assim que o repo chega no robô (`git reset --hard`).
"""
import configparser
import os
import subprocess

import pytest

_RAIZ = os.path.dirname(os.path.abspath(__file__))



def _testpaths():
    cfg = configparser.ConfigParser()
    cfg.read(os.path.join(_RAIZ, 'pytest.ini'))
    return [linha for linha in cfg['pytest']['testpaths'].split() if linha]


def _testes_versionados():
    try:
        saida = subprocess.run(
            ['git', 'ls-files'], cwd=_RAIZ,
            capture_output=True, text=True, timeout=30,
        )
    except (OSError, subprocess.SubprocessError) as erro:
        pytest.skip(f'git indisponível: {erro}')
    if saida.returncode != 0:
        pytest.skip(f'git ls-files falhou: {saida.stderr.strip()}')
    return [
        caminho for caminho in saida.stdout.split()
        if _e_teste(os.path.basename(caminho))
    ]


def _e_teste(nome):
    return (nome.startswith('test_') and nome.endswith('.py')) or nome.endswith('_test.py')


def _coberto(caminho, alvos):
    return any(
        caminho == alvo or caminho.startswith(alvo.rstrip('/') + '/')
        for alvo in alvos
    )


def test_todo_teste_versionado_esta_no_testpaths():
    alvos = _testpaths()
    fora = [c for c in _testes_versionados() if not _coberto(c, alvos)]
    assert not fora, (
        'arquivo de teste versionado fora do testpaths do pytest.ini — o '
        '`pytest` da raiz NÃO vai rodá-lo:\n  ' + '\n  '.join(sorted(fora))
    )


def test_nenhum_testpaths_morto():
    mortos = [
        alvo for alvo in _testpaths()
        if not os.path.exists(os.path.join(_RAIZ, alvo))
    ]
    assert not mortos, (
        'testpaths do pytest.ini que não existem no disco:\n  '
        + '\n  '.join(sorted(mortos))
    )
