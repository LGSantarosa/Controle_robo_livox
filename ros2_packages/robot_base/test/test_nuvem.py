"""A ponte CustomMsg -> PointCloud2 (decisão 017).

Testa a parte pura: o empacotamento. O nó em si não é testável aqui — as
mensagens do `livox_ros_driver2` só existem onde o driver foi compilado, que é
o NUC. É por isso que a lógica mora em `nuvem.py` e não dentro do nó.
"""
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'robot_base'))

from nuvem import (CAMPOS, PASSO_PONTO, desempacota,  # noqa: E402
                   empacota, ponto_valido)


def test_o_ponto_volta_como_entrou():
    dados, n, fora = empacota([(1.0, 2.0, 3.0, 40.0)])
    assert (n, fora) == (1, 0)
    assert len(dados) == PASSO_PONTO
    assert desempacota(dados) == [(1.0, 2.0, 3.0, 40.0)]


def test_o_layout_e_o_que_os_CAMPOS_prometem():
    """O consumidor lê o buffer pelos offsets declarados em `fields`. Se o
    empacotamento e a declaração discordarem, a nuvem chega com x no lugar de y
    e ninguém percebe — ela continua "uma nuvem"."""
    dados, _, _ = empacota([(7.0, 8.0, 9.0, 10.0)])
    for nome, desloc, datatype, quantos in CAMPOS:
        assert datatype == 7 and quantos == 1, nome     # FLOAT32, um por campo
        (v,) = struct.unpack_from('<f', dados, desloc)
        assert v == {'x': 7.0, 'y': 8.0, 'z': 9.0, 'intensity': 10.0}[nome]
    assert CAMPOS[-1][1] + 4 == PASSO_PONTO, 'sobrou ou faltou byte no ponto'


def test_o_ponto_ZERADO_e_DESCARTADO():
    """O Mid-360 emite (0,0,0) quando o raio não volta, e (0,0,0) no frame do
    sensor é o PRÓPRIO ROBÔ. Deixar passar marcaria célula letal em cima dele a
    cada quadro, e o planejador recusaria todo caminho por estar "dentro" de um
    obstáculo — com a nuvem parecendo perfeita."""
    assert not ponto_valido(0.0, 0.0, 0.0)
    dados, n, fora = empacota([(0.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 5.0)])
    assert (n, fora) == (1, 1)
    assert desempacota(dados) == [(1.0, 0.0, 0.0, 5.0)]


def test_zero_em_UM_eixo_nao_e_invalido():
    """Só o ponto inteiro zerado é no-return. Um ponto em (0, 3, 0) é uma
    parede exatamente ao lado do robô — descartar isso apagaria obstáculo real
    no eixo do sensor."""
    assert ponto_valido(0.0, 3.0, 0.0)
    _, n, fora = empacota([(0.0, 3.0, 0.0, 1.0)])
    assert (n, fora) == (1, 0)


def test_nuvem_inteira_invalida_nao_explode():
    """Sensor tampado: zero ponto e a contagem contando. O nó publica nuvem
    vazia em vez de parar de publicar — costmap sem quadro fica NÃO-CURRENT e
    PARA de atualizar, que é falha silenciosa (10-08)."""
    dados, n, fora = empacota([(0.0, 0.0, 0.0, 0.0)] * 5)
    assert (dados, n, fora) == (b'', 0, 5)


def test_o_tamanho_do_buffer_bate_com_a_contagem():
    """`row_step` e `width` saem daqui. Buffer maior que `width × point_step`
    faz o consumidor ler lixo no fim; menor, ele lê fora do buffer."""
    pontos = [(float(i), 1.0, 2.0, 3.0) for i in range(1, 101)]
    dados, n, fora = empacota(pontos)
    assert (n, fora) == (100, 0)
    assert len(dados) == n * PASSO_PONTO
