"""Testes da lei de movimentação (decisão 005).

Cada teste trava uma propriedade MEDIDA no simulador, não um detalhe de
implementação. Se um destes cair, é a decisão 005 que está sendo contrariada.
"""
import math

import pytest

from robot_motion.lei_de_rumo import (
    comando,
    linear_de_avanco,
    norm_ang,
    piso_de_linear,
    wz_de_frenagem,
    wz_minimo_parado,
)

# Números do simulador degradado, que é onde a lei foi validada.
A_DEC = 0.3
WZ_MAX = 1.0
V_MAX = 0.7
ZONA_MORTA = 0.15
BITOLA = 0.20
MARGEM = 0.05
TOL = 0.02


def frenagem_necessaria(wz, a_dec):
    """Rumo que ainda se percorre freando de `wz` até zero."""
    return wz ** 2 / (2.0 * a_dec)


# ---------------------------------------------------------------- a lei

def test_nunca_pede_giro_que_nao_consegue_frear():
    """O ponto inteiro da decisão 005.

    Para qualquer erro, o giro comandado tem que caber na frenagem restante:
    é isso que impede o robô de atravessar o alvo e entrar no S.
    """
    for erro in [0.01, 0.05, 0.2, 0.5, 0.785, 1.57, 3.0, 3.14]:
        wz = wz_de_frenagem(erro, A_DEC, WZ_MAX)
        assert frenagem_necessaria(wz, A_DEC) <= erro + 1e-9, (
            f'erro {erro}: pediu wz={wz}, que precisa de '
            f'{frenagem_necessaria(wz, A_DEC)} rad para parar')


def test_respeita_o_teto_de_giro():
    assert wz_de_frenagem(math.pi, A_DEC, WZ_MAX) == pytest.approx(WZ_MAX)


def test_giro_acompanha_o_sinal_do_erro():
    assert wz_de_frenagem(0.5, A_DEC, WZ_MAX) > 0
    assert wz_de_frenagem(-0.5, A_DEC, WZ_MAX) < 0
    assert wz_de_frenagem(0.0, A_DEC, WZ_MAX) == 0.0


def test_a_dec_invalido_e_erro_e_nao_divisao_silenciosa():
    with pytest.raises(ValueError):
        wz_de_frenagem(0.5, 0.0, WZ_MAX)


def test_errar_a_dec_para_baixo_e_conservador():
    """Regra de ouro medida: subestimar `a_dec` só deixa o robô mais lento.

    Com `a_dec` menor a lei pede MENOS giro para o mesmo erro — freia antes da
    hora, nunca depois. É o que autoriza escrever a movimentação antes de
    medir o robô.
    """
    for erro in [0.1, 0.5, 1.0, 3.0]:
        chute_baixo = abs(wz_de_frenagem(erro, A_DEC / 3, WZ_MAX))
        real = abs(wz_de_frenagem(erro, A_DEC, WZ_MAX))
        assert chute_baixo <= real + 1e-12


# ---------------------------------------------------------------- linear

def test_linear_cede_com_o_desalinhamento():
    assert linear_de_avanco(0.0, V_MAX) == pytest.approx(V_MAX)
    assert linear_de_avanco(math.pi / 3, V_MAX) == pytest.approx(V_MAX * 0.5)
    assert linear_de_avanco(math.pi / 2, V_MAX) == pytest.approx(0.0, abs=1e-9)


def test_nunca_anda_de_re():
    """Erro acima de 90° zera a linear; nunca a torna negativa."""
    for erro in [math.pi / 2 + 0.01, 2.0, math.pi]:
        assert linear_de_avanco(erro, V_MAX) >= 0.0


# ---------------------------------------------------------------- zona morta

def test_piso_tira_a_roda_interna_da_zona_morta():
    """A defesa do BO-3: a roda de dentro tem que sair do lugar.

    Sem isso o robô fica parado encarando o erro — 22 s imóveis no ensaio E9.
    """
    for wz in [0.1, 0.5, 1.0]:
        piso = piso_de_linear(ZONA_MORTA, wz, BITOLA, MARGEM)
        roda_interna = piso - abs(wz) * BITOLA / 2.0
        assert roda_interna > ZONA_MORTA, (
            f'wz={wz}: roda interna a {roda_interna} m/s, dentro da zona morta')


def test_comando_mantem_as_duas_rodas_vivas():
    """Integração da lei: em qualquer erro, nenhuma roda fica na zona morta."""
    for erro in [0.05, 0.5, 1.57, 2.5, 3.14]:
        v, wz = comando(erro, V_MAX, A_DEC, WZ_MAX, ZONA_MORTA, BITOLA,
                        MARGEM, TOL)
        interna = v - abs(wz) * BITOLA / 2.0
        externa = v + abs(wz) * BITOLA / 2.0
        assert externa > ZONA_MORTA
        assert interna > ZONA_MORTA, (
            f'erro {erro}: v={v:.3f} wz={wz:.3f} -> roda interna {interna:.3f}')


def test_piso_nunca_acelera_acima_do_pedido():
    """O piso é defesa, não licença para andar mais rápido que o comandado."""
    v_pedido = 0.25
    for erro in [0.1, 1.0, 3.0]:
        v, _ = comando(erro, v_pedido, A_DEC, WZ_MAX, ZONA_MORTA, BITOLA,
                       MARGEM, TOL)
        assert v <= v_pedido + 1e-12


def test_velocidade_baixa_curva_devagar_em_vez_de_travar():
    """Velocidade pedida abaixo do piso não pode reabrir o BO-3.

    Antes desta cláusula, `v` era cortada em `v_max` e a roda interna voltava
    para a zona morta — robô parado com comando saindo. Agora o giro é que
    cede, e as rodas continuam vivas.
    """
    v, wz = comando(1.0, 0.25, A_DEC, WZ_MAX, ZONA_MORTA, BITOLA, MARGEM, TOL)
    interna = v - abs(wz) * BITOLA / 2.0
    assert interna >= ZONA_MORTA - 1e-9, f'roda interna a {interna} m/s'
    assert 0.0 < abs(wz) < wz_de_frenagem(1.0, A_DEC, WZ_MAX)


def test_velocidade_baixa_demais_nao_curva_e_isso_e_explicito():
    """Abaixo de `zona_morta + margem` não existe curva. Devolve zero, não mente."""
    v, wz = comando(1.0, 0.15, A_DEC, WZ_MAX, ZONA_MORTA, BITOLA, MARGEM, TOL)
    assert wz == 0.0


def test_reduzir_giro_nunca_viola_a_lei_de_frenagem():
    """A cláusula de velocidade baixa não pode criar sobrepasso.

    Menos giro do que se pode frear continua cabendo na frenagem restante.
    """
    for v_ped in [0.2, 0.25, 0.4, 0.7]:
        for erro in [0.1, 0.785, 1.57, 3.14]:
            _, wz = comando(erro, v_ped, A_DEC, WZ_MAX, ZONA_MORTA, BITOLA,
                            MARGEM, TOL)
            assert frenagem_necessaria(wz, A_DEC) <= erro + 1e-9


def test_giro_parado_devagar_e_impossivel():
    """Limite físico, registrado para a navegação não descobrir na marra."""
    assert wz_minimo_parado(0.15, 0.20) == pytest.approx(1.5)
    assert wz_minimo_parado(0.10, 0.32) == pytest.approx(0.625)


# ---------------------------------------------------------------- bordas

def test_dentro_da_tolerancia_nao_gira():
    v, wz = comando(0.01, V_MAX, A_DEC, WZ_MAX, ZONA_MORTA, BITOLA, MARGEM, TOL)
    assert wz == 0.0
    assert v == pytest.approx(V_MAX)


def test_erro_normalizado_pelo_caminho_curto():
    """Alvo a 350° é 10° para o outro lado, não 350° para este."""
    assert norm_ang(math.radians(350)) == pytest.approx(math.radians(-10))
    _, wz = comando(math.radians(350), V_MAX, A_DEC, WZ_MAX, ZONA_MORTA,
                    BITOLA, MARGEM, TOL)
    assert wz < 0, 'girou pelo caminho longo'


def test_sobrepasso_nao_depende_do_tamanho_do_erro():
    """Propriedade medida: 0,1° a 0,8° em erros de 45°, 90° e 180°.

    Aqui ela é checada na forma analítica: a frenagem consumida pelo giro
    comandado nunca ultrapassa o erro disponível, seja ele qual for.
    """
    folgas = []
    for erro in [math.radians(45), math.radians(90), math.radians(180)]:
        wz = wz_de_frenagem(erro, A_DEC, WZ_MAX)
        folgas.append(erro - frenagem_necessaria(wz, A_DEC))
    assert all(f >= -1e-9 for f in folgas)
