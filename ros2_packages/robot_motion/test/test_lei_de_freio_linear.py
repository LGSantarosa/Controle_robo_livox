"""O freio LINEAR — a lei que impede o robô de comer 10 cm de parede.

Os números vêm de `docs/dados/2026-08-13-freio-linear/varredura-2.csv` (bancada)
e de `docs/dados/2026-08-13-re-na-porta/volta-pra-sala.csv` (a batida). Cada
teste trava um defeito que ACONTECEU — a maioria deles na bancada do giro de
14-08, e estão repetidos aqui porque a planta é a mesma placa.
"""
import pytest

from robot_motion.lei_de_freio_linear import FREANDO, SOLTO, FreioLinear


def test_enquanto_alguem_pede_marcha_o_freio_nao_disputa():
    """Freio que briga com quem dirige são dois comandos na mesma roda."""
    f = FreioLinear()
    assert f.passo(v_pedido=0.45, v_medido=0.30, dt=0.05) == 0.45
    assert f.estado == SOLTO


def test_sem_inercia_nao_ha_o_que_frear():
    """Robô parado + ninguém pedindo = zero. Contra-torque aqui seria criar
    marcha para trás do nada — a ré que o dono NÃO quer."""
    f = FreioLinear(solta_em=0.25)
    assert f.passo(0.0, 0.10, 0.05) == 0.0
    assert f.estado == SOLTO


def test_com_inercia_ele_manda_o_CONTRARIO():
    """A única forma de tirar energia desta placa é torque contrário: ela
    entrega 0,298 m/s entre 0,008 e 0,838 de comando (020) e não desacelera."""
    f = FreioLinear(v_comando=0.5, solta_em=0.25)
    assert f.passo(0.0, +0.30, 0.05) < 0, 'indo para frente, freio manda ré'
    assert f.estado == FREANDO
    g = FreioLinear(v_comando=0.5, solta_em=0.25)
    assert g.passo(0.0, -0.30, 0.05) > 0, 'andando de ré, freio manda frente'


def test_o_contra_torque_nao_INVERTE_no_meio_da_frenagem():
    """O sentido é fixado na ENTRADA. Seguir o sinal do instante faz o freio
    bater palma perto do zero; quem responde por ruído é SOLTAR, que é sempre
    mais seguro do que inverter."""
    f = FreioLinear(v_comando=0.5, solta_em=0.15, pico_min=0.12)
    saidas = [f.passo(0.0, v, 0.05) for v in (0.30, 0.26, 0.22, 0.18, 0.10)]
    freando = [s for s in saidas if s != 0.0]
    assert freando and all(s < 0 for s in freando)


def test_ele_SOLTA_antes_de_o_robo_parar():
    """A retenção de 0,52 s vale para o contra-comando também. Soltar com o
    robô já parado deixa meio segundo de empurrão reverso sobrando e ele sai
    andando para trás: medido na bancada, soltar em 0,15 deu −0,084 m."""
    f = FreioLinear(v_comando=0.5, solta_em=0.25, pico_min=0.12)
    f.passo(0.0, 0.30, 0.05)
    assert f.passo(0.0, 0.28, 0.05) < 0
    assert f.passo(0.0, 0.20, 0.05) == 0.0, 'abaixo do limiar tem de SOLTAR'
    assert f.estado == SOLTO


def test_espera_o_PICO_antes_de_poder_soltar():
    """A velocidade sobe DEPOIS do corte. Um freio que solta no primeiro `v`
    baixo solta na SUBIDA e não freia nada — é o `freou por 0,00 s` que
    apareceu em duas linhas da varredura de 13-08, e essas duas linhas
    sobraram +0,107 e +0,127 m, ou seja, bateram."""
    f = FreioLinear(v_comando=0.5, solta_em=0.15, pico_min=0.28)
    f.passo(0.0, 0.20, 0.05)
    assert f.passo(0.0, 0.10, 0.05) < 0, 'não pode soltar antes do pico'
    f.passo(0.0, 0.32, 0.05)
    assert f.passo(0.0, 0.10, 0.05) == 0.0


def test_o_teto_de_tempo_solta_de_qualquer_jeito():
    """`v` medido que nunca cai (robô preso contra a parede, pose travada)
    deixaria o contra-torque ligado para sempre — e aí o freio vira acelerador
    de ré. Foi exatamente a situação da porta: robô encostado, empurrando."""
    f = FreioLinear(v_comando=0.5, solta_em=0.25, pico_min=0.12, teto_s=0.20)
    f.passo(0.0, 0.40, 0.05)
    for _ in range(3):
        saida = f.passo(0.0, 0.40, 0.05)
    assert saida == 0.0
    assert f.estado == SOLTO
    assert 'teto' in f.motivo


def test_pedir_marcha_no_meio_da_frenagem_cancela_o_freio():
    """Humano pegando o controle no meio da frenagem: quem manda é ele."""
    f = FreioLinear(v_comando=0.5, solta_em=0.25)
    f.passo(0.0, 0.35, 0.05)
    assert f.estado == FREANDO
    assert f.passo(0.40, 0.35, 0.05) == 0.40
    assert f.estado == SOLTO


def test_limiar_nao_positivo_e_erro_de_digitacao():
    with pytest.raises(ValueError):
        FreioLinear(solta_em=0.0)


def test_o_perfil_da_BATIDA_de_13_08_com_o_freio_ligado():
    """O caso real: o reflexo zera a saída a 0,30 m da parede e a placa segura.

    Perfil de `volta-pra-sala.csv` a partir de t=21,85 s (o instante em que a
    saída foi a zero), amostrado a ~0,05 s. Sem freio ele andou +0,10 m e
    encostou. O teste exige que a lei tenha mandado contra-torque na primeira
    amostra — antes disso o robô já está gastando folga.
    """
    perfil = [0.30, 0.31, 0.29, 0.26, 0.22, 0.17, 0.11, 0.05, 0.01]
    f = FreioLinear(v_comando=0.5, solta_em=0.25, pico_min=0.12)
    saidas = [f.passo(0.0, v, 0.05) for v in perfil]
    assert saidas[0] < 0, 'o freio tem de agir na PRIMEIRA amostra do corte'
    assert saidas[-1] == 0.0, 'e tem de estar solto quando a marcha morrer'
    negativos = [i for i, s in enumerate(saidas) if s < 0]
    assert max(negativos) < len(perfil) - 2, (
        'soltou tarde: com a retenção de 0,52 s isso vira ré')
