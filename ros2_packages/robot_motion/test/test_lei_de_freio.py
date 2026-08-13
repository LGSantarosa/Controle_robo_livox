"""O freio de giro — a lei que troca "corta e reza" por "corta e freia".

Os números vêm de `docs/dados/2026-08-14-freio-de-giro/` e do robô real em
`docs/dados/2026-08-06-pivo/`. Cada teste trava um defeito que ACONTECEU na
bancada de 14-08, e nenhum deles é hipotético — foram quatro varreduras
inválidas até a lei ficar de pé.
"""
import math

import pytest

from robot_motion.lei_de_freio import FREANDO, SOLTO, FreioDeGiro


def test_enquanto_a_lei_pede_giro_o_freio_nao_disputa():
    """Freio que briga com a lei de rumo é duas leis comandando a mesma roda."""
    f = FreioDeGiro()
    assert f.passo(wz_pedido=0.8, wz_medido=1.2, dt=0.05) == 0.8
    assert f.estado == SOLTO


def test_sem_inercia_nao_ha_o_que_frear():
    """Robô parado + lei calada = zero. Contra-torque aqui seria criar giro."""
    f = FreioDeGiro(solta_em=1.0)
    assert f.passo(0.0, 0.2, 0.05) == 0.0
    assert f.estado == SOLTO


def test_com_inercia_ele_manda_o_CONTRARIO():
    """A única forma de tirar energia desta placa é torque contrário: ela tem
    um módulo só (2,204 rad/s medido) e não sabe desacelerar."""
    f = FreioDeGiro(wz_comando=1.0, solta_em=1.0)
    saida = f.passo(0.0, +1.5, 0.05)
    assert saida < 0, 'girando para +, o freio tem de mandar −'
    assert f.estado == FREANDO
    g = FreioDeGiro(wz_comando=1.0, solta_em=1.0)
    assert g.passo(0.0, -1.5, 0.05) > 0


def test_o_contra_torque_nao_INVERTE_no_meio_da_frenagem():
    """O sentido do freio é fixado na ENTRADA e não recalculado a cada amostra.

    Um freio que seguisse o sinal do `wz` do instante bateria palma perto do
    zero. Quem responde por ruído aqui é a SAÍDA: assim que o componente no
    sentido de origem cai abaixo do limiar, ele solta — e soltar é sempre mais
    seguro do que inverter.
    """
    f = FreioDeGiro(wz_comando=1.0, solta_em=0.3, pico_min=0.6)
    saidas = [f.passo(0.0, wz, 0.05) for wz in (1.5, 1.2, 0.9, 0.6, 0.4)]
    freando = [s for s in saidas if s != 0.0]
    assert freando and all(s < 0 for s in freando), (
        'todo comando de freio tem de ser contrário ao sentido de origem')


def test_ele_SOLTA_antes_de_o_robo_parar():
    """A retenção de 0,52 s vale para o contra-comando também: soltar com o
    robô já parado deixa meio segundo de torque reverso sobrando e ele gira
    para o outro lado. Medido em 14-08: soltar em 0,6 deu −50,7° de giro.
    """
    f = FreioDeGiro(wz_comando=1.0, solta_em=1.0, pico_min=0.6)
    f.passo(0.0, 1.5, 0.05)               # freando
    assert f.passo(0.0, 1.2, 0.05) < 0    # ainda acima do limiar
    assert f.passo(0.0, 0.9, 0.05) == 0.0, 'abaixo do limiar tem de SOLTAR'
    assert f.estado == SOLTO


def test_espera_o_PICO_antes_de_poder_soltar():
    """O `wz` sobe DEPOIS do corte — pico em +0,65 s, medido no Gazebo e
    compatível com `t_parar` de 1,9 s no robô real. Um freio que solta no
    primeiro `wz` baixo solta na SUBIDA e não freia nada: foi exatamente isso
    que fez `freou por 0,00 s` aparecer em toda linha da 1ª varredura.
    """
    f = FreioDeGiro(wz_comando=1.0, solta_em=0.3, pico_min=1.4)
    f.passo(0.0, 0.5, 0.05)                # entra freando com pouca inércia
    # cai abaixo do limiar, mas o pico exigido (1,4) ainda não apareceu
    assert f.passo(0.0, 0.2, 0.05) < 0, 'não pode soltar antes do pico'
    f.passo(0.0, 1.6, 0.05)                # agora sim, a inércia apareceu
    assert f.passo(0.0, 0.2, 0.05) == 0.0


def test_o_teto_de_tempo_solta_de_qualquer_jeito():
    """Sem teto, um `wz` medido que nunca cai (sensor travado, robô preso)
    deixa o contra-torque ligado para sempre — e aí o freio vira acelerador
    para o outro lado."""
    f = FreioDeGiro(wz_comando=1.0, solta_em=0.5, pico_min=0.6, teto_s=0.20)
    f.passo(0.0, 2.0, 0.05)
    for _ in range(3):
        saida = f.passo(0.0, 2.0, 0.05)
    assert saida == 0.0
    assert f.estado == SOLTO
    assert 'teto' in f.motivo


def test_a_lei_voltando_a_pedir_giro_cancela_o_freio():
    """Objetivo novo no meio da frenagem: quem manda é a lei, não o freio."""
    f = FreioDeGiro(wz_comando=1.0, solta_em=1.0)
    f.passo(0.0, 1.5, 0.05)
    assert f.estado == FREANDO
    assert f.passo(0.9, 1.5, 0.05) == 0.9
    assert f.estado == SOLTO


def test_limiar_nao_positivo_e_erro_de_digitacao():
    with pytest.raises(ValueError):
        FreioDeGiro(solta_em=0.0)


def test_o_ciclo_inteiro_com_os_numeros_MEDIDOS():
    """Reproduz o perfil de 14-08: comando cortado, `wz` sobe até ~1,1 em
    0,65 s, e o freio tem de agir na subida/pico e soltar antes do zero.
    """
    perfil = [0.26, 0.64, 0.90, 1.12, 1.05, 0.94, 0.70, 0.48, 0.20, 0.05]
    f = FreioDeGiro(wz_comando=1.0, solta_em=1.0, pico_min=0.6)
    saidas = [f.passo(0.0, wz, 0.05) for wz in perfil]
    assert any(s < 0 for s in saidas), 'o freio precisa ter agido'
    # soltou assim que o giro caiu abaixo de 1,0 tendo passado do pico 0,6
    assert saidas[-1] == 0.0
    negativos = [i for i, s in enumerate(saidas) if s < 0]
    assert max(negativos) < len(perfil) - 2, (
        'soltou tarde demais: com a retenção de 0,52 s isso inverte o giro')
