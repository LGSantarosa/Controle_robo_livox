"""O pivô tem teto próprio (01-10): o wz_max subiu para 2,2 pela curva, e o
pivô, que gira no teto, passou a varrer demais contra a retenção da placa."""
import os

import yaml

AQUI = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(AQUI)


def _par():
    cfg = yaml.safe_load(open(os.path.join(PKG, 'config', 'movimentacao.yaml')))
    return cfg['heading_controller']['ros__parameters']


def test_pivo_tem_teto_proprio_abaixo_do_da_curva():
    p = _par()
    assert 0.0 < p['pivo_wz'] < p['wz_max']
    assert p['pivo_wz'] == 1.25


def test_o_no_usa_o_teto_do_pivo_ao_criar_a_manobra():
    fonte = open(os.path.join(PKG, 'robot_motion', 'heading_controller.py')).read()
    assert "('pivo_wz', 0.0)" in fonte
    assert "wz_comando=(self.par['pivo_wz'] if self.par['pivo_wz'] > 0.0" in fonte
