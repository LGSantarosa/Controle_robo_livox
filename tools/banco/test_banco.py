"""Testes do banco de ensaios.

Existem por causa de 29-07: a bancada do planner não tinha teste nenhum, e foi
assim que uma régua errada sobreviveu por semanas medindo cúspide como curva
fechada. O dente de serra é uma máquina de estados que decide o número nº 1 do
projeto — não vai para o robô sem prova.

Nada aqui sobe ROS. O `dente_de_serra` só toca atributos do próprio objeto, o
que permite exercitá-lo com um dublê e testar a LÓGICA, não a plumbing.
"""
import argparse
import importlib.util
import math
import os

import pytest

AQUI = os.path.dirname(os.path.abspath(__file__))


def _carrega(nome):
    """Importa ensaio.py/medir.py sem instalá-los como pacote."""
    spec = importlib.util.spec_from_file_location(nome, os.path.join(AQUI, f'{nome}.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


medir = _carrega('medir')
sessao = _carrega('sessao')


class Dubles:
    """O mínimo que `dente_de_serra` toca. Evita subir nó ROS num teste."""

    def __init__(self, **kw):
        cfg = dict(rampa_ate=1.0, rampa_seg=20.0, dentes=4, taxa=50.0)
        cfg.update(kw)
        self.cfg = argparse.Namespace(**cfg)
        self.dente = 0
        self.fase = 'sobe'
        self.nivel = 0.0
        self.te_cmd = None
        self.gatilho = None
        self.eventos = []
        self.fim = False
        self.motivo = None

    def parar(self, motivo):
        self.motivo = motivo
        self.fim = True


def _importa_dente():
    """Carrega só o método, sem executar o módulo ensaio.py (que importa rclpy)."""
    import types
    fonte = open(os.path.join(AQUI, 'ensaio.py')).read()
    # Recorta a classe Ensaio inteira seria frágil; em vez disso executa o
    # arquivo com um rclpy de mentira, que é o único import pesado.
    import sys
    falsos = {}
    for nome in ('rclpy', 'rclpy.node', 'rclpy.qos', 'geometry_msgs',
                 'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg'):
        m = types.ModuleType(nome)
        falsos[nome] = m
        sys.modules.setdefault(nome, m)
    sys.modules['rclpy.node'].Node = object
    sys.modules['rclpy.qos'].QoSProfile = object
    sys.modules['rclpy.qos'].ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    sys.modules['geometry_msgs.msg'].TwistStamped = object
    sys.modules['nav_msgs.msg'].Odometry = object
    return _carrega('ensaio')


ensaio = _importa_dente()
dente_de_serra = ensaio.Ensaio.dente_de_serra


def roda_dente(no, limiar_saida, dt=0.02, t_max=600.0):
    """Simula um robô de brinquedo: anda quando |comando| passa do limiar.

    `limiar_saida` é a zona morta fingida. A queda usa 70% dela, para o
    dinâmico ser menor que o estático como na física de verdade.
    """
    te, med, saida = 0.0, 0.0, []
    andando = False
    while te < t_max and not no.fim:
        cmd = dente_de_serra(no, te, med)
        # Espelha o `passo()` de produção: encerrou nesta chamada, NÃO grava
        # linha. É justamente essa guarda que impede o dente fantasma, então o
        # dublê tem de tê-la para o teste valer alguma coisa.
        if no.fim:
            break
        mag = abs(cmd)
        if not andando and mag > limiar_saida:
            andando = True
        elif andando and mag < 0.7 * limiar_saida:
            andando = False
        med = (mag - 0.7 * limiar_saida) if andando else 0.0
        saida.append((te, cmd, med, no.dente, no.fase))
        te += dt
    return saida


# ------------------------------------------------------------ dente de serra

def test_fecha_o_numero_de_dentes_pedido():
    no = Dubles(dentes=4, rampa_ate=1.0)
    roda_dente(no, 0.30)
    assert no.fim, 'a corrida tem que terminar sozinha ao fechar os dentes'
    assert '4 dentes' in no.motivo
    assert no.dente == 4


def test_sentido_alterna_a_cada_dente():
    """Alternar é o que mantém o ensaio linear dentro da sala E o que mede os
    dois sentidos, que nesta máquina não têm por que ser iguais."""
    no = Dubles(dentes=4, rampa_ate=1.0)
    tr = roda_dente(no, 0.30)
    sinais = []
    for _, cmd, _, d, _ in tr:
        if abs(cmd) > 1e-6:
            s = 1 if cmd > 0 else -1
            if not sinais or sinais[-1][0] != d:
                sinais.append((d, s))
    assert [s for _, s in sinais] == [1, -1, 1, -1]


def test_nao_gera_dente_fantasma():
    """Defeito real, achado no Gazebo em 31-07: o contador andava antes do
    encerramento e gravava uma linha de um dente inexistente, que a leitura
    contava como 'não saiu do lugar'."""
    no = Dubles(dentes=4, rampa_ate=1.0)
    tr = roda_dente(no, 0.30)
    assert max(d for _, _, _, d, _ in tr) == 3, 'só podem existir os dentes 0..3'


def test_taxa_da_rampa_e_a_pedida():
    """A taxa É a medida: subir mais rápido infla o limiar pelo atraso de
    detecção. Um dedo em --rampa-seg tem que aparecer aqui."""
    no = Dubles(dentes=1, rampa_ate=1.0, rampa_seg=20.0)
    tr = roda_dente(no, 5.0)   # limiar inalcançável: sobe até o teto
    subida = [(t, abs(c)) for t, c, _, _, f in tr if f == 'sobe']
    t0, v0 = subida[0]
    t1, v1 = subida[-1]
    taxa = (v1 - v0) / (t1 - t0)
    assert taxa == pytest.approx(1.0 / 20.0, rel=0.05)


def test_desce_assim_que_sai_da_inercia():
    """É o que segura o ensaio dentro da sala: virar no evento, não no relógio.
    Sem isso o linear se afastava 5,25 m e a trava de 3 m matava a corrida."""
    no = Dubles(dentes=2, rampa_ate=1.0)
    tr = roda_dente(no, 0.20)
    pico = max(abs(c) for _, c, _, _, _ in tr)
    assert pico < 0.40, ('o comando não pode continuar subindo depois de achar '
                         f'o limiar (pico {pico:.3f} para limiar 0,20)')


def test_sobe_ate_o_teto_quando_o_robo_nao_sai_do_lugar():
    """Zona morta acima do teto da rampa não é falha do ensaio — é o resultado,
    e ele tem que ir até o fim para poder dizer isso."""
    no = Dubles(dentes=2, rampa_ate=1.0)
    tr = roda_dente(no, 5.0)
    assert max(abs(c) for _, c, _, _, _ in tr) == pytest.approx(1.0, abs=1e-3)
    assert any(q == 'teto' for _, q, _ in no.eventos)


def test_cada_dente_parte_do_repouso():
    """A pausa entre dentes não é enfeite: sem ela a próxima saída não seria do
    repouso, e não seria atrito estático que se estaria medindo."""
    no = Dubles(dentes=3, rampa_ate=1.0)
    tr = roda_dente(no, 0.30)
    for d in (1, 2):
        primeiro = next(c for _, c, _, dd, f in tr if dd == d and f == 'sobe'
                        for c in [c])
        assert abs(primeiro) < 0.05, f'dente {d} começou de {primeiro:.3f}'


# ------------------------------------------------------------------- leitura

def csv_dente(saidas, quedas, campo_cmd='cmd_v', campo_med='v_pose'):
    """Monta um CSV sintético com limiares conhecidos, um par por dente."""
    # A cauda acima do limiar precisa ter folga: a leitura exige 8 amostras
    # seguidas em movimento antes de aceitar que ele saiu do lugar (é o que
    # descarta solavanco).
    r, t = [], 0.0
    for d, (zs, zq) in enumerate(zip(saidas, quedas)):
        topo = int(zs * 100) + 16
        for cmd in [i * 0.01 for i in range(1, topo)]:
            t += 0.02
            r.append({'t': round(t, 3), 'dente': float(d), 'fase': 'sobe',
                      campo_cmd: cmd, campo_med: 0.1 if cmd >= zs else 0.0})
        for cmd in [i * 0.01 for i in range(topo - 1, 0, -1)]:
            t += 0.02
            r.append({'t': round(t, 3), 'dente': float(d), 'fase': 'desce',
                      campo_cmd: cmd, campo_med: 0.1 if cmd >= zq else 0.0})
    return r


def test_le_um_limiar_por_dente_e_devolve_a_media():
    r = csv_dente([0.20, 0.24, 0.22, 0.26], [0.15, 0.15, 0.15, 0.15])
    m = medir.zona_morta(r, 'cmd_v', 'v_pose', 'm/s')
    assert m == pytest.approx(0.23, abs=0.011)


def test_queda_sai_menor_que_saida(capsys):
    """Atrito dinâmico < estático. Se sair invertido, a leitura tem que gritar,
    porque é sinal de rampa rápida demais ou de pouca amostra."""
    r = csv_dente([0.20] * 4, [0.14] * 4)
    medir.zona_morta(r, 'cmd_v', 'v_pose', 'm/s')
    saida = capsys.readouterr().out
    assert 'LIMIAR DE QUEDA' in saida
    assert 'arrancar custa' in saida


def test_rampa_cortada_no_meio_nao_vira_falso_negativo(capsys):
    """Corrida cortada pelo teto de tempo deixa um dente parcial. Contá-lo como
    'não saiu do lugar' inventaria um resultado que o robô nunca deu."""
    r = csv_dente([0.20] * 3, [0.15] * 3)
    for i in range(30):        # cauda: um 4º dente que mal começou a subir
        r.append({'t': 900.0 + i * 0.02, 'dente': 3.0, 'fase': 'sobe',
                  'cmd_v': 0.001 * i, 'v_pose': 0.0})
    medir.zona_morta(r, 'cmd_v', 'v_pose', 'm/s')
    saida = capsys.readouterr().out
    assert 'NÃO saíram do lugar' not in saida
    assert 'cortados no meio' in saida


def test_csv_antigo_de_rampa_unica_continua_legivel(capsys):
    """Os CSV já gravados não têm coluna `dente`. Quebrar a leitura deles
    apagaria dado medido."""
    r = [{'t': i * 0.02, 'cmd_v': i * 0.002,
          'v_pose': 0.1 if i * 0.002 >= 0.10 else 0.0} for i in range(200)]
    m = medir.zona_morta(r, 'cmd_v', 'v_pose', 'm/s')
    assert m == pytest.approx(0.10, abs=0.005)
    assert 'rampa única' in capsys.readouterr().out


def test_espalhamento_de_uma_amostra_nao_inventa_desvio():
    m, dp, lo, hi = medir.espalhamento([0.42])
    assert (m, dp, lo, hi) == (0.42, 0.0, 0.42, 0.42)


# ----------------------------------------------------------------- protocolo

def test_expande_gera_um_arquivo_por_repeticao():
    """Repetição só vira medida se as corridas forem independentes — daí um CSV
    por corrida em vez de somarem num só."""
    fora = sessao.expande([dict(csv='4-curva_v02.csv', repete=3, args=[])])
    assert [c['csv'] for c in fora] == ['4-curva_v02-a.csv', '4-curva_v02-b.csv',
                                        '4-curva_v02-c.csv']
    assert [c['k'] for c in fora] == [1, 2, 3]
    assert all(c['de'] == 3 for c in fora)


def test_condicao_unica_mantem_o_nome_limpo():
    fora = sessao.expande([dict(csv='1-zona_morta_linear.csv', args=[])])
    assert [c['csv'] for c in fora] == ['1-zona_morta_linear.csv']


def test_repete_forcado_encurta_a_sessao():
    """`--repete 1` é a saída para bateria acabando: números sem faixa, mas
    sessão inteira."""
    fora = sessao.expande([dict(csv='4-curva_v02.csv', repete=3, args=[])],
                          forcar=1)
    assert len(fora) == 1 and fora[0]['csv'] == '4-curva_v02.csv'


def test_o_protocolo_tem_as_corridas_que_a_folha_de_campo_promete():
    """A folha de campo diz 27 corridas. Se o protocolo mudar e a folha não,
    o dono vai ao laboratório com o roteiro errado."""
    total = sum(len(sessao.expande(p['corridas'])) for p in sessao.PASSOS)
    assert total == 27


def test_passo_6_tem_a_corrida_de_controle_girada():
    """Sem ela, três retas do mesmo ponto medem o caimento do piso e a média sai
    confiante e errada — média não mata erro sistemático."""
    p6 = next(p for p in sessao.PASSOS if p['n'] == 6)
    assert any(c.get('gira_180') for c in p6['corridas'])
