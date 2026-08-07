"""Testes do banco de ensaios.

Existem por causa de 29-07: a bancada do planner não tinha teste nenhum, e foi
assim que uma régua errada sobreviveu por semanas medindo cúspide como curva
fechada. O dente de serra é uma máquina de estados que decide o número nº 1 do
projeto — não vai para o robô sem prova.

Nada aqui sobe ROS. O `dente_de_serra` só toca atributos do próprio objeto, o
que permite exercitá-lo com um dublê e testar a LÓGICA, não a plumbing.
"""
import argparse
import csv
import datetime
import importlib.util
import math
import os
import types
from collections import deque

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

    def parar(self, motivo, normal=False):
        self.motivo = motivo
        self.normal = normal
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

def csv_dente(saidas, quedas, campo_cmd='cmd_v', campo_med='v_pose', movendo=0.1):
    """Monta um CSV sintético com limiares conhecidos, um par por dente.

    `movendo` é o valor do campo medido quando o robô anda. Nos ensaios de giro
    ele precisa ser bem maior: o campo é rad/s e a leitura o converte para borda
    de roda (x L/2), então 0,1 rad/s viram 0,0135 m/s — parado, para o limiar.
    """
    # A cauda acima do limiar precisa ter folga: a leitura exige 8 amostras
    # seguidas em movimento antes de aceitar que ele saiu do lugar (é o que
    # descarta solavanco).
    r, t = [], 0.0
    for d, (zs, zq) in enumerate(zip(saidas, quedas)):
        topo = int(zs * 100) + 16
        for cmd in [i * 0.01 for i in range(1, topo)]:
            t += 0.02
            r.append({'t': round(t, 3), 'dente': float(d), 'fase': 'sobe',
                      campo_cmd: cmd, campo_med: movendo if cmd >= zs else 0.0})
        for cmd in [i * 0.01 for i in range(topo - 1, 0, -1)]:
            t += 0.02
            r.append({'t': round(t, 3), 'dente': float(d), 'fase': 'desce',
                      campo_cmd: cmd, campo_med: movendo if cmd >= zq else 0.0})
    return r


def test_le_um_limiar_por_dente_e_devolve_a_media():
    r = csv_dente([0.20, 0.24, 0.22, 0.26], [0.15, 0.15, 0.15, 0.15])
    m = medir.zona_morta(r, 'cmd_v', 'v_pose', 'm/s')
    assert m == pytest.approx(0.23, abs=0.011)


def test_primeiro_dente_destoante_e_separado_em_vez_de_diluido(capsys):
    """Só o dente #0 parte de repouso longo; os outros, da pausa curta. Atrito
    estático cresce com o tempo parado, então são condições diferentes — e a
    média dos quatro esconderia justamente o caso do BO-3 (robô parado, mandaram
    andar). Visto no Gazebo em 31-07: #0 em 0,203 contra 0,059 dos demais."""
    r = csv_dente([0.40, 0.20, 0.20, 0.20], [0.15] * 4)
    medir.zona_morta(r, 'cmd_v', 'v_pose', 'm/s')
    saida = capsys.readouterr().out
    assert 'o dente #0' in saida and 'destoa' in saida
    assert 'repouso LONGO' in saida


def test_primeiro_dente_alinhado_nao_gera_ruido(capsys):
    """Se todos concordam, não há nada a separar — e o aviso não pode virar
    barulho de fundo que se aprende a ignorar."""
    r = csv_dente([0.20, 0.21, 0.20, 0.19], [0.15] * 4)
    medir.zona_morta(r, 'cmd_v', 'v_pose', 'm/s')
    assert 'destoa' not in capsys.readouterr().out


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


def test_o_giro_e_o_primeiro_ensaio_da_sessao():
    """Decisão de 31-07, e não é ordem de gosto: o ensaio de giro responde a
    pergunta do pivô DIRETAMENTE (o menor wz que gira o robô parado É o limiar
    do pivô, sem converter por bitola). Sessão cortada perde o que está por
    último, e em 30-07 a sessão foi bloqueada sem medir nada."""
    assert sessao.PASSOS[0]['tipo'] == 'zona_morta_giro'
    assert sessao.PASSOS[1]['tipo'] == 'zona_morta_linear'


def test_os_numeros_dos_passos_batem_com_a_posicao():
    """`--so N` e `--de N` indexam por este campo, e a folha de campo cita os
    números. Um passo renumerado sem o resto manda o dono rodar outro ensaio."""
    assert [p['n'] for p in sessao.PASSOS] == list(range(1, len(sessao.PASSOS) + 1))


def test_o_csv_diz_a_que_passo_pertence():
    """O prefixo numérico do CSV é como a pasta se lê em casa. Se ele
    descolar do passo, o dado fica ambíguo meses depois."""
    for p in sessao.PASSOS:
        for c in p['corridas']:
            assert c['csv'].startswith(f'{p["n"]}-'), (p['n'], c['csv'])


def test_o_protocolo_tem_as_corridas_que_a_folha_de_campo_promete():
    """A folha de campo diz 27 corridas. Se o protocolo mudar e a folha não,
    o dono vai ao laboratório com o roteiro errado."""
    total = sum(len(sessao.expande(p['corridas'])) for p in sessao.PASSOS)
    assert total == 27


# -------------------------------------------------------- calibração viva

def test_calibracao_que_bate_com_a_trena_passa_calada():
    linhas = sessao.laudo_calibracao(
        {'wheel_separation': 0.270, 'wheel_radius': 0.080,
         'left_wheel_names': ['left_wheel_joint'],
         'right_wheel_names': ['right_wheel_joint']})
    assert not any('ATENÇÃO' in l for l in linhas)


def test_bitola_velha_e_delatada_com_o_desvio():
    """0,32 num robô de 0,270 desloca TODO limiar medido em 18,5%, e depois é
    indistinguível de derrapada. Tem que aparecer antes de medir."""
    linhas = sessao.laudo_calibracao(
        {'wheel_separation': 0.32, 'wheel_radius': 0.0825,
         'left_wheel_names': ['left_wheel_joint'],
         'right_wheel_names': ['right_wheel_joint']})
    texto = '\n'.join(linhas)
    assert 'ATENÇÃO' in texto
    assert '+18.5%' in texto, texto
    assert 'wheel_radius' in texto


def test_delatar_nao_e_bloquear():
    """Divergir pode ser deliberado. O que não pode é ninguém saber — o dado
    segue interpretável porque a calibração fica gravada ao lado."""
    linhas = sessao.laudo_calibracao({'wheel_separation': 0.32,
                                      'wheel_radius': 0.080})
    assert any('Não estou parando a sessão' in l for l in linhas)


def test_reconhece_o_swap_de_rodas_nos_dois_estados():
    normal = {'left_wheel_names': ['left_wheel_joint'],
              'right_wheel_names': ['right_wheel_joint']}
    trocado = {'left_wheel_names': ['right_wheel_joint'],
               'right_wheel_names': ['left_wheel_joint']}
    assert sessao.swap_aplicado(normal) is False
    assert sessao.swap_aplicado(trocado) is True
    assert sessao.swap_aplicado({}) is None


def test_controlador_mudo_nao_finge_calibracao():
    """Sem resposta, o certo é dizer 'desconhecida'. Assumir os valores do
    fonte é exatamente o erro que esta conferência existe para evitar."""
    linhas = sessao.laudo_calibracao({})
    assert all('[ok]' not in l for l in linhas)
    assert sum('não veio do controlador' in l for l in linhas) == 2


def test_passo_6_tem_a_corrida_de_controle_girada():
    """Sem ela, três retas do mesmo ponto medem o caimento do piso e a média sai
    confiante e errada — média não mata erro sistemático."""
    p6 = next(p for p in sessao.PASSOS if p['n'] == 6)
    assert any(c.get('gira_180') for c in p6['corridas'])


def test_fonte_roda_le_as_colunas_da_roda():
    """O `--fonte roda` existe porque em 31-07 o LIO fabricou 143,8° de giro num
    pivô que os encoders mediram como 8,7°. Se a leitura ignorasse a escolha e
    fosse na coluna do LIO assim mesmo, o ensaio trocaria de gatilho e o número
    continuaria vindo do sinal quebrado."""
    r = csv_dente([0.15] * 4, [0.10] * 4, campo_cmd='cmd_wz',
                  campo_med='wz_roda', movendo=0.3)
    m = medir.LEITURAS('roda')['zona_morta_giro'](r)
    assert m == pytest.approx(0.15, abs=0.011)


def test_fonte_lio_continua_o_padrao():
    """A roda não enxerga derrapagem: ela mede o EIXO, não o robô. Só serve
    enquanto o LIO estiver quebrado, então o padrão não pode escorregar."""
    r = csv_dente([0.20] * 4, [0.15] * 4)
    assert medir.LEITURAS()['zona_morta_linear'](r) == pytest.approx(0.20, abs=0.011)
    assert medir.LEITURAS('lio')['zona_morta_linear'](r) is not None


def test_resumo_respeita_a_fonte(tmp_path, capsys):
    """A divergência silenciosa que este banco existe para não ter: as corridas
    lidas da roda e a média das N repetições saindo do LIO, sem ninguém avisar."""
    arqs = []
    for i in range(3):
        r = csv_dente([0.15] * 4, [0.10] * 4, campo_cmd='cmd_wz',
                      campo_med='wz_roda', movendo=0.3)
        p = tmp_path / f'c{i}.csv'
        with open(p, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(r[0].keys()))
            w.writeheader()
            w.writerows(r)
        arqs.append(str(p))
    medir.resumo('zona_morta_giro', arqs, 'roda')
    saida = capsys.readouterr().out
    assert 'fonte=roda' in saida
    assert 'nenhuma corrida devolveu número' not in saida


def test_rastejo_de_eixo_no_giro_nao_vira_zona_morta():
    """O defeito de 31-07, travado. O giro comparava rad/s cru contra o mesmo
    LIMIAR_PARADO da reta, então 0,03 rad/s (= 0,004 m/s de borda de roda)
    contavam como 'andando'. O ensaio devolveu 0,032 rad/s enquanto o dono via
    o robô PARADO: era folga do eixo, 2° de encoder por dente. Convertido para
    borda de roda, o mesmo dado tem de dizer que ele não saiu do lugar."""
    r = csv_dente([0.032] * 4, [0.012] * 4, campo_cmd='cmd_wz',
                  campo_med='wz_roda', movendo=0.035)
    assert medir.LEITURAS('roda')['zona_morta_giro'](r) is None


# ----------------------------------------------------- trava de dado parado

class PoseFalsa:
    """O mínimo de `nav_msgs/Odometry` que o `passo()` lê."""

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        q = types.SimpleNamespace(x=0.0, y=0.0,
                                  z=math.sin(yaw / 2), w=math.cos(yaw / 2))
        self.pose = types.SimpleNamespace(pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=x, y=y), orientation=q))
        self.twist = types.SimpleNamespace(twist=types.SimpleNamespace(
            linear=types.SimpleNamespace(x=0.0),
            angular=types.SimpleNamespace(z=0.0)))


class DublesPasso:
    """Dublê do nó para exercitar o `passo()` sem subir ROS.

    O relógio é nosso (`relogio`), então dá para simular a fonte calando: basta
    andar com o tempo e NÃO mexer no `t_pose`.
    """

    def __init__(self, **kw):
        cfg = dict(ensaio='reta', fonte='lio', espaco=4.0, dur=20.0,
                   sem_dado=1.0, taxa=50.0, v=0.3, wz=0.5, bitola=0.270,
                   rampa_ate=1.0, rampa_seg=20.0, dentes=4, janela=0.5)
        cfg.update(kw)
        self.cfg = argparse.Namespace(**cfg)
        self.relogio = 0.0
        self.pose = PoseFalsa()
        self.roda = None
        self.t_pose = 0.0
        self.t_roda = None
        self.janela = self.cfg.janela
        self.hist = deque()
        self.hist_roda = deque()
        self.t0 = self.p0 = self.p0_roda = None
        self.t_ant = -1.0
        self.linhas = []
        self.fim = False
        self.motivo = 'concluído'
        self.normal = True
        self.dente, self.fase, self.nivel = 0, 'sobe', 0.0
        self.te_cmd = self.gatilho = None
        self.eventos = []
        self.publicados = []

    def agora(self):
        return self.relogio

    def publica(self, v, wz):
        self.publicados.append((v, wz))

    derivada = ensaio.Ensaio.derivada
    comando = ensaio.Ensaio.comando
    parar = ensaio.Ensaio.parar


passo = ensaio.Ensaio.passo


def roda_passo(no, segundos, fonte_viva=True, dt=0.02):
    """Avança a corrida. `fonte_viva=False` congela a pose, como a rede caindo."""
    fim = no.relogio + segundos
    while no.relogio < fim and not no.fim:
        no.relogio += dt
        if fonte_viva:
            # 0,05 m/s: devagar de propósito, senão a corrida longa bate na
            # trava de ESPAÇO (4 m) antes do teto de tempo e o teste passaria a
            # medir a trava errada.
            no.pose = PoseFalsa(x=0.05 * no.relogio)
            no.t_pose = no.relogio
        passo(no)


def test_fonte_que_cala_no_meio_aborta_a_corrida():
    """O defeito de 31-07: a rede caiu, `self.pose` congelou no último valor, a
    derivada passou a devolver 0,0 — leitura plausível — e as corridas fecharam
    inteiras com 'concluído' e cara de sucesso. Sete saíram assim."""
    no = DublesPasso(dur=20.0, sem_dado=1.0)
    roda_passo(no, 3.0)
    assert not no.fim and len(no.linhas) > 100, 'a corrida tem de ter começado'

    roda_passo(no, 5.0, fonte_viva=False)
    assert no.fim, 'fonte calada tem de ABORTAR, não completar a corrida'
    assert 'calou' in no.motivo and '/Odometry' in no.motivo
    assert not no.normal, 'aborto tem de virar código de saída para o sessao.py'
    assert no.publicados[-1] == (0.0, 0.0), 'e o robô tem de parar'


def test_aborta_antes_de_gravar_a_corrida_inteira():
    """Não basta abortar: tem de abortar CEDO. Se ele só percebesse no fim, o
    CSV já teria os 20 s e a corrida passaria por medida na análise."""
    no = DublesPasso(dur=20.0, sem_dado=1.0)
    roda_passo(no, 2.0)
    n_vivo = len(no.linhas)
    roda_passo(no, 10.0, fonte_viva=False)
    congeladas = (len(no.linhas) - n_vivo) * 0.02
    # Uma volta de malha de folga sobre a trava; e MUITO longe dos 10 s que
    # sobravam de corrida, que é o que ele gravaria sem a trava.
    assert congeladas <= no.cfg.sem_dado + 0.02, (
        f'gravou {congeladas:.2f} s de pose congelada; a trava é de '
        f'{no.cfg.sem_dado} s')


def test_fonte_viva_nao_e_abortada():
    """A trava não pode matar corrida boa: com a fonte falando, os 20 s inteiros
    têm de rodar e terminar como fim NORMAL."""
    no = DublesPasso(dur=20.0, sem_dado=1.0)
    roda_passo(no, 25.0)
    assert no.fim and no.motivo == 'concluído'
    assert no.normal, 'fim normal não pode virar erro para o sessao.py'


def test_corrida_que_nem_comecou_nao_dispara_a_trava():
    """Antes da primeira mensagem não há o que estar parado — quem trata esse
    caso é o `grava()`, que já diz 'sem dados'. A trava aqui só confundiria."""
    no = DublesPasso()
    no.pose = None
    no.t_pose = None
    roda_passo(no, 5.0, fonte_viva=False)
    assert not no.fim and no.linhas == []


# ------------------------------------------- o angulo que enrola (30-07)

def csv_degrau(pico, dyaw_graus, atraso=0.4, dt=0.02, t_corte=4.0):
    """CSV sintético de degrau de giro com PICO conhecido depois do corte.

    Reproduz o robô de verdade: o comando zera em `t_corte` mas a placa segue
    empurrando por `atraso`, e só então ele desacelera. `dyaw_graus` é o giro da
    frenagem — passe mais de 180° para exercitar o enrolamento.
    """
    r, t, yaw = [], 0.0, 0.0
    def linha(cmd_wz, wz):
        nonlocal t, yaw
        yaw += wz * dt
        r.append({'t': round(t, 3), 'cmd_wz': cmd_wz, 'wz_pose': wz,
                  'yaw': math.atan2(math.sin(yaw), math.cos(yaw)),  # SEMPRE enrolado
                  'cmd_v': 0.0, 'v_pose': 0.0, 'x': 0.0, 'y': 0.0})
        t += dt
    # No corte ele ainda NAO esta no pico — e essa a razao de o pico existir.
    # Platô achatado aqui faria o max() cair no proprio corte e o teste nao
    # exercitaria nada.
    wz_corte = 0.7 * pico
    while t < t_corte:                       # acelerando sob comando
        linha(0.6, wz_corte * t / t_corte)
    t0 = t
    while t - t0 < atraso:                   # comando zerado, mas AINDA SOBE
        linha(0.0, wz_corte + (pico - wz_corte) * (t - t0) / atraso)
    # frenagem: rampa linear ate zero, ajustada para dar o dyaw pedido
    alvo = math.radians(dyaw_graus)
    dur = 2.0 * alvo / pico                  # area do triangulo = alvo
    t0 = t
    while t - t0 < dur:
        linha(0.0, pico * max(0.0, 1.0 - (t - t0) / dur))
    for _ in range(30):
        linha(0.0, 0.0)
    return r


def test_a_dec_nao_enrola_quando_a_inercia_passa_de_180():
    """O defeito de 30-07 vivendo no medir.py. A versão antiga fazia
    atan2(sin(yaw_fim - yaw_corte), ...), que le 200° como -160° — e devolve um
    a_dec com sinal e magnitude errados, sem sintoma nenhum."""
    r = csv_degrau(pico=2.0, dyaw_graus=200.0)
    a = medir.a_dec(r)
    assert a is not None
    esperado = 2.0 ** 2 / (2 * math.radians(200.0))
    assert a == pytest.approx(esperado, rel=0.10), (
        f'a_dec {a:.3f} contra {esperado:.3f}: a inercia de 200° foi lida enrolada')


def test_a_dec_mede_do_PICO_e_nao_do_corte():
    """A placa segue empurrando ~0,5 s depois do comando zerar (medido 04-08:
    0,40/0,56/0,60 s). Medir do corte inclui trecho ACIONADO, que nao e
    desaceleracao, e infla o a_dec."""
    r = csv_degrau(pico=2.0, dyaw_graus=60.0, atraso=0.5)
    a = medir.a_dec(r)
    esperado = 2.0 ** 2 / (2 * math.radians(60.0))
    assert a == pytest.approx(esperado, rel=0.10)


def test_a_dec_curto_continua_valendo():
    """A trava nao pode quebrar o caso normal, abaixo de 180°."""
    r = csv_degrau(pico=1.5, dyaw_graus=45.0)
    a = medir.a_dec(r)
    assert a == pytest.approx(1.5 ** 2 / (2 * math.radians(45.0)), rel=0.10)


def _gira(total_graus, passo_graus=5.0):
    """Sequência de yaw ENROLADO para um giro contínuo de `total_graus`.

    Termina no ângulo EXATO — com passo fixo o último degrau ficava de fora e o
    teste comparava 280° contra 281,5°, acusando o acumulador por um defeito do
    próprio dublê.
    """
    n = max(1, int(math.ceil(abs(total_graus) / passo_graus)))
    return [math.atan2(math.sin(math.radians(total_graus * i / n)),
                       math.cos(math.radians(total_graus * i / n)))
            for i in range(n + 1)]


def test_cutucao_le_giro_de_mais_de_meia_volta_com_o_sinal_CERTO():
    """O defeito que bloqueou 30-07. Um giro anti-horário de 281,5° era lido
    como −78,5° — sinal trocado — e diagnosticado como 'rodas espelhadas'. O
    conserto quase aplicado teria quebrado um robô que estava certo."""
    ys = _gira(281.5)
    g = sessao.GiroAcumulado(ys[0])
    for y in ys[1:]:
        g.soma(y)
    assert math.degrees(g.total) == pytest.approx(281.5, abs=1.0)
    assert g.total > 0, 'anti-horário tem de sair POSITIVO, não negativo'


def test_cutucao_pega_o_sentido_de_verdade_se_ele_estiver_invertido():
    """A trava não pode cegar o cutucão: rodas de fato trocadas ainda têm de ser
    acusadas. Aqui o robô gira HORÁRIO com comando anti-horário."""
    ys = _gira(-281.5)
    g = sessao.GiroAcumulado(ys[0])
    for y in ys[1:]:
        g.soma(y)
    assert g.total < 0, 'giro invertido de verdade tem de continuar sendo pego'
    assert math.degrees(g.total) == pytest.approx(-281.5, abs=1.0)


def test_giro_acumulado_conta_varias_voltas():
    """Este robô chega a girar mais de uma volta num cutucão de 2 s."""
    ys = _gira(760.0)
    g = sessao.GiroAcumulado(ys[0])
    for y in ys[1:]:
        g.soma(y)
    assert math.degrees(g.total) == pytest.approx(760.0, abs=2.0)


# ------------------------------------------- curvatura: o arco de 04-08

DADOS_0804 = os.path.join(AQUI, '..', '..', 'docs', 'dados',
                          '2026-08-04-bancada-robo')


def csv_arco(curv, caminho=1.2, cmd_v=0.25, n=200, yaw0=0.0):
    """Arco perfeito de curvatura conhecida, para conferir a régua.

    Anda `caminho` metros com curvatura `curv` [1/m]. De ré (`cmd_v` < 0) o
    corpo aponta para trás da direção de viagem — que é o que o robô faz e o
    que a leitura tem de continuar medindo igual.
    """
    r = []
    for i in range(n):
        s = caminho * i / (n - 1)
        a = curv * s
        if abs(curv) > 1e-9:
            x, y = math.sin(a) / curv, (1 - math.cos(a)) / curv
        else:
            x, y = s, 0.0
        rot = yaw0 + (math.pi if cmd_v < 0 else 0.0)
        r.append({'t': round(0.1 * i, 3), 'cmd_v': cmd_v, 'cmd_wz': 0.0,
                  'x': x * math.cos(yaw0) - y * math.sin(yaw0),
                  'y': x * math.sin(yaw0) + y * math.cos(yaw0),
                  'yaw': medir.norm(a + rot)})
    return r


def test_curvatura_le_um_arco_conhecido(capsys):
    """A régua antes da medida: arco de raio 1,22 m tem de ler −0,82 1/m."""
    assert medir.curvatura(csv_arco(-0.817)) == pytest.approx(-0.817, rel=0.02)
    assert medir.curvatura(csv_arco(-0.098)) == pytest.approx(-0.098, rel=0.02)


def test_curvatura_nao_enrola_passando_de_180(capsys):
    """O defeito de `7a0c364`, na régua nova. A corrida `frente-a` girou 242°;
    com `atan2` entre as pontas ela leria −118°, com o SINAL TROCADO. Aqui o
    arco gira 242° em 3,7 m = −1,14 1/m."""
    r = csv_arco(-1.143, caminho=3.7, n=800)
    assert medir.curvatura(r) == pytest.approx(-1.143, rel=0.02)


def test_curvatura_de_reta_de_verdade_da_zero(capsys):
    """Se a régua não souber ver reta, ela não sabe ver arco."""
    assert abs(medir.curvatura(csv_arco(0.0))) < 1e-6


def test_curvatura_ignora_o_robo_parado(capsys):
    """Antes e depois do comando o robô está parado, e o LIO ainda treme. Essas
    amostras não andam caminho nenhum: entrando na conta, viram giro dividido
    por ~zero e explodem a curvatura."""
    r = csv_arco(-0.817)
    parado = [dict(r[-1], t=r[-1]['t'] + 0.1 * i, cmd_v=0.0,
                   yaw=r[-1]['yaw'] + 0.02 * i) for i in range(1, 30)]
    assert medir.curvatura(r + parado) == pytest.approx(-0.817, rel=0.02)


def test_curvatura_reproduz_o_robo_medido_em_0804(capsys):
    """Regressão contra o dado CRU do robô, não contra um número copiado.

    Estes CSV estão versionados; se a régua mudar de resposta, ou ela quebrou
    ou o alvo do simulador mudou — e as duas coisas têm de doer aqui.
    """
    def med(nome):
        return medir.curvatura(medir.le(os.path.join(DADOS_0804, nome)))

    frente = [med(f'6-reta_frente-{r}.csv') for r in 'bc']
    re = [med(f'6-reta_re-{r}.csv') for r in 'bc']
    mf, mr = sum(frente) / 2, sum(re) / 2

    assert mf == pytest.approx(-0.84, abs=0.03), 'arco de frente'
    assert mr == pytest.approx(-0.11, abs=0.03), 'arco de ré'
    assert mf / mr == pytest.approx(7.4, abs=1.5), 'a assinatura da boba'
    assert mf < 0 and mr < 0, 'os dois sentidos arcam para o mesmo lado do corpo'


# ------------------ do número medido ao comando que o usa (decisão 013)
#
# O caminho 3 da 013 só existe se a medida do dia CHEGAR ao compensador. Até
# 07-08 a régua parava na média impressa: o valor era lido em voz alta na
# bancada e não tinha para onde ir. Estes testes travam a ponte — e travam
# principalmente as RECUSAS, porque uma linha colável impressa a partir de
# medida ruim é pior que nenhuma: ela seria colada.

def arqs_de_arco(tmp_path, curvs, cmd_v=0.25, prefixo='c'):
    """N corridas de curvatura conhecida, gravadas como o ensaio gravaria."""
    saida = []
    for i, c in enumerate(curvs):
        r = csv_arco(c, cmd_v=cmd_v)
        p = tmp_path / f'{prefixo}{i}.csv'
        with open(p, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(r[0].keys()))
            w.writeheader()
            w.writerows(r)
        saida.append(str(p))
    return saida


def test_tres_corridas_que_concordam_viram_linha_de_launch(tmp_path, capsys):
    """O produto do experimento nº 2 da bancada: a média vira o comando que
    sobe a pilha com o ff do dia, com a data de hoje já preenchida."""
    medir.resumo('curvatura', arqs_de_arco(tmp_path, [-0.90, -0.91, -0.92]))
    saida = capsys.readouterr().out
    assert 'curv_frente:=-0.91' in saida
    assert 'pilha.launch.py' in saida
    assert f'curv_medido_em:={datetime.date.today().isoformat()}' in saida


def test_a_re_vira_o_OUTRO_parametro(tmp_path, capsys):
    """Frente e ré são parâmetros diferentes e diferem 8x. Colar a ré em
    `curv_frente` poria o robô arcando com o viés somado por cima."""
    medir.resumo('curvatura', arqs_de_arco(tmp_path, [-0.10, -0.098, -0.102],
                                           cmd_v=-0.25))
    saida = capsys.readouterr().out
    assert 'curv_re:=-0.10' in saida
    assert 'curv_frente' not in saida


def test_dispersao_alta_nao_vira_feedforward(tmp_path, capsys):
    """Dentro do dia este robô repete em 2–3% (04-08 e 05-08) — é essa
    repetibilidade que sustenta a 013. Espalho de 20% descreve duas plantas, e
    a média entre elas não é o ff de nenhuma."""
    medir.resumo('curvatura', arqs_de_arco(tmp_path, [-0.70, -0.90, -1.05]))
    saida = capsys.readouterr().out
    assert 'não vira feedforward' in saida
    assert 'curv_frente:=' not in saida


def test_duas_corridas_nao_viram_feedforward(tmp_path, capsys):
    """O mesmo mínimo de três que o resumo já exige para a média significar
    algo — aqui ele vale como trava, não como aviso."""
    medir.resumo('curvatura', arqs_de_arco(tmp_path, [-0.90, -0.91]))
    saida = capsys.readouterr().out
    assert 'não vira feedforward' in saida
    assert 'curv_frente:=' not in saida


def test_frente_misturada_com_re_nao_vira_feedforward(tmp_path, capsys):
    """Acontece de verdade: as corridas de um dia caem todas na mesma pasta e
    o glob pega as duas famílias. A média de −0,90 com −0,10 não descreve
    sentido nenhum, e sairia parecendo medida."""
    arqs = (arqs_de_arco(tmp_path, [-0.90, -0.91], prefixo='f')
            + arqs_de_arco(tmp_path, [-0.10], cmd_v=-0.25, prefixo='r'))
    medir.resumo('curvatura', arqs)
    saida = capsys.readouterr().out
    assert 'não vira feedforward' in saida
    assert 'curv_frente:=' not in saida


def test_a_receita_so_aparece_na_curvatura(tmp_path, capsys):
    """Nenhuma outra leitura do banco vira parâmetro do compensador."""
    r = csv_dente([0.20] * 4, [0.15] * 4)
    p = tmp_path / 'zm.csv'
    with open(p, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(r[0].keys()))
        w.writeheader()
        w.writerows(r)
    medir.resumo('zona_morta_linear', [str(p)] * 3)
    assert 'pilha.launch.py' not in capsys.readouterr().out


# ------------------------------- campo de visão do LiDAR (decisão 012)

campo = _carrega('campo_de_visao')
DEP7 = math.radians(7.0)   # depressão do Mid-360


def nuvem_de_chao(altura, depressao, n=360, ate=8.0):
    """Nuvem sintética: um leque de feixes batendo num chão plano.

    Feixe acima da horizontal nunca encontra o chão e não entra — é o que
    cria a zona cega, e o teste tem de reproduzir a causa, não o número.
    """
    pts = []
    for i in range(n):
        a = 2 * math.pi * i / n
        d = altura / math.tan(depressao)
        while d <= ate:
            pts.append((d * math.cos(a), d * math.sin(a), -altura))
            d += 0.25
    return pts


def test_zona_cega_e_a_conta_da_decisao_012():
    """Mid-360 a 0,27 m com −7° não vê o chão dentro de ~2,2 m. É o dado que
    diz que obstáculo baixo e perto é invisível POR GEOMETRIA."""
    assert campo.zona_cega(0.270, DEP7) == pytest.approx(2.20, abs=0.02)


def test_zona_cega_escala_com_a_altura():
    """Ela é ~8,1x a altura de montagem — por isso medir a altura do Mid-360
    com trena é item de bancada barato e de alto retorno."""
    for h in (0.20, 0.27, 0.40):
        assert campo.zona_cega(h, DEP7) == pytest.approx(8.14 * h, rel=0.01)


def test_sensor_que_nao_olha_para_baixo_nunca_ve_o_chao():
    assert campo.zona_cega(0.27, 0.0) == float('inf')


def test_altura_minima_visivel_bate_com_a_tabela_da_012():
    """A tabela que a decisão 012 previu e a bancada confirmou em 05-08."""
    for d, esperado in ((0.5, 0.21), (1.0, 0.15), (1.5, 0.09)):
        assert campo.altura_minima_visivel(0.270, DEP7, d) == \
            pytest.approx(esperado, abs=0.01), f'a {d} m'


def test_alem_da_zona_cega_nada_e_exigido_de_altura():
    """Passada a zona cega o feixe já está no chão: objeto de qualquer altura
    aparece. A função tem de saturar em zero, não devolver negativo."""
    assert campo.altura_minima_visivel(0.270, DEP7, 3.0) == 0.0


def test_separa_chao_isola_os_pontos_impossiveis():
    """Retorno ABAIXO do plano do chão não é geometria — é artefato de
    rasância. Foi ele que sequestrou a primeira medida (2,27 m em vez de
    2,04 m) por entrar num mínimo. Tem de sair separado, não misturado."""
    pts = nuvem_de_chao(0.27, DEP7) + [(5.0, 0.0, -0.31), (6.0, 0.0, -0.33)]
    chao, acima, abaixo = campo.separa_chao(pts, 0.27)
    assert len(abaixo) == 2, 'os dois impossíveis têm de ser isolados'
    assert chao and not acima


def test_raio_do_chao_nao_e_envenenado_pelo_artefato():
    """O teste que trava o defeito de 05-08: com pontos afundados na nuvem, a
    zona cega medida NÃO pode mudar."""
    limpa = nuvem_de_chao(0.27, DEP7)
    suja = limpa + [(2.4, 0.0, -0.308), (5.2, 0.0, -0.33)]
    assert campo.raio_do_chao_visto(suja, 0.27) == \
        pytest.approx(campo.raio_do_chao_visto(limpa, 0.27), abs=1e-9)


def test_nuvem_sem_chao_devolve_none_em_vez_de_mentir():
    """Sala pequena, sensor para cima, nuvem vazia: 'não dá para medir' é
    resposta. Devolver um número aqui seria inventar zona cega."""
    assert campo.raio_do_chao_visto([(1.0, 0.0, 0.5)], 0.27) is None
    assert campo.raio_do_chao_visto([], 0.27) is None


def test_perfil_de_cegueira_marca_faixa_vazia():
    """Faixa sem retorno nenhum é diferente de faixa que vê o chão. Uma é
    'não sei', a outra é 'enxergo'. Confundir as duas é como o costmap ganha
    buraco fantasma."""
    pts = nuvem_de_chao(0.27, DEP7)
    perfil = campo.perfil_de_cegueira(pts, 0.27, [(0.5, 1.0), (3.0, 4.0)])
    assert perfil[0][2] is None and perfil[0][3] == 0, 'dentro da zona cega'
    assert perfil[1][2] == pytest.approx(0.0, abs=0.01), 'já vê o chão'


# ------------------------------------------- a régua de "ele bateu?" (05-08)

folga_mod = _carrega('folga')


def grid_com_parede(res=0.05, largura=60, altura=60, col_parede=30):
    """Sala vazia com uma parede vertical numa coluna conhecida."""
    dados = [0] * (largura * altura)
    for lin in range(altura):
        dados[lin * largura + col_parede] = 100
    return folga_mod.Grid(dados, largura, altura, res, 0.0, 0.0)


def test_folga_mede_a_distancia_ate_a_parede():
    """Parede na coluna 30 com 0,05 m/célula = x de 1,50 m. Um ponto a 1,00 m
    tem de ler 0,50 m de folga."""
    g = grid_com_parede()
    assert folga_mod.folga(g, 1.00, 1.0) == pytest.approx(0.50, abs=0.05)
    assert folga_mod.folga(g, 1.40, 1.0) == pytest.approx(0.10, abs=0.05)


def test_folga_satura_e_diz_que_saturou():
    """Longe de tudo a resposta é o alcance, não uma distância inventada.
    Quem chama precisa saber que é um piso — senão 'folga 1,5 m' viraria
    verdade num mapa onde a parede está a 40 m."""
    g = grid_com_parede()
    assert folga_mod.folga(g, 0.1, 1.0, alcance=0.6) == 0.6


def test_fora_do_mapa_nao_e_obstaculo():
    """Desconhecido não é ocupado. Tratar como ocupado faria toda corrida
    perto da borda parecer colisão — e o robô nasce a 0,2 m da parede."""
    g = grid_com_parede()
    assert folga_mod.folga(g, -5.0, -5.0, alcance=0.5) == 0.5


def test_perfil_separa_INVASAO_de_raspao():
    """A distinção que o dono precisa: 'bateu' (o corpo ocupou a célula do
    obstáculo) é diferente de 'passou raspando'. Misturar os dois faria uma
    sintonia que só raspa parecer tão ruim quanto uma que bate."""
    g = grid_com_parede()
    raio = 0.36
    # x=1,50 é a parede: 1,20 dá folga 0,30 (invade), 1,10 dá 0,40 (raspa),
    # 0,50 dá 1,00 (limpo).
    pior, inv, rasp, _ = folga_mod.perfil_de_folga(
        g, [(0.50, 1.0), (1.15, 1.0), (1.20, 1.0)], raio, margem=0.05)
    assert inv == 1, 'o ponto a ~0,33 m da parede invadiu'
    assert rasp == 1, 'o ponto a ~0,38 m raspou'
    assert pior == pytest.approx(0.33, abs=0.03)


def test_perfil_devolve_o_pior_ponto_para_ir_olhar():
    """Número sem lugar não se investiga. O pior ponto é o que se abre no
    RViz."""
    g = grid_com_parede()
    _, _, _, ponto = folga_mod.perfil_de_folga(
        g, [(0.5, 1.0), (1.45, 2.0)], 0.36)
    assert ponto == (1.45, 2.0)


def test_le_o_mapa_de_verdade_da_pista():
    """Regressão contra o PGM versionado. A pista tem perímetro de parede, e
    o ponto de largada do robô (2,0 · 5,0) é livre com folga de sobra."""
    import os as _os
    raiz = _os.path.dirname(_os.path.dirname(AQUI))
    g = folga_mod.carrega_pgm(_os.path.join(raiz, 'maps', 'pista_obstaculos.pgm'),
                              resolucao=0.05, ox=0.0, oy=0.0)
    assert g.largura > 100 and g.altura > 100
    assert folga_mod.folga(g, 2.0, 5.0) > 0.5, 'a largada tem de ser livre'
    assert folga_mod.folga(g, 0.1, 4.0) < 0.2, 'a borda oeste é parede'


# ------------------------------------------- o pré-voo (checa_pilha.py)

def _carrega_checa_pilha():
    """Carrega o pré-voo com o ROS DUBLADO.

    O módulo importa rclpy e meia dúzia de pacotes de mensagem no topo, e a
    suíte roda sem `source /opt/ros`. O que se quer testar aqui não toca ROS
    nenhum: é a contagem de processos, que é onde mora a armadilha.
    """
    import sys
    falsos = {}
    for nome in ('rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.time',
                 'lifecycle_msgs', 'lifecycle_msgs.srv', 'nav2_msgs',
                 'nav2_msgs.srv', 'nav_msgs', 'nav_msgs.msg', 'sensor_msgs',
                 'sensor_msgs.msg', 'geometry_msgs', 'geometry_msgs.msg',
                 'tf2_ros'):
        mod = types.ModuleType(nome)
        mod.__getattr__ = lambda _nome: type('Falso', (), {})
        falsos[nome] = mod
    guardados = {n: sys.modules.get(n) for n in falsos}
    sys.modules.update(falsos)
    try:
        return _carrega('checa_pilha')
    finally:
        for n, antigo in guardados.items():
            if antigo is None:
                sys.modules.pop(n, None)
            else:
                sys.modules[n] = antigo


def test_a_contagem_de_processos_nao_conta_a_si_mesma():
    """A armadilha que mordeu duas vezes neste projeto: `pgrep -c -f "gz sim"`
    devolvia 2 com ZERO Gazebos vivos, porque casava com a linha de comando do
    próprio shell que o executou (roteiro, apêndice; de novo em 07-08).

    Aqui a busca é por um padrão que só existe no comando de teste — se a
    função contasse a si mesma, este número não seria zero.
    """
    cp = _carrega_checa_pilha()
    assert cp.quantos('padrao_que_nao_existe_em_processo_nenhum_xyz') == 0
    # E ela acha o que existe de verdade: o próprio interpretador.
    assert cp.quantos('python') >= 1


def test_o_pre_voo_sabe_que_o_reflexo_nao_republica_zero():
    """Medido em 07-08: 3 s de zeros em `/auto_vel_raw` dão 0 mensagens em
    `/auto_vel`; 3 s de 0,05 m/s dão 150. Com o robô parado o silêncio do
    reflexo é a resposta CERTA, e um pré-voo que o marcasse como falha mandaria
    o dono caçar um defeito que não existe — foi o que a leitura de 06-08 fez.
    """
    cp = _carrega_checa_pilha()
    exigencia = {t: e for t, _, e in cp.CADEIA}
    assert exigencia['/auto_vel'] == 'se_nao_nulo'
    assert exigencia['/auto_vel_raw'] is True


def test_o_pre_voo_exige_hardware_so_quando_nao_ha_simulador():
    """`0 fastlio_mapping` é correto no simulador e é falha GRAVE no robô. Um
    pré-voo que dissesse ✅ para os dois seria pior que pré-voo nenhum."""
    cp = _carrega_checa_pilha()
    assert cp.UNICOS['fastlio_mapping'][1] is True
    assert cp.UNICOS['livox_ros_driver2_node'][1] is True
    assert cp.UNICOS['ros2 launch robot_motion'][1] is False
