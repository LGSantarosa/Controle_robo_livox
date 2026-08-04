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
