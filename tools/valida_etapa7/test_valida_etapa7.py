#!/usr/bin/env python3
"""O julgamento do objetivo curto (etapa 6, passo 7) — sem Gazebo, sem robô.

🔴 ESTES TESTES NASCEM VERMELHOS, DE PROPÓSITO. O `tools/valida_etapa7/julga.py`
ainda NÃO existe: o contrato foi fechado antes do código (`6718c57`,
`docs/PLANO_ETAPA6_ROBO3.md` §4.6.1 e decisão 056 §4.1), e estes testes são a
primeira escrita do passo 7. Eles definem a interface que o `julga.py` vai ter
de cumprir; nenhum deles sobe pilha, nenhum deles manda objetivo.

O contrato, em três critérios simultâneos:

  1. a ação devolve `SUCCEEDED`;
  2. a pose final cai dentro do `xy_goal_tolerance` VIVO — e a pose INICIAL
     fica registrada, senão "chegou" pode ser "já estava lá";
  3. um comando **acima do patamar vivo** chega ao **consumidor final**,
     `/hoverboard_base_controller/cmd_vel`, depois do modelo de atuador.

O terceiro é o que pega a zona morta, e é onde quase toda a asneira cabe:

  · observar no `/cmd_vel_bruto` não vale — o que passa por ele ainda pode ser
    ENGOLIDO pela placa. Ele é diagnóstico;
  · `patamar = deadband_speed · escala_real · raio`, e
    `comando_efetivo = max(|v - wz·bitola/2|, |v + wz·bitola/2|)`;
  · os parâmetros são CONSULTADOS no dump de `/placa_simulada`, nunca
    redigitados — a `bitola` viva (0,32) NÃO é o default do nó (0,270), e há
    teste aqui que só passa se o julgador tiver lido o valor vivo;
  · com `modelo: ideal` a placa vira fio e o critério não prova nada;
  · a amostra tem de cair DENTRO da janela entre objetivo aceito e resultado —
    nem antes, nem no teardown (a quinta corrida mostrou o que o teardown
    escreve: decisão 057).

⚠️ LIMITE HONESTO: isto valida a PLACA SIMULADA herdada do robô 2. Não mede a
zona morta real do robô 3, e passar aqui não diz nada sobre o atuador físico.
"""
import importlib.util
import os

import pytest

AQUI = os.path.dirname(os.path.abspath(__file__))
JULGA = os.path.join(AQUI, 'julga.py')

TOPICO_FINAL = '/hoverboard_base_controller/cmd_vel'
TOPICO_DIAG = '/cmd_vel_bruto'

# Os nomes dos itens, como eles vão sair no resultado.csv do passo 7.
SUCESSO = ' 7.1 a ação devolveu SUCCEEDED'
TOLERANCIA = ' 7.2 pose final dentro do xy_goal_tolerance vivo'
NASCEU_FORA = ' 7.2 o objetivo não nasceu dentro da tolerância'
PATAMAR = ' 7.3 comando acima do patamar vivo no consumidor final'
TOPOLOGIA = ' 7.3 topologia nominal do tópico observado'
MODELO = ' 7.3 a placa está no modelo medido'
JANELA = ' 7.3 a amostra está dentro da janela do objetivo'


@pytest.fixture(scope='module')
def julga():
    """Carrega o julgador — que ainda não existe, e é por isso que tudo está
    vermelho. O vermelho aqui é UM só e é o certo: falta o código."""
    if not os.path.exists(JULGA):
        pytest.fail(
            f'{JULGA} ainda não existe — passo 7, primeiro movimento: o '
            'contrato (§4.6.1) está escrito e os testes vêm antes do código.')
    spec = importlib.util.spec_from_file_location('julga_etapa7', JULGA)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


# ─── a evidência sintética ───────────────────────────────────────────────────
#
# Os números são os VIVOS da quinta corrida (~/etapa6/20260924_132510,
# captura/parametros_normalizados.yaml, nó /placa_simulada):
#
#   modelo medido · deadband_speed 100,0 · escala_real 0,0372 · raio 0,0825
#   → patamar = 0,3069 m/s ;  bitola 0,32 (o default do nó é 0,270)

PLACA_VIVA = {'modelo': 'medido', 'deadband_speed': 100.0,
              'escala_real': 0.0372, 'raio': 0.0825, 'bitola': 0.32}
PATAMAR_VIVO = 100.0 * 0.0372 * 0.0825          # 0,3069 m/s


def _evidencia_boa():
    """Uma corrida que deve aprovar nos três critérios.

    Goal 1,0 m à frente; a pose inicial está a 1,0 m do alvo (fora da
    tolerância de 0,25) e a final a 0,08 m (dentro). A amostra cai no meio da
    janela e sai EXATAMENTE no patamar — que é o caso comum, não um caso de
    borda inventado: no modelo `medido` a placa multiplica os dois lados por
    `k = deadband_speed/mx`, então o comando que sobrevive sai no patamar.
    """
    return {
        'resultado_acao': 'SUCCEEDED',
        'goal': {'x': 1.0, 'y': 0.0},
        'pose_inicial': {'x': 0.0, 'y': 0.0},
        'pose_final': {'x': 0.92, 'y': 0.0},
        'xy_goal_tolerance': 0.25,
        'placa': dict(PLACA_VIVA),
        'grafo': {
            TOPICO_FINAL: {'publicadores': ['/placa_simulada'],
                           'assinantes': ['/hoverboard_base_controller']},
            TOPICO_DIAG: {'publicadores': ['/compensador_rumo'],
                          'assinantes': ['/placa_simulada']},
        },
        'janela': {'objetivo_aceito': 10.0, 'resultado': 35.0},
        'amostras': [
            {'topico': TOPICO_FINAL, 't': 12.0, 'v': 0.0, 'wz': 0.0},
            {'topico': TOPICO_FINAL, 't': 20.0, 'v': PATAMAR_VIVO, 'wz': 0.0},
            {'topico': TOPICO_DIAG, 't': 20.0, 'v': 0.05, 'wz': 0.0},
        ],
    }


def _so_no_diagnostico(v=1.0):
    """Comando forte, mas SÓ no `/cmd_vel_bruto`: no consumidor final nada
    passou do zero. É o retrato do comando engolido pela placa."""
    ev = _evidencia_boa()
    ev['amostras'] = [
        {'topico': TOPICO_DIAG, 't': 20.0, 'v': v, 'wz': 0.0},
        {'topico': TOPICO_FINAL, 't': 20.0, 'v': 0.0, 'wz': 0.0},
    ]
    return ev


def _com_amostra(topico, t, v, wz=0.0):
    ev = _evidencia_boa()
    ev['amostras'] = [{'topico': topico, 't': t, 'v': v, 'wz': wz}]
    return ev


# ─── o caso que aprova ───────────────────────────────────────────────────────

def test_corrida_boa_aprova_os_tres_criterios(julga):
    itens = julga.avalia(_evidencia_boa())
    assert all(ok for ok, _ in itens.values()), itens


# ─── 7.1 SUCCEEDED ───────────────────────────────────────────────────────────

def test_acao_abortada_reprova(julga):
    ev = _evidencia_boa()
    ev['resultado_acao'] = 'ABORTED'
    assert not julga.avalia(ev)[SUCESSO][0]


def test_acao_cancelada_reprova(julga):
    """CANCELED não é sucesso, e o teto de 60 s simulados cancela."""
    ev = _evidencia_boa()
    ev['resultado_acao'] = 'CANCELED'
    assert not julga.avalia(ev)[SUCESSO][0]


# ─── 7.2 tolerância final e pose inicial ─────────────────────────────────────

def test_pose_final_fora_da_tolerancia_reprova(julga):
    ev = _evidencia_boa()
    ev['pose_final'] = {'x': 0.60, 'y': 0.0}      # 0,40 m > 0,25
    assert not julga.avalia(ev)[TOLERANCIA][0]


def test_a_tolerancia_e_a_viva_nao_um_numero_redigitado(julga):
    """Mesma pose final, tolerância viva menor: tem de reprovar. Se o julgador
    tivesse 0,25 escrito por dentro, este caso passaria."""
    ev = _evidencia_boa()
    ev['xy_goal_tolerance'] = 0.05                # 0,08 m de erro > 0,05
    assert not julga.avalia(ev)[TOLERANCIA][0]


def test_pose_final_desviada_so_em_y_reprova(julga):
    """🔴 O caso que um julgador de uma dimensão só deixa passar: `x` cravado no
    alvo e 0,30 m de desvio lateral. Medindo só `|x|`, o erro dá ZERO e a
    corrida aprova com o robô 30 cm fora."""
    ev = _evidencia_boa()
    ev['pose_final'] = {'x': 1.0, 'y': 0.30}      # distância 0,30 > 0,25
    assert not julga.avalia(ev)[TOLERANCIA][0]


def test_pose_final_em_diagonal_dentro_do_raio_aprova(julga):
    """O espelho: erro nas duas coordenadas, mas a distância cabe — 0,212 no
    raio. Quem reprovaria corrida boa aqui é SOMA (Manhattan): 0,15 + 0,15 =
    0,30 > 0,25. Coordenada a coordenada isto passaria, como passa no raio."""
    ev = _evidencia_boa()
    ev['pose_final'] = {'x': 0.85, 'y': 0.15}     # distância 0,2121
    assert julga.avalia(ev)[TOLERANCIA][0]


def test_objetivo_que_nasceu_dentro_da_tolerancia_reprova(julga):
    """"Chegou" que era "já estava lá" — o modo de falha mais provável com um
    alvo curto, e o motivo de a pose inicial ser evidência obrigatória."""
    ev = _evidencia_boa()
    ev['pose_inicial'] = {'x': 0.90, 'y': 0.0}    # 0,10 m do goal < 0,25
    assert not julga.avalia(ev)[NASCEU_FORA][0]


def test_pose_inicial_desviada_so_em_y_esta_fora_e_aprova(julga):
    """Mesma armadilha, do outro lado, e o sentido é este: partida em
    `x` = 1,0 (o `x` do alvo) com 0,30 m de desvio lateral.

    🔴 O julgador 1D lê `|Δx| = 0` e conclui que o objetivo NASCEU DENTRO —
    reprovando corrida boa. Pela distância XY a partida está a 0,30 m, fora da
    tolerância de 0,25, e o item APROVA. É a versão que separa os dois
    cálculos: com `y` = 0,10 os dois reprovariam, e o teste não provaria nada.
    """
    ev = _evidencia_boa()
    ev['pose_inicial'] = {'x': 1.0, 'y': 0.30}    # distância 0,30 > 0,25
    assert julga.avalia(ev)[NASCEU_FORA][0]


def test_a_caixa_nao_e_o_raio_nas_duas_poses(julga):
    """🔴 A armadilha da CAIXA: 0,20 em `x` E 0,20 em `y`.

    Cada coordenada isolada cabe na tolerância (0,20 < 0,25), mas a distância é
    0,2828 — está FORA. Um julgador que testasse `|Δx| < tol and |Δy| < tol`
    aprovaria a chegada (robô 28 cm do alvo) e, na partida, diria que o objetivo
    nasceu dentro. Os dois vereditos ficam invertidos pela mesma conta errada,
    então o caso cobra as duas pontas de uma vez.
    """
    ev = _evidencia_boa()
    ev['pose_final'] = {'x': 0.80, 'y': 0.20}     # distância 0,2828 > 0,25
    assert not julga.avalia(ev)[TOLERANCIA][0]

    ev = _evidencia_boa()
    ev['pose_inicial'] = {'x': 0.80, 'y': 0.20}   # distância 0,2828 > 0,25
    assert julga.avalia(ev)[NASCEU_FORA][0]


def test_pose_inicial_em_diagonal_fora_do_raio_aprova(julga):
    ev = _evidencia_boa()
    ev['pose_inicial'] = {'x': 0.80, 'y': 0.80}   # distância 0,8246 > 0,25
    assert julga.avalia(ev)[NASCEU_FORA][0]


def test_pose_inicial_ausente_reprova(julga):
    """Sem o registro não se PROVA que o objetivo não nasceu dentro; ausência
    de evidência não vira aprovação."""
    ev = _evidencia_boa()
    ev.pop('pose_inicial')
    assert not julga.avalia(ev)[NASCEU_FORA][0]


# ─── 7.3 as duas bordas do patamar ───────────────────────────────────────────

def test_comando_exatamente_no_patamar_aprova(julga):
    """A borda de cima, e ela é o caso NORMAL: no modelo medido a saída cai
    exatamente no patamar."""
    ev = _com_amostra(TOPICO_FINAL, 20.0, PATAMAR_VIVO)
    assert julga.avalia(ev)[PATAMAR][0]


def test_comando_logo_abaixo_do_patamar_reprova(julga):
    """A borda de baixo: 2 mm/s abaixo. É o robô parado em silêncio — o critério
    existe para pegar exatamente isto."""
    ev = _com_amostra(TOPICO_FINAL, 20.0, PATAMAR_VIVO - 2e-3)
    assert not julga.avalia(ev)[PATAMAR][0]


def test_diferenca_de_float_no_patamar_ainda_aprova(julga):
    """A tolerância de 1e-3 não é cosmética: a saída sai exatamente no patamar,
    e comparar float no fio da navalha reprovaria corrida boa."""
    ev = _com_amostra(TOPICO_FINAL, 20.0, PATAMAR_VIVO - 5e-4)
    assert julga.avalia(ev)[PATAMAR][0]


def test_comando_nulo_reprova(julga):
    ev = _com_amostra(TOPICO_FINAL, 20.0, 0.0)
    assert not julga.avalia(ev)[PATAMAR][0]


def test_giro_puro_conta_pela_roda_mais_rapida(julga):
    """`v` zero e `wz` alto é comando efetivo: a roda anda. Julgar por `v`
    sozinho deixaria passar como nulo."""
    ev = _com_amostra(TOPICO_FINAL, 20.0, 0.0, wz=2.5)   # 0,40 m/s de borda
    assert julga.avalia(ev)[PATAMAR][0]


def test_comando_negativo_conta_pelo_modulo(julga):
    ev = _com_amostra(TOPICO_FINAL, 20.0, -PATAMAR_VIVO)
    assert julga.avalia(ev)[PATAMAR][0]


# ─── 7.3 o /cmd_vel_bruto NÃO satisfaz o contrato ────────────────────────────

def test_so_o_cmd_vel_bruto_nao_satisfaz(julga):
    """🔴 O centro do contrato. Comando forte no diagnóstico e zero no
    consumidor final é o comando ENGOLIDO pela placa — e era exatamente o que o
    critério antigo ("não nulo", observado antes do modelo) aprovaria."""
    assert not julga.avalia(_so_no_diagnostico())[PATAMAR][0]


def test_amostra_do_diagnostico_nao_entra_na_conta(julga):
    """Nem por engano de topologia: se o julgador somar os dois tópicos, este
    caso (só diagnóstico, nada no final) passa."""
    itens = julga.avalia(_so_no_diagnostico(v=5.0))
    assert not itens[PATAMAR][0]


# ─── 7.3 topologia nominal ───────────────────────────────────────────────────

def test_publicador_diferente_da_placa_reprova(julga):
    """Se quem publica no consumidor final não é a `placa_simulada`, o comando
    não atravessou o modelo de atuador — e o critério não prova o que diz."""
    ev = _evidencia_boa()
    ev['grafo'][TOPICO_FINAL]['publicadores'] = ['/compensador_rumo']
    assert not julga.avalia(ev)[TOPOLOGIA][0]


def test_consumidor_ausente_reprova(julga):
    """Publicar para ninguém não é "chegar ao controlador"."""
    ev = _evidencia_boa()
    ev['grafo'][TOPICO_FINAL]['assinantes'] = []
    assert not julga.avalia(ev)[TOPOLOGIA][0]


def test_dois_publicadores_no_topico_final_reprova(julga):
    """Com dois, não se sabe de quem é a amostra que aprovou."""
    ev = _evidencia_boa()
    ev['grafo'][TOPICO_FINAL]['publicadores'] = ['/placa_simulada', '/teleop']
    assert not julga.avalia(ev)[TOPOLOGIA][0]


# ─── 7.3 modelo == medido ────────────────────────────────────────────────────

def test_modelo_ideal_reprova(julga):
    """Com `modelo: ideal` o nó vira fio: passa qualquer coisa, e o critério
    não prova nada sobre zona morta."""
    ev = _evidencia_boa()
    ev['placa']['modelo'] = 'ideal'
    assert not julga.avalia(ev)[MODELO][0]


def test_modelo_cru_reprova(julga):
    ev = _evidencia_boa()
    ev['placa']['modelo'] = 'cru'
    assert not julga.avalia(ev)[MODELO][0]


# ─── 7.3 a janela temporal ───────────────────────────────────────────────────

def test_amostra_antes_do_objetivo_reprova(julga):
    """Comando anterior ao objetivo aceito é de outra coisa — teleop, reflexo,
    o que for. Não prova que a navegação mandou."""
    ev = _com_amostra(TOPICO_FINAL, 9.0, PATAMAR_VIVO)
    assert not julga.avalia(ev)[JANELA][0]


def test_amostra_do_teardown_reprova(julga):
    """Depois do resultado é teardown — e a quinta corrida mostrou o que o
    teardown escreve (decisão 057)."""
    ev = _com_amostra(TOPICO_FINAL, 36.0, PATAMAR_VIVO)
    assert not julga.avalia(ev)[JANELA][0]


def test_amostra_fora_da_janela_nao_aprova_o_patamar(julga):
    """A janela não é um item decorativo ao lado: amostra fora dela não pode
    aprovar o 7.3 por outro caminho."""
    ev = _com_amostra(TOPICO_FINAL, 36.0, PATAMAR_VIVO)
    assert not julga.avalia(ev)[PATAMAR][0]


def test_amostra_nas_bordas_da_janela_aprova(julga):
    """Inclusiva nas duas pontas (decidido em 24-09): o instante do aceite e o
    do resultado são da corrida — `objetivo_aceito <= t <= resultado`."""
    for t in (10.0, 35.0):
        ev = _com_amostra(TOPICO_FINAL, t, PATAMAR_VIVO)
        assert julga.avalia(ev)[JANELA][0], t


def test_o_teardown_comeca_estritamente_depois_do_resultado(julga):
    """1 ms depois do resultado já é teardown. Sem folga temporal inventada: a
    janela é a janela, e tolerância artificial aqui só serviria para salvar
    corrida ruim."""
    ev = _com_amostra(TOPICO_FINAL, 35.001, PATAMAR_VIVO)
    assert not julga.avalia(ev)[JANELA][0]


# ─── 7.3 os parâmetros são os VIVOS, nunca redigitados ───────────────────────

def test_patamar_segue_o_deadband_vivo(julga):
    """Dobrar o `deadband_speed` dobra o patamar: a mesma amostra que aprovava
    passa a reprovar. Se o 0,3069 estivesse escrito no julgador, isto passaria.
    """
    ev = _evidencia_boa()
    ev['placa']['deadband_speed'] = 200.0         # patamar 0,6138
    assert not julga.avalia(ev)[PATAMAR][0]


def test_patamar_segue_o_raio_vivo(julga):
    ev = _evidencia_boa()
    ev['placa']['raio'] = 0.165                   # patamar 0,6138
    assert not julga.avalia(ev)[PATAMAR][0]


def test_a_bitola_usada_e_a_viva_e_nao_o_default_do_no(julga):
    """🔴 O caso que o default estraga. Giro puro com `wz` 2,0:

        bitola VIVA 0,32   → 2,0 · 0,16  = 0,320 m/s  ≥ 0,3069 → APROVA
        default do nó 0,270 → 2,0 · 0,135 = 0,270 m/s  < 0,3069 → reprovaria

    Um julgador que lesse o default (ou redigitasse 0,270) reprovaria corrida
    boa, e o erro seria invisível: o número tem a cara certa.
    """
    ev = _com_amostra(TOPICO_FINAL, 20.0, 0.0, wz=2.0)
    assert julga.avalia(ev)[PATAMAR][0]


def test_bitola_menor_muda_o_veredito_do_mesmo_comando(julga):
    """O espelho do anterior: com a bitola do robô 2 viva no dump, o MESMO
    comando reprova. É a prova de que a bitola entra na conta pelo dump."""
    ev = _com_amostra(TOPICO_FINAL, 20.0, 0.0, wz=2.0)
    ev['placa']['bitola'] = 0.270
    assert not julga.avalia(ev)[PATAMAR][0]


@pytest.mark.parametrize('campo, item', [
    ('modelo', MODELO),
    ('deadband_speed', PATAMAR),
    ('escala_real', PATAMAR),
    ('raio', PATAMAR),
    ('bitola', PATAMAR),
])
def test_campo_vivo_ausente_reprova_em_vez_de_assumir(julga, campo, item):
    """Faltando QUALQUER um dos cinco, o julgador NÃO pode cair num default: ele
    reprova e diz o que faltou.

    🔴 É a mesma armadilha da `bitola` vista de frente. O nó tem default para
    todos eles (`placa_simulada.py:144-178`), então um julgador distraído
    "completa" o dump e devolve veredito com número que não é o da corrida —
    erro invisível, porque o resultado tem a cara certa. Ausência de evidência
    reprova; nunca vira aprovação nem número assumido.
    """
    ev = _evidencia_boa()
    ev['placa'].pop(campo)
    assert not julga.avalia(ev)[item][0]
