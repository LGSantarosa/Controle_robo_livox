"""Testes da lei de movimentação (decisão 005).

Cada teste trava uma propriedade MEDIDA no simulador, não um detalhe de
implementação. Se um destes cair, é a decisão 005 que está sendo contrariada.
"""
import math

import pytest

from robot_motion.lei_de_rumo import (
    GatilhoDeGiro,
    ajusta_para_zona_morta,
    comando,
    comando_de_re,
    erro_antecipado,
    linear_de_avanco,
    norm_ang,
    piso_de_linear,
    teto_de_pivo,
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
    """Nenhuma roda dentro da banda morta — checado em MÓDULO.

    A roda interna girando ao contrário está tão viva quanto girando pra
    frente: é assim que o robô pivota. Checar só o lado positivo foi o defeito
    que proibiu o pivô por aritmética.
    """
    for erro in [0.05, 0.5, 1.57, 2.5, 3.14]:
        v, wz = comando(erro, V_MAX, A_DEC, WZ_MAX, ZONA_MORTA, BITOLA,
                        MARGEM, TOL)
        interna = v - abs(wz) * BITOLA / 2.0
        externa = v + abs(wz) * BITOLA / 2.0
        assert abs(externa) >= ZONA_MORTA
        assert abs(interna) >= ZONA_MORTA, (
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


# ------------------------------------------------- pivô (a saída por baixo)

def test_pivo_existe_quando_o_giro_sozinho_tira_a_roda_da_banda():
    """A roda interna girando ao contrário também está fora da zona morta.

    Ignorar essa saída proibiu o robô de virar no lugar por aritmética, e o
    obrigou a arcos enormes — chegando a orbitar pontos próximos sem alcançá-los.
    """
    # meia bitola de giro (1.0*0.20/2 = 0.10) já supera o limiar (0.05+0.02),
    # então v=0 é válido: a roda interna gira para trás, fora da banda.
    v, wz = ajusta_para_zona_morta(0.0, 1.0, 0.05, BITOLA, 0.02, v_teto=0.5)
    assert wz == pytest.approx(1.0), 'o giro não pode ser cortado'
    assert v == pytest.approx(0.0), 'parado e girando é válido — é o pivô'
    assert abs(v - abs(wz) * BITOLA / 2.0) >= 0.05

    # e o teto do pivô é a maior linear que ainda mantém a roda interna
    # girando ao contrário
    limite = teto_de_pivo(0.05, 1.0, BITOLA, 0.02)
    assert limite == pytest.approx(0.10 - 0.07)
    v2, _ = ajusta_para_zona_morta(0.05, 1.0, 0.05, BITOLA, 0.02, v_teto=0.5)
    assert v2 == pytest.approx(limite), 'dentro da banda, sai pelo lado mais perto'


def test_sem_zona_morta_a_lei_nao_e_tocada():
    """Zona morta zero (o simulador) não pode inventar piso nenhum."""
    for erro in [0.1, 1.0, 2.0, 3.0]:
        v, wz = comando(erro, V_MAX, A_DEC, WZ_MAX, 0.0, BITOLA, 0.0, TOL)
        assert v == pytest.approx(linear_de_avanco(erro, V_MAX))
        assert wz == pytest.approx(wz_de_frenagem(erro, A_DEC, WZ_MAX))


def test_erro_grande_prefere_pivotar_a_acelerar():
    """Com o rumo muito torto, avançar é o que menos interessa."""
    v, wz = comando(3.0, V_MAX, A_DEC, WZ_MAX, 0.05, BITOLA, 0.02, TOL)
    assert abs(wz) > 0.5
    assert v < 0.1, f'deveria quase parar para virar, mas v={v:.3f}'


def test_erro_pequeno_nao_pivota():
    """Giro pequeno não tira a roda da banda: aí a saída é acelerar."""
    v, wz = comando(0.1, V_MAX, A_DEC, WZ_MAX, 0.05, BITOLA, 0.02, TOL)
    assert v > 0.1
    assert abs(v - abs(wz) * BITOLA / 2.0) >= 0.05


def test_raio_de_curva_encolhe_com_o_pivo_liberado():
    """O defeito visível era o raio: v/wz grande demais para pegar pontos perto.

    Com a saída por baixo, o mesmo erro de rumo passa a produzir raio bem menor.
    """
    def raio(zona_morta):
        # erro grande: é onde a linear desejada é baixa e o pivô pode ganhar
        v, wz = comando(2.5, V_MAX, A_DEC, WZ_MAX, zona_morta, BITOLA, 0.02, TOL)
        return v / abs(wz)

    # zona morta pequena: pivota, raio ~zero. Grande: obrigado a arco largo.
    assert raio(0.05) < 0.05, 'com zona morta pequena tem que pivotar'
    assert raio(0.15) > 0.20, 'com zona morta grande o arco é inevitável'


def test_pivo_disponivel_depende_da_bitola():
    """A bitola decide se o robô consegue virar no próprio eixo.

    Medido no simulador: bitola 0,20 com zona morta 0,10 exige 1,3 rad/s para
    pivotar, contra um teto de 1,0 — impossível, e pontos próximos ficam
    inalcançáveis (o robô os orbita). Supunha-se que a bitola real (0,32)
    resolvesse o mesmo caso, e por isso medi-la era urgente.

    MEDIDA em 2026-07-29: 0,270 — entre os dois palpites, e ela NÃO resolveu
    a pergunta. Com zona morta 0,10 o pivô existe, mas por 4% de folga; com
    0,15 ele some. Quem decide agora é a ZONA MORTA, ainda não medida.
    """
    from robot_motion.lei_de_rumo import pivo_disponivel
    assert not pivo_disponivel(zona_morta=0.10, bitola=0.20, margem=0.03,
                               wz_max=1.0)
    assert pivo_disponivel(zona_morta=0.10, bitola=0.32, margem=0.03,
                           wz_max=1.0)
    # sem zona morta, sempre disponível
    assert pivo_disponivel(zona_morta=0.0, bitola=0.20, margem=0.0, wz_max=1.0)

    # A bitola MEDIDA, nos dois cenários de zona morta que estão em jogo.
    # Trava o resultado: o pivô depende da zona morta, não mais da bitola.
    assert pivo_disponivel(zona_morta=0.10, bitola=0.270, margem=0.03,
                           wz_max=1.0), 'com zm 0,10 o pivô existe (folga 4%)'
    assert not pivo_disponivel(zona_morta=0.15, bitola=0.270, margem=0.05,
                               wz_max=1.0), 'com zm 0,15 o pivô some'


# ---------------------------------------------------------------- a ré

def test_re_nunca_gira():
    """Ré é RETA. Decisão do dono, e ela protege o que não sabemos.

    Curvar de ré inverte a geometria da direção e coloca a boba na frente
    enquanto ela ainda decide para onde apontar. Como manobra de espaço, a ré
    não precisa curvar: recuar já abre o raio de curva de que o alvo precisa.
    """
    for v in [-0.05, -0.2, -0.5, -5.0]:
        assert comando_de_re(v, ZONA_MORTA, MARGEM, V_MAX)[1] == 0.0


def test_re_nunca_fica_abaixo_da_zona_morta():
    """Ré fraca demais é o BO-3 outra vez, agora andando para trás.

    Pedir 0,02 m/s de ré com zona morta de 0,15 é pedir robô parado em
    silêncio no meio de uma manobra — pior que não manobrar, porque o log
    fica limpo.
    """
    for pedida in [-0.001, -0.05, -0.1]:
        v, _ = comando_de_re(pedida, ZONA_MORTA, MARGEM, V_MAX)
        assert abs(v) >= ZONA_MORTA + MARGEM


def test_re_respeita_o_teto_de_velocidade():
    v, _ = comando_de_re(-10.0, ZONA_MORTA, MARGEM, V_MAX)
    assert abs(v) <= V_MAX


def test_re_e_sempre_negativa():
    for pedida in [-0.001, -0.2, -10.0]:
        assert comando_de_re(pedida, ZONA_MORTA, MARGEM, V_MAX)[0] < 0.0


def test_re_recusa_velocidade_para_frente():
    """A ré é modo explícito, não um sinal que se descobre no meio da lei."""
    for pedida in [0.0, 0.3]:
        with pytest.raises(ValueError):
            comando_de_re(pedida, ZONA_MORTA, MARGEM, V_MAX)


# ------------------------------- A HISTERESE DO GIRO (14-08, corridas A a F)
#
# Um limiar só (`tolerancia_rumo` 0,02 rad = 1,15°) contra um atuador cujo
# MENOR golpe é de 14 a 28° (medido na 037, com freio): a lei pedia correção
# 12 a 24x mais fina do que a placa sabe entregar, e o resultado era
# ciclo-limite. O período medido em cinco corridas ficou em 2,0-2,8 s e NÃO
# mudou quando lei, ganho, mira, pivô e frame mudaram — assinatura de planta
# (relé com tempo morto L oscila em ~4L; `atraso_desliga` = 0,52 s -> 2,08 s).

ENTRA = math.radians(16.0)
SAI = math.radians(5.0)


def test_nao_reengata_por_ruido_abaixo_do_gatilho():
    """O defeito: com 1,15° de gate, 2° de ruído já mandava girar."""
    g = GatilhoDeGiro(entra=ENTRA, sai=SAI)
    for erro_deg in (0.5, 2.0, 8.0, 15.0):
        assert g.deve_girar(math.radians(erro_deg)) is False


def test_entra_a_girar_com_erro_da_ordem_do_golpe_da_maquina():
    g = GatilhoDeGiro(entra=ENTRA, sai=SAI)
    assert g.deve_girar(math.radians(17.0)) is True


def test_uma_vez_GIRANDO_ele_aperta_ate_o_limiar_de_SAIDA():
    """Entrada larga, saída apertada — é o que faz o erro assentado ser <=5°
    sem que o reengate vire caça a ruído."""
    g = GatilhoDeGiro(entra=ENTRA, sai=SAI)
    g.deve_girar(math.radians(30.0))
    for erro_deg in (20.0, 12.0, 8.0, 6.0):        # ainda acima da saída
        assert g.deve_girar(math.radians(erro_deg)) is True
    assert g.deve_girar(math.radians(4.0)) is False


def test_o_sinal_do_erro_nao_importa_so_o_modulo():
    g = GatilhoDeGiro(entra=ENTRA, sai=SAI)
    assert g.deve_girar(math.radians(-17.0)) is True


def test_sem_histerese_a_construcao_e_RECUSADA():
    with pytest.raises(ValueError):
        GatilhoDeGiro(entra=SAI, sai=SAI)


def test_comando_com_girar_FALSE_nao_pede_giro():
    v, wz = comando(math.radians(10.0), v_max=0.4, a_dec=0.3, wz_max=1.0,
                    zona_morta=0.02, bitola=0.27, margem_piso=0.05,
                    tolerancia=SAI, girar=False)
    assert wz == 0.0 and v == pytest.approx(0.4)


def test_comando_com_girar_TRUE_pede_giro_mesmo_com_erro_pequeno():
    """É o outro lado da histerese: já girando, ele fecha até a saída."""
    _, wz = comando(math.radians(6.0), v_max=0.4, a_dec=0.3, wz_max=1.0,
                    zona_morta=0.02, bitola=0.27, margem_piso=0.05,
                    tolerancia=SAI, girar=True)
    assert wz != 0.0


def test_sem_o_argumento_a_lei_ANTIGA_e_preservada():
    """`girar=None` = um limiar só. É o que trava os testes da 005."""
    v, wz = comando(math.radians(0.5), v_max=0.4, a_dec=0.3, wz_max=1.0,
                    zona_morta=0.02, bitola=0.27, margem_piso=0.05,
                    tolerancia=math.radians(1.15))
    assert wz == 0.0 and v == pytest.approx(0.4)


# ── antecipação da retenção da placa (20-08, medido no robô) ─────────────────

def test_retencao_zero_e_identidade():
    """Quem não mediu a retenção não desconta nada — o default não muda a lei."""
    for e in (0.0, 0.3, -1.2, 3.0):
        assert erro_antecipado(e, 1.0, 0.0) == pytest.approx(norm_ang(e))


def test_desconta_o_giro_que_ja_esta_na_fila():
    """1,0 rad/s x 0,52 s = 0,52 rad: o erro que sobra é o de agora menos isso.

    É a conta que fecha com o medido no robô — 29,9° de sobra depois do erro
    zerar, contra 29,8° previstos.
    """
    assert erro_antecipado(0.52, 1.0, 0.52) == pytest.approx(0.0, abs=1e-9)
    assert erro_antecipado(1.0, 1.0, 0.52) == pytest.approx(0.48, abs=1e-9)


def test_girando_para_o_lado_certo_o_comando_CAI_a_zero():
    """O ponto todo: o comando decai sozinho, sem pulso contrário.

    Com o robô girando na direção do alvo, o erro previsto encolhe e a
    `wz_de_frenagem` — que é contínua — vai junto até zero.
    """
    a_dec, wz_max = 0.3, 1.0
    anterior = None
    for wz_real in (0.0, 0.3, 0.6, 0.9, 1.0):
        e = erro_antecipado(0.52, wz_real, 0.52)
        wz = wz_de_frenagem(e, a_dec, wz_max)
        if anterior is not None:
            assert wz <= anterior + 1e-9, 'o comando tem de CAIR, nunca subir'
        anterior = wz
    assert wz == pytest.approx(0.0, abs=1e-9)


def test_se_passar_do_alvo_o_contra_giro_e_PROPORCIONAL_nao_pulso():
    """A diferença para o `FreioDeGiro`, e é o pedido do dono.

    Girando muito mais do que o erro pede, o comando inverte — mas suave, e
    bem abaixo do teto. Contra-torque cheio seria `wz_max`.
    """
    e = erro_antecipado(0.10, 1.0, 0.52)      # sobra ~0,42 rad para o outro lado
    wz = wz_de_frenagem(e, 0.3, 1.0)
    assert wz < 0.0, 'tem de inverter'
    assert abs(wz) < 0.6, f'contra-giro alto demais ({abs(wz):.2f}) — virou pulso'


def test_teto_impede_que_um_pico_de_wz_vire_instabilidade():
    """O pico medido no robô foi 4,16 rad/s; sem teto isso descontaria 124°."""
    e = erro_antecipado(0.0, 4.16, 0.52)
    assert abs(e) <= math.radians(60.0) + 1e-9


def test_wz_nao_finito_nao_envenena_a_lei():
    assert erro_antecipado(0.4, float('nan'), 0.52) == pytest.approx(0.4)
