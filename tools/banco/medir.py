#!/usr/bin/env python3
"""Lê os CSVs do banco e cospe os NÚMEROS do robô.

    python3 medir.py zona_morta_linear zm_lin.csv
    python3 medir.py degrau_giro deg_10.csv
    python3 medir.py curva curva_v04_wz05.csv
    python3 medir.py --fonte roda zona_morta_giro zm_giro.csv

Cada leitura devolve o parâmetro que vai para o YAML da movimentação, com a
evidência ao lado — não um veredito solto.

`--fonte` tem de repetir o que o `ensaio.py` usou naquela corrida (`lio` é o
padrão dos dois). Não é preferência de leitura: é a mesma escolha, do outro
lado do CSV.
"""
import csv
import math
import os
import sys

LIMIAR_PARADO = 0.02   # m/s e rad/s abaixo disso é considerado imóvel


def norm(a):
    """Ângulo em (-pi, pi] — diferença de rumo não pode dar 359°."""
    return math.atan2(math.sin(a), math.cos(a))


def _num(v):
    """Número quando dá, texto quando não dá. A coluna `fase` do dente de serra
    é texto ('sobe'/'desce'/'pausa') e converter tudo em float estourava aqui."""
    if v in ('', None):
        return None
    try:
        return float(v)
    except ValueError:
        return v


def le(p):
    return [{k: _num(v) for k, v in l.items()} for l in csv.DictReader(open(p))]


def espalhamento(vs):
    """Média e faixa. Um número sozinho não é medida — é uma amostra."""
    m = sum(vs) / len(vs)
    if len(vs) == 1:
        return m, 0.0, m, m
    dp = math.sqrt(sum((v - m) ** 2 for v in vs) / (len(vs) - 1))
    return m, dp, min(vs), max(vs)


def zona_morta(r, campo_cmd, campo_med, unidade, escala=1.0):
    """Limiares de SAÍDA e de QUEDA, um par por dente do dente de serra.

    `escala` converte o campo medido para a grandeza em que LIMIAR_PARADO faz
    sentido — velocidade de BORDA DE RODA. Na reta é 1,0 (o campo já é m/s de
    roda); no giro é L/2, porque comparar rad/s cru contra o mesmo limiar torna
    o ensaio de giro ~7x mais sensível e faz a leitura achar rastejo de eixo
    onde o robô não saiu do lugar (31-07). Tem de ser a MESMA conversão que o
    ensaio.py usou para virar o dente.

    Duas medidas diferentes, e o projeto precisa das duas:

      · SAÍDA (subindo, partindo do repouso) — atrito estático. É o número do
        BO-3: abaixo dele o robô fica plantado sem erro nenhum, e é ele que
        decide se este robô consegue pivotar.
      · QUEDA (descendo, já andando) — atrito dinâmico, sempre menor. É o que
        o piso de velocidade do seguidor precisa, porque manter andando custa
        menos que arrancar. Usar a saída no lugar da queda deixa o piso alto
        demais e o robô mais rápido do que precisa perto do alvo.

    Cada dente é uma amostra independente (rotor parado em outro ponto), e os
    dentes alternam de sentido. Por isso a leitura sai com FAIXA: um limiar de
    atrito não é um número, é uma distribuição, e a decisão do pivô se joga
    dentro dela.
    """
    def med(l):
        return abs(l[campo_med]) * escala

    tem_dente = r and r[0].get('dente') is not None
    if not tem_dente:
        return _zona_morta_rampa_unica(r, campo_cmd, campo_med, unidade, escala)

    # Segmenta por (dente, fase): o ensaio já marcou onde cada rampa começa.
    # Rampa interrompida no meio (teto de tempo, trava de espaço) não vale como
    # "não saiu do lugar" — ela não chegou a tentar. Sem este piso, uma corrida
    # cortada no fim vira um falso negativo com cara de resultado.
    pico = max(abs(l[campo_cmd]) for l in r) if r else 0.0
    MINIMO_TENTATIVA = 0.5 * pico

    saidas, quedas, sem_saida, parciais = [], [], [], []
    for d in sorted({int(l['dente']) for l in r}):
        sobe = [l for l in r if int(l['dente']) == d and l['fase'] == 'sobe']
        desce = [l for l in r if int(l['dente']) == d and l['fase'] == 'desce']

        achou = None
        for i, l in enumerate(sobe):
            if med(l) > LIMIAR_PARADO:
                seg = sobe[i:i + 8]
                if len(seg) == 8 and all(med(k) > LIMIAR_PARADO / 2
                                         for k in seg):
                    achou = l
                    break
        if achou:
            saidas.append((d, abs(achou[campo_cmd]), achou['t']))
        elif sobe:
            tentou = max(abs(l[campo_cmd]) for l in sobe)
            if tentou >= MINIMO_TENTATIVA:
                sem_saida.append((d, tentou))
            else:
                parciais.append(d)

        # A queda é a ÚLTIMA amostra ainda em movimento da descida: abaixo dela
        # o comando já não sustenta o movimento.
        movendo = [l for l in desce if med(l) > LIMIAR_PARADO]
        if movendo and achou:
            quedas.append((d, abs(movendo[-1][campo_cmd]), movendo[-1]['t']))

    if not saidas:
        print('  ROBÔ NÃO SAIU DO LUGAR EM DENTE NENHUM.')
        if sem_saida:
            print(f'    comando máximo tentado: '
                  f'{max(v for _, v in sem_saida):.3f} {unidade}')
        print('    -> refazer com --rampa-ate maior')
        return None

    m, dp, lo, hi = espalhamento([v for _, v, _ in saidas])
    print(f'  ZONA MORTA (saída, do repouso) = {m:.3f} {unidade}')
    print(f'    {len(saidas)} dente(s): ' +
          '  '.join(f'#{d}={v:.3f}' for d, v, _ in saidas))
    print(f'    faixa {lo:.3f} a {hi:.3f}  (desvio {dp:.3f})')

    # O dente #0 parte de um repouso LONGO; os demais, da pausa curta entre
    # dentes. Atrito estático cresce com o tempo parado, então os dois são
    # condições físicas diferentes e a média dos quatro mistura as duas — bem
    # no número do BO-3, que é "o robô estava parado e mandaram andar".
    # Não corrijo a média: separo e mostro, porque qual dos dois usar depende
    # de quanto tempo o robô fica parado na operação real.
    if len(saidas) >= 3:
        primeiro = saidas[0][1]
        resto = sorted(v for _, v, _ in saidas[1:])
        mediana = resto[len(resto) // 2]
        if mediana > 0 and abs(primeiro - mediana) / mediana > 0.30:
            print(f'    [!] o dente #0 ({primeiro:.3f}) destoa dos outros '
                  f'(mediana {mediana:.3f}).')
            print(f'        Ele é o único que parte de repouso LONGO; os demais '
                  f'partem da pausa')
            print(f'        curta entre dentes, e atrito estático cresce com o '
                  f'tempo parado.')
            print(f'        Para arrancar depois de o robô ficar parado, use '
                  f'{primeiro:.3f};')
            print(f'        para arrancar em manobra encadeada, {mediana:.3f}.')
    if sem_saida:
        print(f'    [atenção] {len(sem_saida)} dente(s) NÃO saíram do lugar até '
              f'o teto da rampa —')
        print(f'              a saída é maior que {max(v for _, v in sem_saida):.3f} '
              f'{unidade} em pelo menos um sentido.')

    if quedas:
        mq, dpq, loq, hiq = espalhamento([v for _, v, _ in quedas])
        print(f'  LIMIAR DE QUEDA (já andando) = {mq:.3f} {unidade}')
        print(f'    {len(quedas)} dente(s): ' +
              '  '.join(f'#{d}={v:.3f}' for d, v, _ in quedas))
        print(f'    faixa {loq:.3f} a {hiq:.3f}  (desvio {dpq:.3f})')
        if mq < m:
            print(f'    -> arrancar custa {m - mq:.3f} {unidade} a mais que '
                  f'manter andando ({100 * (m - mq) / m:.0f}%)')
        else:
            print('    -> a queda saiu MAIOR que a saída, o que é fisicamente '
                  'esquisito:')
            print('       atrito estático deveria ser o maior dos dois. '
                  'Suspeitar da rampa')
            print('       rápida demais ou de pouca amostra na descida.')

    # A dispersão entre sentidos é dado, não ruído: nesta máquina ir e voltar
    # não são simétricos (motriz na frente, boba atrás).
    if len(saidas) >= 2 and hi > 0:
        print(f'    dispersão entre dentes: {100 * (hi - lo) / hi:.0f}% do maior')
    if parciais:
        print(f'    ({len(parciais)} dente(s) cortados no meio da rampa, '
              f'ignorados — corrida terminou antes)')
    return m


def _zona_morta_rampa_unica(r, campo_cmd, campo_med, unidade, escala=1.0):
    """Leitura dos CSV antigos, de rampa única e subida só. Mantida para os
    dados já gravados continuarem legíveis."""
    for i, l in enumerate(r):
        if abs(l[campo_med]) * escala > LIMIAR_PARADO:
            seg = r[i:i + 25]
            if len(seg) == 25 and all(abs(k[campo_med]) * escala > LIMIAR_PARADO / 2
                                      for k in seg):
                print(f'  ZONA MORTA = {abs(l[campo_cmd]):.3f} {unidade}  '
                      f'(rampa única — UMA amostra, sem faixa)')
                print(f'    (saiu do lugar em t={l["t"]:.2f} s, '
                      f'comando {l[campo_cmd]:.3f}, medido {l[campo_med]:.3f})')
                return abs(l[campo_cmd])
    print('  ROBÔ NÃO SAIU DO LUGAR EM NENHUM PONTO DA RAMPA.')
    print(f'    comando máximo tentado: {max(abs(l[campo_cmd]) for l in r):.3f} {unidade}')
    print('    -> refazer com --rampa-ate maior')
    return None


def a_dec(r):
    """Desaceleração angular: wz no instante do corte, e quanto ainda girou."""
    corte = None
    for i in range(1, len(r)):
        if abs(r[i - 1]['cmd_wz']) > 1e-6 and abs(r[i]['cmd_wz']) < 1e-6:
            corte = i
            break
    if corte is None:
        print('  não achei o corte do comando de giro no CSV')
        return None
    wz_corte = r[corte - 1]['wz_pose']
    yaw_corte = r[corte]['yaw']
    # rumo em que ele efetivamente parou de girar
    parou = None
    for l in r[corte:]:
        if abs(l['wz_pose']) < LIMIAR_PARADO:
            parou = l
            break
    if parou is None:
        print('  ele não parou de girar dentro do ensaio — aumentar --dur')
        return None
    dyaw = abs(math.atan2(math.sin(parou['yaw'] - yaw_corte),
                          math.cos(parou['yaw'] - yaw_corte)))
    if dyaw < 1e-3:
        print('  girou de menos depois do corte para medir')
        return None
    a = wz_corte ** 2 / (2.0 * dyaw)
    print(f'  a_dec = {a:.3f} rad/s²')
    print(f'    (cortou com wz={wz_corte:.3f} rad/s e ainda girou '
          f'{math.degrees(dyaw):.1f}° em {parou["t"] - r[corte]["t"]:.2f} s)')
    print(f'  -> SOBREPASSO do controlador velho seria {wz_corte**2/(2*a):.3f} rad '
          f'({math.degrees(wz_corte**2/(2*a)):.0f}°)')
    return a


def curva(r):
    """wz realizado contra comandado, e derrapada (roda contra pose)."""
    seg = [l for l in r if l['t'] >= 4.0 and abs(l['cmd_wz']) > 1e-6]
    if not seg:
        print('  sem trecho de curva sustentada no CSV')
        return
    cmd = sum(l['cmd_wz'] for l in seg) / len(seg)
    med = sum(l['wz_pose'] for l in seg) / len(seg)
    v = sum(l['v_pose'] for l in seg) / len(seg)
    print(f'  a v={v:.2f} m/s:  wz pedido {cmd:.3f}  ->  wz realizado {med:.3f} '
          f'rad/s  ({100*med/cmd:.0f}% do pedido)')
    print(f'    raio da curva: {v/med:.2f} m' if abs(med) > 1e-3 else '')
    if seg[0]['yaw_roda'] not in (None, ''):
        d0 = seg[0]['yaw'] - seg[0]['yaw_roda']
        d1 = seg[-1]['yaw'] - seg[-1]['yaw_roda']
        giro = abs(seg[-1]['yaw'] - seg[0]['yaw'])
        if giro > 0.1:
            print(f'    DERRAPADA: a roda acha que girou {math.degrees(abs(d1-d0)):.1f}° '
                  f'a mais que a pose, numa curva de {math.degrees(giro):.0f}°')
    # O número agregável desta leitura é a RAZÃO realizado÷comandado: é ela que
    # se compara entre velocidades e entre corridas (em 29-07 o giro entregou
    # 79-86% do comandado no Gazebo). O raio e a derrapada saem dela.
    return med / cmd if abs(cmd) > 1e-6 else None


def aceleracao(r):
    subida = [l for l in r if 2.0 <= l['t'] < 6.0]
    if len(subida) < 10:
        print('  ensaio curto demais')
        return
    v_max = max(l['v_pose'] for l in subida)
    # tempo do 10% ao 90% da velocidade atingida
    t10 = next((l['t'] for l in subida if l['v_pose'] > 0.1 * v_max), None)
    t90 = next((l['t'] for l in subida if l['v_pose'] > 0.9 * v_max), None)
    print(f'  velocidade atingida: {v_max:.3f} m/s')
    a_lin = None
    if t10 and t90 and t90 > t10:
        a_lin = 0.8 * v_max / (t90 - t10)
        print(f'  aceleração ≈ {a_lin:.3f} m/s² '
              f'(10%→90% em {t90-t10:.2f} s)')
    descida = [l for l in r if l['t'] >= 6.0]
    if descida:
        t_parou = next((l['t'] for l in descida
                        if abs(l['v_pose']) < LIMIAR_PARADO), None)
        if t_parou:
            print(f'  desaceleração ≈ {v_max/(t_parou-6.0):.3f} m/s² '
                  f'(parou {t_parou-6.0:.2f} s depois do corte)')
    # Agregável: a aceleração de arranque, que é o número que vai ao YAML.
    return a_lin


def reta(r):
    """Cutuca o rumo andando reto e olha se o desvio VOLTA ou CRESCE.

    Serve para comparar ida e ré na mesma velocidade. De ré a boba deixa de ser
    arrastada e passa a ser empurrada — vira roda dianteira, que é a
    configuração instável do carrinho de supermercado.

    Não adianta olhar o rumo final: um diferencial não tem nada que traga o
    rumo de volta sozinho: solto o giro, ele segue reto no rumo em que ficou.
    Quem denuncia instabilidade é a VELOCIDADE DE GIRO depois de soltar. Se ela
    cai a zero e fica, a boba está sendo dominada pelas rodas motrizes; se ela
    sobrevive ou cresce, a boba dianteira está mandando no rumo e a ré precisa
    de teto de velocidade.
    """
    solta = None
    for i in range(1, len(r)):
        if abs(r[i - 1]['cmd_wz']) > 1e-6 and abs(r[i]['cmd_wz']) < 1e-6:
            solta = i
            break
    if solta is None:
        print('  sem cutucão neste CSV (rodar sem --wz 0) — nada a comparar')
        return

    pos = r[solta:]
    if len(pos) < 20:
        print('  ensaio curto demais depois do cutucão')
        return
    v_med = sum(abs(l['v_pose']) for l in pos) / len(pos)
    sentido = 'RÉ' if pos[0]['cmd_v'] < 0 else 'FRENTE'
    wz_solta = r[solta - 1]['wz_pose']
    yaw_solta = pos[0]['yaw']

    parou = next((l for l in pos if abs(l['wz_pose']) < LIMIAR_PARADO), None)
    extra = (abs(norm(parou['yaw'] - yaw_solta)) if parou else
             abs(norm(pos[-1]['yaw'] - yaw_solta)))

    cauda = [l for l in pos if l['t'] >= pos[-1]['t'] - 2.0]
    wz_cauda = max(abs(l['wz_pose']) for l in cauda)
    meio = len(pos) // 2
    wz1 = max(abs(l['wz_pose']) for l in pos[:meio])
    wz2 = max(abs(l['wz_pose']) for l in pos[meio:])

    print(f'  {sentido} a {v_med:.2f} m/s, cutucão solto com '
          f'wz={wz_solta:.3f} rad/s')
    print(f'  girou mais {math.degrees(extra):.1f}° depois de soltar' +
          (f', parou de girar em {parou["t"] - pos[0]["t"]:.2f} s'
           if parou else ' e NÃO PAROU dentro do ensaio'))
    print(f'  pico de |wz|: {wz1:.3f} (1ª metade) -> {wz2:.3f} rad/s (2ª)')
    print(f'  últimos 2 s: |wz| máximo = {wz_cauda:.3f} rad/s')
    if wz_cauda > LIMIAR_PARADO or wz2 > wz1:
        print('  -> o rumo NÃO assentou: a boba está mandando. INSTÁVEL')
    else:
        print('  -> o rumo assentou e ficou: as motrizes dominam. ESTÁVEL')


def resumo(tipo, arqs, fonte='lio', bitola=0.270):
    """As N repetições de uma condição, juntas: média, faixa e dispersão.

    Existe porque uma corrida é uma amostra, e a leitura corrida a corrida não
    responde a pergunta que decide se o número serve: **as três concordam?**
    Rodar isto ainda no laboratório é o que permite repetir uma condição
    esquisita com o robô ligado, em vez de descobrir em casa.

    Não reimplementa nenhuma medida: chama a mesma leitura de sempre em cada
    arquivo e junta o que ela devolveu. Uma medida, um lugar.
    """
    leituras = LEITURAS(fonte, bitola)
    print(f'\n=== RESUMO {tipo} — {len(arqs)} corridas, fonte={fonte}')
    if tipo not in leituras:
        # `reta` é o caso: a leitura dela é um laudo (o rumo assentou ou não),
        # não uma grandeza. Média de laudo não existe, e dizer "nenhuma corrida
        # devolveu número" soa como falha quando não é.
        print(f'  este ensaio não devolve grandeza agregável — a leitura dele é')
        print(f'  um laudo por corrida. Comparar as {len(arqs)} acima, a olho.')
        return
    vals = []
    for a in arqs:
        try:
            r = le(a)
        except FileNotFoundError:
            print(f'  [falta] {os.path.basename(a)} — corrida não gravou')
            continue
        v = leituras[tipo](r) if tipo in leituras else None
        if v is not None:
            vals.append((os.path.basename(a), v))

    if not vals:
        print('  nenhuma corrida devolveu número — ver as leituras acima')
        return
    ns = [v for _, v in vals]
    m, dp, lo, hi = espalhamento(ns)
    print(f'  {UNIDADES.get(tipo, "valor")}')
    print(f'  média = {m:.4f}   faixa {lo:.4f} a {hi:.4f}   desvio {dp:.4f}')
    for nome, v in vals:
        print(f'    {nome}: {v:.4f}')
    if len(ns) < len(arqs):
        print(f'  [atenção] {len(arqs) - len(ns)} de {len(arqs)} corridas não '
              f'devolveram número — a repetição encolheu sozinha.')
        print(f'            Repetir a condição: sessao.py --so <passo>')
    if len(ns) < 3:
        print(f'  [atenção] só {len(ns)} corrida(s) válida(s). Três é o mínimo '
              f'para uma média significar algo.')
    elif m and abs(dp / m) > 0.15:
        print(f'  [atenção] dispersão de {100 * abs(dp / m):.0f}% da média. '
              f'Alguma coisa mudou entre as corridas')
        print(f'            (ponto de partida, piso, bateria) — vale repetir '
              f'antes de guardar.')
    else:
        print(f'  dispersão de {100 * abs(dp / m):.0f}% da média — as corridas '
              f'concordam.')


def _silencioso(f):
    """Roda a leitura sem imprimir: no resumo interessa o número, não o laudo
    de cada corrida (que já saiu na tela quando ela rodou)."""
    def g(r):
        real, sys.stdout = sys.stdout, open(os.devnull, 'w')
        try:
            return f(r)
        finally:
            sys.stdout.close()
            sys.stdout = real
    return g


# O que cada tipo devolve como NÚMERO agregável. Nem todo ensaio tem um: a
# `reta` devolve um laudo (assentou / não assentou), não uma grandeza.
UNIDADES = {
    'zona_morta_linear': 'zona morta de saída [m/s]',
    'zona_morta_giro': 'zona morta de saída [rad/s]',
    'degrau_giro': 'a_dec [rad/s²]',
    'curva': 'giro realizado ÷ comandado [1,0 = entrega o que se pede]',
    'aceleracao_linear': 'aceleração de arranque [m/s²]',
}

def LEITURAS(fonte='lio', bitola=0.270):
    """As leituras agregáveis, amarradas à fonte escolhida.

    É função, e não dicionário fixo, porque `--fonte` não pode valer só na
    leitura de uma corrida: o resumo que junta as N repetições tem de ler o
    MESMO sinal, senão a média sai do LIO enquanto as corridas saíram da roda,
    e ninguém percebe — divergência silenciosa é o defeito que este banco
    inteiro existe para não ter.
    """
    s = '_roda' if fonte == 'roda' else '_pose'
    return {
        'zona_morta_linear': _silencioso(
            lambda r: zona_morta(r, 'cmd_v', 'v' + s, 'm/s')),
        'zona_morta_giro': _silencioso(
            lambda r: zona_morta(r, 'cmd_wz', 'wz' + s, 'rad/s', bitola / 2)),
        'degrau_giro': _silencioso(a_dec),
        'curva': _silencioso(curva),
        'aceleracao_linear': _silencioso(aceleracao),
    }


def main():
    # `--fonte roda` lê as colunas derivadas da odometria de roda em vez das do
    # LIO. Tem de casar com o `--fonte` do ensaio: quem virou o dente e quem lê
    # o limiar precisam concordar sobre o que é "andando", senão a leitura
    # procura a saída num sinal que não foi o que disparou a troca de fase.
    argv = list(sys.argv[1:])
    bitola = 0.270
    if '--bitola' in argv:
        i = argv.index('--bitola')
        try:
            bitola = float(argv[i + 1])
        except (IndexError, ValueError):
            print('uso: medir.py [--bitola M] [--fonte lio|roda] <tipo> <csv>')
            raise SystemExit(1)
        del argv[i:i + 2]
    fonte = 'lio'
    if '--fonte' in argv:
        i = argv.index('--fonte')
        if i + 1 >= len(argv) or argv[i + 1] not in ('lio', 'roda'):
            print('uso: medir.py [--fonte lio|roda] <tipo> <csv>')
            raise SystemExit(1)
        fonte = argv[i + 1]
        del argv[i:i + 2]

    if argv and argv[0] == '--resumo':
        if len(argv) < 3:
            print('uso: medir.py [--fonte lio|roda] --resumo <tipo> <csv> [csv...]')
            raise SystemExit(1)
        resumo(argv[1], argv[2:], fonte, bitola)
        return
    if len(argv) != 2:
        print(__doc__)
        raise SystemExit(1)
    sufixo = '_roda' if fonte == 'roda' else '_pose'
    cv, cwz = 'v' + sufixo, 'wz' + sufixo
    tipo, arq = argv[0], argv[1]
    r = le(arq)
    if fonte == 'roda' and not any(cwz in l for l in r[:1]):
        print(f'{arq} não tem coluna {cwz} — CSV anterior ao --fonte',
              file=sys.stderr)
        raise SystemExit(1)
    print(f'=== {tipo}  ({arq}, {len(r)} amostras, fonte={fonte})')
    if tipo == 'zona_morta_linear':
        zona_morta(r, 'cmd_v', cv, 'm/s')
    elif tipo == 'zona_morta_giro':
        zm = zona_morta(r, 'cmd_wz', cwz, 'rad/s', bitola / 2)
        if zm:
            print(f'    -> em m/s de roda, com bitola L: zona_morta = {zm:.3f}·L/2')
    elif tipo == 'degrau_giro':
        a_dec(r)
    elif tipo == 'curva':
        curva(r)
    elif tipo == 'aceleracao_linear':
        aceleracao(r)
    elif tipo == 'reta':
        reta(r)
    else:
        print(f'tipo desconhecido: {tipo}')


if __name__ == '__main__':
    main()
