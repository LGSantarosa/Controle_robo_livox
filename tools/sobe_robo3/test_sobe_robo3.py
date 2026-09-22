"""`bin/sobe-robo3` só derruba o que ELE subiu — etapa 4, passo 6a (D5, plano §7).

O de hoje escolhe quem matar por NOME (`joy_node`, `twist_mux`…) na máquina
inteira: derrubaria o robô 2 ou o simulador no mesmo PC. O contrato:

- **subida**: não mata nada. Registro próprio vivo → recusa. `ros2 node list`
  com `/robot_state_publisher`, `/twist_mux`, `/joy_node` ou
  `/teleop_twist_joy_node` → recusa e lista. `node list` falhando ou travando
  → recusa (nunca "sem conflito"). Cada grupo (`setsid`) entra no registro
  logo depois de criado, com o PID conferido como PGID;
- **`--mata`**: só os grupos do registro; idempotente; registro ausente ou
  obsoleto → sai 0 sem matar; malformado → recusa sem matar;
- consulta, subida, registro e `--mata` sob `flock`.

Registro: `$HOME/.local/state/sobe-robo3/grupos` (diretório 0700, arquivo
0600), uma linha por grupo, `PGID STARTTIME PAPEL`, com o STARTTIME do líder
(campo 22 do `/proc/PID/stat`). Um PGID não volta a ser alocado enquanto houver
membro no grupo — então líder morto com membro vivo é grupo NOSSO (mata), e
líder vivo com STARTTIME diferente é PID reusado (não é nosso: descarta).

**Sem namespace** (o AppArmor desta máquina bloqueia user namespace sem
root), o isolamento é por calços (`calcos/`): `BASH_ENV` troca `kill`,
`pkill`, `killall`, `sleep` e o `source` do ROS por funções; o PATH começa por
`kill`, `ps`, `pkill`, `killall`, `pgrep` e `ros2` falsos. O `kill` só entrega
sinal a PID/PGID que o teste criou. Contorno (`/bin/kill`, `builtin kill`…) é
proibido no fonte pelos testes estáticos.
"""
import os
import re
import signal
import subprocess
import time

import pytest

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.abspath(os.path.join(AQUI, '..', '..'))
SCRIPT = os.path.join(RAIZ, 'bin', 'sobe-robo3')
CALCOS = os.path.join(AQUI, 'calcos')

# Nomes que um robô 2 ou o simulador têm no mesmo PC — o `vivos()` de hoje casa
# todos (o `robot_state_publisher` o plano manda recusar no grafo).
EXTERNOS = ('/opt/ros/jazzy/lib/joy/joy_node',
            '/opt/ros/jazzy/lib/teleop_twist_joy/teleop_node teleop_twist_joy',
            '/opt/ros/jazzy/lib/twist_mux/twist_mux',
            '/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher')
NOS_CONFLITO = ('/joy_node', '/teleop_twist_joy_node', '/twist_mux',
                '/robot_state_publisher')


# ─── /proc ───────────────────────────────────────────────────────────────────

def _stat(pid):
    """Devolve (estado, pgrp, starttime), ou None se o processo não existe.

    Campos DEPOIS do último ')': o comm pode ter espaço e parêntese.
    """
    try:
        with open(f'/proc/{pid}/stat') as f:
            s = f.read()
    except (FileNotFoundError, ProcessLookupError):
        return None
    resto = s[s.rindex(')') + 2:].split()
    return resto[0], int(resto[2]), int(resto[19])


def _vivo(pid):
    st = _stat(pid)
    return st is not None and st[0] not in ('Z', 'X')


def _membros(pgid):
    out = []
    for d in os.listdir('/proc'):
        if d.isdigit():
            st = _stat(int(d))
            if st and st[1] == pgid and st[0] not in ('Z', 'X'):
                out.append(int(d))
    return out


def _espera(cond, limite=5.0):
    fim = time.monotonic() + limite
    while time.monotonic() < fim:
        if cond():
            return True
        time.sleep(0.05)
    return cond()


# ─── bancada fingida ─────────────────────────────────────────────────────────

class Bancada:
    def __init__(self, tmp):
        self.home = tmp / 'home'
        self.c = tmp / 'calco'
        self.home.mkdir()
        self.c.mkdir()
        self.porta = tmp / 'ttyACM_fingida'
        self.porta.write_text('')
        self.estado = self.home / '.local' / 'state' / 'sobe-robo3'
        self.registro = self.estado / 'grupos'
        self._filhos = []
        self.env = {
            'HOME': str(self.home),
            'PATH': f'{CALCOS}:/usr/bin:/bin',
            'BASH_ENV': os.path.join(CALCOS, 'bash_env.sh'),
            'CALCO': str(self.c),
            'CALCO_BIN': CALCOS,
            'LANG': 'C.UTF-8',
            # Segunda linha de defesa: se um `ros2` REAL escapar de novo, fica
            # fora do domínio 0 e sem descobrir outras máquinas (Jazzy: o
            # LOCALHOST_ONLY está obsoleto, e o DISCOVERY_RANGE é o atual).
            'ROS_DOMAIN_ID': '77',
            'ROS_LOCALHOST_ONLY': '1',
            'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
        }

    # listas do calço de kill
    def _anota(self, arquivo, n):
        with open(self.c / arquivo, 'a') as f:
            f.write(f'{n}\n')

    def _linhas(self, arquivo):
        p = self.c / arquivo
        return p.read_text().splitlines() if p.exists() else []

    def violacoes(self):
        return self._linhas('violacoes')

    def chamadas_kill(self):
        return self._linhas('kill_chamadas')

    def chamadas_ros2(self):
        return self._linhas('ros2_chamadas')

    def botao(self, nome, conteudo=''):
        (self.c / nome).write_text(conteudo)

    def processo(self, argv0, *, grupo_proprio=True, stdin=None):
        """Sobe um `sleep` com o argv[0] pedido (o que o `ps` mostra).

        Filho do teste, em sessão própria como um launch do robô 2, e
        permitido ao calço: se o script matar, a sobrevivência reprova, não o
        calço.
        """
        p = subprocess.Popen(
            ['bash', '-c', f'exec -a {argv0!r} /usr/bin/sleep 3600'],
            start_new_session=grupo_proprio, stdin=stdin)
        self._filhos.append(p)
        assert _espera(lambda: _vivo(p.pid))
        self._anota('permitidos', p.pid)
        self._anota('grupos', p.pid)
        return p

    def externos(self):
        return [self.processo(n) for n in EXTERNOS]

    def grupo_lider_morto(self, argv0_filho):
        """Grupo cujo líder morre e deixa um filho vivo NO MESMO grupo."""
        lider = subprocess.Popen(
            ['bash', '-c', f'(exec -a {argv0_filho!r} /usr/bin/sleep 3600) & '
                           'echo $!; read -r _'],
            start_new_session=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            text=True)
        self._filhos.append(lider)
        filho = int(lider.stdout.readline())
        _, pgid, start = _stat(lider.pid)
        assert pgid == lider.pid and _stat(filho)[1] == pgid
        for n in (lider.pid, filho):
            self._anota('permitidos', n)
        self._anota('grupos', pgid)
        return lider, filho, pgid, start

    def escreve_registro(self, linhas):
        self.estado.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.registro.write_text(''.join(f'{linha}\n' for linha in linhas))
        os.chmod(self.registro, 0o600)

    def roda(self, *args, timeout=60):
        return subprocess.run(['bash', SCRIPT, *args], env=self.env, cwd=RAIZ,
                              capture_output=True, text=True, timeout=timeout,
                              stdin=subprocess.DEVNULL)

    def sobe(self, *args, **kw):
        return self.roda(f'porta:={self.porta}', *args, **kw)

    def dispara(self, *args):
        return subprocess.Popen(['bash', SCRIPT, f'porta:={self.porta}', *args],
                                env=self.env, cwd=RAIZ, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True,
                                stdin=subprocess.DEVNULL)

    def marcados(self):
        """Lista TODO processo vivo que herdou a marca `CALCO` deste teste.

        Tudo o que o script cria descende dele e carrega a marca, por qualquer
        caminho (calço certo ou errado). Nenhum processo da máquina a tem.
        """
        marca = f'CALCO={self.c}'.encode() + b'\0'
        out = []
        for d in os.listdir('/proc'):
            if not d.isdigit() or int(d) == os.getpid():
                continue
            try:
                with open(f'/proc/{d}/environ', 'rb') as f:
                    env = f.read()
            except OSError:
                continue
            if env.startswith(marca[:-1] + b'\0') or b'\0' + marca in env:
                if _vivo(int(d)):
                    out.append(int(d))
        return out

    def vazados(self):
        """Marcados que o teste não conhece: sinal de calço furado."""
        conhecidos = {int(x) for x in self._linhas('permitidos')}
        grupos = {int(x) for x in self._linhas('grupos')}
        return [n for n in self.marcados()
                if n not in conhecidos and _stat(n) and _stat(n)[1] not in grupos]

    def limpa(self):
        """Derruba só o que o teste criou.

        Os permitidos, os grupos anotados e os processos com a marca deste
        teste.
        """
        for n in self.marcados():
            try:
                os.kill(n, signal.SIGKILL)
            except ProcessLookupError:
                pass
        for p in self._filhos:
            if p.poll() is None:
                p.kill()
            p.wait()
        for g in {int(x) for x in self._linhas('grupos')}:
            if g > 1 and _membros(g):
                try:
                    os.killpg(g, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        for n in {int(x) for x in self._linhas('permitidos')}:
            if n > 1 and _vivo(n):
                try:
                    os.kill(n, signal.SIGKILL)
                except ProcessLookupError:
                    pass


@pytest.fixture
def b(tmp_path):
    bancada = Bancada(tmp_path)
    yield bancada
    vazados = [(n, open(f'/proc/{n}/cmdline', 'rb').read()[:120])
               for n in bancada.vazados() if _vivo(n)]
    bancada.limpa()
    assert vazados == [], f'processo fora dos calços (a bancada furou): {vazados}'


def _todos_vivos(procs):
    return [p.pid for p in procs if p.poll() is not None]


# ─── os calços mordem (senão a fronteira é de papel) ─────────────────────────

@pytest.mark.parametrize('alvo', ['1', '-1', '0', '-0', '1x', str(os.getpid()),
                                  f'-{os.getpgid(0)}'])
def test_calco_de_kill_recusa_alvo_que_o_teste_nao_criou(b, alvo):
    r = subprocess.run([os.path.join(CALCOS, 'kill'), '-0', '--', alvo],
                       env=b.env, capture_output=True, text=True)
    assert r.returncode != 0
    assert b.violacoes(), 'o calço deixou passar um alvo de fora'


def test_calco_de_kill_trata_o_primeiro_menos_numero_como_sinal(b):
    """`kill -5` é o sinal 5 (não previsto), não o grupo 5."""
    r = subprocess.run([os.path.join(CALCOS, 'kill'), '-5'], env=b.env,
                       capture_output=True, text=True)
    assert r.returncode != 0 and b.violacoes()


def test_calco_de_kill_entrega_ao_permitido(b):
    p = b.processo('/fingido/qualquer')
    r = subprocess.run([os.path.join(CALCOS, 'kill'), '-KILL', '--', f'-{p.pid}'],
                       env=b.env, capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert _espera(lambda: p.poll() is not None)
    assert b.violacoes() == []


@pytest.mark.parametrize('forma', ['kill -0 1', 'pkill -f joy', 'killall twist_mux',
                                   'echo 1 | xargs -r kill -0'])
def test_bash_env_e_path_desviam_as_formas_comuns(b, forma):
    r = subprocess.run(['bash', '-c', forma], env=b.env, capture_output=True, text=True)
    assert r.returncode != 0
    assert b.violacoes(), f'{forma!r} passou por fora dos calços'


@pytest.mark.parametrize('arquivo', ['install/setup.bash', '/opt/ros/jazzy/setup.bash',
                                     './install/local_setup.bash'])
def test_source_de_setup_nao_mexe_no_path(b, arquivo):
    r = subprocess.run(['bash', '-c', f'source {arquivo}; echo "$PATH"; command -v ros2'],
                       env=b.env, cwd=RAIZ, capture_output=True, text=True)
    path, ros2 = r.stdout.splitlines()
    assert path == b.env['PATH']
    assert ros2 == os.path.join(CALCOS, 'ros2'), 'o ros2 REAL entraria'


def test_detector_de_vazamento_acha_o_processo_marcado_que_o_teste_nao_criou(b):
    """A rede de segurança não depende dos calços.

    Um processo solto com a marca deste teste é achado, e a limpeza o derruba.
    """
    subprocess.run(['bash', '-c', 'setsid /usr/bin/sleep 300 >/dev/null 2>&1 &'],
                   env=b.env, check=True)
    assert _espera(lambda: b.vazados() != [])
    solto = b.vazados()
    b.limpa()
    assert _espera(lambda: not any(_vivo(n) for n in solto))


def test_ps_falso_nao_enxerga_a_maquina(b):
    p = b.processo('/fingido/visivel')
    r = subprocess.run(['bash', '-c', 'ps -eo pid,args'], env=b.env,
                       capture_output=True, text=True)
    pids = [linha.split()[0] for linha in r.stdout.splitlines()[1:]]
    assert pids == [str(p.pid)]


# ─── estático: nada de matar por nome nem contornar os calços ────────────────

def _codigo():
    """O fonte sem comentários (o de hoje CITA o `pkill -f` para proibi-lo)."""
    with open(SCRIPT) as f:
        linhas = f.read().splitlines()
    return '\n'.join(re.sub(r'(^|\s)#.*$', '', linha) for linha in linhas)


@pytest.mark.parametrize('proibido', [
    r'\bpkill\b', r'\bkillall\b', r'\bpgrep\b',
    r'\bxargs\b[^\n|;]*\bkill\b',
    r'/(usr/)?bin/kill\b', r'\bbuiltin\s+kill\b', r'\bcommand\s+kill\b',
    r'\benv\s+kill\b', r'\bexec\s+kill\b', r'\\kill\b',
    r'(^|[;&|]\s*)\.\s+\S',          # só `source` carrega arquivo (o calço o vigia)
])
def test_fonte_nao_mata_por_nome_nem_contorna_os_calcos(proibido):
    achou = re.search(proibido, _codigo(), re.MULTILINE)
    assert achou is None, f'{proibido!r} no sobe-robo3: {achou.group(0)!r}'


def test_fonte_usa_flock():
    assert re.search(r'\bflock\b', _codigo()), 'consulta, subida e --mata sob flock'


# ─── --mata ──────────────────────────────────────────────────────────────────

def test_mata_sem_registro_nao_mata_ninguem(b):
    ext = b.externos()
    r = b.roda('--mata')
    assert r.returncode == 0, r.stdout + r.stderr
    assert _todos_vivos(ext) == [], 'matou processo que não é dele (por nome)'
    assert b.chamadas_kill() == [], 'sem registro, não há o que sinalizar'
    assert b.violacoes() == []


def test_mata_e_idempotente(b):
    ext = b.externos()
    for _ in range(2):
        r = b.roda('--mata')
        assert r.returncode == 0, r.stdout + r.stderr
    assert _todos_vivos(ext) == []
    assert b.violacoes() == []


def test_mata_registro_de_grupo_que_nao_existe_mais(b):
    ext = b.externos()
    morto = subprocess.Popen(['/usr/bin/sleep', '30'], start_new_session=True)
    _, pgid, start = _stat(morto.pid)
    morto.kill()
    morto.wait()
    b.escreve_registro([f'{pgid} {start} launch'])
    r = b.roda('--mata')
    assert r.returncode == 0, r.stdout + r.stderr
    assert not b.registro.exists(), 'registro obsoleto descartado'
    assert b.chamadas_kill() == []
    assert _todos_vivos(ext) == []
    assert b.violacoes() == []


def test_mata_starttime_diferente_nao_mata_pid_reutilizado(b):
    """O líder registrado existe, mas é OUTRO processo (PID reusado)."""
    ext = b.externos()
    alheio = b.processo('/fingido/lib/twist_mux')
    _, pgid, start = _stat(alheio.pid)
    b.escreve_registro([f'{pgid} {start + 1} launch'])
    r = b.roda('--mata')
    assert r.returncode == 0, r.stdout + r.stderr
    assert alheio.poll() is None, 'matou o dono novo do PID reusado'
    assert not b.registro.exists(), 'registro obsoleto descartado'
    assert _todos_vivos(ext) == []
    assert b.violacoes() == []


def test_mata_lider_morreu_filho_do_grupo_ficou_vivo(b):
    ext = b.externos()
    lider, filho, pgid, start = b.grupo_lider_morto('/fingido/sem_nome_de_no')
    b.escreve_registro([f'{pgid} {start} launch'])
    lider.stdin.close()
    lider.wait()
    assert not _vivo(pgid) and _vivo(filho), 'montagem: líder morto, filho vivo'
    r = b.roda('--mata')
    assert r.returncode == 0, r.stdout + r.stderr
    assert _espera(lambda: not _vivo(filho)), 'membro do NOSSO grupo ficou vivo'
    assert _membros(pgid) == []
    assert not b.registro.exists()
    assert _todos_vivos(ext) == []
    assert b.violacoes() == []


def test_mata_escala_para_kill_quem_ignora_o_int(b):
    ext = b.externos()
    teimoso = subprocess.Popen(
        ['bash', '-c', "trap '' INT; exec -a /fingido/teimoso /usr/bin/sleep 3600"],
        start_new_session=True)
    b._filhos.append(teimoso)
    assert _espera(lambda: _vivo(teimoso.pid))
    b._anota('permitidos', teimoso.pid)
    b._anota('grupos', teimoso.pid)
    _, pgid, start = _stat(teimoso.pid)
    b.escreve_registro([f'{pgid} {start} launch'])
    r = b.roda('--mata')
    assert r.returncode == 0, r.stdout + r.stderr
    assert _espera(lambda: teimoso.poll() is not None)
    assert teimoso.returncode == -signal.SIGKILL, 'o INT foi ignorado; o KILL resolveu'
    assert not b.registro.exists()
    assert _todos_vivos(ext) == []
    assert b.violacoes() == []


@pytest.mark.parametrize('linha', ['0 123 launch', '1 123 launch', '-5 123 launch',
                                   'abc 123 launch', '4242', '4242 x launch',
                                   '4242 123 launch extra', ''])
def test_mata_registro_malformado_recusa_sem_matar(b, linha):
    ext = b.externos()
    b.escreve_registro([linha])
    r = b.roda('--mata')
    assert r.returncode != 0, 'registro malformado não pode passar como vazio'
    assert b.chamadas_kill() == []
    assert b.registro.exists(), 'o arquivo fica para um humano olhar'
    assert _todos_vivos(ext) == []
    assert b.violacoes() == []


def test_mata_registro_misto_valida_o_arquivo_inteiro_antes_de_sinalizar(b):
    """Registro com uma linha válida e depois uma malformada.

    A válida é de um grupo nosso, vivo. Nenhuma ação parcial: nem a linha boa
    é sinalizada.
    """
    ext = b.externos()
    nosso = b.processo('/fingido/sem_nome_de_no')
    _, pgid, start = _stat(nosso.pid)
    b.escreve_registro([f'{pgid} {start} launch', '0 123 bag'])
    r = b.roda('--mata')
    assert r.returncode != 0
    assert b.chamadas_kill() == [], 'sinalizou antes de validar o arquivo inteiro'
    assert nosso.poll() is None
    assert b.registro.exists()
    assert _todos_vivos(ext) == []
    assert b.violacoes() == []


# ─── subida ──────────────────────────────────────────────────────────────────

def test_subida_com_conflito_no_grafo_recusa_sem_matar(b):
    ext = b.externos()
    b.botao('nos', '\n'.join(('/mapa', *NOS_CONFLITO, '/outro')) + '\n')
    r = b.sobe()
    saida = r.stdout + r.stderr
    assert r.returncode != 0
    for no in NOS_CONFLITO:
        assert no in saida, f'a recusa lista quem está no domínio: {no}'
    assert b.chamadas_kill() == [], 'recusar é não matar'
    assert not any(c.startswith(('launch', 'bag')) for c in b.chamadas_ros2())
    assert not b.registro.exists()
    assert _todos_vivos(ext) == []
    assert b.violacoes() == []


def test_subida_nao_confia_no_daemon_que_ainda_nao_descobriu(b):
    """Vazio com código 0 NÃO é "sem conflito" quando a descoberta é incompleta.

    Os testes de `node_list_falha`/`node_list_trava` cobriam erro e
    travamento da consulta, mas não isto: em 22-09 (20260922_132801) o
    `ros2 node list` pelo daemon recém-nascido voltou vazio com código 0,
    enquanto `--no-daemon --spin-time 5`, logo antes, via os quatro. O
    sobe-robo3 subiu por cima e o domínio ficou com nomes duplicados.
    """
    ext = b.externos()
    b.botao('nos', '\n'.join(NOS_CONFLITO) + '\n')
    b.botao('node_list_daemon_frio')
    r = b.sobe()
    saida = r.stdout + r.stderr
    assert r.returncode != 0, 'subiu por cima de quem está no domínio'
    for no in NOS_CONFLITO:
        assert no in saida
    assert not any(c.startswith(('launch', 'bag')) for c in b.chamadas_ros2())
    assert b.chamadas_kill() == []
    assert not b.registro.exists()
    assert _todos_vivos(ext) == []
    assert b.violacoes() == []


@pytest.mark.parametrize('defeito', ['node_list_falha', 'node_list_trava'])
def test_subida_com_node_list_sem_resposta_recusa(b, defeito):
    ext = b.externos()
    b.botao(defeito)
    r = b.sobe(timeout=90)
    assert r.returncode != 0, 'sem resposta do grafo não é "sem conflito"'
    assert b.chamadas_kill() == []
    assert not any(c.startswith(('launch', 'bag')) for c in b.chamadas_ros2())
    assert not b.registro.exists()
    assert _todos_vivos(ext) == []
    assert b.violacoes() == []


def _le_registro(b):
    return [linha.split() for linha in b.registro.read_text().splitlines()]


def test_subida_limpa_registra_os_grupos_e_mata_so_eles(b):
    ext = b.externos()
    r = b.sobe()
    assert r.returncode == 0, r.stdout + r.stderr
    assert b.chamadas_kill() == [], 'a subida não mata nada'

    assert os.stat(b.estado).st_mode & 0o777 == 0o700
    assert os.stat(b.registro).st_mode & 0o777 == 0o600
    reg = _le_registro(b)
    assert sorted(p for _, _, p in reg) == ['bag', 'launch']
    lideres = {int(x) for x in b._linhas('grupos')} - {p.pid for p in ext}
    for pgid, start, _ in reg:
        pgid, start = int(pgid), int(start)
        assert pgid in lideres, 'PGID registrado é o líder que o launch/bag criou'
        assert _stat(pgid)[1] == pgid, 'PID conferido como PGID'
        assert _stat(pgid)[2] == start
    nossos = [n for g, _, _ in reg for n in _membros(int(g))]
    assert len(nossos) >= 2 + 7

    r = b.roda('--mata')
    assert r.returncode == 0, r.stdout + r.stderr
    assert _espera(lambda: not any(_vivo(n) for n in nossos)), 'sobrou nosso'
    assert not any('KILL' in c for c in b.chamadas_kill()), \
        'launch e bag atendem o INT: o caminho normal não precisa de KILL'
    assert not b.registro.exists()
    assert _todos_vivos(ext) == [], 'o --mata derrubou os externos de mesmo nome'
    assert b.roda('--mata').returncode == 0, 'idempotente depois da subida'
    assert b.violacoes() == []


def test_falha_ao_gravar_o_registro_derruba_o_grupo_recem_criado(b, tmp_path):
    """Grupo confirmado, mas o registro não grava: derruba o grupo, não órfão.

    O `mv` falso falha (disco cheio, permissão). O grupo do launch já existe e
    ninguém o acharia depois: tem de ser derrubado, o temporário removido e a
    subida reprovada.
    """
    ext = b.externos()
    quebrado = tmp_path / 'mv_quebrado'
    quebrado.mkdir()
    (quebrado / 'mv').write_text('#!/usr/bin/env bash\necho "mv falso: falhou" >&2\nexit 1\n')
    os.chmod(quebrado / 'mv', 0o755)
    b.env['PATH'] = f'{quebrado}:{b.env["PATH"]}'
    r = b.sobe()
    assert r.returncode != 0, r.stdout + r.stderr
    lideres = {int(x) for x in b._linhas('grupos')} - {p.pid for p in ext}
    assert lideres, 'montagem: o launch falso chegou a subir'
    assert _espera(lambda: all(_membros(g) == [] for g in lideres)), 'grupo órfão vivo'
    assert not b.registro.exists()
    assert list(b.estado.glob('grupos.novo.*')) == [], 'temporário ficou'
    assert not any(c.startswith('bag') for c in b.chamadas_ros2())
    assert _todos_vivos(ext) == []
    assert b.violacoes() == []


# ─── a conferência da placa não confunde descoberta com defeito ─────────────

def _tentativas(b, d):
    return (d / 'battery_check.log').read_text().count('--- tentativa')


def test_aviso_de_descoberta_e_depois_a_mensagem_boa(b):
    """O falso negativo de 22-09: aviso no stdout lido como placa desligada."""
    b.externos()
    b.botao('battery_avisos', '1')
    r = b.sobe()
    saida = r.stdout + r.stderr
    assert r.returncode == 0, saida
    assert 'placa responde:     🟢 sim' in saida
    bag = sorted((b.home / 'bancada_robo3').glob('controle_*'))[-1]
    assert _tentativas(b, bag) >= 2, 'a segunda tentativa é que trouxe a leitura'
    assert b.violacoes() == []


def test_aviso_permanente_reprova_como_sem_leitura(b):
    b.externos()
    b.botao('battery_avisos', '999')
    r = b.sobe(timeout=120)
    saida = r.stdout + r.stderr
    assert r.returncode != 0
    assert 'sem leitura válida em 10 s' in saida, saida
    assert 'placa desligada' not in saida, 'aviso de descoberta não é diagnóstico de placa'


def test_echo_travado_reprova_como_sem_leitura(b):
    b.externos()
    b.botao('battery_travado')
    r = b.sobe(timeout=180)
    assert r.returncode != 0
    assert 'sem leitura válida em 10 s' in (r.stdout + r.stderr)


def test_present_false_reprova_como_placa_muda(b):
    b.externos()
    b.botao('battery_present_false')
    r = b.sobe()
    saida = r.stdout + r.stderr
    assert r.returncode != 0
    assert 'placa desligada' in saida, 'leitura VÁLIDA com a placa muda'
    # a linha da TENSÃO também diz "sem leitura válida"; aqui importa a da placa
    assert 'sem leitura válida em 10 s' not in saida


def test_present_true_com_tensao_aprova(b):
    b.externos()
    r = b.sobe()
    saida = r.stdout + r.stderr
    assert r.returncode == 0, saida
    assert 'placa responde:     🟢 sim' in saida and 'bateria das rodas:  🟢 38.5 V' in saida


def test_subida_com_registro_vivo_recusa(b):
    ext = b.externos()
    nosso = b.processo('/fingido/lib/mega_bridge')
    _, pgid, start = _stat(nosso.pid)
    b.escreve_registro([f'{pgid} {start} launch'])
    r = b.sobe()
    assert r.returncode != 0, 'já está no ar: recusa (use --mata)'
    assert nosso.poll() is None
    assert b.chamadas_kill() == []
    assert not any(c.startswith(('launch', 'bag')) for c in b.chamadas_ros2())
    assert _todos_vivos(ext) == []
    assert b.violacoes() == []


def test_duas_subidas_concorrentes_so_uma_passa(b):
    ext = b.externos()
    b.botao('node_list_atraso', '1')   # alarga a janela entre consulta e registro
    ps = [b.dispara(), b.dispara()]
    rcs = [p.wait(timeout=90) for p in ps]
    saidas = [p.stdout.read() for p in ps]
    assert sorted(rc == 0 for rc in rcs) == [False, True], (rcs, saidas)
    lancados = [c for c in b.chamadas_ros2() if c.startswith('launch')]
    assert len(lancados) == 1, f'duas subidas passaram juntas: {lancados}'
    assert [p for _, _, p in _le_registro(b)].count('launch') == 1
    assert b.chamadas_kill() == []
    assert _todos_vivos(ext) == []
    assert b.violacoes() == []
