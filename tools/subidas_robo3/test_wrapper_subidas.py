#!/usr/bin/env python3
"""O `bin/subidas-robo3` EXECUTADO com comandos calçados (decisão 061 §4).

Prova, sem ROS e sem Gazebo:
  · falha de subida é registrada e a bateria CONTINUA;
  · resíduo depois da limpeza INTERROMPE a bateria (a próxima não sobe);
  · o manifesto fecha, conferido de fora.

🔴 Isolamento — o calço por PATH já falhou uma vez neste projeto e subiu a pilha
real (um `source` relativo re-prefixou o ROS de verdade):
  · o wrapper roda num REPOSITÓRIO TEMPORÁRIO (git init + só os arquivos que
    ele usa), com árvore limpa de verdade;
  · o wrapper re-prefixa `SUBIDAS_CALCOS` no PATH depois de CADA `source`;
  · domínio 91, nunca o 50;
  · todo calço deixa `CALCO <comando>` num arquivo, e o teste exige as marcas;
  · no fim, nenhum `gz sim` real pode existir no /proc.
"""
import csv
import os
import shutil
import stat
import subprocess
import textwrap

import pytest

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(os.path.dirname(AQUI))
WRAPPER = os.path.join(RAIZ, 'bin', 'subidas-robo3')

COPIAS = ('bin/subidas-robo3', 'tools/valida_etapa6/lib.sh',
          'tools/valida_etapa6/processos.py', 'tools/subidas_robo3/mede.py')

# O `ros2` falso. Cenário por subida em $CALCO_DIR/cenario (linha "N tipo",
# tipo ok | falha_jsb | residuo); a subida corrente é o contador de launches.
ROS2 = textwrap.dedent(r'''
    #!/usr/bin/env bash
    D="$CALCO_DIR"
    echo "CALCO $*" >> "$D/chamadas.txt"
    n="$(cat "$D/launches" 2>/dev/null || echo 0)"
    tipo="$(awk -v n="$n" '$1==n{print $2}' "$D/cenario" 2>/dev/null)"
    case "$1 $2" in
      "launch robot_motion")
        n=$((n + 1)); echo "$n" > "$D/launches"
        tipo="$(awk -v n="$n" '$1==n{print $2}' "$D/cenario")"
        trap 'echo "[x-1] [INFO] [1790000100.0] [rclcpp]: signal_handler(SIGINT/SIGTERM)"; touch "$D/encerrado_$n"; exit 0' INT TERM
        echo "[gazebo-1] [INFO] [1790000001.000000000] [controller_manager]: Activating controllers: [ joint_state_broadcaster ]"
        if [ "$tipo" = falha_jsb ]; then
          echo "[gazebo-1] [ERROR] [1790000006.000000000] [controller_manager]: Switch controller timed out after 5 seconds!"
          echo "[spawner-20] [ERROR] [1790000006.1] [spawner_joint_state_broadcaster]: Failed to activate controller : joint_state_broadcaster"
        else
          echo "[spawner-20] [INFO] [1790000001.500000000] [spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster"
        fi
        echo "[gazebo-1] [INFO] [1790000002.000000000] [controller_manager]: Activating controllers: [ hoverboard_base_controller ]"
        echo "[spawner-21] [INFO] [1790000002.200000000] [spawner_hoverboard_base_controller]: Configured and activated hoverboard_base_controller"
        while :; do sleep 0.1; done ;;
      "lifecycle get") echo "active [3]" ;;
      "action list") echo "/navigate_to_pose" ;;
      "run tf2_ros") echo "- Translation: [0.000, 0.000, 0.000]" ;;
      "control list_controllers")
        if [ "$tipo" = falha_jsb ]; then e=inactive; else e=active; fi
        echo "joint_state_broadcaster     joint_state_broadcaster/JointStateBroadcaster  $e"
        echo "hoverboard_base_controller  diff_drive_controller/DiffDriveController      active" ;;
      "topic echo")
        [ "$tipo" = falha_jsb ] || for i in $(seq 1 30); do echo 1; echo ---; done ;;
      "node list")
        if [ "$tipo" = residuo ] && [ -e "$D/encerrado_$n" ]; then echo /fantasma; fi ;;
      *) echo "calço: comando não previsto: $*" >&2; exit 97 ;;
    esac
''').lstrip()

COLCON = textwrap.dedent('''
    #!/usr/bin/env bash
    echo "CALCO colcon $*" >> "$CALCO_DIR/chamadas.txt"
    echo "Summary: 3 packages finished"
''').lstrip()


def _executavel(caminho, texto):
    with open(caminho, 'w') as f:
        f.write(texto)
    os.chmod(caminho, os.stat(caminho).st_mode | stat.S_IXUSR)


def _gz_real():
    achados = []
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                argv = f.read().replace(b'\0', b' ').decode(errors='replace')
        except OSError:
            continue
        if 'gz sim' in argv or 'gzserver' in argv:
            achados.append(argv)
    return achados


def _bateria(tmp_path, cenario, n):
    if not os.path.exists(WRAPPER):
        pytest.fail(f'{WRAPPER} ainda não existe — decisão 061, passo 2.')
    repo = tmp_path / 'repo'
    for rel in COPIAS:
        destino = repo / rel
        destino.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(os.path.join(RAIZ, rel), destino)
    (repo / 'install').mkdir()
    (repo / 'install' / 'setup.bash').write_text('# calço\n')
    (repo / '.gitignore').write_text('install/\n')
    git = ['git', '-C', str(repo), '-c', 'user.name=t', '-c', 'user.email=t@t']
    subprocess.run(git + ['init', '-q'], check=True)
    subprocess.run(git + ['add', '-A'], check=True)
    subprocess.run(git + ['commit', '-qm', 'calço'], check=True)

    calcos = tmp_path / 'calcos'
    calcos.mkdir()
    _executavel(calcos / 'ros2', ROS2)
    _executavel(calcos / 'colcon', COLCON)
    (calcos / 'cenario').write_text(
        ''.join(f'{i} {cenario.get(i, "ok")}\n' for i in range(1, n + 1)))
    saidas = tmp_path / 'saidas'
    env = dict(os.environ, SUBIDAS_CALCOS=str(calcos), CALCO_DIR=str(calcos),
               SUBIDAS_RAIZ_SAIDA=str(saidas), SUBIDAS_DOMINIO='91')
    env.pop('VALIDA_ETAPA4_MARCA', None)
    r = subprocess.run(['bash', str(repo / 'bin' / 'subidas-robo3'), str(n)],
                       capture_output=True, text=True, timeout=300, env=env)
    pastas = sorted(saidas.iterdir()) if saidas.exists() else []
    assert len(pastas) == 1, (r.stdout[-3000:], r.stderr[-2000:])
    pasta = pastas[0]
    chamadas = (calcos / 'chamadas.txt').read_text()
    assert 'CALCO launch robot_motion' in chamadas, 'o calço não foi usado'
    assert not _gz_real(), 'subiu Gazebo de verdade'
    linhas = list(csv.DictReader(open(pasta / 'subidas.csv')))
    launches = int((calcos / 'launches').read_text())
    return r, pasta, linhas, launches


def _manifesto_de_fora(pasta):
    return subprocess.run(['sha256sum', '-c', 'SHA256SUMS', '--quiet'],
                          cwd=pasta, capture_output=True, text=True)


def test_tres_subidas_nominais_sao_estaveis_e_o_manifesto_fecha(tmp_path):
    r, pasta, linhas, launches = _bateria(tmp_path, {}, 3)
    assert r.returncode == 0, r.stdout[-3000:]
    assert launches == 3 and len(linhas) == 3
    assert [l['subida_ok'] for l in linhas] == ['1', '1', '1']
    assert [l['limpeza_ok'] for l in linhas] == ['1', '1', '1']
    assert (pasta / 'veredito.txt').read_text().startswith('ESTÁVEL')
    m = _manifesto_de_fora(pasta)
    assert m.returncode == 0, m.stdout + m.stderr
    assinados = (pasta / 'SHA256SUMS').read_text()
    assert 'console.txt' not in assinados
    for i in (1, 2, 3):
        assert f'./subida_{i:02d}/launch.log' in assinados


def test_falha_de_subida_e_registrada_e_a_bateria_continua(tmp_path):
    r, pasta, linhas, launches = _bateria(tmp_path, {2: 'falha_jsb'}, 3)
    assert r.returncode == 1
    assert launches == 3 and len(linhas) == 3, 'a bateria tinha de continuar'
    assert [l['subida_ok'] for l in linhas] == ['1', '0', '1']
    assert linhas[1]['switch_timeout'] == '1'
    assert linhas[1]['jsb_estado'] == 'inactive'
    assert linhas[1]['joint_states_msgs'] == '0'
    assert (pasta / 'veredito.txt').read_text().startswith('INSTÁVEL')
    assert _manifesto_de_fora(pasta).returncode == 0


def test_residuo_depois_da_limpeza_interrompe_a_bateria(tmp_path):
    r, pasta, linhas, launches = _bateria(tmp_path, {2: 'residuo'}, 3)
    assert r.returncode == 1
    assert launches == 2, 'a subida 3 não podia ter subido sobre resíduo'
    assert len(linhas) == 2 and linhas[1]['limpeza_ok'] == '0'
    assert (pasta / 'veredito.txt').read_text().startswith('INCOMPLETA')
    assert _manifesto_de_fora(pasta).returncode == 0


def test_o_wrapper_nunca_usa_pkill_nem_apaga_evidencia():
    texto = open(WRAPPER).read() if os.path.exists(WRAPPER) else ''
    assert texto, 'wrapper ainda não existe'
    assert 'pkill' not in texto and 'rm -rf' not in texto


def test_dominio_e_os_argumentos_da_corrida_do_passo_7():
    texto = open(WRAPPER).read() if os.path.exists(WRAPPER) else ''
    assert 'SUBIDAS_DOMINIO:-50' in texto
    for arg in ('robo:=3', 'sim:=true', 'gui:=false', 'rviz:=false',
                'localizacao:=fixa', 'bag:=false'):
        assert arg in texto, arg
