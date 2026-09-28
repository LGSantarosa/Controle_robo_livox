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
          'tools/valida_etapa6/processos.py', 'tools/subidas_robo3/mede.py',
          'tools/subidas_robo3/shm_recupera.py')

# O `ros2` falso. Cenário por subida em $CALCO_DIR/cenario (linha "N tipo",
# tipo ok | falha_jsb | residuo); a subida corrente é o contador de launches.
ROS2 = textwrap.dedent(r'''
    #!/usr/bin/env bash
    D="$CALCO_DIR"
    echo "CALCO $* | AMENT=$AMENT_PREFIX_PATH" >> "$D/chamadas.txt"
    # Como o `ros2` real (achado de 28-09, 14h02): sem o ambiente do ROS
    # carregado ele quebra (PackageNotFoundError: ros2cli). Aqui, "carregado"
    # é o prefixo sujo que o teste injetou TER SAÍDO (reexecução limpa) E o
    # /opt/ros/jazzy TER ENTRADO (`source`).
    case ":$AMENT_PREFIX_PATH:" in
      *prefixo_sujo*|*"::"*) echo "calço: ambiente do ROS não carregado" >&2; exit 1 ;;
    esac
    case ":$AMENT_PREFIX_PATH:" in
      *:/opt/ros/jazzy:*) ;;
      *) echo "calço: ambiente do ROS não carregado" >&2; exit 1 ;;
    esac
    n="$(cat "$D/launches" 2>/dev/null || echo 0)"
    tipo="$(awk -v n="$n" '$1==n{print $2}' "$D/cenario" 2>/dev/null)"
    case "$1 $2" in
      "launch robot_motion")
        n=$((n + 1)); echo "$n" > "$D/launches"
        tipo="$(awk -v n="$n" '$1==n{print $2}' "$D/cenario")"
        [ "$tipo" = sigsegv_antes ] && echo "[ERROR] [collision_monitor-15]: process has died [pid 9, exit code -11, cmd 'x']"
        echo "[gazebo-1] [INFO] [1790000001.000000000] [controller_manager]: Activating controllers: [ joint_state_broadcaster ]"
        if [ "$tipo" = falha_jsb ]; then
          echo "[gazebo-1] [ERROR] [1790000006.000000000] [controller_manager]: Switch controller timed out after 5 seconds!"
          echo "[spawner-20] [ERROR] [1790000006.1] [spawner_joint_state_broadcaster]: Failed to activate controller : joint_state_broadcaster"
        else
          echo "[spawner-20] [INFO] [1790000001.500000000] [spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster"
        fi
        echo "[gazebo-1] [INFO] [1790000002.000000000] [controller_manager]: Activating controllers: [ hoverboard_base_controller ]"
        echo "[spawner-21] [INFO] [1790000002.200000000] [spawner_hoverboard_base_controller]: Configured and activated hoverboard_base_controller"
        # Python, não `trap`: processo lançado com `&` por shell não interativo
        # nasce com SIGINT IGNORADO, e o bash não reinstala sinal ignorado. A
        # `ros2 launch` real instala o próprio tratador; o calço também.
        exec python3 -c '
    import os, signal, sys, time
    def fim(*_):
        print("[x-1] [INFO] [1790000100.0] [rclcpp]: signal_handler(SIGINT/SIGTERM)", flush=True)
        tipo, shm = sys.argv[3], os.environ["SUBIDAS_SHM"]
        mortos = {"sigsegv_teardown": "collision_monitor-15",
                  "vivo_no_clean": "collision_monitor-15",
                  "shm_teimoso": "collision_monitor-15",
                  "sigsegv_outro": "heading_controller-14"}
        if tipo in mortos:
            print("[ERROR] [%s]: process has died [pid 9, exit code -11, cmd 'x']"
                  % mortos[tipo], flush=True)
        mortos["sigsegv_com_el"] = "collision_monitor-15"
        if tipo == "sigsegv_com_el":
            print("[ERROR] [collision_monitor-15]: process has died [pid 9, exit code -11, cmd 'x']", flush=True)
        orfaos = {"sigsegv_com_el": ("fastrtps_port7001", "fastrtps_port7001_el",
                                     "sem.fastrtps_port7001_mutex"),
                  "shm_teimoso": ("fastrtps_port7001", "sem.fastrtps_port7001_mutex",
                                  "fastrtps_abc123")}
        if tipo in mortos or tipo == "residuo":
            for n in orfaos.get(tipo, ("fastrtps_port7001", "sem.fastrtps_port7001_mutex")):
                open(os.path.join(shm, n), "w").close()
        if tipo == "shm_com_dono":
            alvo = os.path.join(shm, "fastrtps_port7009")
            open(alvo, "w").close()
            # Um dono SEM marca (a limpeza da subida não o vê): segura o
            # arquivo aberto; o teste o derruba no fim.
            import subprocess
            env = dict(os.environ); env.pop("VALIDA_ETAPA4_MARCA", None)
            p = subprocess.Popen(["python3", "-c",
                "import sys,time; f=open(sys.argv[1]); time.sleep(60)", alvo],
                env=env, start_new_session=True)
            open(os.path.join(sys.argv[1], "dono_pid"), "w").write(str(p.pid))
        open(os.path.join(sys.argv[1], "encerrado_" + sys.argv[2]), "w").close()
        sys.exit(0)
    signal.signal(signal.SIGINT, fim)
    signal.signal(signal.SIGTERM, fim)
    while True:
        time.sleep(0.1)
    ' "$D" "$n" "${tipo:-ok}" ;;
      "lifecycle get") echo "active [3]" ;;
      "action list") echo "/navigate_to_pose" ;;
      "run tf2_ros") echo "- Translation: [0.000, 0.000, 0.000]" ;;
      "service call")
        if [ "$tipo" = falha_jsb ]; then e=inactive; else e=active; fi
        echo "response:"
        echo "controller_manager_msgs.srv.ListControllers_Response(controller=[controller_manager_msgs.msg.ControllerState(name='joint_state_broadcaster', state='$e', type='x/Y'), controller_manager_msgs.msg.ControllerState(name='hoverboard_base_controller', state='active', type='x/Y')])" ;;
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
    # Cenário "0 residuo_no_build": o resíduo aparece ENTRE a varredura inicial
    # e a da subida 1 — o caminho do defeito de 14h12 (limpeza do `trap` numa
    # subida que não subiu, escrevendo depois do manifesto).
    grep -qx '0 residuo_no_build' "$CALCO_DIR/cenario" && touch "$SUBIDAS_SHM/fastrtps_build"
    echo "Summary: 3 packages finished"
''').lstrip()

# O `fastdds shm clean` falso: remove os órfãos do /dev/shm DE TESTE, salvo no
# cenário shm_teimoso, em que um fica.
FASTDDS = textwrap.dedent(r'''
    #!/usr/bin/env bash
    echo "CALCO fastdds $*" >> "$CALCO_DIR/chamadas.txt"
    [ "$1 $2" = "shm clean" ] || exit 97
    n="$(cat "$CALCO_DIR/launches")"
    tipo="$(awk -v n="$n" '$1==n{print $2}' "$CALCO_DIR/cenario")"
    # Como o real (medido em 28-09, 14h28): só remove o que tem a trava _el.
    for el in "$SUBIDAS_SHM"/*_el; do
      [ -e "$el" ] || continue
      base="${el%_el}"; porta="$(basename "$base")"
      rm -f "$el" "$base" "$SUBIDAS_SHM/sem.${porta}_mutex"
    done
    # Cenário vivo_no_clean: algo com cara de ROS fica vivo DEPOIS do clean.
    if [ "$tipo" = vivo_no_clean ]; then
      setsid bash -c 'exec -a ros2 sleep 60' < /dev/null > /dev/null 2>&1 &
      echo $! > "$CALCO_DIR/vivo_pid"
    fi
    echo "shm.clean:"; echo "1 zombie segments cleaned"
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


def _bateria(tmp_path, cenario, n, shm_sujo=False, extra_env=None,
             exige_calco=True):
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
    _executavel(calcos / 'fastdds', FASTDDS)
    (calcos / 'cenario').write_text(
        ''.join(f'{i} {cenario.get(i, "ok")}\n' for i in range(0, n + 1)))
    saidas = tmp_path / 'saidas'
    shm = tmp_path / 'shm'          # nunca o /dev/shm real: o teste é hermético
    shm.mkdir()
    if shm_sujo:
        (shm / 'fastrtps_port7018').write_text('')
    env = dict(os.environ, SUBIDAS_CALCOS=str(calcos), CALCO_DIR=str(calcos),
               SUBIDAS_RAIZ_SAIDA=str(saidas), SUBIDAS_DOMINIO='91',
               SUBIDAS_SHM=str(shm),
               # Sujo DE PROPÓSITO: a reexecução limpa tem de tirá-lo, e o
               # `source` tem de pôr o /opt/ros/jazzy — o calço confere os dois.
               AMENT_PREFIX_PATH='/nao/existe/prefixo_sujo')
    env.pop('VALIDA_ETAPA4_MARCA', None)
    env.pop('SUBIDAS_AMBIENTE_LIMPO', None)
    env.update(extra_env or {})
    r = subprocess.run(['bash', str(repo / 'bin' / 'subidas-robo3'), str(n)],
                       capture_output=True, text=True, timeout=300, env=env)
    pastas = sorted(saidas.iterdir()) if saidas.exists() else []
    assert len(pastas) == 1, (r.stdout[-3000:], r.stderr[-2000:])
    pasta = pastas[0]
    if not exige_calco:
        return r, pasta, calcos
    chamadas = (calcos / 'chamadas.txt').read_text()
    assert 'CALCO launch robot_motion' in chamadas, 'o calço não foi usado'
    if not cenario and not shm_sujo:
        assert not list(shm.iterdir()), 'sobrou segmento no shm de teste'
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
    assert [l['subida_nominal'] for l in linhas] == ['1', '1', '1']
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
    assert [l['subida_nominal'] for l in linhas] == ['1', '0', '1']
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


def test_os_controladores_sao_lidos_pelo_servico_e_nao_pelo_ros2_control():
    """`ros2 control` não existe neste PC; usá-lo faria TODA subida sair não
    nominal pela ferramenta, não pela pilha."""
    texto = open(WRAPPER).read() if os.path.exists(WRAPPER) else ''
    assert 'ros2 control ' not in texto
    assert ('ros2 service call /controller_manager/list_controllers '
            'controller_manager_msgs/srv/ListControllers') in texto


def test_dominio_e_os_argumentos_da_corrida_do_passo_7():
    texto = open(WRAPPER).read() if os.path.exists(WRAPPER) else ''
    assert 'SUBIDAS_DOMINIO:-50' in texto
    for arg in ('robo:=3', 'sim:=true', 'gui:=false', 'rviz:=false',
                'localizacao:=fixa', 'bag:=false'):
        assert arg in texto, arg


def test_segmento_dds_orfao_impede_a_primeira_subida(tmp_path):
    """Fast DDS morto por KILL deixa segmento em /dev/shm (achado de 28-09: o
    `ros2 bag record` do robô 2 no `valida-etapa4`). É resíduo: nada sobe."""
    if not os.path.exists(WRAPPER):
        pytest.fail('wrapper ainda não existe')
    repo_e_calcos = tmp_path
    with pytest.raises(AssertionError, match='o calço não foi usado'):
        _bateria(repo_e_calcos, {}, 2, shm_sujo=True)
    pasta = next((tmp_path / 'saidas').iterdir())
    assert 'fastrtps_port7018' in (pasta / 'residuo_inicial.txt').read_text()
    assert not (pasta / 'subida_01').exists()


def test_a_ordem_limpa_source_e_so_entao_ros2(tmp_path):
    """A reexecução REAL do wrapper, de ponta a ponta: o prefixo sujo some, o
    `source /opt/ros/jazzy/setup.bash` entra, e só então a PRIMEIRA chamada
    ao `ros2` — que é a varredura de resíduo inicial. Falhou assim em 28-09
    (14h02): a varredura vinha antes do `source`."""
    r, pasta, linhas, launches = _bateria(tmp_path, {}, 1)
    assert r.returncode == 0, r.stdout[-3000:]
    primeira = (tmp_path / 'calcos' / 'chamadas.txt').read_text().splitlines()[0]
    assert primeira.startswith('CALCO node list'), primeira
    assert '/opt/ros/jazzy' in primeira and 'prefixo_sujo' not in primeira


def test_source_que_falha_para_antes_de_qualquer_ros2(tmp_path):
    ruim = tmp_path / 'setup_ruim.bash'
    ruim.write_text('return 1\n')
    r, pasta, calcos = _bateria(tmp_path, {}, 1, exige_calco=False,
                                extra_env={'SUBIDAS_ROS_SETUP': str(ruim)})
    assert r.returncode == 1
    assert not (calcos / 'chamadas.txt').exists(), 'chamou ros2 sem o ambiente'
    assert not (pasta / 'subida_01').exists()


# ─── o teardown com SIGSEGV e a limpeza recuperada (regra de 28-09, 14h) ──────

def _chamadas(tmp_path):
    return (tmp_path / 'calcos' / 'chamadas.txt').read_text()


def test_sigsegv_do_collision_monitor_no_teardown_e_limpeza_recuperada(tmp_path):
    r, pasta, linhas, launches = _bateria(tmp_path, {2: 'sigsegv_teardown'}, 3)
    assert launches == 3 and len(linhas) == 3, 'a bateria tinha de continuar'
    assert [l['subida_nominal'] for l in linhas] == ['1', '1', '1']
    assert [l['sigsegv_teardown'] for l in linhas] == ['0', '1', '0']
    assert [l['limpeza_recuperada'] for l in linhas] == ['0', '1', '0']
    assert linhas[1]['shm_orfaos'] == '2' and linhas[0]['shm_orfaos'] == '0'
    # sem _el, o fastdds não remove: quem recupera é a remoção manual controlada
    assert [l['limpeza_manual_recuperada'] for l in linhas] == ['0', '1', '0']
    assert [l['teardown_anomalo'] for l in linhas] == ['0', '1', '0']
    rem = (pasta / 'subida_02' / 'shm_remocao_manual.txt').read_text()
    assert 'fastrtps_port7001' in rem and 'sem.fastrtps_port7001_mutex' in rem
    assert _chamadas(tmp_path).count('CALCO fastdds shm clean') == 1
    inv = (pasta / 'subida_02' / 'shm_inventario.txt').read_text()
    assert 'fastrtps_port7001' in inv and 'fuser' in inv
    assert 'collision_monitor-15' in inv and 'exit code -11' in inv
    assert not list((tmp_path / 'shm').iterdir())
    v = (pasta / 'veredito.txt').read_text()
    assert '1 limpeza(s) recuperada(s)' in v and '1 com remoção manual' in v
    assert _manifesto_de_fora(pasta).returncode == 0


def test_segmento_que_o_fastdds_nao_remove_interrompe(tmp_path):
    r, pasta, linhas, launches = _bateria(tmp_path, {1: 'shm_teimoso'}, 3)
    assert r.returncode == 1 and launches == 1
    # A parada é NA subida 1, pela recontagem — não na varredura da 2 — e a
    # linha não pode dizer "recuperada" de uma limpeza que não recuperou.
    assert linhas[0]['limpeza_recuperada'] == '0', linhas[0]
    assert not (pasta / 'subida_02').exists()
    assert 'a remoção manual controlada recusou' in r.stdout
    rel = (pasta / 'subida_01' / 'shm_remocao_manual.txt').read_text()
    assert 'fastrtps_abc123' in rel and 'nada removido' in rel
    # um fora do padrão e NENHUM sai — nem os dois que seriam válidos
    restos = sorted(p.name for p in (tmp_path / 'shm').iterdir())
    assert restos == ['fastrtps_abc123', 'fastrtps_port7001',
                      'sem.fastrtps_port7001_mutex'], restos
    assert (pasta / 'veredito.txt').read_text().startswith('INCOMPLETA')
    assert _manifesto_de_fora(pasta).returncode == 0


@pytest.mark.parametrize('tipo', ('sigsegv_outro', 'sigsegv_antes'))
def test_sinal_fora_da_regra_interrompe_sem_limpar_shm(tmp_path, tipo):
    r, pasta, linhas, launches = _bateria(tmp_path, {1: tipo}, 3)
    assert r.returncode == 1 and launches == 1
    assert 'CALCO fastdds' not in _chamadas(tmp_path)
    assert (pasta / 'veredito.txt').read_text().startswith('INCOMPLETA')


def test_residuo_vivo_com_shm_nao_chama_o_fastdds(tmp_path):
    """Nó vivo no domínio E segmento órfão: nada de `fastdds shm clean` —
    ele só roda com nada vivo."""
    r, pasta, linhas, launches = _bateria(tmp_path, {1: 'residuo'}, 3)
    assert r.returncode == 1 and launches == 1
    assert 'CALCO fastdds' not in _chamadas(tmp_path)


def test_residuo_antes_da_subida_nao_escreve_depois_do_manifesto(tmp_path):
    """O defeito de 14h12: barrada ANTES de subir, a subida ganhava uma limpeza
    do `trap` que escrevia depois do SHA256SUMS; e o escopo exigia o
    `launch.log` de uma subida que não subiu."""
    r, pasta, calcos = _bateria(tmp_path, {0: 'residuo_no_build'}, 2,
                                exige_calco=False)
    assert r.returncode == 1
    assert 'CALCO launch' not in (calcos / 'chamadas.txt').read_text()
    m = _manifesto_de_fora(pasta)
    assert m.returncode == 0, m.stdout + m.stderr
    assert 'escopo sem os artefatos' not in r.stdout
    assert (pasta / 'veredito.txt').read_text().startswith('INCOMPLETA')


def test_segmento_com_dono_nao_e_limpo(tmp_path):
    """Só se roda `fastdds shm clean` com TODOS os segmentos sem dono."""
    try:
        r, pasta, linhas, launches = _bateria(tmp_path, {1: 'shm_com_dono'}, 3)
        assert r.returncode == 1 and launches == 1
        assert 'CALCO fastdds' not in _chamadas(tmp_path)
        assert 'fastrtps_port7009' in (pasta / 'subida_01' / 'shm_inventario.txt').read_text()
    finally:
        dono = tmp_path / 'calcos' / 'dono_pid'
        if dono.exists():
            try:
                os.kill(int(dono.read_text()), 9)
            except ProcessLookupError:
                pass


def test_fastdds_sozinho_basta_quando_ha_el(tmp_path):
    r, pasta, linhas, launches = _bateria(tmp_path, {1: 'sigsegv_com_el'}, 2)
    assert launches == 2
    assert linhas[0]['limpeza_recuperada'] == '1'
    assert linhas[0]['limpeza_manual_recuperada'] == '0'
    assert linhas[0]['teardown_anomalo'] == '1'
    assert not (pasta / 'subida_01' / 'shm_remocao_manual.txt').exists()


def test_algo_vivo_antes_da_remocao_manual_interrompe(tmp_path):
    """O "nada vivo" é RECONFERIDO imediatamente antes da remoção manual."""
    try:
        r, pasta, linhas, launches = _bateria(tmp_path, {1: 'vivo_no_clean'}, 2)
        assert r.returncode == 1 and launches == 1
        assert 'algo vivo antes da remoção manual' in r.stdout
        assert not (pasta / 'subida_01' / 'shm_remocao_manual.txt').exists()
        assert (tmp_path / 'shm' / 'fastrtps_port7001').exists()
    finally:
        vivo = tmp_path / 'calcos' / 'vivo_pid'
        if vivo.exists():
            try:
                os.kill(int(vivo.read_text()), 9)
            except ProcessLookupError:
                pass
