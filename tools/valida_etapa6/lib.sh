# Funções comuns do bin/valida-etapa6 — CÓPIA da lib da etapa 5
# (tools/valida_etapa5/lib.sh, que já era cópia da etapa 4). Não refatorada de
# propósito: o `bin/valida-etapa5` é evidência congelada de uma etapa fechada, e
# uma lib compartilhada faria uma correção de hoje mudar o significado de uma
# prova de ontem. Etapa 6, docs/PLANO_ETAPA6_ROBO3.md §5.2. Carregado com
# `source`; separado para a limpeza e a saída antecipada serem testáveis
# (test_valida_etapa6.py) sem subir Gazebo nenhum.
#
# Espera definidos: SAIDA, GRUPOS, RESULTADO, V (tools/valida_etapa6) e
# DOMINIOS_USADOS. A marca VALIDA_ETAPA4_MARCA é exportada por quem carrega,
# DEPOIS das pré-condições, e só então `instala_limpeza`.
#
# 🔴 O NOME DA MARCA CONTINUA SENDO `VALIDA_ETAPA4_MARCA`, e isso é de
# propósito: é ele que a pré-condição de TODOS os validadores procura em
# /proc/*/environ. Renomear por etapa faria a rodada de hoje não ver o processo
# sobrevivente da rodada de ontem — e a sobra invisível é exatamente o que essa
# pré-condição existe para pegar.
#
# Sinal NUNCA por PGID ou nome cru, e NUNCA `pkill -f`: quem sinaliza é o
# processos.py, que classifica cada grupo (líder com o STARTTIME registrado, ou
# órfão com todos os membros marcados) e reconfere pid + starttime + marca logo
# antes de cada kill. O `pkill -f` já matou uma sessão ssh neste projeto
# (bin/smoke-gazebo-robo3).

# 🔧 MUDANÇA DA ETAPA 6 em relação à cópia: `anota` escreve TAMBÉM uma linha de
# CSV, porque o §5.2 do plano pede "os itens do §4, um por linha do CSV, com
# veredito por item e no fim". O texto alinhado continua, para ler no console;
# o CSV é o que eu leio depois sem contar colunas. Uma fonte, dois formatos —
# escrever os dois em lugares separados é como eles passam a discordar.
anota() {   # anota <item> <APROVADO|REPROVADO|ANOTADO> [detalhe]
  printf '%-58s %s %s\n' "$1" "$2" "${3:-}" >> "$RESULTADO"
  if [ -n "${RESULTADO_CSV:-}" ]; then
    # Aspas duplas dobradas: é o escape do CSV, e detalhe com vírgula dentro
    # (lista de nós, caminho) é a regra aqui, não a exceção.
    printf '"%s","%s","%s"\n' "${1//\"/\"\"}" "$2" "${3//\"/\"\"}" >> "$RESULTADO_CSV"
  fi
  local m
  case "$2" in APROVADO) m=🟢 ;; REPROVADO) m=🔴 ;; *) m=⚪ ;; esac
  echo "   $m $1 — $2 ${3:-}"
}

registra_cmd() { echo "$(date +%T.%3N) [dominio ${ROS_DOMAIN_ID}] $*" >> "$SAIDA/comandos.txt"; }

roda() {     # roda <log> <cmd...> — primeiro plano; saída no log e (fim) no console
  local log="$1" rc; shift
  registra_cmd "$* > $(basename "$log")"
  "$@" > "$log" 2>&1
  rc=$?
  sed 's/^/      /' "$log" | tail -40
  return $rc
}

stat_de() {  # "ESTADO PGRP STARTTIME" — campos depois do último ") "
  local s; { read -r s < "/proc/$1/stat"; } 2>/dev/null || return 1
  s="${s##*") "}"
  # shellcheck disable=SC2086
  set -- $s
  echo "$1 $3 ${20}"
}

sobe_grupo() {   # sobe_grupo <papel> <log> <cmd...> — grupo próprio, registrado
  local papel="$1" log="$2" pid st i est pg start=""; shift 2
  registra_cmd "setsid $* > $(basename "$log")"
  setsid "$@" > "$log" 2>&1 < /dev/null &
  pid=$!
  for i in $(seq 1 40); do
    if st="$(stat_de "$pid")"; then
      read -r est pg start <<< "$st"
      [ "$pg" = "$pid" ] && break
    fi
    start=""; sleep 0.05
  done
  if [ -z "$start" ]; then
    # Não virou líder: o PID é filho direto deste shell, então sinal nele é
    # seguro — mas o grupo não é nosso para sinalizar.
    kill -INT "$pid" 2>/dev/null
    anota "$papel: subiu como líder do próprio grupo" REPROVADO "(PID $pid)"
    return 1
  fi
  echo "$pid $start $papel" >> "$GRUPOS"
  echo "   $papel: PID=PGID $pid (starttime $start)"
  ULTIMO_PID=$pid
}

# Lista crua de nós, sem daemon. DEVOLVE o código do ros2: saída vazia com
# erro NÃO é "domínio vazio" — todo chamador testa o retorno.
nos_do_dominio() {   # nos_do_dominio <dominio> <saida.txt>
  ROS_DOMAIN_ID="$1" ros2 node list --no-daemon --spin-time 5 > "$2" 2> "$2.err"
}

# ── dispositivos /dev/input/js* (pré-condição do mega-fingida) ──────────────
#
# O kernel cria jsN para qualquer aparelho com eixo absoluto — inclusive mouse
# virtual. Quem decide o que o joy_node (SDL) abre é a marca do udev. Veredito:
#   JOYSTICK      ID_INPUT_JOYSTICK=1                    → recusa
#   MOUSE         ID_INPUT_MOUSE=1 e sem ID_INPUT_JOYSTICK → permite, registrado
#   INCONCLUSIVO  udevadm falhou, ou nenhuma das duas     → recusa por segurança
# A comparação exata dos frames é defesa ADICIONAL, não substitui isto.
classifica_js() {   # classifica_js </dev/input/jsN> — imprime o veredito
  local props
  if ! props="$(udevadm info --query=property --name="$1" 2>/dev/null)" || [ -z "$props" ]; then
    echo INCONCLUSIVO; return
  fi
  if grep -qx 'ID_INPUT_JOYSTICK=1' <<< "$props"; then echo JOYSTICK
  elif grep -qx 'ID_INPUT_MOUSE=1' <<< "$props"; then echo MOUSE
  else echo INCONCLUSIVO
  fi
}

descreve_js() {     # descreve_js </dev/input/jsN> — tudo o que se sabe dele
  local s="/sys/class/input/$(basename "$1")/device"
  echo "$1: veredito $(classifica_js "$1")"
  echo "   nome: $(cat "$s/name" 2>/dev/null) · id $(cat "$s/id/bustype" "$s/id/vendor" "$s/id/product" 2>/dev/null | paste -sd:)"
  echo "   sysfs: $(readlink -f "$s" 2>/dev/null)"
  udevadm info --query=property --name="$1" 2>&1 | grep -E '^(ID_INPUT|DEVNAME|ID_VENDOR|ID_MODEL)' | sed 's/^/   /'
}

# ── limpeza (idempotente; o `trap EXIT` garante que roda em toda saída) ──────

LIMPO=0
limpa() {
  [ "$LIMPO" -eq 1 ] && return 0
  LIMPO=1
  local L="$SAIDA/limpeza.txt" i n ok=1 d f rc
  echo "== limpeza ==" | tee -a "$L"
  echo "grupos do wrapper (PGID STARTTIME PAPEL):" >> "$L"
  sed 's/^/   /' "$GRUPOS" >> "$L"
  if [ -s "$GRUPOS" ]; then
    python3 "$V/processos.py" sinaliza_grupos "$GRUPOS" INT >> "$L" 2>&1
    for i in $(seq 1 100); do
      n="$(python3 "$V/processos.py" vivos_grupos "$GRUPOS" 2>>"$L")"
      [ "$n" = 0 ] && break
      sleep 0.2
    done
    if [ "$n" != 0 ]; then
      echo "KILL: $n membro(s) não saíram com INT em 20 s" >> "$L"
      python3 "$V/processos.py" sinaliza_grupos "$GRUPOS" KILL >> "$L" 2>&1
      sleep 1
    fi
  fi
  # Quem ainda tem a marca DESTA pasta (ex.: o ros2-daemon que o sobe-robo3
  # sobe ao consultar o grafo) é nosso por construção — anotado e derrubado,
  # cada um reconferido antes do sinal.
  if ! python3 "$V/processos.py" marcados > "$SAIDA/sobras_marcadas.txt" 2>&1; then
    echo "sobras com a marca desta rodada:" >> "$L"
    sed 's/^/   /' "$SAIDA/sobras_marcadas.txt" >> "$L"
    python3 "$V/processos.py" sinaliza_marcados INT >> "$L" 2>&1
    sleep 3
    python3 "$V/processos.py" sinaliza_marcados KILL >> "$L" 2>&1
    sleep 1
  fi
  if python3 "$V/processos.py" marcados >> "$L" 2>&1; then
    echo "processos marcados depois: 0" >> "$L"
  else
    echo "🔴 AINDA HÁ PROCESSO MARCADO" >> "$L"; ok=0
  fi
  for d in $DOMINIOS_USADOS; do
    f="$SAIDA/nos_depois_da_limpeza_dominio_$d.txt"
    nos_do_dominio "$d" "$f"; rc=$?
    if [ "$rc" -ne 0 ]; then
      echo "🔴 domínio $d: consulta ao grafo FALHOU (código $rc) — não prova nada" >> "$L"; ok=0
    elif [ -s "$f" ]; then
      echo "🔴 domínio $d ainda tem nós:" >> "$L"; sed 's/^/   /' "$f" >> "$L"; ok=0
    else
      echo "domínio $d: nenhum nó (consulta com código 0)" >> "$L"
    fi
  done
  if [ "$ok" -eq 1 ]; then
    anota "limpeza: nada marcado, domínios vazios" APROVADO
  else
    anota "limpeza: nada marcado, domínios vazios" REPROVADO "(limpeza.txt)"
  fi
  sed 's/^/   /' "$L"
}

instala_limpeza() {
  trap 'limpa' EXIT
  trap 'echo; echo "   interrompido — limpando antes de sair"; exit 130' INT TERM
}
