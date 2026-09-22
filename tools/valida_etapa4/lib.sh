# Funções comuns do bin/valida-etapa4 — etapa 4, passo 7. Carregado com
# `source`; separado para a limpeza e a saída antecipada serem testáveis
# (test_valida_etapa4.py) sem subir cenário nenhum.
#
# Espera definidos: SAIDA, GRUPOS, RESULTADO, V (tools/valida_etapa4) e
# DOMINIOS_USADOS. A marca VALIDA_ETAPA4_MARCA é exportada por quem carrega,
# DEPOIS das pré-condições, e só então `instala_limpeza`.
#
# Sinal NUNCA por PGID ou nome cru: quem sinaliza é o processos.py, que
# classifica cada grupo (líder com o STARTTIME registrado, ou órfão com todos
# os membros marcados) e reconfere pid + starttime + marca logo antes de cada
# kill.

anota() {   # anota <item> <APROVADO|REPROVADO|ANOTADO> [detalhe]
  printf '%-58s %s %s\n' "$1" "$2" "${3:-}" >> "$RESULTADO"
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
