#!/usr/bin/env bash
# Continuidade da baseline v2 (21-09). Rodar da raiz do repo; só lê arquivos.
V2=docs/dados/2026-09-21-baseline-v2-robo2/02-baseline-v2-aprovada
P2=docs/dados/2026-09-21-passo2-robo2/captura
ORIG=docs/dados/2026-09-21-baseline-robo2/02-baseline-aprovada
python3 - "$V2" "$P2" "$ORIG" <<'PY'
import importlib.util, sys, yaml
spec = importlib.util.spec_from_file_location('nz', 'tools/linha_de_base/normaliza.py')
nz = importlib.util.module_from_spec(spec); spec.loader.exec_module(nz)
v2d, p2d, od = sys.argv[1:4]
v2, p2, orig = (nz.le(f'{d}/parametros_normalizados.yaml') for d in (v2d, p2d, od))
NOVOS = {'/collision_monitor', '/controller_manager'}
tira = lambda d: {k: v for k, v in d.items() if k not in NOVOS}
print('nós com dump na v2:', len(v2), '| antigos (sem os 2 novos):', len(tira(v2)))
print('os 2 novos na original e no passo 2:', {k: len(orig[k]) for k in sorted(NOVOS)},
      {k: len(p2[k]) for k in sorted(NOVOS)}, '(vazios: o furo)')
print('\n1) v2 x passo 2, 23 nós antigos, sem permissão:')
print(nz.relatorio(nz.compara(tira(p2), tira(v2))))
bv2 = yaml.safe_load(open(f'{v2d}/parametros_brutos.yaml'))
bp2 = yaml.safe_load(open(f'{p2d}/parametros_brutos.yaml'))
print('   brutos, nós diferentes:', [k for k in set(bv2) | set(bp2) if k not in NOVOS and bv2.get(k) != bp2.get(k)])
print('\n2) v2 x baseline original, 23 nós, com a permissão do passo 2:')
print(nz.relatorio(nz.compara(tira(orig), tira(v2),
      nz.le_permitidas('tools/linha_de_base/permitidas/passo2_robo2.yaml'))))
print('\n3) conteúdo novo:', {k: len(v2[k]) for k in sorted(NOVOS)})
src = yaml.safe_load(open('ros2_packages/robot_motion/config/collision_monitor.yaml'))
src = src['collision_monitor']['ros__parameters']
for pol in ('PolygonStop', 'PolygonApproach'):
    print(f'   {pol}.points vivo == collision_monitor.yaml:', v2['/collision_monitor'][f'{pol}.points'] == src[pol]['points'])
PY
echo
echo "   fontes dos 2 nós desde a baseline original (feb064a) até a captura (52bbffc):"
git diff --stat feb064a 52bbffc -- ros2_packages/robot_motion/config/collision_monitor.yaml \
  ros2_packages/robot_base/config/ ros2_packages/robot_base/description/ \
  ros2_packages/robot_base/launch/sim.launch.py ros2_packages/robot_motion/launch/pilha.launch.py
echo "   (fim do diff — vazio = nada mudou)"
