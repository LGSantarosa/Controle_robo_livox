2026-09-15 — robô 3: pivô no chão pelo Xbox, PARCIAL (notebook caiu)
====================================================================

Diário: docs/DIARIO.md, 15-09 (NOTEBOOK, ~17:00). Roteiro de repetição:
ESTADO_PROJETO.md, "AMANHÃ (16-09)".

pivo_chao_170422.csv   gravador só de leitura ao lado da pilha do Xbox
                       (versão solta; a versionada é tools/grava_pivo.py, que
                       também grava bateria). Colunas: t (s, chegada), topico
                       (rodas | mega), FL/FR = rpm canal L/R da placa,
                       steer/speed = o que a MEGA escreveu na placa.

Janelas |steer| = 484, regime a partir de 1 s:
  112,7–118,3 s  steer −484, speed −17…−21   FL −125,9  FR +78,9   (L, a de trás, mais rápida)
  125,9–127,3 s  steer +484, speed 0          FL +91,0   FR −122,5  (R, a de trás, mais rápida)

17:06:30 USB da MEGA desconectou (queda do notebook); o resto do arquivo é a
pilha presa em /dev/ttyACM0 sem comando. NÃO CONCLUSIVO: pivô esquerdo com speed
contaminado, direito com 21 amostras.
