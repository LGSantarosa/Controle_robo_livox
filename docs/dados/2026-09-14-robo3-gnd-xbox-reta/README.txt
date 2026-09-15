2026-09-14 — robô 3: GND, Xbox e a reta que puxa para a direita
================================================================

Diário: docs/DIARIO.md, entradas de 14-09 (noite, PC dev) e 14-09 (notebook).
Decisão: docs/decisoes/048-xbox-do-robo3-sinal-giro-e-direcional.md.
MEGA serial 55632313039351D05132 em todos.

ANTES DO GND (PC dev, fio do GND com defeito)
---------------------------------------------
sonda_1940_sem_pullup_rodas_robo.txt   hover_probe sem pull-up, rodas do robô: 0 quadros
sonda_1945_sem_pullup_rodas_dia1.txt   idem com as rodas do dia 1: 0 quadros (rodas descartadas)
sonda_1949_com_pullup.txt              hover_probe + pull-up no 19: 0 quadros; placa armou e
                                       girou sozinha com a MEGA mandando zero; 'g' sem efeito
teclado_20260914_195954.csv            hover_ponte: 20 ms cravados, todo comando != 0 veio de tecla;
                                       rodas giraram sozinhas com o PC em zero
teclado_20260914_200548.csv            + leitura da volta: 66 695 bytes, 0 quadros 0xABCD;
                                       3 marcas 'm' de giro sozinho, PC em zero
teclado_20260914_201104.csv(.rx.bin)   bytes crus: 4,4 % ASCII, 0xBF/0xFF dominam, 'cd ab' vazando
teclado_chkerrado_20260914_201521.csv(.rx.bin)
                                       checksum invertido: rodas imóveis (a placa lê serial);
                                       mesmo ruído no 19

DEPOIS DO GND (fio trocado, mesmos pontos)
------------------------------------------
teclado_20260914_202729.csv(.rx.bin)   7 310 quadros válidos (99,6/s), bateria 40,8 V,
                                       cmd1/cmd2 = mandado, 0 giro sozinho.
                                       No ar: speed ±250 reto ~117–121 rpm;
                                       steer 150 em pivô ~17 rpm (base da decisão 048);
                                       +250: canal L = R (−0,1 %); −250: L +2,7 %.

XBOX NO NOTEBOOK
----------------
joy_dpad_204304.csv                    ros2 topic echo /joy --csv: direcional cima/baixo =
                                       eixo 7 (+1 cima), esquerda/direita = eixo 6 (+1 esquerda),
                                       LB = botão 6
reta_chao_211439.csv                   NO CHÃO, cabos das rodas trocados na placa, LB + direcional
                                       (steer 0 exato). Colunas: t (s, chegada), topico
                                       (rodas|dpad), FL/FR = rpm canal L/R da placa, dpad_x.
                                       Frente: FL +42,3 x FR +38,1 rpm (+9,8 %, 3/3 corridas);
                                       ré: −1,8 %.
grava_reta.py                          o gravador que produziu reta_chao (rclpy, só leitura)

Para recalcular: regime a partir de 1 s de cada janela de dpad_x != 0, só
amostras com |FL| ou |FR| > 5 rpm.

Não guardado: bags mcap (ignorados pelo .gitignore; controle_20260914_205608
ficou sem índice no notebook).
