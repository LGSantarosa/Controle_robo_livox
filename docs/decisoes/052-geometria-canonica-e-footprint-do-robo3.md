# 052 — Geometria canônica do robô 3: footprint sem padding, num arquivo só

**Data**: 2026-09-18 (PC de dev, robô desligado)
**Status**: aplicada. ~~Nenhum consumidor ainda~~ — **consumidor desde
2026-09-22**: o perfil do robô 3 (decisão 053) lê o `poligono` e o
`raio_varrido_pivo` deste artefato. A pilha ainda recusa `robo:=3` (etapa 6).
**Toca**: `robot_base/config/geometria_robo3.yaml` (novo), `robot_base/test/test_urdf_robo3.py`,
`docs/PLANO_NAV2_ROBO3.md` (linhas das etapas 3 e 4).
**Vem de**: decisão 051 (URDF girado) e §3 item 5 / §5 (b) do plano.

---

## 1. A conta (do URDF girado, no `base_link`)

    rígido:     x de −0,2485 (traseira da caixa) a +0,0825 (frente do pneu), y ±0,190
    boba:       raio varrido = √((trail + r)² + (larg/2)²) = √(0,040² + 0,015²) = 0,04272
                pivôs em (−0,2485, ±0,105) → traseira varrida x = −0,29122
                lateral varrida 0,14772 < 0,190 → o pneu continua mandando na largura

Conta conferida de forma independente pelo dono.

## 2. O que foi decidido

1. **Footprint = geometria pura, classe (b).** A folga é `footprint_padding`,
   classe (c), no perfil. O robô 2 (decisão 032) embute ~5 cm dentro do
   polígono; aqui não, e há teste que reprova folga embutida.
2. **Forma: caixa delimitadora dos extremos** —
   `[[0.0825, 0.19], [0.0825, -0.19], [-0.2913, -0.19], [-0.2913, 0.19]]`.
   É envolvente **retangular conservadora**: tem área **vazia** nas quinas de
   trás (fora dos círculos das bobas) e na frente (fora dos pneus). Isso é
   aproximação de forma, **não margem de segurança**, e está escrito no arquivo.
3. **Arredondado para FORA**: −0,29122 vira −0,2913. O −0,2912 deixaria 0,02 mm
   da varredura de fora — o teste de cobertura reprova (conferido por mutação).
4. **Mora num artefato legível por máquina**, não dentro do teste nem desta
   decisão. Condição do dono: um polígono que só existe no teste é teste de um
   artefato que ninguém consome — o "teste que não afirma nada" que o plano já
   recusou.
5. **Na etapa 4**, o perfil Nav2 do robô 3 lê este arquivo ou é amarrado a ele
   por igualdade — vértices nunca redigitados — e declara `footprint_padding`
   **explícito**. Ausente, o Nav2 aplica o padrão dele como margem escondida.

## 3. Os testes, e que eles mordem

- `test_a_varredura_das_bobas_sai_4_cm_atras_e_nao_alarga` — raio e traseira
  calculados do URDF.
- `test_footprint_canonico_cobre_corpo_e_varredura` — quinas da caixa e dos
  pneus + 360 pontos do círculo de cada boba, dentro do polígono.
- `test_footprint_canonico_NAO_carrega_folga` — cada lado encosta no extremo,
  com tolerância só de arredondamento (0,1 mm, para fora).

Mutações feitas à mão e revertidas: traseira em −0,2912 → cobertura e folga
reprovam; 5 cm embutidos → folga reprova; traseira na caixa (sem varredura) →
cobertura reprova.

## 4. Fronteira das etapas 3 e 4, mudada formalmente

O plano dizia que a etapa 3 entregava footprints e que "o Nav2 passa a ter
contorno". Com o consumidor nascendo na 4, **não dá** para marcar a 3 como feita
dizendo isso. Opção escolhida: **mover a fronteira**. A 3 entrega o URDF girado
e o artefato geométrico travado; a 4 herda, escrito na linha dela, o consumo
pelo Nav2, o `footprint_padding` explícito, os outros consumidores de geometria
(`collision_monitor`, meia largura, corredor de ré, recuo do para-choque) e o
`robot_state_publisher` — que já estava listado nas duas.

## 5. Alternativas

| alternativa | por que não |
|---|---|
| polígono só no teste/decisão | teste sem consumidor (condição do dono, §2.4) |
| escrever já no `nav2.yaml` | é o perfil do robô 2; perfis são a etapa 4 |
| polígono justo (seguindo os círculos das bobas) | ganharia ~105 cm² na faixa de trás (163 cm² de faixa − 57 de meios-círculos) — real, e fica como opção se a traseira apertar em algum lugar; hoje nada medido depende disso. Retângulo é mais fácil de conferir a olho |
| folga dentro do polígono, como no robô 2 | mistura (b) com (c); some dos dois lugares |
