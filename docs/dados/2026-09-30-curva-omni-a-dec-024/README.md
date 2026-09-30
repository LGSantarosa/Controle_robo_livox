# Curvas com roda omni — `a_dec: 0,24`

Sessão no robô real em 2026-09-30, com o commit `3d4edcf` instalado.

Configuração relevante:

- `heading_controller.a_dec: 0,24`;
- teto angular do Nav2: `1,25 rad/s`;
- velocidade linear, compensador e lei do pivô não foram alterados.

Resultado relatado pelo dono: o robô ficou **ótimo e melhorou muito**. O pivô
está funcionando muito bem. O problema restante aparece nas diagonais e nas
curvas abertas: a roda omni arrasta lateralmente, o conjunto fica pesado e o
Nav2 ainda precisa de mais autoridade de giro para seguir o plano.

## Próximo passo

Fazer uma única mudança experimental no Nav2: aumentar `a_dec` em mais 15%, de
`0,24` para `0,276`. Como a lei usa `wz = sqrt(2*a_dec*erro)`, isso representa
aproximadamente 7,2% a mais de comando angular para o mesmo erro. Preservar o
pivô, a velocidade linear, o compensador e os tetos atuais.

## Evidência salva

Os CSVs e logs de texto desta sessão estão neste diretório. O bag completo foi
fechado corretamente e ficou no NUC, pois tem 8.200.572.569 bytes e não deve
entrar no Git:

`/home/bara/logs_robo2/corrida_2026-09-30_182443/`

O `metadata.yaml` do bag foi copiado para cá. `SHA256SUMS` registra a integridade
dos arquivos versionados e também o hash do MCAP preservado no NUC.

SHA-256 do MCAP completo:

`c3ea3a0b0350e6e0eeaf62ea99d51a9ea4cc459fc343521c9ea06ad81c03cfec`
