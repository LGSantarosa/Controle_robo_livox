# Controle Web — Flask + Socket.IO

UI web e ponte ROS2 para o robô. A documentação completa do projeto (modos
TELEOP/SLAM/NAV2/TREKKING, lançamento via `launch.sh`, integração com a
MEGA, mapa, waypoints, gamepad PS4, etc.) está no [README do
repositório](../README.md).

## Ajuste ao vivo do Nav2

Em modo `nav2`, o painel mostra dois multiplicadores que valem somente durante
a sessão:

- **Velocidade pedida** multiplica a velocidade calculada pelo seguidor, até o
  teto físico de 0,70 m/s.
- **Força de curva** multiplica `a_dec` na lei contínua de rumo; o pivô e
  `wz_max` não são alterados.

Os valores aplicados aparecem no CSV do `path_follower` nas colunas
`escala_linear_nav` e `escala_curva_nav`. Reiniciar a web repõe ambos em
`1,00×`.
