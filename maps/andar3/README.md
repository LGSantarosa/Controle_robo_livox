# Mapas do andar 3

Os dois mapas do andar 3 que o robô do estágio usa, com a régua que importa
para ESTE robô medida em 12-08.

```
mapa                       tamanho        folga p10   corpo não cabe
scan_andar3_ajustado    73,4 x 20,0 m      0,400 m        7,5%
mapa_3_andar            48,7 x 40,4 m      0,461 m        6,7%
scan_andar3 (não copiado, o cru)           0,212 m       13,3%
```

**`scan_andar3_ajustado` é o padrão do roteiro.** É a versão limpa do
`scan_andar3` (mesma geometria, 1468 x 399 células a 5 cm, origem
[-28,672 · -15,476]) e a limpeza é medível: o p10 de folga quase dobra e a
fração de célula livre onde o corpo não cabe cai de 13,3% para 7,5%.

`mapa_3_andar` é OUTRO recorte (973 x 808, origem [-23,7 · -11,6]) e tem a
melhor folga dos três — mas cobre uma área diferente. Está aqui para poder
trocar por argumento, sem chute, se a sala do teste não estiver no
`ajustado`.

⚠️ **A régua**: `folga p10` é o décimo percentil da distância de cada célula
livre até a célula ocupada mais próxima; "corpo não cabe" é a fração dessas
células onde essa distância é menor que o `robot_radius` de 0,32 m. O p50 NÃO
serve aqui — ele satura no alcance de busca de 1,5 m do `folga.py` nos cinco
mapas testados, e portanto não separa nada.

⚠️ Estes números **não** são comparáveis com a "mediana de folga 0,35 m"
registrada para o `meu_mapa` em 12-08: aquela veio de outro método, e a
comparação entre eles não foi feita.
