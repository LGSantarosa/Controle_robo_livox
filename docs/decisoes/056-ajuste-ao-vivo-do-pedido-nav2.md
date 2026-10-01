# 056 — A web ajusta o pedido do Nav2, não apenas seus tetos

**Data**: 2026-10-01
**Status**: implementado e provado em ROS isolado; falta chão real
**Toca**: web, `path_follower`, `heading_controller` e CSV de corrida

## Contexto

Com a roda omni, curvas abertas e diagonais exigem mais esforço lateral. Subir
`wz_max` de 1,00 para 1,25 não resolveu: a corrida real pediu no máximo 0,53
rad/s. O gargalo estava no pedido, não no teto. Ajustar `a_dec` em pequenos
passos melhorou muito, mas exigia editar, compilar e instalar código a cada
tentativa.

A barra de velocidade que já existia na web também não era solução: ela atua
somente no teleop manual `/web_vel`, que é outra entrada do `twist_mux`.

## Decisão

O modo Nav2 ganha dois multiplicadores de sessão:

1. `linear_scale` (`0,50×–1,40×`) multiplica o pedido linear **depois** de o
   seguidor calcular o limite por curva e frenagem. O resultado não passa de
   0,70 m/s, teto do `diff_drive_controller`.
2. `curve_scale` (`0,50×–3,00×`) multiplica o `a_dec` usado pela lei contínua
   de rumo. O teto `wz_max`, `pivo_a_dec` e a manobra de pivô permanecem
   inalterados.

A web publica os valores em `/nav_tuning/linear_scale` e
`/nav_tuning/curve_scale` como `Float64`, `reliable` e `transient_local`. Assim
o ajuste entra no ciclo seguinte e um nó reiniciado durante a mesma sessão
recebe o último valor. Reiniciar a web publica `1,00×` de novo; o teste não
reescreve YAML silenciosamente.

O CSV do seguidor grava as duas escalas em cada amostra. Isso é obrigatório
porque o operador pode mudá-las no meio da corrida e, sem o valor junto do
dado, não haveria como atribuir a melhora ao ajuste correto.

## Alternativas descartadas

- **Usar a barra manual existente:** altera `/web_vel`, não a autonomia.
- **Subir apenas `v_max`/`wz_max`:** um teto maior não muda um pedido que já
  está abaixo dele; foi exatamente o resultado medido em 30-09.
- **Escalar o `Twist` no fim da cadeia:** também alteraria pivô, ré e outras
  manobras, apagando a separação entre elas.
- **Editar parâmetros ROS pela web:** os nós guardavam os parâmetros num
  dicionário lido na subida; mudar o servidor de parâmetros não mudaria a lei
  em execução sem callbacks adicionais e coordenação entre dois nós.

## Limites da decisão

O multiplicador muda o **pedido**, não promete a velocidade física medida: o
reflexo de colisão, o controlador de base e a própria tração ainda podem
reduzir o resultado. A escala linear também aumenta os pedidos limitados por
curva/frenagem, de propósito; cada valor novo precisa ser avaliado no chão. O
ensaio de oscilação em `a_dec=0,30` foi feito com a roda boba anterior e não é
tratado como limite da planta com omni.
