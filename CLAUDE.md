# Controle_robo_livox — instruções pro assistente

Robô diferencial (2 rodas hover + boba) com Livox Mid-360 e Intel NUC.
**Projeto PIBIT**: vira artigo científico. Nasceu como clone do
`Controle_robo_web` (robô 1, skid-steer 4 rodas) — histórico git completo.

## Método de trabalho (inegociável)

1. **PERGUNTE ANTES DE AGIR**: diga o que entendeu + o que pretende fazer e
   ESPERE o ok. Confirme a CAUSA antes de propor SOLUÇÃO.
2. **1 mudança pequena por vez**, cada uma com justificativa escrita ANTES.
   Ritmo devagar de propósito — aqui método vale mais que velocidade.
3. **Documentação PIBIT é parte do trabalho, não burocracia**:
   - decisão técnica → registro em `docs/decisoes/NNN-slug.md` (contexto,
     alternativas, por que descartamos as outras, referências);
   - sessão de trabalho → entrada no `docs/DIARIO.md` (inclusive fracassos);
   - escolhas de abordagem embasadas em literatura → `docs/REFERENCIAS.md`.
4. **`ESTADO_PROJETO.md` é o estado vivo** — manter atualizado; é o handoff
   entre PCs e sessões (a memória do assistente NÃO cruza máquinas).
5. **Eu leio logs/CSV e diagnostico; o dono SÓ RODA**: instrumentar em CSV
   que se puxa via ssh; nunca pedir pro dono relatar console.
6. Trabalhar linear, sem subagentes. Testes hands-on: anunciar e esperar o
   "pode". Avisar quando o robô precisa estar LIGADO vs DESLIGADO.
7. Commits SEM rodapé de autoria do assistente (sem Co-Authored-By).
8. Deploy = dev → commit → push → `git fetch && git reset --hard origin/main`
   no robô. NUNCA scp de arquivos soltos.
9. Complicou? Abortar e reverter pro último estado bom (branch de backup antes).

## Cuidados herdados do robô 1 que NÃO valem aqui

- Os knobs anti-skid (zona-morta 1.7, autoridade de giro 6.0, spin_calib,
  slow_wz_cap, proibição de arco) eram do atrito do skid-steer 4 rodas.
  O diferencial gira fácil: calibrar do zero, não herdar.
- O que VALE dos dois lados: nunca escalar wz parcialmente sem conhecer a
  zona-morta do atuador; humano tem prioridade sobre goal; mudança grande
  não vai blind pro robô.

## Referências rápidas

- Plano de migração: `MIGRACAO_LIVOX.md` (raiz).
- Robô 1 (base deste repo): github `LGSantarosa/Controle_robo_web` — portar
  correção boa de lá é `git cherry-pick` manual, repos independentes.
