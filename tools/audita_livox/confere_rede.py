#!/usr/bin/env python3
"""Checagens 7 e 8 do ``bin/audita-livox`` pelo contrato da decisão 059.

Sob a decisão 058 não existe mais um JSON universal para comparar byte a
byte: o certo depende da máquina (perfil) e da unidade (sensor escolhido no
setup). Então o JSON ativo é certo quando é o template materializado com os
PRÓPRIOS IPs dele, e esses IPs cabem num perfil versionado. As regras vêm do
``tools/prepara_config_livox.py`` por import, nunca reescritas: se o setup
mudar a regra, o auditor muda junto.

Offline e sem efeito colateral: nada de ping, MAC, nuvem ou /Odometry.

Entrada de interfaces: o texto de ``ip -brief addr`` no stdin (o mesmo que o
auditor já lia). Saída: duas linhas ``nome<TAB>VEREDITO<TAB>detalhe``, uma
para ``config`` e outra para ``host``; o bash registra as duas.
"""

from __future__ import annotations

import argparse
import ipaddress
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from prepara_config_livox import (  # noqa: E402
    ErroConfiguracao,
    PerfilMaquina,
    TOKEN_HOST,
    TOKEN_LIDAR,
    carrega_perfil,
    renderiza_config,
    valida_ip_lidar,
)

CAMPOS_HOST = ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip")


class Reprovado(Exception):
    pass


def _ipv4_concreto(valor, rotulo: str) -> ipaddress.IPv4Address:
    if valor in (TOKEN_HOST, TOKEN_LIDAR):
        raise Reprovado(f"{rotulo} ainda é o placeholder {valor}")
    try:
        endereco = ipaddress.ip_address(valor)
    except (TypeError, ValueError):
        raise Reprovado(f"{rotulo} não é IPv4: {valor!r}") from None
    if not isinstance(endereco, ipaddress.IPv4Address):
        raise Reprovado(f"{rotulo} não é IPv4: {valor!r}")
    return endereco


def le_ips(config) -> tuple[ipaddress.IPv4Address, ipaddress.IPv4Address]:
    try:
        host_net = config["MID360"]["host_net_info"]
    except (KeyError, TypeError):
        raise Reprovado("sem MID360.host_net_info") from None
    # Estrutura errada é defeito do JSON (REPROVADO), não queda do conferidor.
    if not isinstance(host_net, dict):
        raise Reprovado(
            f"MID360.host_net_info não é objeto ({type(host_net).__name__})")
    hosts = [_ipv4_concreto(host_net.get(campo), campo) for campo in CAMPOS_HOST]
    if len(set(hosts)) != 1:
        lista = ", ".join(f"{c}={h}" for c, h in zip(CAMPOS_HOST, hosts))
        raise Reprovado(f"os quatro campos de host divergem ({lista})")

    lidares = config.get("lidar_configs") if isinstance(config, dict) else None
    if not isinstance(lidares, list) or len(lidares) != 1:
        raise Reprovado("lidar_configs precisa ter exatamente uma entrada")
    if not isinstance(lidares[0], dict):
        raise Reprovado(
            f"lidar_configs[0] não é objeto ({type(lidares[0]).__name__})")
    lidar = _ipv4_concreto(lidares[0].get("ip"), "lidar_configs[0].ip")
    return hosts[0], lidar


def perfil_do_host(perfis_path: Path, host: ipaddress.IPv4Address) -> PerfilMaquina:
    try:
        documento = json.loads(perfis_path.read_text(encoding="utf-8"))
        nomes = list(documento["perfis"])
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise Reprovado(f"não li os perfis {perfis_path}: {exc}") from None
    try:
        perfis = [carrega_perfil(perfis_path, nome) for nome in nomes]
    except ErroConfiguracao as exc:
        raise Reprovado(str(exc)) from None
    casados = [p for p in perfis if p.host_ip == host]
    if not casados:
        raise Reprovado(
            f"o host {host} não corresponde a nenhum perfil de máquina "
            f"({', '.join(f'{p.nome}={p.host_ip}' for p in perfis)})")
    if len(casados) > 1:
        raise Reprovado(
            f"o host {host} corresponde a mais de um perfil "
            f"({', '.join(p.nome for p in casados)})")
    return casados[0]


def confere_config(runtime: Path, modelo: Path, perfis: Path):
    """Checagem 7. Devolve (veredito, detalhe, perfil ou None, host ou None)."""
    for arquivo in (modelo, perfis):
        if not arquivo.is_file():
            return "REPROVADO", f"sem o versionado {arquivo}", None, None
    try:
        config = json.loads(runtime.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        return "REPROVADO", f"JSON de runtime ilegível: {exc}", None, None

    host = perfil = None
    try:
        host, lidar = le_ips(config)
        perfil = perfil_do_host(perfis, host)
        try:
            valida_ip_lidar(str(lidar), perfil)
        except ErroConfiguracao as exc:
            raise Reprovado(f"sensor {lidar}: {exc}") from None
        try:
            esperado = renderiza_config(modelo, host, lidar)
        except ErroConfiguracao as exc:
            raise Reprovado(str(exc)) from None
        if config != esperado:
            raise Reprovado(
                f"difere do template materializado com os próprios IPs "
                f"({host}/{lidar}) — editado à mão? rode o setup_livox.sh")
    except Reprovado as exc:
        return "REPROVADO", str(exc), perfil, host
    return (
        "APROVADO",
        f"perfil {perfil.nome}: host {host}/{perfil.prefix_length}, sensor "
        f"{lidar} (não consultado), igual ao template materializado",
        perfil,
        host,
    )


def le_ip_brief(texto: str):
    """Linhas de ``ip -brief addr``: interface, estado, endereços."""
    enderecos = []
    for linha in texto.splitlines():
        campos = linha.split()
        if len(campos) < 3:
            continue
        for campo in campos[2:]:
            try:
                interface = ipaddress.ip_interface(campo)
            except ValueError:
                continue
            if isinstance(interface, ipaddress.IPv4Interface):
                enderecos.append((campos[0], campos[1], interface))
    return enderecos


def confere_host(perfil, host, texto_ip: str | None):
    """Checagem 8. Host do RUNTIME, máscara do perfil que casou."""
    if host is None:
        return "INCONCLUSIVO", "não consegui ler o host do JSON de runtime"
    if perfil is None:
        return "INCONCLUSIVO", f"o host {host} não tem perfil — sem máscara esperada"
    if texto_ip is None:
        return "INCONCLUSIVO", "comando 'ip' indisponível"

    enderecos = le_ip_brief(texto_ip)
    esperado = f"{host}/{perfil.prefix_length}"
    exatos = [(i, e) for i, e, a in enderecos
              if a.ip == host and a.network.prefixlen == perfil.prefix_length]
    if len(exatos) == 1:
        interface, estado = exatos[0]
        return "APROVADO", f"{esperado} em {interface} ({estado})"
    if len(exatos) > 1:
        return "REPROVADO", (
            f"{esperado} aparece em mais de uma interface "
            f"({', '.join(i for i, _ in exatos)}); a rota do lidar é ambígua")

    outra_mascara = [(i, a) for i, _, a in enderecos if a.ip == host]
    if outra_mascara:
        lista = ", ".join(f"{i} {a}" for i, a in outra_mascara)
        return "REPROVADO", f"o JSON espera {esperado}, mas o host está como {lista}"

    na_rede = [(i, a) for i, _, a in enderecos if a.ip in perfil.rede]
    if na_rede:
        interface, endereco = na_rede[0]
        return "REPROVADO", (
            f"o JSON espera {host}, mas esta máquina está na rede como "
            f"{interface} {endereco}")
    return "INCONCLUSIVO", (
        f"{host} não está em nenhuma interface, e não há nada na {perfil.rede} "
        "(cabo fora?)")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--runtime", required=True, type=Path)
    parser.add_argument("--modelo", required=True, type=Path)
    parser.add_argument("--perfis", required=True, type=Path)
    parser.add_argument("--sem-ip", action="store_true",
                        help="o comando 'ip' não existe nesta máquina")
    args = parser.parse_args(argv)

    veredito, detalhe, perfil, host = confere_config(
        args.runtime, args.modelo, args.perfis)
    print(f"config\t{veredito}\t{detalhe}")
    texto_ip = None if args.sem_ip else sys.stdin.read()
    print("host\t%s\t%s" % confere_host(perfil, host, texto_ip))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
