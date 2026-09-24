#!/usr/bin/env python3
"""Resolve a rede do Mid-360 sem confundir máquina com unidade do sensor.

O perfil escolhe somente o endereço da máquina. O endereço do lidar vem de
uma varredura viva ou de ``--lidar-ip`` explícito. A saída é um JSON temporário
que o ``setup_livox.sh`` instala dentro do clone descartável do driver.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import ipaddress
import json
import os
from dataclasses import dataclass
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from typing import Callable, Iterable, Sequence


TOKEN_HOST = "__HOST_IP__"
TOKEN_LIDAR = "__LIDAR_IP__"
OUI_LIVOX = "e4:7a:2c:"


class ErroConfiguracao(RuntimeError):
    """Erro que deve parar o setup antes de qualquer mudança persistente."""


@dataclass(frozen=True)
class PerfilMaquina:
    nome: str
    descricao: str
    host_ip: ipaddress.IPv4Address
    prefix_length: int

    @property
    def rede(self) -> ipaddress.IPv4Network:
        return ipaddress.ip_network(
            f"{self.host_ip}/{self.prefix_length}", strict=False)


@dataclass(frozen=True)
class EnderecoLocal:
    interface: str
    ip: ipaddress.IPv4Address
    prefix_length: int


@dataclass(frozen=True)
class SensorLivox:
    ip: ipaddress.IPv4Address
    mac: str
    interface: str


def _carrega_json(caminho: Path, descricao: str):
    try:
        with caminho.open(encoding="utf-8") as arquivo:
            return json.load(arquivo)
    except (OSError, json.JSONDecodeError) as exc:
        raise ErroConfiguracao(
            f"não foi possível ler {descricao} {caminho}: {exc}"
        ) from exc


def carrega_perfil(caminho: Path, nome: str) -> PerfilMaquina:
    documento = _carrega_json(caminho, "os perfis de máquina")
    perfis = documento.get("perfis") if isinstance(documento, dict) else None
    if not isinstance(perfis, dict) or not perfis:
        raise ErroConfiguracao(f"{caminho} não contém o objeto 'perfis'")
    if nome not in perfis:
        disponiveis = ", ".join(sorted(perfis))
        raise ErroConfiguracao(
            f"perfil de máquina '{nome}' não existe; use um de: {disponiveis}"
        )

    bruto = perfis[nome]
    try:
        host_ip = ipaddress.ip_address(bruto["host_ip"])
        prefix_length = int(bruto["prefix_length"])
        descricao = str(bruto["descricao"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ErroConfiguracao(f"perfil '{nome}' inválido em {caminho}: {exc}") from exc
    if not isinstance(host_ip, ipaddress.IPv4Address):
        raise ErroConfiguracao(f"host_ip do perfil '{nome}' não é IPv4")
    if not 0 <= prefix_length <= 32:
        raise ErroConfiguracao(f"prefix_length do perfil '{nome}' é inválido")

    perfil = PerfilMaquina(nome, descricao, host_ip, prefix_length)
    if perfil.rede.num_addresses > 256:
        raise ErroConfiguracao(
            f"perfil '{nome}' cobre {perfil.rede.num_addresses} endereços; "
            "a varredura é limitada a uma rede /24 ou menor"
        )
    return perfil


def _executa_json(comando: Sequence[str], descricao: str):
    try:
        resultado = subprocess.run(
            comando,
            check=True,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError as exc:
        raise ErroConfiguracao(f"comando necessário ausente: {comando[0]}") from exc
    except subprocess.CalledProcessError as exc:
        detalhe = (exc.stderr or exc.stdout or "sem detalhe").strip()
        raise ErroConfiguracao(f"falha ao {descricao}: {detalhe}") from exc
    try:
        return json.loads(resultado.stdout)
    except json.JSONDecodeError as exc:
        raise ErroConfiguracao(f"saída inválida ao {descricao}: {exc}") from exc


def enderecos_locais() -> list[EnderecoLocal]:
    interfaces = _executa_json(
        ["ip", "-j", "-4", "addr", "show"],
        "ler os endereços IPv4 locais",
    )
    encontrados = []
    for interface in interfaces:
        nome = str(interface.get("ifname", "?"))
        for endereco in interface.get("addr_info", []):
            if endereco.get("family") != "inet":
                continue
            try:
                encontrados.append(
                    EnderecoLocal(
                        nome,
                        ipaddress.ip_address(endereco["local"]),
                        int(endereco["prefixlen"]),
                    )
                )
            except (KeyError, ValueError):
                continue
    return encontrados


def confirma_host_local(
    perfil: PerfilMaquina,
    enderecos: Iterable[EnderecoLocal],
) -> EnderecoLocal:
    enderecos = list(enderecos)
    mesmo_ip = [endereco for endereco in enderecos if endereco.ip == perfil.host_ip]
    if not mesmo_ip:
        presentes = ", ".join(
            f"{item.interface}={item.ip}/{item.prefix_length}" for item in enderecos
        ) or "nenhum IPv4"
        raise ErroConfiguracao(
            f"o perfil '{perfil.nome}' exige {perfil.host_ip}/{perfil.prefix_length}, "
            f"mas esse IP não existe em nenhuma interface local (presentes: {presentes})"
        )

    exato = [
        endereco for endereco in mesmo_ip
        if endereco.prefix_length == perfil.prefix_length
    ]
    if not exato:
        mascaras = ", ".join(
            f"{item.interface}=/{item.prefix_length}" for item in mesmo_ip
        )
        raise ErroConfiguracao(
            f"{perfil.host_ip} existe, mas com máscara diferente da /"
            f"{perfil.prefix_length} do perfil ({mascaras})"
        )
    if len(exato) > 1:
        interfaces = ", ".join(item.interface for item in exato)
        raise ErroConfiguracao(
            f"{perfil.host_ip}/{perfil.prefix_length} aparece em mais de uma "
            f"interface ({interfaces}); a rota do lidar é ambígua"
        )
    return exato[0]


def valida_ip_lidar(
    valor: str,
    perfil: PerfilMaquina,
) -> ipaddress.IPv4Address:
    try:
        endereco = ipaddress.ip_address(valor)
    except ValueError as exc:
        raise ErroConfiguracao(f"IP do lidar inválido: {valor!r}") from exc
    if not isinstance(endereco, ipaddress.IPv4Address):
        raise ErroConfiguracao("o IP do lidar precisa ser IPv4")
    if endereco not in perfil.rede:
        raise ErroConfiguracao(
            f"o lidar {endereco} não pertence à rede {perfil.rede} do perfil "
            f"'{perfil.nome}'"
        )
    proibidos = {
        perfil.host_ip,
        perfil.rede.network_address,
        perfil.rede.broadcast_address,
    }
    if endereco in proibidos:
        raise ErroConfiguracao(
            f"{endereco} não pode ser o IP do lidar nesse perfil"
        )
    return endereco


def _ping(endereco: ipaddress.IPv4Address) -> bool:
    if shutil.which("ping") is None:
        raise ErroConfiguracao("comando necessário ausente: ping")
    resultado = subprocess.run(
        ["ping", "-c", "1", "-W", "1", "-w", "2", str(endereco)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return resultado.returncode == 0


def _sensores_na_vizinhanca(
    respondentes: set[ipaddress.IPv4Address],
) -> list[SensorLivox]:
    vizinhos = _executa_json(
        ["ip", "-j", "neigh", "show"],
        "ler a tabela de vizinhos depois da varredura",
    )
    sensores = []
    for vizinho in vizinhos:
        try:
            endereco = ipaddress.ip_address(vizinho["dst"])
            mac = str(vizinho["lladdr"]).lower()
        except (KeyError, ValueError):
            continue
        if endereco in respondentes and mac.startswith(OUI_LIVOX):
            sensores.append(
                SensorLivox(endereco, mac, str(vizinho.get("dev", "?")))
            )
    return sorted(set(sensores), key=lambda sensor: int(sensor.ip))


def descobre_sensores(
    perfil: PerfilMaquina,
    ping: Callable[[ipaddress.IPv4Address], bool] = _ping,
) -> list[SensorLivox]:
    alvos = [ip for ip in perfil.rede.hosts() if ip != perfil.host_ip]
    respondentes: set[ipaddress.IPv4Address] = set()
    with concurrent.futures.ThreadPoolExecutor(
        max_workers=min(128, len(alvos) or 1)
    ) as executor:
        futuros = {executor.submit(ping, ip): ip for ip in alvos}
        for futuro in concurrent.futures.as_completed(futuros):
            if futuro.result():
                respondentes.add(futuros[futuro])
    return _sensores_na_vizinhanca(respondentes)


def resolve_lidar(
    perfil: PerfilMaquina,
    lidar_ip_explicito: str | None,
) -> tuple[ipaddress.IPv4Address, SensorLivox | None, str]:
    if lidar_ip_explicito is not None:
        endereco = valida_ip_lidar(lidar_ip_explicito, perfil)
        if not _ping(endereco):
            return endereco, None, "explícito, mas NÃO confirmado na rede"
        sensores = _sensores_na_vizinhanca({endereco})
        if not sensores:
            raise ErroConfiguracao(
                f"{endereco} respondeu, mas seu MAC não tem o OUI Livox "
                f"{OUI_LIVOX[:-1]}"
            )
        return endereco, sensores[0], "explícito e confirmado pelo MAC"

    sensores = descobre_sensores(perfil)
    if not sensores:
        raise ErroConfiguracao(
            f"nenhum Livox respondeu em {perfil.rede}; ligue o sensor e repita "
            "ou informe conscientemente --lidar-ip para preparar offline"
        )
    if len(sensores) > 1:
        lista = ", ".join(f"{s.ip} ({s.mac})" for s in sensores)
        raise ErroConfiguracao(
            f"mais de um Livox respondeu em {perfil.rede}: {lista}; "
            "escolha um com --lidar-ip"
        )
    return sensores[0].ip, sensores[0], "descoberto pela varredura e pelo MAC"


def _substitui_tokens(valor, host_ip: str, lidar_ip: str, contagem: dict[str, int]):
    if isinstance(valor, dict):
        return {
            chave: _substitui_tokens(item, host_ip, lidar_ip, contagem)
            for chave, item in valor.items()
        }
    if isinstance(valor, list):
        return [
            _substitui_tokens(item, host_ip, lidar_ip, contagem)
            for item in valor
        ]
    if valor == TOKEN_HOST:
        contagem["host"] += 1
        return host_ip
    if valor == TOKEN_LIDAR:
        contagem["lidar"] += 1
        return lidar_ip
    return valor


def renderiza_config(
    modelo_path: Path,
    host_ip: ipaddress.IPv4Address,
    lidar_ip: ipaddress.IPv4Address,
) -> dict:
    modelo = _carrega_json(modelo_path, "o modelo da configuração")
    contagem = {"host": 0, "lidar": 0}
    config = _substitui_tokens(
        modelo,
        str(host_ip),
        str(lidar_ip),
        contagem,
    )
    if contagem != {"host": 4, "lidar": 1}:
        raise ErroConfiguracao(
            f"tokens inesperados em {modelo_path}: esperado host=4/lidar=1, "
            f"encontrado host={contagem['host']}/lidar={contagem['lidar']}"
        )
    serializado = json.dumps(config)
    if TOKEN_HOST in serializado or TOKEN_LIDAR in serializado:
        raise ErroConfiguracao(f"sobrou token sem resolver em {modelo_path}")
    return config


def grava_config_atomica(config: dict, saida: Path) -> None:
    saida.parent.mkdir(parents=True, exist_ok=True)
    temporario = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=saida.parent,
            prefix=f".{saida.name}.",
            delete=False,
        ) as arquivo:
            temporario = Path(arquivo.name)
            json.dump(config, arquivo, indent=2)
            arquivo.write("\n")
        os.replace(temporario, saida)
    finally:
        if temporario is not None and temporario.exists():
            temporario.unlink()


def cria_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Valida máquina + sensor e gera o MID360_config.json ativo."
    )
    parser.add_argument("--perfis", required=True, type=Path)
    parser.add_argument("--modelo", required=True, type=Path)
    parser.add_argument("--perfil", required=True)
    parser.add_argument(
        "--lidar-ip",
        help=(
            "IP deliberadamente informado; sem esta opção, varre a rede e "
            "exige exatamente um MAC Livox"
        ),
    )
    parser.add_argument("--saida", required=True, type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = cria_parser().parse_args(argv)
    try:
        perfil = carrega_perfil(args.perfis, args.perfil)
        endereco_local = confirma_host_local(perfil, enderecos_locais())
        lidar_ip, sensor, procedencia = resolve_lidar(perfil, args.lidar_ip)
        config = renderiza_config(args.modelo, perfil.host_ip, lidar_ip)
        grava_config_atomica(config, args.saida)
    except ErroConfiguracao as exc:
        print(f"ERRO: {exc}", file=sys.stderr)
        return 1

    print(f"    perfil de máquina: {perfil.nome} — {perfil.descricao}")
    print(
        f"    host local:        {perfil.host_ip}/{perfil.prefix_length} "
        f"em {endereco_local.interface}"
    )
    if sensor is None:
        print(f"    AVISO: lidar {lidar_ip}: {procedencia}", file=sys.stderr)
        print("           Isto prepara o arquivo; NÃO prova que essa unidade está presente.",
              file=sys.stderr)
    else:
        print(f"    lidar:             {lidar_ip}, MAC {sensor.mac}")
        print(f"    procedência:       {procedencia}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
