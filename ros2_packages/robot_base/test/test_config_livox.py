"""Contrato da configuração de rede do Mid-360 (decisão 058)."""

import importlib.util
import ipaddress
import json
from pathlib import Path
import sys

import pytest


RAIZ = Path(__file__).resolve().parents[3]
FERRAMENTA = RAIZ / "tools" / "prepara_config_livox.py"
PERFIS = RAIZ / "ros2_packages" / "robot_base" / "config" / "livox_host_profiles.json"
MODELO = RAIZ / "ros2_packages" / "robot_base" / "config" / "MID360_config.template.json"
SETUP = RAIZ / "setup_livox.sh"

spec = importlib.util.spec_from_file_location("prepara_config_livox", FERRAMENTA)
livox = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = livox
spec.loader.exec_module(livox)


def test_perfis_descrevem_maquinas_sem_amarrar_sensor_a_robo():
    documento = json.loads(PERFIS.read_text())
    assert documento == {
        "perfis": {
            "nuc": {
                "descricao": "Intel NUC",
                "host_ip": "192.168.1.2",
                "prefix_length": 24,
            },
            "notebook": {
                "descricao": "notebook de bancada",
                "host_ip": "192.168.1.5",
                "prefix_length": 24,
            },
        }
    }
    serializado = json.dumps(documento)
    assert "192.168.1.158" not in serializado
    assert "192.168.1.169" not in serializado
    assert "robo2" not in serializado and "robo3" not in serializado


def test_modelo_nao_petrifica_host_nem_unidade_do_sensor():
    texto = MODELO.read_text()
    assert texto.count(livox.TOKEN_HOST) == 4
    assert texto.count(livox.TOKEN_LIDAR) == 1
    assert "192.168.1.2" not in texto
    assert "192.168.1.5" not in texto
    assert "192.168.1.158" not in texto
    assert "192.168.1.169" not in texto
    assert not (MODELO.parent / "MID360_config.json").exists()


def test_host_do_perfil_precisa_existir_localmente():
    perfil = livox.carrega_perfil(PERFIS, "notebook")
    locais = [
        livox.EnderecoLocal(
            "enp1s0", ipaddress.ip_address("192.168.1.2"), 24
        )
    ]
    with pytest.raises(livox.ErroConfiguracao, match="não existe"):
        livox.confirma_host_local(perfil, locais)


def test_mascara_e_interface_ambiguas_tambem_reprovam():
    perfil = livox.carrega_perfil(PERFIS, "notebook")
    ip = ipaddress.ip_address("192.168.1.5")
    with pytest.raises(livox.ErroConfiguracao, match="máscara diferente"):
        livox.confirma_host_local(
            perfil, [livox.EnderecoLocal("enp1s0", ip, 16)]
        )
    with pytest.raises(livox.ErroConfiguracao, match="mais de uma interface"):
        livox.confirma_host_local(
            perfil,
            [
                livox.EnderecoLocal("enp1s0", ip, 24),
                livox.EnderecoLocal("enp2s0", ip, 24),
            ],
        )


@pytest.mark.parametrize(
    "endereco",
    ["não-é-ip", "10.0.0.169", "192.168.1.5", "192.168.1.0", "192.168.1.255"],
)
def test_lidar_ip_explicito_precisa_ser_valido_na_rede(endereco):
    perfil = livox.carrega_perfil(PERFIS, "notebook")
    with pytest.raises(livox.ErroConfiguracao):
        livox.valida_ip_lidar(endereco, perfil)


def test_renderizacao_aplica_os_dois_eixos_sem_deixar_token():
    config = livox.renderiza_config(
        MODELO,
        ipaddress.ip_address("192.168.1.5"),
        ipaddress.ip_address("192.168.1.169"),
    )
    host = config["MID360"]["host_net_info"]
    assert {
        host["cmd_data_ip"],
        host["push_msg_ip"],
        host["point_data_ip"],
        host["imu_data_ip"],
    } == {"192.168.1.5"}
    assert host["log_data_ip"] == ""
    assert config["lidar_configs"][0]["ip"] == "192.168.1.169"
    assert "__" not in json.dumps(config)


def test_varredura_exige_exatamente_um_livox(monkeypatch):
    perfil = livox.carrega_perfil(PERFIS, "notebook")
    sensor_169 = livox.SensorLivox(
        ipaddress.ip_address("192.168.1.169"),
        "e4:7a:2c:90:1d:f1",
        "enp1s0",
    )
    sensor_158 = livox.SensorLivox(
        ipaddress.ip_address("192.168.1.158"),
        "e4:7a:2c:95:df:da",
        "enp1s0",
    )

    monkeypatch.setattr(livox, "descobre_sensores", lambda _perfil: [])
    with pytest.raises(livox.ErroConfiguracao, match="nenhum Livox"):
        livox.resolve_lidar(perfil, None)

    monkeypatch.setattr(
        livox, "descobre_sensores", lambda _perfil: [sensor_158, sensor_169]
    )
    with pytest.raises(livox.ErroConfiguracao, match="mais de um Livox"):
        livox.resolve_lidar(perfil, None)

    monkeypatch.setattr(
        livox, "descobre_sensores", lambda _perfil: [sensor_169]
    )
    endereco, sensor, procedencia = livox.resolve_lidar(perfil, None)
    assert endereco == ipaddress.ip_address("192.168.1.169")
    assert sensor == sensor_169
    assert "varredura" in procedencia


def test_lidar_ip_explicito_offline_e_visivel_mas_nao_finge_confirmacao(monkeypatch):
    perfil = livox.carrega_perfil(PERFIS, "notebook")
    monkeypatch.setattr(livox, "_ping", lambda _endereco: False)
    endereco, sensor, procedencia = livox.resolve_lidar(
        perfil, "192.168.1.169"
    )
    assert endereco == ipaddress.ip_address("192.168.1.169")
    assert sensor is None
    assert "NÃO confirmado" in procedencia


def test_lidar_ip_explicito_que_responde_sem_oui_livox_reprova(monkeypatch):
    perfil = livox.carrega_perfil(PERFIS, "notebook")
    monkeypatch.setattr(livox, "_ping", lambda _endereco: True)
    monkeypatch.setattr(livox, "_sensores_na_vizinhanca", lambda _ips: [])
    with pytest.raises(livox.ErroConfiguracao, match="MAC não tem o OUI Livox"):
        livox.resolve_lidar(perfil, "192.168.1.169")


def test_setup_faz_preflight_antes_de_qualquer_passo_mutavel():
    script = SETUP.read_text()
    chamada = script.index('"${PREPARA_CONFIG[@]}"')
    assert chamada < script.index('echo "==> 0/5')
    assert chamada < script.index("git clone")
    assert chamada < script.index("sudo cmake --install")
    assert "--perfil é obrigatório" in script


def test_setup_copia_a_config_gerada_e_confere_a_copia():
    script = SETUP.read_text()
    assert 'cp -f "$CFG_GERADA" "$CFG_ATIVA"' in script
    assert 'cmp -s "$CFG_GERADA" "$CFG_ATIVA"' in script
    assert "robot_base/config/MID360_config.json" not in script
