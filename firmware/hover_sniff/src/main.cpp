// Contador de bytes crus no RX1 (pino 19) da MEGA.
//
// Varre bauds, 3 s em cada, mandando o frame de speed=0 a 50 Hz pelo TX1
// (pino 18) — o mesmo que a sonda manda em repouso, então NADA se move.
// Para cada baud imprime uma linha:
//
//   baud=115200 rx=540 cdab=30 eco=0  primeiros: CD AB ...
//
//   rx    bytes que chegaram no pino 19 na janela
//   cdab  quantas vezes o par CD AB (início de frame da placa) apareceu
//   eco   bytes que batem com o frame que a MEGA acabou de mandar
//
// Leitura: rx=0 em todos os bauds -> ninguém fala no 19 (cabo, GND, conector,
// placa desligada). rx>0 só num baud com cdab>0 -> placa viva nesse baud.
// rx grande com eco≈rx -> curto/loopback TX↔RX, não é a placa.

#include <Arduino.h>

static const uint16_t START_FRAME = 0xABCD;
static const uint32_t BAUDS[] = {9600, 19200, 38400, 57600, 115200};
static const uint32_t JANELA_MS = 3000;

#pragma pack(push, 1)
struct Command { uint16_t start; int16_t steer; int16_t speed; uint16_t checksum; };
#pragma pack(pop)

static Command zero() {
    Command c;
    c.start = START_FRAME; c.steer = 0; c.speed = 0;
    c.checksum = START_FRAME;  // START ^ 0 ^ 0
    return c;
}

static void janela(uint32_t baud) {
    Serial1.end();
    Serial1.begin(baud);
    while (Serial1.available()) Serial1.read();

    const Command c = zero();
    const uint8_t* tx = (const uint8_t*)&c;
    uint32_t rx = 0, cdab = 0, eco = 0, t_tx = 0;
    uint8_t prev = 0, primeiros[16];
    uint8_t n_prim = 0, pos_eco = 0;

    const uint32_t t0 = millis();
    while (millis() - t0 < JANELA_MS) {
        if (millis() - t_tx >= 20) { t_tx = millis(); Serial1.write(tx, sizeof(c)); }
        while (Serial1.available()) {
            const uint8_t b = (uint8_t)Serial1.read();
            rx++;
            if (n_prim < sizeof(primeiros)) primeiros[n_prim++] = b;
            if (prev == 0xCD && b == 0xAB) cdab++;
            prev = b;
            // eco: o byte recebido é o próximo do frame enviado?
            if (b == tx[pos_eco]) { eco++; pos_eco = (pos_eco + 1) % sizeof(c); }
            else pos_eco = (b == tx[0]) ? 1 : 0;
        }
    }

    Serial.print(F("baud=")); Serial.print(baud);
    Serial.print(F(" rx="));   Serial.print(rx);
    Serial.print(F(" cdab=")); Serial.print(cdab);
    Serial.print(F(" eco="));  Serial.print(eco);
    Serial.print(F("  primeiros:"));
    for (uint8_t i = 0; i < n_prim; i++) {
        Serial.print(' ');
        if (primeiros[i] < 0x10) Serial.print('0');
        Serial.print(primeiros[i], HEX);
    }
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {}
    Serial.println(F("\n== CONTADOR DE BYTES CRUS  TX1=18 RX1=19  speed=0 sempre =="));
}

void loop() {
    for (uint32_t b : BAUDS) janela(b);
    Serial.println(F("--"));
}
