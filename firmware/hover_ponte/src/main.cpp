// Ponte transparente: USB (Serial) <-> placa de hover (Serial1), ambos 115200.
//
// Sem lógica nenhuma de propósito: o protocolo 0xABCD, o watchdog e a
// segurança continuam no PC (hoverboard_driver) e na placa (timeout do EFeru).
// Se o PC calar, a MEGA cala, e a placa desarma pelo timeout dela.
//
// Abrir a porta USB reseta a MEGA (DTR): os primeiros ~2 s depois do open se
// perdem. Quem abre a porta tem de esperar antes de mandar comando.

#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    // Pull-up no RX1: com a placa sem transmitir o 19 flutua e vira ruído
    // (medido em 2026-09-10 com o hover_sniff).
    pinMode(19, INPUT_PULLUP);
}

void loop() {
    while (Serial.available())  Serial1.write((uint8_t)Serial.read());
    while (Serial1.available()) Serial.write((uint8_t)Serial1.read());
}
